"""Project an arm pose into the target RGB optical frame for manual alignment."""

import cv2
import numpy as np
from trimesh import ray

from .geometry import validate_rigid


def rasterize_arm_mask(mesh, transform, k, shape, *, near_m=0.02, crop_origin=(0, 0), scale=1):
    """Compute occupancy without a dense R-tree query, including near-plane clipping."""
    optical = (mesh.vertices - transform[:3, 3]) @ transform[:3, :3]
    front = optical[:, 2] >= near_m
    # Sample our requested grid from a 2× canvas to reduce OpenCV's pixel
    # rounding at polygon edges, while keeping coordinates in original pixels.
    supersample, shift = 2, 8
    mask = np.zeros(tuple(size * supersample for size in shape), dtype=np.uint8)
    offset = np.asarray(crop_origin) - 0.5
    gain = scale * supersample * (1 << shift)
    half = 0.5 * supersample * (1 << shift)

    def pixels(points):
        homogeneous = points @ k.T
        uv = homogeneous[:, :2] / np.maximum(homogeneous[:, 2, None], near_m)
        return np.rint((uv - offset) * gain - half).astype(np.int32)

    faces = mesh.faces
    visibility = front[faces]
    triangles = pixels(optical)[faces[visibility.all(axis=1)]]
    # Avoid native draw calls for triangles wholly outside the crop.
    limit = np.array(mask.shape[::-1]) << shift
    intersects = ((triangles.max(axis=1) >= 0) & (triangles.min(axis=1) < limit)).all(axis=1)
    for triangle in triangles[intersects]:
        cv2.fillConvexPoly(mask, triangle, 1, shift=shift)

    for triangle in optical[faces[visibility.any(axis=1) & ~visibility.all(axis=1)]]:
        clipped = []
        previous = triangle[-1]
        for current in triangle:
            if (previous[2] >= near_m) != (current[2] >= near_m):
                clipped.append(previous + (current - previous) * ((near_m - previous[2]) / (current[2] - previous[2])))
            if current[2] >= near_m:
                clipped.append(current)
            previous = current
        cv2.fillConvexPoly(mask, pixels(np.asarray(clipped)), 1, shift=shift)
    return np.ascontiguousarray(mask[::supersample, ::supersample])


def project_arm_mask(torso_mesh, torso_from_optical, intrinsics, shape, *, near_m=0.02):
    """Rasterize the silhouette of a posed upper-limb mesh in torso metres."""
    transform = np.asarray(torso_from_optical, dtype=float)
    validate_rigid(transform, "target torso-from-camera transform")
    k = np.asarray(intrinsics, dtype=float)
    if (
        not np.isfinite(transform).all()
        or k.shape != (3, 3)
        or not np.isfinite(k).all()
        or min(k[0, 0], k[1, 1]) <= 0
        or k[0, 1] != 0
        or k[1, 0] != 0
        or not np.allclose(k[2], [0, 0, 1])
        or not np.isfinite(near_m)
        or near_m <= 0
    ):
        raise ValueError("finite aligned optical intrinsics and a positive near plane required")

    if not ray.has_embree:
        return rasterize_arm_mask(torso_mesh, transform, k, shape, near_m=near_m).astype(bool)
    optical_mesh = torso_mesh.copy()
    optical_mesh.apply_transform(np.linalg.inv(transform))
    v, u = np.indices(shape)
    rays = np.column_stack(((u.ravel() - k[0, 2]) / k[0, 0], (v.ravel() - k[1, 2]) / k[1, 1], np.ones(u.size)))
    return optical_mesh.ray.intersects_any(near_m * rays, rays).reshape(shape)


def overlay_arm_target(rgb, mask):
    """Draw a thin outline and a light tint on an OpenCV BGR image."""
    if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8 or rgb.shape[:2] != mask.shape:
        raise ValueError("the original uint8 color image and target mask must have matching sizes")
    result = rgb.copy()
    target = np.array([245, 215, 52], dtype=np.uint8)
    result[mask] = np.rint(0.86 * result[mask] + 0.14 * target).astype(np.uint8)
    interior = cv2.erode(mask.astype(np.uint8), np.ones((3, 3), dtype=np.uint8)).astype(bool)
    result[mask & ~interior] = target
    return result


def rigid_hand_arm_mesh(model, arm, joint_positions, *, data=None):
    """Pose the G1 fixed hand and arm in torso metres using its native MJCF geometry."""
    import mujoco
    import trimesh

    if arm not in ("left", "right"):
        raise ValueError("arm must be left or right")
    if data is None:
        data = mujoco.MjData(model)
    bodies = {
        index
        for index in range(model.nbody)
        if model.body(index).name.startswith(tuple(f"{arm}_{s}_" for s in ("shoulder", "elbow", "wrist")))
    }
    wrist = model.body(f"{arm}_wrist_yaw_link").id
    fixed_hand = model.mesh(f"{arm}_rubber_hand").id
    visual_ids = tuple(
        index
        for index in range(model.ngeom)
        if model.geom_bodyid[index] in bodies
        and model.geom_group[index] == 1
        and model.geom_type[index] == mujoco.mjtGeom.mjGEOM_MESH
    )
    if sum(model.geom_bodyid[idx] == wrist and model.geom_dataid[idx] == fixed_hand for idx in visual_ids) != 1:
        raise ValueError(f"expected one authored {arm} fixed hand on the wrist yaw link")
    for index in range(model.njnt):
        if model.jnt_bodyid[index] not in bodies:
            continue
        name = model.joint(index).name
        angle = joint_positions.get(name)
        if model.jnt_type[index] != mujoco.mjtJoint.mjJNT_HINGE or angle is None or not np.isfinite(angle):
            raise ValueError(f"a finite hinge position is required for {name}")
        data.qpos[model.jnt_qposadr[index]] = angle
    mujoco.mj_forward(model, data)

    torso = data.body("torso_link")
    world_from_torso = torso.xmat.reshape(3, 3)
    meshes = []
    for index in visual_ids:
        mesh_id = model.geom_dataid[index]
        verts = slice(model.mesh_vertadr[mesh_id], model.mesh_vertadr[mesh_id] + model.mesh_vertnum[mesh_id])
        faces = slice(model.mesh_faceadr[mesh_id], model.mesh_faceadr[mesh_id] + model.mesh_facenum[mesh_id])
        world_vertices = model.mesh_vert[verts] @ data.geom_xmat[index].reshape(3, 3).T + data.geom_xpos[index]
        torso_vertices = (world_vertices - torso.xpos) @ world_from_torso
        meshes.append(trimesh.Trimesh(torso_vertices, model.mesh_face[faces], process=False))
    return trimesh.util.concatenate(meshes), visual_ids
