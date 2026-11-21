"""Estimate a passive G1 head mount from directional RGB contours."""

import cv2
import numpy as np
from scipy.ndimage import map_coordinates
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from trimesh import ray

from .geometry import validate_rigid
from .overlay import rasterize_arm_mask

# All image distances are in original RGB pixels.
MAX_DISTANCE = 8.0
MIN_COVERAGE = 0.64
AMBIGUITY_GAP = 0.05
MIN_SAMPLES = 24
HAND_NEIGHBORHOOD_M = 0.10


def contour_pose_transform(zero, parameters):
    """Return torso-from-optical, using positive back degrees and pivot X/Z in mm."""
    angle, x, z = parameters
    rotation = Rotation.from_rotvec([0, -np.deg2rad(angle), 0]).as_matrix()
    pivot = np.array([x, 0, z]) / 1000
    mount = np.eye(4)
    mount[:3, :3] = rotation
    mount[:3, 3] = pivot - rotation @ pivot
    return mount @ zero


def _project(points, transform, k):
    optical = (points - transform[:3, 3]) @ transform[:3, :3]
    uv = optical @ k.T
    return uv[:, :2] / np.maximum(uv[:, 2, None], 0.02), optical[:, 2]


def _sparse_hits(mesh, origin, rays, uv, projected, depth):
    """Query candidate triangle boxes in the image, then intersect sparse rays in 3D."""
    from rtree.index import Index

    face_depth = depth[mesh.faces]
    in_front = face_depth >= 0.02
    projected_faces = projected[mesh.faces]
    lower, upper = projected_faces.min(axis=1), projected_faces.max(axis=1)
    visible = in_front.all(axis=1) & ((upper >= uv.min(axis=0)) & (lower <= uv.max(axis=0))).all(axis=1)
    fully_visible = np.flatnonzero(visible)
    tree = (
        Index(
            zip(
                fully_visible.tolist(),
                np.column_stack((lower[visible], upper[visible])).tolist(),
                [None] * len(fully_visible),
                strict=True,
            )
        )
        if len(fully_visible)
        else None
    )
    straddling = np.flatnonzero(in_front.any(axis=1) & ~in_front.all(axis=1))
    candidate_ids, ray_ids = [], []
    for index, (x, y) in enumerate(uv):
        faces = list(tree.intersection((x, y, x, y))) if tree is not None else []
        faces.extend(straddling)
        candidate_ids.extend(faces)
        ray_ids.extend([index] * len(faces))
    if not candidate_ids:
        return np.empty((0, 3)), np.empty(0, dtype=int)

    triangles = mesh.triangles[candidate_ids]
    ray_ids = np.asarray(ray_ids)
    directions = rays[ray_ids]
    edge1, edge2 = triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]
    normal = np.cross(directions, edge2)
    determinant = np.einsum("ij,ij->i", edge1, normal)
    valid = np.abs(determinant) > 1e-12
    inverse = np.zeros_like(determinant)
    np.divide(1.0, determinant, out=inverse, where=valid)
    relative = origin - triangles[:, 0]
    side = np.cross(relative, edge1)
    u = np.einsum("ij,ij->i", relative, normal) * inverse
    v = np.einsum("ij,ij->i", directions, side) * inverse
    z = np.einsum("ij,ij->i", edge2, side) * inverse
    valid &= (u >= -1e-9) & (v >= -1e-9) & (u + v <= 1 + 1e-9) & (z >= 0.02)
    z, ray_ids = z[valid], ray_ids[valid]
    order = np.lexsort((z, ray_ids))
    _, first = np.unique(ray_ids[order], return_index=True)
    nearest = order[first]
    return origin + z[nearest, None] * rays[ray_ids[nearest]], ray_ids[nearest]


def _silhouette(mesh, transform, k, shape):
    """Return visible boundary points in torso and 2D normals from a subpixel ray mask."""
    height, width = shape
    r, origin = transform[:3, :3], transform[:3, 3]
    projected, depth = _project(mesh.vertices, transform, k)
    if not (depth > 0.02).any():
        return np.empty((0, 3)), np.empty((0, 2))
    lo = np.maximum(np.floor(projected.min(axis=0)).astype(int) - 2, 0)
    hi = np.minimum(np.ceil(projected.max(axis=0)).astype(int) + 3, [width, height])
    if np.any(hi <= lo):
        return np.empty((0, 3)), np.empty((0, 2))
    # The mask is cropped in the original image frame. Ray origins move with the
    # camera so Embree's acceleration structure stays in the posed torso frame.
    scale = 3
    mask_shape = tuple(((hi - lo) * scale)[::-1])
    if ray.has_embree:
        v, u = np.indices(mask_shape)
        grid = (np.column_stack((u.ravel(), v.ravel())) + 0.5) / scale - 0.5 + lo
        rays = np.column_stack(((grid - k[:2, 2]) / k.diagonal()[:2], np.ones(len(grid)))) @ r.T
        occupied = mesh.ray.intersects_any(origin + 0.02 * rays, rays).reshape(u.shape).astype(np.uint8)
    else:
        occupied = rasterize_arm_mask(mesh, transform, k, mask_shape, crop_origin=lo, scale=scale)
    contours, _ = cv2.findContours(occupied, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    samples, normals = [], []
    for contour in contours:
        contour = contour[:, 0, :]
        if len(contour) < 24:
            continue
        tangent = np.roll(contour, -6, axis=0) - np.roll(contour, 6, axis=0)
        normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
        length = np.linalg.norm(normal, axis=1)
        uv = (contour + 0.5) / scale - 0.5 + lo
        valid = (length > 0) & ((uv >= 3) & (uv < [width - 3, height - 3])).all(axis=1)
        indices = np.flatnonzero(valid)[::6]
        samples.append(uv[indices])
        normals.append(normal[indices] / length[indices, None])
    if not samples:
        return np.empty((0, 3)), np.empty((0, 2))
    uv, normal = np.vstack(samples), np.vstack(normals)
    rays = np.column_stack(((uv - k[:2, 2]) / k.diagonal()[:2], np.ones(len(uv)))) @ r.T
    if ray.has_embree:
        points, ids, _ = mesh.ray.intersects_location(origin + 0.02 * rays, rays, multiple_hits=False)
    else:
        points, ids = _sparse_hits(mesh, origin, rays, uv, projected, depth)
    # The last occupied subpixel is just inside the contour. Refine that
    # projected center toward the boundary while retaining the ray-hit depth.
    z = ((points - origin) @ r)[:, 2]
    offset = normal[ids] * z[:, None] / (2 * scale * k.diagonal()[:2])
    return points + offset @ r[:, :2].T, normal[ids]


def _image_edges(rgb):
    grey = cv2.GaussianBlur(cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY), (3, 3), 0)
    edge = cv2.Canny(grey, 20, 60, L2gradient=True)
    v, u = np.where(edge)
    if len(u) < MIN_SAMPLES:
        return ()
    gx = cv2.Sobel(grey, cv2.CV_32F, 1, 0)
    gy = cv2.Sobel(grey, cv2.CV_32F, 0, 1)
    magnitude = np.hypot(gx, gy)
    normal = np.column_stack((gx[v, u], gy[v, u])) / magnitude[v, u, None]
    uv = np.column_stack((u, v)).astype(float)
    before = map_coordinates(magnitude, (uv - normal).T[::-1], order=1, mode="nearest")
    after = map_coordinates(magnitude, (uv + normal).T[::-1], order=1, mode="nearest")
    curvature = before + after - 2 * magnitude[v, u]
    displacement = np.zeros(len(uv))
    np.divide(0.5 * (before - after), curvature, out=displacement, where=curvature < -1e-3)
    uv += np.clip(displacement, -0.5, 0.5)[:, None] * normal

    # Unsigned direction bins permit a boundary to appear brighter or darker
    # than its background. Each tree contains only compatible edge normals.
    directions = np.arange(12) * np.pi / 12
    filters = abs(normal @ np.vstack((np.cos(directions), np.sin(directions)))) >= np.cos(np.deg2rad(28))
    return tuple((cKDTree(uv[ok]), uv[ok], normal[ok]) for ok in filters.T)


def _correspond(uv, normal, edges):
    bins = np.rint(np.arctan2(normal[:, 1], normal[:, 0]) * len(edges) / np.pi).astype(int) % len(edges)
    distances = np.full(len(uv), np.inf)
    targets = np.zeros_like(uv)
    directions = np.zeros_like(uv)
    for index, (tree, points, normals) in enumerate(edges):
        selected = np.flatnonzero(bins == index)
        if len(points) and len(selected):
            distance, nearest = tree.query(uv[selected])
            distances[selected] = distance
            targets[selected] = points[nearest]
            directions[selected] = normals[nearest]
    return distances, targets, directions


def _different_pose(first, second):
    return abs(first[0] - second[0]) >= 0.5 or np.linalg.norm(first[1:] - second[1:]) >= 10


def _supported(by_side):
    return all(
        side["edge_coverage"] >= MIN_COVERAGE
        and side["contour_samples"] >= MIN_SAMPLES
        and (
            "hand_edge_coverage" not in side
            or (side["hand_edge_coverage"] >= MIN_COVERAGE and side["hand_contour_samples"] >= MIN_SAMPLES)
        )
        for side in by_side.values()
    )


def _select_pose(candidates, lower, upper, *, explored=()):
    candidates.sort(key=lambda row: row[0])
    cost, pose, by_side = candidates[0]
    alternative = min(
        (
            other_cost
            for other_cost, other_pose, other_sides in (*candidates, *explored)
            if _different_pose(pose, other_pose) and _supported(other_sides)
        ),
        default=None,
    )
    gap = alternative - cost if alternative is not None else None
    reason = None
    if not _supported(by_side):
        reason = "insufficient_arm_contour_match"
    elif np.any(np.minimum(pose - lower, upper - pose) < [0.05, 0.5, 0.5]):
        reason = "solution_at_search_boundary"
    elif gap is not None and gap < AMBIGUITY_GAP:
        reason = "ambiguous_contour_pose"
    return cost, pose, by_side, gap, reason


def _quality(contours, transform, k, edges, hand_points):
    scores = {}
    for side, (points, normal) in contours.items():
        uv, _ = _project(points, transform, k)
        distance, _, _ = _correspond(uv, normal, edges)
        scores[side] = {
            "mean_capped_distance_px": float(np.minimum(distance, MAX_DISTANCE).mean()) if len(distance) else 8.0,
            "edge_coverage": float((distance <= 2.5).mean()) if len(distance) else 0.0,
            "contour_samples": len(distance),
        }
        if hand_points is not None:
            hand = np.linalg.norm(points - hand_points[side], axis=1) <= HAND_NEIGHBORHOOD_M
            scores[side]["hand_contour_samples"] = int(hand.sum())
            scores[side]["hand_edge_coverage"] = float((distance[hand] <= 2.5).mean()) if hand.any() else 0.0
    return float(np.mean([row["mean_capped_distance_px"] for row in scores.values()])), scores


def check_contour_pose(rgb, arms, intrinsics, transform, hand_points):
    """Score the existing transform on a different arm pose without refitting."""
    k = np.asarray(intrinsics, dtype=float)
    transform = np.asarray(transform, dtype=float)
    validate_rigid(transform, "held-out torso-from-RGB")
    edges = _image_edges(rgb)
    if not edges:
        return {"accepted": False, "reason": "insufficient_image_edges"}
    shape = rgb.shape[:2]
    centers, depths = _project(np.asarray([hand_points[side] for side in arms]), transform, k)
    if np.any(depths < 0.02) or np.any(centers < 3) or np.any(centers >= [shape[1] - 3, shape[0] - 3]):
        return {"accepted": False, "reason": "arm_out_of_view"}
    contours = {side: _silhouette(mesh, transform, k, shape) for side, mesh in arms.items()}
    cost, by_side = _quality(contours, transform, k, edges, hand_points)
    supported = _supported(by_side)
    return {
        "accepted": supported,
        "reason": None if supported else "insufficient_arm_contour_match",
        "cost_px": cost,
        "quality_by_side": by_side,
    }


def estimate_contour_pose(rgb, arms, intrinsics, zero, initial, *, hand_points=None):
    """Fit one frozen RGB frame. No camera pose or segmentation truth is accepted."""
    if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
        raise ValueError("contour fitting requires a uint8 BGR image")
    if not arms or set(arms) - {"left", "right"}:
        raise ValueError("provide named meshes for the visible arm or arms")
    k, zero, initial = np.asarray(intrinsics, float), np.asarray(zero, float), np.asarray(initial, float)
    validate_rigid(zero, "RGB zero optical mount")
    if (
        k.shape != (3, 3)
        or not np.isfinite(k).all()
        or k[0, 1] != 0
        or k[1, 0] != 0
        or not np.array_equal(k[2], [0, 0, 1])
        or min(k[0, 0], k[1, 1]) <= 0
        or initial.shape != (3,)
        or not np.isfinite(initial).all()
        or not 12 <= initial[0] <= 60
    ):
        raise ValueError("finite rectified intrinsics and the nominal passive head mounting are required")
    edges = _image_edges(rgb)
    if not edges:
        return {"accepted": False, "reason": "insufficient_image_edges"}

    lower, upper = initial - [7, 36, 36], initial + [7, 36, 36]
    transform = contour_pose_transform(zero, initial)
    shape = rgb.shape[:2]
    silhouettes = {side: _silhouette(mesh, transform, k, shape) for side, mesh in arms.items()}
    if any(len(points) < MIN_SAMPLES for points, _ in silhouettes.values()):
        return {"accepted": False, "reason": "arm_out_of_view"}
    if hand_points is not None:
        height, width = shape
        centers = np.asarray([hand_points[side] for side in arms])
        uv, depth = _project(centers, transform, k)
        if np.any(depth < 0.02) or np.any(uv < 3) or np.any(uv >= [width - 3, height - 3]):
            return {"accepted": False, "reason": "arm_out_of_view"}

    def quality(parameters, contours):
        return _quality(contours, contour_pose_transform(zero, parameters), k, edges, hand_points)

    offsets = np.stack(np.meshgrid(np.arange(-6, 7, 1.5), [-24, -12, 0, 12, 24], [-24, -12, 0, 12, 24]), axis=-1)
    grid = initial + offsets.reshape(-1, 3)
    ranked = sorted(enumerate(grid), key=lambda row: quality(row[1], silhouettes)[0])
    seeds = []
    for _, candidate in ranked:
        if all(np.linalg.norm((candidate - chosen) / [1.5, 16, 16]) >= 1.5 for chosen in seeds):
            seeds.append(candidate)
        if len(seeds) == 4:
            break

    candidates, explored = [], []
    for seed in seeds:
        parameters = seed
        for _ in range(7):
            transform = contour_pose_transform(zero, parameters)
            contours = {side: _silhouette(mesh, transform, k, shape) for side, mesh in arms.items()}
            cost, by_side = quality(parameters, contours)
            # Fixed correspondences optimize a local surrogate. Preserve the
            # real contour cost before the update for the ambiguity gate.
            explored.append((cost, parameters, by_side))
            samples = []
            for points, normal in contours.values():
                uv, _ = _project(points, transform, k)
                distance, targets, direction = _correspond(uv, normal, edges)
                matched = distance < 6
                if matched.sum() < MIN_SAMPLES:
                    break
                samples.append((points[matched], targets[matched], direction[matched]))
            if len(samples) != len(arms):
                break
            reference_count = np.mean([len(row[0]) for row in samples])
            weights = np.concatenate([np.full(len(points), reference_count / len(points)) for points, _, _ in samples])

            def residual(candidate, samples=samples):
                transform = contour_pose_transform(zero, candidate)
                return np.concatenate(
                    [
                        np.sum((_project(points, transform, k)[0] - targets) * direction, axis=1)
                        for points, targets, direction in samples
                    ]
                )

            def weighted_huber(squared, weights=weights):
                # Normalize the loss per side after applying the 1 px Huber
                # breakpoint, so large residuals retain the same arm weight.
                tail = squared > 1
                root = np.sqrt(squared[tail])
                rho = np.vstack((squared, np.ones(len(squared)), np.zeros(len(squared))))
                rho[:, tail] = [2 * root - 1, 1 / root, -0.5 / root**3]
                return rho * weights

            step = least_squares(
                residual, parameters, bounds=(lower, upper), x_scale=[1, 12, 12], loss=weighted_huber, max_nfev=18
            ).x
            movement = np.linalg.norm((step - parameters) / [1, 12, 12])
            parameters = step
            if movement < 0.025:
                break

        transform = contour_pose_transform(zero, parameters)
        contours = {side: _silhouette(mesh, transform, k, shape) for side, mesh in arms.items()}
        cost, quality_by_side = quality(parameters, contours)
        candidates.append((cost, parameters, quality_by_side))

    cost, pose, by_side, gap, reason = _select_pose(candidates, lower, upper, explored=explored)
    transform = contour_pose_transform(zero, pose)
    if hand_points is not None and reason is None:
        uv, depth = _project(centers, transform, k)
        if np.any(depth < 0.02) or np.any(uv < 3) or np.any(uv >= [width - 3, height - 3]):
            reason = "arm_out_of_view"
    return {
        "accepted": reason is None,
        "reason": reason,
        "head_back_deg": float(pose[0]),
        "pivot_torso_m": [float(pose[1] / 1000), 0.0, float(pose[2] / 1000)],
        "T_torso_camera_optical": transform.tolist(),
        "cost_px": cost,
        "quality_by_side": by_side,
        "alternative_gap_px": gap,
        "candidates": [{"parameters_deg_mm": p.tolist(), "cost_px": c} for c, p, _ in candidates],
        "explored_candidates": [{"parameters_deg_mm": p.tolist(), "cost_px": c} for c, p, _ in explored],
    }
