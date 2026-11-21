"""Construct the fixed-base MuJoCo scene used for live head calibration."""

import importlib.util
import io
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROBOT_XML = Path("third_party/unitree_mujoco/unitree_robots/g1/g1_29dof.xml")


def load_robot_model(root: Path):
    """Load the native G1 fixed-hand robot without constructing a test scene."""
    import mujoco

    return mujoco.MjModel.from_xml_path(str(root / ROBOT_XML))


def _terrain_module(root):
    path = root / "apps/simulation/terrain.py"
    spec = importlib.util.spec_from_file_location("g1_deploy_calibration_terrain", path)
    terrain = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(terrain)
    return terrain


def load_scene_profile(root: Path):
    """Read the native door profile, retaining the simulator's RGB camera conventions."""
    import mujoco

    profile_path = root / "profiles/door.json"
    profile = json.loads(profile_path.read_text())
    robot_path = (profile_path.parent / profile["robot"]["config"]).resolve()
    camera_path = (profile_path.parent / profile["camera"]["config"]).resolve()
    camera = json.loads(camera_path.read_text())
    target_back_deg = float(json.loads(robot_path.read_text())["head_back_deg"])
    rgb = camera["rgb"]
    height, width = (int(rgb["stream"][axis]) for axis in ("height", "width"))
    focal = height / (2 * math.tan(math.radians(rgb["vertical_fov_deg"]) / 2))
    k = [[focal, 0, (width - 1) / 2], [0, focal, (height - 1) / 2], [0, 0, 1]]
    terrain = _terrain_module(root)
    transforms = []
    for tilt in (0.0, target_back_deg):
        scene = ET.fromstring('<mujoco><worldbody><body name="torso_link"/></worldbody></mujoco>')
        terrain._inject_depth_camera(scene, camera, tilt)
        element = scene.find(f".//camera[@name='{rgb['scene_camera']}']")
        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, np.fromstring(element.get("quat"), sep=" "))
        transform = np.eye(4)
        transform[:3, :3] = rotation.reshape(3, 3) @ np.diag([1, -1, -1])
        transform[:3, 3] = np.fromstring(element.get("pos"), sep=" ")
        transforms.append(transform)
    zero, target = transforms
    return {
        "frame_shape": (height, width),
        "intrinsics_K": k,
        "target_head_back_deg": target_back_deg,
        "T_torso_camera_optical_zero": zero,
        "T_torso_camera_optical_target": target,
        "nominal_pivot_torso_m": terrain.HEAD_PIVOT_TORSO_M,
    }


def build_live_scene(root: Path, *, profile, initial_back_deg, pivot_torso_m):
    """Keep the native arm dynamics and model the head as a manually locked hinge.

    Back tilt is positive joint qpos about torso -Y. The equality's constant
    coefficient is the initial tilt in radians. Set MjData's head qpos to the
    same value before the first step. The zero reference remains at qpos0=0.
    The pelvis, waist and legs stay fixed. Render with geom group 0 disabled
    to show the native robot visual meshes without their collision copies.
    """
    import mujoco

    torso_from_optical_zero = profile["T_torso_camera_optical_zero"]
    pivot = np.asarray(pivot_torso_m, dtype=float)
    if pivot.shape != (3,) or not np.isfinite(np.r_[pivot, initial_back_deg]).all():
        raise ValueError("head fixture requires a finite pivot and angle")
    height, width = profile["frame_shape"]
    focal = profile["intrinsics_K"][1][1]

    def vector(values):
        return " ".join(format(float(value), ".17g") for value in values)

    robot_path = root / ROBOT_XML
    xml = ET.parse(robot_path).getroot()
    compiler = xml.find("compiler")
    compiler.set("meshdir", str((robot_path.parent / compiler.get("meshdir", "meshes")).resolve()))
    world = xml.find("worldbody")
    pelvis = world.find("body[@name='pelvis']")
    torso = pelvis.find(".//body[@name='torso_link']")
    arm_prefixes = tuple(f"{side}_{joint}_" for side in ("left", "right") for joint in ("shoulder", "elbow", "wrist"))
    arm_joints, removed_joints = set(), set()
    for body in pelvis.iter("body"):
        for joint in (*body.findall("joint"), *body.findall("freejoint")):
            name = joint.get("name", "")
            if name.startswith(arm_prefixes):
                arm_joints.add(name)
            else:
                removed_joints.add(name)
                body.remove(joint)
    actuator = xml.find("actuator")
    motors = [motor for motor in actuator if motor.get("joint") in arm_joints]
    if len(arm_joints) != 14 or len(motors) != 14 or {motor.get("joint") for motor in motors} != arm_joints:
        raise ValueError("expected one native motor for each of the 14 named arm joints")
    for section in (actuator, xml.find("sensor")):
        for entry in section[:]:
            if entry.get("joint") in removed_joints:
                section.remove(entry)

    ET.SubElement(xml, "option", timestep="0.002")
    visual = ET.SubElement(xml, "visual")
    ET.SubElement(visual, "global", offheight=str(height), offwidth=str(width))
    assets = xml.find("asset")
    ET.SubElement(
        assets, "material", name="calibration_floor_mat", rgba="0.14 0.17 0.19 1", specular="0", shininess="0"
    )
    ET.SubElement(
        world,
        "geom",
        name="calibration_floor",
        type="plane",
        size="0 0 0.1",
        group="2",
        material="calibration_floor_mat",
    )
    terrain = _terrain_module(root)
    terrain._build_door_scene(xml, json.loads(terrain.SETTINGS.read_text()), io.BytesIO())
    xml.set("model", "g1_fixed_base_head_calibration_door")
    door = world.find("body[@name='door_frame']")
    for geom in (
        *door.iter("geom"),
        *world.findall("geom[@name='door_wall_negative']"),
        *world.findall("geom[@name='door_wall_positive']"),
    ):
        geom.set("group", "2")
    for body in door.iter("body"):
        for site in body.findall("site"):
            body.remove(site)
    ET.SubElement(
        world,
        "light",
        name="calibration_key",
        pos="0.9 -0.6 2.1",
        dir="-0.3 0.2 -1",
        ambient="0.15 0.15 0.15",
        diffuse="0.6 0.6 0.6",
        specular="0.1 0.1 0.1",
        castshadow="false",
    )
    overview_position = np.array([1.65, -1.65, 1.25])
    overview_backward = overview_position - [0, 0, 0.73]
    overview_backward /= np.linalg.norm(overview_backward)
    overview_right = np.cross([0, 0, 1], overview_backward)
    overview_right /= np.linalg.norm(overview_right)
    overview_up = np.cross(overview_backward, overview_right)
    ET.SubElement(
        world,
        "camera",
        name="overview",
        mode="fixed",
        pos=vector(overview_position),
        xyaxes=vector(np.r_[overview_right, overview_up]),
        fovy="43",
    )

    head = ET.SubElement(torso, "body", name="calibration_head", pos=vector(pivot))
    ET.SubElement(head, "joint", name="calibration_head_pitch", type="hinge", axis="0 -1 0", damping="0.2")
    # Source deployment head mass properties. The merged torso stays welded to
    # the world in this fixture, so its original inertia does not enter arm dynamics.
    ET.SubElement(
        head,
        "inertial",
        pos=vector(np.array([0.0092305, 0.000299, 0.405869]) - pivot),
        mass="1.036",
        fullinertia="0.004085051 0.004185212 0.001807911 -2.543e-06 -6.9455e-05 -3.726e-06",
    )
    for geom in torso.findall("geom[@mesh='head_link']"):
        position = np.fromstring(geom.get("pos", "0 0 0"), sep=" ")
        geom.set("pos", vector(position - pivot))
        torso.remove(geom)
        head.append(geom)
    optical_from_mj_camera = np.diag([1, -1, -1])
    camera_matrix = torso_from_optical_zero[:3, :3] @ optical_from_mj_camera
    camera_quat = np.empty(4)
    mujoco.mju_mat2Quat(camera_quat, camera_matrix.ravel())
    ET.SubElement(
        head,
        "camera",
        name="calibration_rgb",
        mode="fixed",
        pos=vector(torso_from_optical_zero[:3, 3] - pivot),
        quat=vector(camera_quat),
        fovy=format(math.degrees(2 * math.atan(height / (2 * focal))), ".17g"),
    )
    # The source puts both collision meshes on the torso. Splitting the head
    # into a moving body must not create self-contact at the mounting interface.
    contact = ET.SubElement(xml, "contact")
    ET.SubElement(contact, "exclude", body1="torso_link", body2="calibration_head")
    equality = ET.SubElement(xml, "equality")
    ET.SubElement(
        equality,
        "joint",
        name="calibration_head_lock",
        joint1="calibration_head_pitch",
        polycoef=vector([math.radians(initial_back_deg), 0, 0, 0, 0]),
        solref="0.004 1",
        solimp="0.9999 0.9999 0.001",
    )
    xml_text = ET.tostring(xml, encoding="unicode")
    return mujoco.MjModel.from_xml_string(xml_text), xml_text
