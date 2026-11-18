"""Build repeatable G1 obstacle courses with real gaps in the walking surface."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np


TERRAINS = ("original", "mixed", "platforms", "stones", "gaps", "stairs", "hurdles", "doors")
SETTINGS = Path(__file__).with_name("terrain.json")
DOOR_ASSETS = Path(__file__).with_name("assets") / "doors"
DOOR_ASSET = DOOR_ASSETS / "shared_doorman_v1.mjcf.xml"
DOOR_SCENE_JSON = DOOR_ASSETS / "doorman_right_out_with_walls.json"

# 可选的静态头部后仰（不是机械关节）：复刻 OpenDoor-GraspPlan 分支 codex/g1-head-pitch20
# 的 head_pose.py 安装公式——整块 head_link 绕躯干 +Y 轴向后旋（负向=抬头），轴心为用户
# 2026-11-13 在躯干 X-Z 图上指定的 [0, 0, 0.320] m；头部质量/质心/惯量并回躯干重算。
HEAD_PIVOT_TORSO_M = np.array([0.0, 0.0, 0.320])
# 头部数值来自 unitree_ros g1_description URDF（head_joint rpy=0，head_link <inertial>），
# 与 OpenDoor head_pose.py original_head_inertial() 同源；质心=安装点+质心偏移。
HEAD_MASS_KG = 1.036
HEAD_CENTRE_TORSO_M = np.array([0.0092305, 0.000299, 0.405869])
HEAD_INERTIA_TORSO = np.array([
    [0.004085051, -2.543e-06, -6.9455e-05],
    [-2.543e-06, 0.004185212, -3.726e-06],
    [-6.9455e-05, -3.726e-06, 0.001807911]])


def _vector(text):
    return np.array([float(value) for value in text.split()], dtype=float)


def _head_back_rotation(back_deg):
    angle = -math.radians(back_deg)
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rotation_to_quat(rotation):
    """3x3 旋转矩阵 -> MuJoCo wxyz 四元数。"""
    trace = np.trace(rotation)
    if trace > 0:
        scale = math.sqrt(trace + 1.0) * 2
        quat = np.array([0.25 * scale,
                         (rotation[2, 1] - rotation[1, 2]) / scale,
                         (rotation[0, 2] - rotation[2, 0]) / scale,
                         (rotation[1, 0] - rotation[0, 1]) / scale])
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2
        quat = np.array([(rotation[2, 1] - rotation[1, 2]) / scale, 0.25 * scale,
                         (rotation[0, 1] + rotation[1, 0]) / scale,
                         (rotation[0, 2] + rotation[2, 0]) / scale])
    elif rotation[1, 1] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2
        quat = np.array([(rotation[0, 2] - rotation[2, 0]) / scale,
                         (rotation[0, 1] + rotation[1, 0]) / scale, 0.25 * scale,
                         (rotation[1, 2] + rotation[2, 1]) / scale])
    else:
        scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2
        quat = np.array([(rotation[1, 0] - rotation[0, 1]) / scale,
                         (rotation[0, 2] + rotation[2, 0]) / scale,
                         (rotation[1, 2] + rotation[2, 1]) / scale, 0.25 * scale])
    return quat / np.linalg.norm(quat)


def head_back_deg():
    """terrain.json 的可选头部后仰角（度）；head 节缺失 = 0 = 标准姿态。"""
    section = json.loads(SETTINGS.read_text()).get("head")
    if section is None:
        return 0.0
    value = float(section["back_deg"])
    if not math.isfinite(value) or value <= 0:
        raise ValueError("head.back_deg must be finite positive; remove the head section to disable")
    return value


def apply_head_back_camera(adapter, back_deg):
    """让深度相机随头部一起后仰：方向同角度旋转，安装点绕头部枢点变换（光轴下俯角减小 back_deg）。

    位置必须用与 head_geom 相同的 H = Trans(p)·Ry·Trans(−p)（枢点 p=[0,0,0.320]），
    不能绕躯干原点旋转——否则相机原点会被甩到肩部区域。adapter.cosines 是射线
    相对相机光轴的夹角，头+相机一起转时不变，保持原样。
    """
    rotation = _head_back_rotation(back_deg)
    adapter.rays = np.ascontiguousarray(adapter.rays @ rotation.T)
    adapter.offset = HEAD_PIVOT_TORSO_M + rotation @ (adapter.offset - HEAD_PIVOT_TORSO_M)


def _apply_head_back(scene, back_deg):
    """把机器人头部的几何与躯干合并惯量按静态安装公式后仰 back_deg（覆盖所有生成场景）。"""
    rotation = _head_back_rotation(back_deg)
    pivot = HEAD_PIVOT_TORSO_M
    torso = next(body for body in scene.iter("body")
                 if body.find("geom[@mesh='head_link']") is not None)
    for geom in torso.findall("geom[@mesh='head_link']"):
        if geom.get("quat"):
            raise ValueError("head geom already carries a quat; composition would be required")
        pos = pivot + rotation @ (_vector(geom.get("pos")) - pivot)
        geom.set("pos", " ".join(f"{value:.9g}" for value in pos))
        half = math.radians(back_deg) / 2
        geom.set("quat", f"{math.cos(half):.9g} 0 {-math.sin(half):.9g} 0")
    inertial = torso.find("inertial")
    mass = float(inertial.get("mass"))
    centre = _vector(inertial.get("pos"))
    axes = np.eye(3)
    if inertial.get("quat"):
        # wxyz 四元数 -> 3x3（与 _rotation_to_quat 互逆的标准公式）
        w, x, y, z = _vector(inertial.get("quat"))
        axes = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                         [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                         [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    inertia = axes @ np.diag(_vector(inertial.get("diaginertia"))) @ axes.T
    new_head_centre = pivot + rotation @ (HEAD_CENTRE_TORSO_M - pivot)
    new_centre = centre + HEAD_MASS_KG / mass * (new_head_centre - HEAD_CENTRE_TORSO_M)
    # reposition_merged_head（head_pose.py）的直接移植：总质量不变，只移动头部贡献。
    inertia = (inertia
               + mass * ((centre @ centre - new_centre @ new_centre) * np.eye(3)
                         - np.outer(centre, centre) + np.outer(new_centre, new_centre))
               + rotation @ HEAD_INERTIA_TORSO @ rotation.T - HEAD_INERTIA_TORSO
               + HEAD_MASS_KG * ((new_head_centre @ new_head_centre - HEAD_CENTRE_TORSO_M @ HEAD_CENTRE_TORSO_M)
                                 * np.eye(3) - np.outer(new_head_centre, new_head_centre)
                                 + np.outer(HEAD_CENTRE_TORSO_M, HEAD_CENTRE_TORSO_M)))
    values, vectors = np.linalg.eigh(inertia)
    if np.linalg.det(vectors) < 0:
        vectors[:, -1] *= -1
    inertial.set("pos", " ".join(f"{value:.9g}" for value in new_centre))
    inertial.set("quat", " ".join(f"{value:.9g}" for value in _rotation_to_quat(vectors)))
    inertial.set("diaginertia", " ".join(f"{value:.9g}" for value in values))
    custom = scene.find("custom")
    if custom is None:
        custom = ET.SubElement(scene, "custom")
    ET.SubElement(custom, "numeric", name="g1_head_back_deg", data=f"{back_deg:.1f}")
COLORS = {
    "ground": "0.27 0.32 0.36 1",
    "platforms": "0.78 0.55 0.25 1",
    "stones": "0.35 0.65 0.46 1",
    "gaps": "0.32 0.53 0.74 1",
    "stairs": "0.63 0.48 0.77 1",
    "hurdles": "0.80 0.34 0.27 1",
}


class Course:
    def __init__(self, world, settings, y=0.0, prefix="", add_start=True):
        self.world, self.settings = world, settings
        self.y, self.prefix = y, prefix
        self.half_width = settings["lane_width"] / 2
        self.pit_depth = settings["pit_depth"]
        self.cursor = settings["start_length"]
        self.sections = []
        if add_start:
            self.ground(-4.0, self.cursor, "start")

    def box(self, name, start, end, top=0, color="ground"):
        # Every tile reaches down to the pit floor. No plane exists at walking height.
        bottom = -self.pit_depth
        appearance = {"material": "terrain_sandstone_ground"} if top == 0 else {"rgba": COLORS[color]}
        ET.SubElement(self.world, "geom", name=f"terrain_{self.prefix}{name}", type="box",
                      pos=f"{(start + end) / 2} {self.y} {(top + bottom) / 2}",
                      size=f"{(end - start) / 2} {self.half_width} {(top - bottom) / 2}",
                      friction="1 0.005 0.0001", **appearance)

    def ground(self, start, end, name, color="ground"):
        self.box(name, start, end, color=color)

    def platforms(self):
        cfg = self.settings["platforms"]
        for index, height in enumerate(cfg["heights"]):
            start = self.cursor
            self.box(f"platform_{index}", start, start + cfg["length"], height, "platforms")
            self.cursor += cfg["length"]
            self.ground(self.cursor, self.cursor + cfg["spacing"], f"platform_exit_{index}")
            self.cursor += cfg["spacing"]

    def stones(self):
        cfg = self.settings["stones"]
        for index, (height, radius) in enumerate(zip(cfg["heights"], cfg["radii"])):
            x = self.cursor + cfg["stride"] / 2
            y = self.y + cfg["lateral_offset"] * (1 if index % 2 else -1)
            ET.SubElement(self.world, "geom", name=f"terrain_{self.prefix}stone_{index}", type="cylinder",
                          pos=f"{x} {y} {(height - self.pit_depth) / 2}",
                          size=f"{radius} {(height + self.pit_depth) / 2}",
                          rgba=COLORS["stones"], friction="1 0.005 0.0001")
            self.cursor += cfg["stride"]

    def gaps(self):
        cfg = self.settings["gaps"]
        for index, width in enumerate(cfg["widths"]):
            # Deliberately leave this x interval empty across the whole lane width.
            self.cursor += width
            self.ground(self.cursor, self.cursor + cfg["landing_length"],
                        f"gap_landing_{index}", "gaps")
            self.cursor += cfg["landing_length"]

    def stairs(self):
        cfg = self.settings["stairs"]
        for flight, rise in enumerate(cfg["rises"]):
            heights = [rise * step for step in range(1, cfg["steps"] + 1)]
            for index, height in enumerate(heights + heights[-2::-1]):
                self.box(f"stairs_{flight}_{index}", self.cursor,
                         self.cursor + cfg["tread_length"], height, "stairs")
                self.cursor += cfg["tread_length"]
            self.ground(self.cursor, self.cursor + cfg["landing_length"], f"stairs_exit_{flight}")
            self.cursor += cfg["landing_length"]

    def hurdles(self):
        cfg = self.settings["hurdles"]
        for index, height in enumerate(cfg["heights"]):
            self.box(f"hurdle_{index}", self.cursor, self.cursor + cfg["thickness"], height, "hurdles")
            self.cursor += cfg["thickness"]
            self.ground(self.cursor, self.cursor + cfg["spacing"], f"hurdle_exit_{index}")
            self.cursor += cfg["spacing"]

    def build(self, selection):
        start = self.cursor
        getattr(self, selection)()
        self.sections.append((selection, start, self.cursor, self.y))
        end = self.cursor + self.settings["transition_length"]
        self.ground(self.cursor, end, f"after_{selection}")
        self.cursor = end
        self.ground(self.cursor, self.cursor + 5, "finish")
        return self.sections


def _inline_robot(scene, robot_dir):
    """把机器人定义内联进场景，使网格路径不依赖临时目录。"""
    for include in list(scene.findall("include")):
        scene.remove(include)
    robot = ET.parse(robot_dir / "g1_29dof.xml").getroot()
    for child in reversed(list(robot)):
        if child.tag == "compiler":
            for attribute in ("meshdir", "texturedir"):
                if attribute in child.attrib:
                    child.set(attribute, str((robot_dir / child.get(attribute)).resolve()))
        scene.insert(0, child)


def _build_door_scene(scene, settings, destination):
    """在原始平地场景里，把 shared_doorman_v1 门放到机器人正前方（右铰链、朝 +X 外开）。"""
    door_x = settings["doors"]["x"]
    # 场景自身的 worldbody（灯光+地面）在内联机器人之后，门排在机器人后面即可保证文档顺序安全。
    world = scene.findall("worldbody")[-1]
    door = ET.parse(DOOR_ASSET).getroot().find("worldbody/body")
    scene_config = json.loads(DOOR_SCENE_JSON.read_text())
    thickness = scene_config["wall_thickness_m"]
    height = scene_config["wall_height_m"]
    outer = scene_config["wall_outer_y_m"]
    # 资产自带的 1 m 小侧墙由正式墙体取代（对齐 OpenDoor-GraspPlan DoorScene 的 robot_side_flush 语义）。
    flank = door.find("geom[@name='frame_wall_positive_y']")
    inner = abs(float(flank.get("pos").split()[1])) - float(flank.get("size").split()[1])
    for geom in list(door.findall("geom")):
        if geom.get("name", "").startswith("frame_wall_"):
            door.remove(geom)
    door.set("pos", f"{door_x} 0 0")
    world.append(door)
    # 把手铰链覆写（不改资产文件）：真实门轴锁芯有摩擦，柄帽重力力矩（~0.134 N·m）会顶开
    # 回位弹簧让把手静置下坠 ~5°；frictionloss 需配合 solimpfriction 提高阻抗——MuJoCo 的
    # 摩擦约束是软约束，默认参数下恒定负载会蠕变到弹簧平衡点，硬化后下坠 <0.05°。
    handle = door.find(".//joint[@name='handle_joint']")
    friction = settings["doors"].get("handle_friction")
    if friction is not None:
        handle.set("frictionloss", f"{float(friction):g}")
        handle.set("solimpfriction", "0.9999 0.99999 1e-06 0.5 2")
    stiffness = settings["doors"].get("handle_stiffness")
    if stiffness is not None:
        handle.set("stiffness", f"{float(stiffness):g}")
    # 墙体：从门洞边缘到 wall_outer_y，机器人侧 (-X) 面与门板 -X 面齐平。
    panel_thickness = 2 * float(door.find(".//geom[@name='panel']").get("size").split()[0])
    center_x = (thickness - panel_thickness) / 2
    center_y, half_y = (inner + outer) / 2, (outer - inner) / 2
    for name in ("negative", "positive"):
        sign = 1 if name == "positive" else -1
        ET.SubElement(world, "geom", name=f"door_wall_{name}", type="box", rgba="0.72 0.73 0.75 1",
                      pos=f"{door_x + center_x} {sign * center_y} {height / 2}",
                      size=f"{thickness / 2} {half_y} {height / 2}")
    scene.set("model", "G1 door scene")
    ET.indent(scene, space="  ")
    ET.ElementTree(scene).write(destination, encoding="utf-8", xml_declaration=True)
    return [("doors", door_x, door_x, 0.0)]


def generate_scene(root, selection, destination):
    """Copy scene/robot definitions into a temporary MJCF and replace only terrain."""
    if selection not in TERRAINS or selection == "original":
        raise ValueError(f"Cannot generate terrain: {selection}")
    robot_dir = Path(root) / "unitree_robots/g1"
    scene = ET.parse(robot_dir / "scene_29dof.xml").getroot()
    # Inline the robot so compiler/mesh paths do not depend on the temporary directory.
    _inline_robot(scene, robot_dir)
    settings = json.loads(SETTINGS.read_text())

    def validate_numbers(values):
        for value in values.values():
            if isinstance(value, dict):
                validate_numbers(value)
                continue
            numbers = value if isinstance(value, list) else [value]
            if not numbers or any(not isinstance(number, (int, float)) or
                                  not math.isfinite(number) or number <= 0 for number in numbers):
                raise ValueError("Terrain dimensions must be finite positive numbers")

    validate_numbers(settings)
    back_deg = float(settings.get("head", {}).get("back_deg", 0.0) or 0.0)
    if back_deg:
        _apply_head_back(scene, back_deg)
    if selection == "doors":
        return _build_door_scene(scene, settings, destination)
    for world in scene.findall("worldbody"):
        for floor in list(world.findall("geom")):
            if floor.get("type") == "plane" or floor.get("name") == "floor":
                world.remove(floor)
    # Soft daylight sky, with a matching horizon instead of the original black gradient.
    for asset in scene.findall("asset"):
        for texture in list(asset.findall("texture")):
            if texture.get("type") == "skybox":
                asset.remove(texture)
    visual = scene.find("visual")
    if visual is None:
        visual = ET.SubElement(scene, "visual")
    rgba = visual.find("rgba")
    if rgba is None:
        rgba = ET.SubElement(visual, "rgba")
    rgba.set("haze", "0.86 0.89 0.91 1")
    assets = ET.SubElement(scene, "asset")
    ET.SubElement(assets, "texture", name="terrain_daylight_sky", type="skybox", builtin="gradient",
                  rgb1="0.55 0.72 0.88", rgb2="0.92 0.92 0.88", width="512", height="3072")
    ET.SubElement(assets, "texture", name="terrain_sandstone_texture", type="2d", builtin="flat",
                  mark="random", rgb1="0.66 0.59 0.47", markrgb="0.58 0.51 0.40", random="0.035",
                  width="512", height="512")
    ET.SubElement(assets, "material", name="terrain_sandstone_ground", texture="terrain_sandstone_texture",
                  texuniform="true", texrepeat="0.5 0.5", reflectance="0", specular="0.04", shininess="0.05")
    world = ET.SubElement(scene, "worldbody")
    if len(settings["stones"]["heights"]) != len(settings["stones"]["radii"]):
        raise ValueError("Each stepping stone requires a height and radius")
    if not isinstance(settings["stairs"]["steps"], int):
        raise ValueError("Stair step count must be an integer")
    if settings["lane_spacing"] < settings["lane_width"]:
        raise ValueError("lane_spacing must be at least lane_width to keep courses separate")
    ET.SubElement(world, "geom", name="terrain_pit_floor", type="plane",
                  pos=f"0 0 {-settings['pit_depth']}", size="0 0 .05",
                  material="terrain_sandstone_ground", friction="1 0.005 0.0001")
    if selection == "mixed":
        categories = ("platforms", "stones", "gaps", "stairs", "hurdles")
        # All lanes start at the same X; alternating Y keeps platforms at the spawn heading.
        offsets = [((index + 1) // 2) * settings["lane_spacing"] * (1 if index % 2 else -1)
                   for index in range(len(categories))]
        plaza_settings = {**settings, "lane_width": max(offsets) - min(offsets) + settings["lane_width"]}
        Course(world, plaza_settings, prefix="plaza_")
        sections = []
        for category, y in zip(categories, offsets):
            sections.extend(Course(world, settings, y=y, prefix=f"{category}_", add_start=False).build(category))
    else:
        sections = Course(world, settings).build(selection)
    scene.set("model", f"G1 {selection} course")
    ET.indent(scene, space="  ")
    ET.ElementTree(scene).write(destination, encoding="utf-8", xml_declaration=True)
    return sections
