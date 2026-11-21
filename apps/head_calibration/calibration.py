"""Fit and verify bilateral RGB contours using the same frame contract in sim and on hardware."""

import hashlib
import json

import cv2
import numpy as np

from .contour import check_contour_pose, contour_pose_transform, estimate_contour_pose
from .overlay import overlay_arm_target, project_arm_mask, rigid_hand_arm_mesh

REASONS = {
    "insufficient_image_edges": "画面对比度不足，未找到足够的边缘",
    "arm_out_of_view": "手或腕部离开视野",
    "insufficient_arm_contour_match": "手或腕部的轮廓匹配不足",
    "solution_at_search_boundary": "结果落在搜索边缘",
    "ambiguous_contour_pose": "存在多个接近的位姿解",
}


def score_frame(result, truth, score_path):
    """Post-fit evaluation for simulated experiments with observed camera ground truth."""
    if truth.get("expect_rejected"):
        score = {"expect_rejected": True, "passed": not result["accepted"], "reason": result["reason"]}
    elif "head_back_deg" in result:
        angle = abs(result["head_back_deg"] - truth["head_back_deg"])
        axis = 1000 * np.linalg.norm((np.asarray(result["pivot_torso_m"]) - np.asarray(truth["pivot_torso_m"]))[[0, 2]])
        score = {
            "angle_error_deg": angle,
            "pivot_xz_error_mm": float(axis),
            "passed": bool(result["accepted"] and angle <= 0.5 and axis <= 10),
            "accepted_by_image": result["accepted"],
            "reason": result["reason"],
        }
    else:
        score = {"passed": False, "accepted_by_image": False, "reason": result["reason"]}
    score_path.write_text(json.dumps(score, indent=2) + "\n")
    return score


def show_result(path, result):
    from PIL import Image, ImageDraw, ImageFont

    image = cv2.imread(str(path))
    width = image.shape[1]
    bar = Image.new("RGB", (width, 108), (24, 31, 38))
    draw = ImageDraw.Draw(bar)
    font = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", 19)
    if result["accepted"]:
        line = f"估计后仰 {result['head_back_deg']:.3f}°   轴心 X/Z  {result['pivot_torso_m'][0] * 1000:.1f} / {result['pivot_torso_m'][2] * 1000:.1f} mm"
    else:
        line = REASONS[result["reason"]]
    draw.text((12, 9), line, font=font, fill=(224, 244, 248))
    sides = result.get("quality_by_side", {})
    if sides:
        coverage = "   ".join(
            f"{label}侧匹配 {sides[side]['edge_coverage']:.0%}" for side, label in (("left", "左"), ("right", "右"))
        )
        draw.text((12, 44), coverage, font=font, fill=(190, 211, 220))
    draw.text((12, 76), "青色：左侧   橙色：右侧   Esc 关闭", font=font, fill=(190, 211, 220))
    window = "G1 - Bilateral contour fit"
    cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
    try:
        header = cv2.cvtColor(np.asarray(bar), cv2.COLOR_RGB2BGR)
        cv2.imshow(window, np.vstack((header, image)))
        while cv2.waitKeyEx(50) != 27 and cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) >= 1:
            pass
    finally:
        cv2.destroyAllWindows()


def _posed_arms(model, joints):
    import mujoco

    data = mujoco.MjData(model)
    arms, hand_points = {}, {}
    for side in ("left", "right"):
        arms[side], geom_ids = rigid_hand_arm_mesh(model, side, joints, data=data)
        hand_mesh = model.mesh(f"{side}_rubber_hand").id
        hand_geom = next(index for index in geom_ids if model.geom_dataid[index] == hand_mesh)
        torso = data.body("torso_link")
        hand_points[side] = (data.geom_xpos[hand_geom] - torso.xpos) @ torso.xmat.reshape(3, 3)
    return arms, hand_points


def overlay_frame(model, rgb, joints, *, k, transform):
    arms, _ = _posed_arms(model, joints)
    mask = np.zeros(rgb.shape[:2], dtype=bool)
    for mesh in arms.values():
        mask |= project_arm_mask(mesh, transform, k, mask.shape)
    return overlay_arm_target(rgb, mask)


def fit_frame(model, rgb_path, joints, out, *, k, zero, initial):
    return _process_frame(model, rgb_path, joints, out, k=k, zero=zero, initial=initial)


def check_frame(model, rgb_path, joints, out, *, k, transform):
    return _process_frame(model, rgb_path, joints, out, k=k, transform=transform)


def _process_frame(model, rgb_path, joints, out, *, k, zero=None, initial=None, transform=None):
    rgb_bytes = rgb_path.read_bytes()
    rgb = cv2.imdecode(np.frombuffer(rgb_bytes, np.uint8), cv2.IMREAD_COLOR)
    if rgb is None:
        raise ValueError(f"无法读取确认帧 {rgb_path}")
    arms, hand_points = _posed_arms(model, joints)
    fitting = transform is None
    if fitting:
        result = estimate_contour_pose(rgb, arms, k, zero, initial, hand_points=hand_points)
        transform = result.get("T_torso_camera_optical", contour_pose_transform(zero, initial))
        result["T_torso_camera_optical_zero"] = np.asarray(zero).tolist()
    else:
        result = check_contour_pose(rgb, arms, k, transform, hand_points)
        result["T_torso_camera_optical"] = np.asarray(transform).tolist()

    result.update(
        mode="rigid_hand_directional_contour" if fitting else "check_without_refit",
        reference_frame="torso_link",
        camera_frame="head_rgb_optical",
        length_unit="m",
        head_angle_unit="deg, positive backward",
        arms=list(arms),
        joint_positions_rad=joints,
        intrinsics_K=np.asarray(k).tolist(),
        source_rgb_sha256=hashlib.sha256(rgb_bytes).hexdigest(),
    )

    out.mkdir(exist_ok=False)
    (out / "report.json").write_text(json.dumps(result, indent=2) + "\n")
    overlay = rgb.copy()
    for side, mesh in arms.items():
        mask = project_arm_mask(mesh, transform, k, rgb.shape[:2]).astype(np.uint8)
        edge = mask != cv2.erode(mask, np.ones((3, 3), np.uint8))
        overlay[edge] = (245, 215, 52) if side == "left" else (40, 157, 255)
    if not cv2.imwrite(str(out / "overlay.png"), overlay):
        raise OSError(f"无法保存轮廓叠加图 {out}")
    if fitting and result["accepted"]:
        (out / "calibration.json").write_text(json.dumps(result, indent=2) + "\n")
    return result
