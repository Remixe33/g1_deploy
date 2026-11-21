"""Control and hold G1's arms in MuJoCo, then calibrate one confirmed RGB frame."""

import argparse
import hashlib
import json
import os
import tempfile
import time
from pathlib import Path

import cv2
import numpy as np

from .motion import CALIBRATION_POSE_RAD, CalibrationArmControl
from .overlay import overlay_arm_target, project_arm_mask, rigid_hand_arm_mesh
from .sim_scene import ROBOT_XML, build_live_scene, load_robot_model, load_scene_profile

# Test scene geometry. The fitter receives the independent nominal mount only.
SIM_PIVOT_M = (0.008, 0, 0.326)
INITIAL_BACK_DEG = 16.0
HEAD_LIMITS_DEG = (14.0, 26.0)
ARM_TIMEOUT_S = 8.0


def capture_live(root, out, *, config, headless, demo_back_deg):
    import mujoco
    from PIL import Image, ImageDraw, ImageFont

    joints = CALIBRATION_POSE_RAD
    k = config["intrinsics_K"]
    height, width = config["frame_shape"]
    model, xml = build_live_scene(
        root,
        profile=config,
        initial_back_deg=INITIAL_BACK_DEG,
        pivot_torso_m=SIM_PIVOT_M,
    )
    (out / "live-scene.xml").write_text(xml)
    data = mujoco.MjData(model)
    hinge = data.joint("calibration_head_pitch")
    lock_id = model.equality("calibration_head_lock").id
    requested_back_deg = INITIAL_BACK_DEG
    head_actions = []

    def hand_turn(angle):
        nonlocal requested_back_deg
        # Quasistatic manual repositioning in simulation, followed by the
        # mechanical lock. The head joint has no motor.
        requested_back_deg = float(np.clip(angle, *HEAD_LIMITS_DEG))
        radians = np.deg2rad(requested_back_deg)
        hinge.qpos[0], hinge.qvel[0] = radians, 0.0
        model.eq_data[lock_id, 0] = radians
        mujoco.mj_forward(model, data)
        head_actions.append([float(data.time), requested_back_deg])

    hand_turn(INITIAL_BACK_DEG)
    control = CalibrationArmControl(model, data, joints)
    (out / "motion-config.json").write_text(
        json.dumps(
            {
                "joint_names": control.names,
                "start_rad": control.start.tolist(),
                "target_rad": joints,
                "motor_control_limits": model.actuator_ctrlrange[control.motors].tolist(),
                "timestep_s": float(model.opt.timestep),
                "travel_s": 3.0,
                "position_tolerance_rad": control.POSITION_TOL_RAD,
                "velocity_tolerance_rad_s": control.VELOCITY_TOL_RAD_S,
                "minimum_hold_s": control.HOLD_S,
                "root_mode": "fixed pelvis and waist",
                "head_input": "scripted virtual manual adjustment"
                if headless
                else "keyboard virtual manual adjustment",
            },
            indent=2,
        )
        + "\n"
    )

    motion = []
    option = mujoco.MjvOption()
    option.geomgroup[0] = 0
    target_camera = np.asarray(config["T_torso_camera_optical_target"])
    target_mask = None
    mask_pose = None
    preview_data = mujoco.MjData(model)
    window = "G1 - Live head calibration"

    def render(renderer, camera):
        renderer.update_scene(data, camera=camera, scene_option=option)
        return cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)

    def target_overlay(rgb):
        nonlocal target_mask, mask_pose
        if mask_pose is None or np.max(np.abs(control.position_rad - mask_pose)) > 0.002:
            mask_pose = control.position_rad
            measured = dict(zip(control.names, mask_pose.tolist(), strict=True))
            target_mask = np.zeros((height, width), bool)
            for side in ("left", "right"):
                mesh, _ = rigid_hand_arm_mesh(model, side, measured, data=preview_data)
                target_mask |= project_arm_mask(mesh, target_camera, k, (height, width))
        return overlay_arm_target(rgb, target_mask)

    font = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", 19)

    def display_frame(head_rgb, overview, caption):
        header = Image.new("RGB", (2 * width, 70), (24, 31, 38))
        draw = ImageDraw.Draw(header)
        draw.text((12, 7), "头部 RGB · 青色为目标轮廓", font=font, fill=(224, 244, 248))
        draw.text((width + 12, 7), "固定 45° · 实际双臂姿态", font=font, fill=(224, 244, 248))
        draw.text((12, 40), caption, font=font, fill=(190, 211, 220))
        return np.vstack((cv2.cvtColor(np.asarray(header), cv2.COLOR_RGB2BGR), np.hstack((head_rgb, overview))))

    def save_image(name, image):
        if not cv2.imwrite(str(out / name), image):
            raise OSError(f"could not save simulation image {name}")

    if not headless:
        cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
    visible_steps = max(1, round(1 / (15 * model.opt.timestep)))
    try:
        with mujoco.Renderer(model, width=width, height=height) as renderer:
            save_image(
                "initial-views.png",
                display_frame(
                    render(renderer, "calibration_rgb"), render(renderer, "overview"), "双臂起始姿态 · 电机控制即将开始"
                ),
            )
            while True:
                tick = time.monotonic()
                for _ in range(visible_steps):
                    control.step()
                    motion.append(
                        (
                            control.command_time_s,
                            float(data.time),
                            control.reference_rad.copy(),
                            control.position_rad,
                            control.velocity_rad_s,
                            data.ctrl[control.motors],
                        )
                    )
                if data.time > ARM_TIMEOUT_S and (control.first_ready_s is None or headless):
                    raise RuntimeError("双臂在规定时间内未达到稳定姿态。")
                if headless:
                    if control.ready:
                        delta = demo_back_deg - requested_back_deg
                        if abs(delta) < 1e-8:
                            break
                        hand_turn(requested_back_deg + np.clip(delta, -0.5, 0.5))
                    continue

                mujoco.mj_forward(model, data)
                raw = render(renderer, "calibration_rgb")
                if control.ready:
                    shown = target_overlay(raw)
                    caption = (
                        f"双臂已稳定 · 目标后仰 {config['target_head_back_deg']:g}° · A / D 或 ← / → 调头 · Enter 确认"
                    )
                else:
                    shown = raw
                    caption = f"双臂正在就位 · 最大关节差 {np.rad2deg(control.max_error_rad):.2f}° · Esc 退出"
                cv2.imshow(window, display_frame(shown, render(renderer, "overview"), caption))
                wait = max(1, round(1000 * (visible_steps * model.opt.timestep - (time.monotonic() - tick))))
                key = cv2.waitKeyEx(wait)
                if key == 27 or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                    return None
                if control.ready:
                    if key in (ord("a"), ord("A"), 0x1000012, 65361):
                        hand_turn(requested_back_deg - 0.5)
                    elif key in (ord("d"), ord("D"), 0x1000014, 65363):
                        hand_turn(requested_back_deg + 0.5)
                    elif key in (10, 13):
                        break

            # Refresh body poses from integrated q and freeze both observations
            # without a time step between the kinematic readback and rendering.
            mujoco.mj_forward(model, data)
            if not control.ready:
                raise RuntimeError("确认时双臂未保持稳定。")
            capture_time = float(data.time)
            measured = dict(zip(control.names, control.position_rad.tolist(), strict=True))
            velocity = dict(zip(control.names, control.velocity_rad_s.tolist(), strict=True))
            raw = render(renderer, "calibration_rgb")
            save_image("rgb.png", raw)
            save_image(
                "confirmed-views.png",
                display_frame(
                    target_overlay(raw), render(renderer, "overview"), "确认帧 · 同一仿真时刻 · 双臂保持标定姿态"
                ),
            )
            capture = {
                "capture_sim_time_s": capture_time,
                "joint_feedback_sim_time_s": capture_time,
                "joint_positions_rad": measured,
                "joint_velocities_rad_s": velocity,
                "rgb_sha256": hashlib.sha256((out / "rgb.png").read_bytes()).hexdigest(),
                "intrinsics_K": k,
                "first_stable_sim_time_s": control.first_ready_s,
            }
            (out / "observation.json").write_text(json.dumps(capture, indent=2) + "\n")
            return capture, float(np.rad2deg(hinge.qpos[0]))
    finally:
        if not headless:
            cv2.destroyAllWindows()
        np.savez_compressed(
            out / "motion.npz",
            joint_names=np.array(control.names),
            command_time_s=[row[0] for row in motion],
            feedback_time_s=[row[1] for row in motion],
            reference_rad=[row[2] for row in motion],
            actual_rad=[row[3] for row in motion],
            actual_velocity_rad_s=[row[4] for row in motion],
            motor_ctrl=[row[5] for row in motion],
        )
        (out / "manual-head.json").write_text(
            json.dumps([{"sim_time_s": stamp, "lock_target_deg": angle} for stamp, angle in head_actions], indent=2)
            + "\n"
        )


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=root, help=argparse.SUPPRESS)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--headless", action="store_true", help="perform a scripted virtual manual adjustment")
    parser.add_argument("--demo-back-deg", type=float)
    parser.add_argument("--score-sim", action="store_true", help="score against simulated truth after fitting")
    args = parser.parse_args()
    # MuJoCo reads this on import, before creating an OpenGL context.
    os.environ.setdefault("MUJOCO_GL", "egl" if args.headless else "glfw")
    if args.demo_back_deg is not None and (
        not args.headless or not HEAD_LIMITS_DEG[0] <= args.demo_back_deg <= HEAD_LIMITS_DEG[1]
    ):
        parser.error("--demo-back-deg requires --headless and an angle between 14 and 26 degrees")
    demo_back_deg = args.demo_back_deg if args.demo_back_deg is not None else 19.5
    root = args.root.resolve()
    if not all((root / path).is_file() for path in ("profiles/door.json", "apps/simulation/terrain.py", ROBOT_XML)):
        parser.error(f"找不到 g1_deploy 门场景与固定手模型：{root}")

    from .calibration import fit_frame, score_frame, show_result

    config = load_scene_profile(root)
    if args.out is None:
        parent = root / "logs/head-calibration/sim"
        parent.mkdir(parents=True, exist_ok=True)
        out = Path(tempfile.mkdtemp(prefix="run-", dir=parent))
    else:
        out = args.out.resolve()
        out.mkdir(parents=True, exist_ok=False)
    print(f"结果目录 {out}", flush=True)
    frozen = capture_live(root, out, config=config, headless=args.headless, demo_back_deg=demo_back_deg)
    if frozen is None:
        print("已退出；轨迹保存在结果目录。", flush=True)
        return 130
    observation, actual_angle = frozen
    fit = out / "fit"
    print(f"双臂已稳定，使用 {observation['capture_sim_time_s']:.3f} s 的原图和关节反馈求解。", flush=True)
    # Keep the nominal fit prior separate from the live hinge and model.
    nominal_pivot = config["nominal_pivot_torso_m"]
    result = fit_frame(
        load_robot_model(root),
        out / "rgb.png",
        observation["joint_positions_rad"],
        fit,
        k=config["intrinsics_K"],
        zero=config["T_torso_camera_optical_zero"],
        initial=[config["target_head_back_deg"], 1000 * nominal_pivot[0], 1000 * nominal_pivot[2]],
    )
    summary = {"accepted": result["accepted"], "reason": result["reason"]}
    if args.score_sim:
        truth = {"head_back_deg": actual_angle, "pivot_torso_m": SIM_PIVOT_M}
        (out / "simulation-ground-truth.json").write_text(json.dumps(truth, indent=2) + "\n")
        summary["simulation"] = score_frame(result, truth, out / "score.json")
    print(json.dumps(summary, ensure_ascii=False), flush=True)
    if not args.headless:
        show_result(fit / "overlay.png", result)
    return 0 if result["accepted"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
