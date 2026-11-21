"""Read robot state and RGB-D for operator-confirmed head calibration over loopback."""

import argparse
import hashlib
import json
import secrets
import tempfile
import time
from collections import OrderedDict
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from threading import Lock
from urllib.parse import urlsplit

import cv2
import numpy as np

MIN_POSE_CHANGE_RAD = 0.10
FRAME_RETENTION = 16


def _json_default(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(f"cannot serialize {type(value).__name__}")


@dataclass
class _Preview:
    sample: dict
    png: bytes
    ticket: str = ""


class ReviewSession:
    def __init__(self, model, out, profile, *, zero, initial, fit, check, max_confirm_age_s=1.0):
        self.model = model
        self.out = out
        self.profile = profile
        self.zero = zero
        self.initial = initial
        self.fit = fit
        self.check = check
        self.max_confirm_age_s = max_confirm_age_s
        self.phase = "fit"
        self._frames = OrderedDict()
        self._sequence = 0
        self._generation = 0
        self._phase_started = time.monotonic()
        self._lock = Lock()
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="g1-calibration")
        self._future = None
        self._fitted_pose = None
        self._fitted_transform = None
        self._result = None
        self._result_overlay = None
        self._depth = None
        self._error = None

    def publish(self, sample, png, *, generation=None):
        sample = {
            **sample,
            **{key: sample[key].copy() for key in ("raw_bgr", "bgr", "intrinsics_K")},
        }
        if sample.get("depth_z16") is not None:
            sample["depth_z16"] = sample["depth_z16"].copy()
        with self._lock:
            if generation is not None and generation != self._generation:
                return None
            self._sequence += 1
            self._frames[self._sequence] = _Preview(sample, png)
            while len(self._frames) > FRAME_RETENTION:
                self._frames.popitem(last=False)
            return self._sequence

    def _eligible(self, frame):
        if self.phase not in ("fit", "validation"):
            return False, "当前阶段不能确认新帧。"
        if not frame or time.monotonic() - frame.sample["capture"]["host_arrival_s"] > self.max_confirm_age_s:
            return False, "画面已过期，等待新帧。"
        sample = frame.sample
        if sample["capture"]["host_arrival_s"] < self._phase_started:
            return False, "等待当前步骤的新画面。"
        if sample["joints"] is None or not sample["capture"]["stable"]:
            return False, sample["capture"].get("stability_reason") or "等待双臂稳定和同帧反馈。"
        if self.phase == "validation":
            angles = sample["joints"]
            if max(abs(angle - self._fitted_pose[name]) for name, angle in angles.items()) < MIN_POSE_CHANGE_RAD:
                return False, "请切换到另一组双手可见的姿态，停稳后再确认。"
        return True, None

    def state(self):
        with self._lock:
            frame_id = next(reversed(self._frames), None)
            eligible, reason = self._eligible(self._frames.get(frame_id))
            return {
                "phase": self.phase,
                "frame": frame_id,
                "eligible": eligible,
                "frame_reason": reason,
                "result": self._result,
                "result_overlay": self._result_overlay is not None,
                "result_id": self._result_overlay.parent.parent.name if self._result_overlay else None,
                "depth_ready": self._depth is not None,
                "error": self._error,
            }

    def preview(self, frame_id):
        with self._lock:
            frame = self._frames.get(frame_id)
            if frame is None:
                return None
            if not frame.ticket:
                frame.ticket = secrets.token_urlsafe(24)
            eligible, _ = self._eligible(frame)
            return frame.png, frame.ticket, eligible

    def preview_context(self):
        with self._lock:
            return self._generation, self.phase, self._fitted_transform

    def confirm(self, frame_id, ticket):
        with self._lock:
            frame = self._frames.get(frame_id)
            if not frame or not frame.ticket or not secrets.compare_digest(frame.ticket, ticket):
                return False, "该画面未显示或已处理，请等候下一帧。"
            accepted, reason = self._eligible(frame)
            newest = self._frames[next(reversed(self._frames))].sample
            if not accepted:
                return False, reason
            if (
                not newest["capture"]["stable"]
                or newest["joints"] is None
                or max(abs(value - newest["joints"][name]) for name, value in frame.sample["joints"].items()) > 0.01
            ):
                return False, "双臂状态已变化，停稳后请重新确认。"
            stage = self.phase
            self.phase = "fitting" if stage == "fit" else "checking"
            self._generation += 1
            self._frames.clear()
            self._error = None
            self._future = self._executor.submit(self._evaluate, frame_id, frame, stage)
            return True, None

    def _evaluate(self, frame_id, frame, stage):
        sample = frame.sample
        candidate = self.out / f"frame-{frame_id:06d}"
        native = candidate / "rgb_raw.png"
        rectified = candidate / "rgb_rectified.png"
        evaluation_dir = candidate / ("fit" if stage == "fit" else "check")
        next_phase = stage
        result = depth_path = transform = None
        error = None
        try:
            candidate.mkdir()
            for path, image in ((native, sample["raw_bgr"]), (rectified, sample["bgr"])):
                if not cv2.imwrite(str(path), image):
                    raise OSError(f"无法保存 RGB {path}")
            if sample.get("depth_z16") is not None:
                np.save(candidate / "depth_raw_z16.npy", sample["depth_z16"], allow_pickle=False)
            (candidate / "browser-preview.png").write_bytes(frame.png)
            observation = {
                "browser_frame_id": frame_id,
                "phase": stage,
                "capture": sample["capture"],
                "joint_positions_rad": sample["joints"],
                "joint_velocities_rad_s": sample["joint_velocities"],
                "K_rgb_rectified": sample["intrinsics_K"],
                "profile": self.profile,
                "browser_preview_sha256": hashlib.sha256(frame.png).hexdigest(),
                "rgb_raw_sha256": hashlib.sha256(native.read_bytes()).hexdigest(),
                "rgb_rectified_sha256": hashlib.sha256(rectified.read_bytes()).hexdigest(),
            }
            (candidate / "observation.json").write_text(
                json.dumps(observation, indent=2, default=_json_default) + "\n", encoding="utf-8"
            )
            if stage == "fit":
                result = self.fit(
                    self.model,
                    rectified,
                    sample["joints"],
                    evaluation_dir,
                    k=sample["intrinsics_K"],
                    zero=self.zero,
                    initial=self.initial,
                )
                if result["accepted"]:
                    transform = np.asarray(result["T_torso_camera_optical"], float)
                    rgb_calibration = evaluation_dir / "calibration.json"
                    depth = {
                        "schema": "g1_head_depth_camera_calibration_v1",
                        "reference_frame": "torso_link",
                        "camera_frame": "head_depth_optical",
                        "length_unit": "m",
                        "T_torso_camera_optical": transform @ np.asarray(self.profile["T_rgb_depth"], float),
                        "intrinsics_K": self.profile["K_depth"],
                        "depth_scale_m_per_unit": self.profile["depth_scale_m"],
                        "device_serial": self.profile["serial"],
                        "depth_stream": self.profile["depth"],
                        "rgb_fit_sha256": hashlib.sha256(rgb_calibration.read_bytes()).hexdigest(),
                        "fit_observation": str((candidate / "observation.json").relative_to(self.out)),
                        "validation_status_at_export": "pending_separate_posture",
                    }
                    depth_path = self.out / "depth-camera-calibration.json"
                    depth_path.write_text(json.dumps(depth, indent=2, default=_json_default) + "\n", encoding="utf-8")
                    next_phase = "validation"
            else:
                result = self.check(
                    self.model,
                    rectified,
                    sample["joints"],
                    evaluation_dir,
                    k=sample["intrinsics_K"],
                    transform=self._fitted_transform,
                )
                if result["accepted"]:
                    next_phase = "complete"
        except (OSError, ValueError, RuntimeError, LookupError, cv2.error) as exc:
            # A failed capture or numerical job must keep the browser ready for a fresh frame.
            error = f"{type(exc).__name__}: {exc}"
            print(error, flush=True)
        overlay = evaluation_dir / "overlay.png"
        with self._lock:
            self._frames.clear()
            self.phase = next_phase
            self._generation += 1
            self._phase_started = time.monotonic()
            self._error = error
            if result is not None:
                self._result = {
                    "stage": stage,
                    "accepted": bool(result["accepted"]),
                    "reason": result.get("reason"),
                    "cost_px": result.get("cost_px"),
                    "head_back_deg": result.get("head_back_deg"),
                    "pivot_torso_m": result.get("pivot_torso_m"),
                    "frame": frame_id,
                    "coverage": {
                        side: {
                            key: float(quality[key])
                            for key in ("edge_coverage", "hand_edge_coverage")
                            if key in quality
                        }
                        for side, quality in result.get("quality_by_side", {}).items()
                    },
                }
                self._result_overlay = overlay if overlay.is_file() else None
            if depth_path is not None and error is None:
                self._depth = depth_path
                self._fitted_pose = sample["joints"]
                self._fitted_transform = transform

    def artifact(self, kind):
        with self._lock:
            return self._depth if kind == "depth" else self._result_overlay

    def wait_for_job(self):
        with self._lock:
            future = self._future
        if future is not None:
            future.result()

    def close(self):
        self._executor.shutdown(wait=True)


def browser_server(session, port):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format, *args):
            pass

        def _is_local(self):
            host = self.headers.get("Host", "")
            origin = self.headers.get("Origin")
            return host.split(":")[0] in ("127.0.0.1", "localhost") and (origin is None or origin == f"http://{host}")

        def _send(self, status, data, kind, headers=()):
            try:
                self.send_response(status)
                self.send_header("Content-Type", kind)
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Content-Type-Options", "nosniff")
                self.send_header("Content-Length", str(len(data)))
                for name, value in headers:
                    self.send_header(name, value)
                self.end_headers()
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                # Preview fetches may be replaced when the displayed frame advances.
                return

        def _json(self, status, payload):
            self._send(status, json.dumps(payload, ensure_ascii=False).encode(), "application/json; charset=utf-8")

        def do_GET(self):
            if not self._is_local():
                self._json(HTTPStatus.FORBIDDEN, {"error": "只允许本机连接。"})
                return
            path = urlsplit(self.path).path
            if path == "/":
                self._send(
                    HTTPStatus.OK, Path(__file__).with_name("real.html").read_bytes(), "text/html; charset=utf-8"
                )
                return
            if path == "/api/state":
                self._json(HTTPStatus.OK, session.state())
                return
            if path.startswith("/api/frame/") and path.endswith(".png"):
                value = path.removeprefix("/api/frame/").removesuffix(".png")
                preview = (
                    session.preview(int(value)) if value.isascii() and value.isdigit() and len(value) <= 12 else None
                )
                if preview is not None:
                    png, ticket, eligible = preview
                    self._send(
                        HTTPStatus.OK,
                        png,
                        "image/png",
                        (("X-Frame-Ticket", ticket), ("X-Frame-Eligible", str(int(eligible)))),
                    )
                    return
            artifacts = {
                "/api/depth.json": ("depth", "application/json; charset=utf-8"),
                "/api/result.png": ("overlay", "image/png"),
            }
            if path in artifacts:
                artifact, kind = artifacts[path]
                file = session.artifact(artifact)
                if file is not None:
                    self._send(HTTPStatus.OK, file.read_bytes(), kind)
                    return
            self._json(HTTPStatus.NOT_FOUND, {"error": "画面或文件已不可用。"})

        def do_POST(self):
            if not self._is_local():
                self._json(HTTPStatus.FORBIDDEN, {"error": "只允许本机连接。"})
                return
            if self.path != "/api/confirm":
                self._json(HTTPStatus.NOT_FOUND, {"error": "未知请求。"})
                return
            size = self.headers.get("Content-Length", "")
            if (
                self.headers.get_content_type() != "application/json"
                or not size.isascii()
                or not size.isdigit()
                or len(size) > 4
                or not 0 < int(size) <= 512
            ):
                self._json(HTTPStatus.BAD_REQUEST, {"error": "确认请求格式错误。"})
                return
            try:
                data = json.loads(self.rfile.read(int(size)))
            except (json.JSONDecodeError, UnicodeDecodeError):
                self._json(HTTPStatus.BAD_REQUEST, {"error": "确认请求格式错误。"})
                return
            if (
                not isinstance(data, dict)
                or type(data.get("frame")) is not int
                or not isinstance(data.get("ticket"), str)
                or not data["ticket"].isascii()
            ):
                self._json(HTTPStatus.BAD_REQUEST, {"error": "确认请求格式错误。"})
                return
            accepted, reason = session.confirm(data["frame"], data["ticket"])
            self._json(
                HTTPStatus.ACCEPTED if accepted else HTTPStatus.CONFLICT, {"accepted": accepted, "error": reason}
            )

    return ThreadingHTTPServer(("127.0.0.1", port), Handler)


def main():
    from threading import Thread

    from .calibration import check_frame, fit_frame, overlay_frame
    from .contour import contour_pose_transform
    from .geometry import validate_rigid
    from .real_input import RealInput
    from .sim_scene import load_robot_model

    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    config = json.loads(args.config.read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        parser.error("配置必须是一个 JSON 对象。")
    for key, label in (("interface", "DDS 网卡"), ("serial", "相机序列号")):
        value = config.get(key)
        if (
            not isinstance(value, str)
            or not value.strip()
            or value != value.strip()
            or value.startswith("REPLACE_WITH_")
        ):
            parser.error(f"请在配置中填写实机{label}（{key}）。")
    port = config.get("port", 8768)
    if type(port) is not int or not 1 <= port <= 65535:
        parser.error("port 必须是 1–65535 的整数。")
    try:
        zero = np.asarray(config.get("T_torso_camera_optical_zero"), float)
        validate_rigid(zero, "T_torso_camera_optical_zero")
        if not np.isfinite(zero).all():
            raise ValueError("平移值必须有效")
    except (ValueError, TypeError) as exc:
        parser.error(f"请提供光学相机到躯干的有效零位刚体变换：{exc}")
    pivot = np.asarray(config.get("nominal_pivot_torso_m", [0, 0, 0.32]), float)
    angle = float(config.get("nominal_head_back_deg", 20))
    if pivot.shape != (3,) or not np.isfinite(pivot).all() or not 12 <= angle <= 60:
        parser.error("请提供有效名义轴心和 12–60° 的后仰角。")
    initial = np.array([angle, pivot[0] * 1000, pivot[2] * 1000])
    target = contour_pose_transform(zero, initial)
    model = load_robot_model(Path(config.get("g1_deploy_root", root)).resolve())
    if args.out is None:
        parent = root / "logs/head-calibration"
        parent.mkdir(parents=True, exist_ok=True)
        out = Path(tempfile.mkdtemp(prefix=time.strftime("real-%Y%m%d-%H%M%S-"), dir=parent))
    else:
        out = args.out.resolve()
        out.mkdir(parents=True, exist_ok=False)
    print(f"结果目录 {out}", flush=True)
    with RealInput(config) as source:
        profile = source.profile
        session = ReviewSession(model, out, profile, zero=zero, initial=initial, fit=fit_frame, check=check_frame)
        server = browser_server(session, port)
        thread = Thread(target=server.serve_forever, daemon=True)
        thread.start()
        print(f"浏览器地址 http://127.0.0.1:{server.server_port}", flush=True)
        try:
            last_preview = float("-inf")
            while True:
                sample = source.read()
                if sample is None:
                    time.sleep(0.003)
                    continue
                now = time.monotonic()
                if now - last_preview < 0.10:
                    continue
                last_preview = now
                generation, phase, fitted = session.preview_context()
                image = sample["bgr"]
                if sample["joints"] is not None and phase in ("fit", "validation"):
                    image = overlay_frame(
                        model,
                        image,
                        sample["joints"],
                        k=sample["intrinsics_K"],
                        transform=fitted if fitted is not None else target,
                    )
                ok, encoded = cv2.imencode(".png", image)
                if not ok:
                    raise OSError("无法编码标定预览图。")
                session.publish(sample, encoded.tobytes(), generation=generation)
        except KeyboardInterrupt:
            return 0
        finally:
            server.shutdown()
            server.server_close()
            thread.join()
            session.close()


if __name__ == "__main__":
    raise SystemExit(main())
