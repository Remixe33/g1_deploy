"""G1 29DOF 机器人数据可视化与录制服务（DDS 订阅 + 相机源 + 录制，只读不改控制）。"""
import argparse
import math
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import sys
import threading
import time
from urllib.parse import urlsplit

import cv2
import numpy as np

from camera_sources import MockSource, RealSenseSource, SimSource, load_camera_module
from depth_stream import DEFAULT_CAMERA_CONFIG, DepthCadence, profile_camera
from recorder import Recorder

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from web_addresses import print_web_addresses

DEPTH_RANGE_M = 2.5  # 预览伪彩归一化区间


JOINT_NAMES = (
    [f"{side}_{joint}_joint" for side in ("left", "right")
     for joint in ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")]
    + [f"waist_{joint}_joint" for joint in ("yaw", "roll", "pitch")]
    + [f"{side}_{joint}_joint" for side in ("left", "right")
       for joint in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
                     "wrist_roll", "wrist_pitch", "wrist_yaw")]
)


def read_imu(msg):
    """LowState quaternion is w,x,y,z; derive RPY for bridges omitting rpy."""
    return read_imu_state(getattr(msg, "imu_state", None))


def read_imu_state(source):
    if source is None:
        return None
    def vector(name, size):
        values = [float(v) for v in getattr(source, name, [])]
        return values if len(values) == size and all(math.isfinite(v) for v in values) else None
    quaternion = vector("quaternion", 4)
    rpy = None
    if quaternion is not None:
        norm = math.sqrt(sum(v * v for v in quaternion))
        if norm > 1e-9:
            w, x, y, z = [v / norm for v in quaternion]
            rpy = [math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y)),
                   math.asin(max(-1.0, min(1.0, 2 * (w*y - z*x)))),
                   math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))]
    return {"quaternion": quaternion, "rpy": rpy,
            "gyroscope": vector("gyroscope", 3),
            "accelerometer": vector("accelerometer", 3)}


class JointMonitor:
    def __init__(self, host="127.0.0.1", port=8765, rate_hz=50,
                 camera_source=None, camera_module=None, record_root="logs/recordings",
                 record_start=False):
        if not math.isfinite(rate_hz) or rate_hz <= 0:
            raise ValueError("Joint monitor rate must be positive")
        self.period = 1.0 / rate_hz
        self.last_capture = None
        self.torso_imu = None
        self.torso_at = None
        self.torso_count = 0
        self.lock = threading.Lock()
        self.changed = threading.Condition(self.lock)
        self.stopping = threading.Event()
        self.snapshot = {"sample_id": 0, "simulation_time": 0, "joints": []}
        self.names = JOINT_NAMES
        self.command = None
        self.command_at = None
        self.target = None
        self.target_at = None
        self.dry_target = None
        self.dry_target_at = None
        self.enable = None
        self.enable_at = None
        self.started_at = time.monotonic()
        self.next_capture = self.started_at
        self.html = (Path(__file__).parent / "static" / "joint_monitor.html").read_bytes()
        monitor = self
        self.camera_source = camera_source
        self.camera_module = camera_module or {}
        self.record_root = Path(record_root)
        self.recorder = None
        if record_start:
            self.start_recording()
        # 相机 MJPEG 信箱：相机线程写入编码后的 JPEG，端点流式推送。
        self.camera_lock = threading.Condition()
        self.camera_jpegs = None
        self.camera_sequence = 0
        self.camera_thread = None
        self.subscribers = []
        if camera_source is not None:
            self.camera_thread = threading.Thread(target=self.camera_loop,
                                                  name="camera-source", daemon=True)

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                path = urlsplit(self.path).path
                if path == "/api/stream":
                    self.stream()
                    return
                if path in ("/depth.mjpg", "/raw.mjpg", "/model.mjpg"):
                    monitor.mjpeg(self, path[1:-5])
                    return
                if path in ("/", "/index.html"):
                    content, mime = monitor.html, "text/html; charset=utf-8"
                elif path == "/api/joints":
                    with monitor.lock:
                        state = monitor.snapshot_locked()
                    content = json.dumps(state, allow_nan=False).encode()
                    mime = "application/json; charset=utf-8"
                elif path == "/api/status":
                    with monitor.lock:
                        state = monitor.snapshot_locked()
                    state.update(monitor.camera_status())
                    content = json.dumps(state, allow_nan=False).encode()
                    mime = "application/json; charset=utf-8"
                else:
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header("Content-Type", mime)
                self.send_header("Content-Length", str(len(content)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                try:
                    self.wfile.write(content)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def do_POST(self):
                path = urlsplit(self.path).path
                if path == "/api/record/start":
                    monitor.start_recording()
                    content = json.dumps({"recording": True,
                                          "dir": str(monitor.recorder.dir)}, ensure_ascii=False).encode()
                    mime = "application/json; charset=utf-8"
                elif path == "/api/record/stop":
                    meta = monitor.stop_recording()
                    content = json.dumps({"recording": False, "meta": meta}, ensure_ascii=False).encode()
                    mime = "application/json; charset=utf-8"
                else:
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header("Content-Type", mime)
                self.send_header("Content-Length", str(len(content)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                try:
                    self.wfile.write(content)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def log_message(self, *_):
                pass

            def stream(self):
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Accel-Buffering", "no")
                self.end_headers()
                self.connection.settimeout(5)
                previous = -1
                try:
                    while not monitor.stopping.is_set():
                        with monitor.changed:
                            monitor.changed.wait_for(
                                lambda: monitor.stopping.is_set() or
                                monitor.snapshot["sample_id"] != previous, timeout=1)
                            if monitor.stopping.is_set():
                                break
                            previous = monitor.snapshot["sample_id"]
                            state = monitor.snapshot_locked()
                        content = json.dumps(state, allow_nan=False, separators=(",", ":"))
                        self.wfile.write(("data: " + content + "\n\n").encode())
                        self.wfile.flush()
                except (OSError, TimeoutError):
                    pass

        self.server = ThreadingHTTPServer((host, port), Handler)
        self.server.daemon_threads = True
        self.thread = threading.Thread(target=self.server.serve_forever,
                                       name="joint-monitor", daemon=True)

    def camera_info(self):
        if self.camera_source is not None:
            return self.camera_source.info()
        return {"source": "none"}

    def start_recording(self):
        name = time.strftime("%Y%m%d-%H%M%S")
        source_name = self.camera_info().get("source", "none")
        meta = {"profile_camera_module": self.camera_module}
        intrinsics = self.camera_source.intrinsics() if self.camera_source is not None \
            and hasattr(self.camera_source, "intrinsics") else None
        self.recorder = Recorder(self.record_root / f"{source_name}-{name}",
                                 self.camera_info(), meta_extra=meta, intrinsics=intrinsics)
        print(f"Recording started: {self.recorder.dir}", flush=True)

    def stop_recording(self):
        meta = self.recorder.stop(self.camera_info)
        print(f"Recording stopped: {json.dumps(meta, ensure_ascii=False)}", flush=True)
        return meta

    def camera_loop(self):
        """保存实际收到的相机帧，并独立限制 MJPEG 预览刷新率。"""
        panes = getattr(self.camera_source, "panes", ["depth"]) if self.camera_source is not None else []
        cadence = DepthCadence(self.camera_source.stream)
        while not self.stopping.is_set():
            try:
                frame = self.camera_source.read()
            except Exception as error:
                print(f"Camera source error: {error}", flush=True)
                time.sleep(1)
                continue
            if frame is None:
                continue
            try:
                if self.recorder is not None and self.recorder.recording:
                    self.recorder.add_frame(frame)
            except Exception as error:
                print(f"Recorder error: {error}", flush=True)
            now = time.monotonic_ns()
            if now < cadence.next_ns:
                continue
            cadence.advance(now)
            jpegs = {}
            for pane in panes:
                image = self.pane_image(frame, pane)
                if image is None:
                    continue
                ok, buffer = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    jpegs[pane] = buffer.tobytes()
            with self.camera_lock:
                self.camera_jpegs = jpegs
                self.camera_sequence += 1
                self.camera_lock.notify_all()

    @staticmethod
    def pane_image(frame, pane):
        """生成供 JPEG 编码的 BGR 预览图。"""
        if pane == "model" and getattr(frame, "model", None) is not None:
            values = np.clip(np.nan_to_num(np.asarray(frame.model), nan=0., posinf=1.), 0., 1.)
        else:
            values = np.clip(np.nan_to_num(np.asarray(frame.raw) / DEPTH_RANGE_M, posinf=1.), 0., 1.)
        # 与 MuJoCo 预览一致：近处用暖色，远处用冷色。
        level = np.rint((1 - values) * 255).astype(np.uint8)
        image = cv2.applyColorMap(level, cv2.COLORMAP_TURBO)
        if pane == "raw":
            image[~np.isfinite(frame.raw)] = 0  # Parkour 未命中的射线显示为黑色。
        return image

    def mjpeg(self, handler, key):
        handler.send_response(200)
        handler.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        handler.send_header("Cache-Control", "no-store")
        handler.end_headers()
        handler.connection.settimeout(5)
        seen = -1
        try:
            while not self.stopping.is_set():
                with self.camera_lock:
                    if self.camera_jpegs is None or self.camera_sequence == seen:
                        self.camera_lock.wait_for(
                            lambda: self.stopping.is_set() or
                            (self.camera_jpegs is not None and self.camera_sequence != seen), timeout=1)
                        continue
                    if self.stopping.is_set():
                        break
                    seen = self.camera_sequence
                    data = self.camera_jpegs.get(key)
                if data is None:
                    continue
                handler.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                                    str(len(data)).encode() + b"\r\n\r\n" + data + b"\r\n")
        except (OSError, TimeoutError):
            pass

    def camera_status(self):
        with self.camera_lock:
            panes = sorted(self.camera_jpegs or {})
            sequence = self.camera_sequence
        source_info = self.camera_info()
        recording = self.recorder.recording if self.recorder is not None else False
        return {"camera": {**source_info,
                           "online": bool(panes),
                           "sequence": sequence,
                           "panes": panes},
                "recording": {"active": recording,
                              "dir": str(self.recorder.dir) if recording else None,
                              "frames": self.recorder.frames if self.recorder is not None else 0,
                              "state_rows": self.recorder.state_rows if self.recorder is not None else 0,
                              "sequence_gaps": self.recorder.gaps if self.recorder is not None else 0}}

    def start(self):
        self.thread.start()
        if self.camera_thread is not None:
            self.camera_thread.start()
        print_web_addresses("Data web", self.server.server_address)

    def start_subscribers(self, domain, interface, init_factory=True):
        """创建 DDS 订阅；init_factory=False 时由宿主进程先完成 ChannelFactoryInitialize。"""
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, LowCmd_, IMUState_
        if init_factory:
            ChannelFactoryInitialize(domain, interface)
        for topic, message_type, callback in (
                ("rt/lowstate", LowState_, self.on_state),
                ("rt/secondary_imu", IMUState_, self.on_torso_imu),
                ("rt/lowcmd", LowCmd_, self.on_command),
                ("rt/g1/reset_target", LowCmd_, self.on_target),
                ("rt/g1/dry_run_target", LowCmd_, self.on_dry_target),
                ("rt/g1/reset_enable", LowCmd_, self.on_enable)):
            subscriber = ChannelSubscriber(topic, message_type)
            self.subscribers.append(subscriber)
            subscriber.Init(callback, 1)

    def snapshot_locked(self):
        # Called with self.lock held. Recompute torso freshness even if LowState stops.
        now = time.monotonic()
        state = dict(self.snapshot)
        state["age_ms"] = None if self.last_capture is None else round((now-self.last_capture)*1000, 1)
        age = None if self.torso_at is None else (now-self.torso_at)*1000
        state["torso_imu"] = {"topic": "rt/secondary_imu", "age_ms": None if age is None else round(age, 1),
                              "fresh": age is not None and age < 100 and self.torso_imu is not None,
                              "valid": self.torso_imu is not None, "samples": self.torso_count,
                              "data": self.torso_imu}
        return state

    def on_torso_imu(self, msg):
        data = read_imu_state(msg)
        if data is not None:
            q = data["quaternion"]
            norm = sum(v*v for v in q) if q is not None else 0
            if not .5 <= norm <= 1.5 or data["gyroscope"] is None:
                data = None
            else:
                w,x,y,z = q
                data["projected_gravity"] = [2*(w*y-x*z)/norm, -2*(w*x+y*z)/norm,
                                             (2*(x*x+y*y)-norm)/norm]
        with self.changed:
            self.torso_imu = data
            self.torso_at = time.monotonic()
            self.torso_count += 1
            self.changed.notify_all()

    def on_command(self, msg):
        if len(msg.motor_cmd) < len(self.names):
            return
        command = [tuple(float(getattr(m, k)) for k in ("q", "dq", "kp", "kd", "tau"))
                   for m in msg.motor_cmd[:len(self.names)]]
        if not all(math.isfinite(v) for motor in command for v in motor):
            return
        with self.lock:
            self.command = command
            self.command_at = time.monotonic()

    def on_state(self, msg):
        now = time.monotonic()
        if len(msg.motor_state) < len(self.names):
            return
        if getattr(msg, "mode_machine", 0) not in (0, 5):
            return  # refuse known non-29DOF modes
        with self.lock:
            if now < self.next_capture:
                return
            command = self.command
            command_fresh = self.command_at is not None and now - self.command_at < 0.5
            target = self.target
            target_fresh = self.target_at is not None and now - self.target_at < 0.5
            enable = self.enable
            enable_fresh = self.enable_at is not None and now - self.enable_at < 0.5
            dry_target = self.dry_target
            dry_run = self.dry_target_at is not None and now - self.dry_target_at < 0.5 and not (enable_fresh and enable[0])
        joints = []
        for i, name in enumerate(self.names):
            motor = msg.motor_state[i]
            q, dq, tau = float(motor.q), float(motor.dq), float(motor.tau_est)
            desired = dry_target if dry_run else target if target_fresh else command
            q_target, dq_target, kp, kd, _ = (
                desired[i] if desired is not None else (0, 0, 0, 0, 0))
            actual_command = command[i] if command is not None else (0, 0, 0, 0, 0)
            joints.append({
                "index": i, "name": name, "q": q, "dq": dq, "tau": tau,
                "q_target": q_target, "dq_target": dq_target,
                "kp": kp, "kd": kd, "error": q_target - q,
                "target_available": dry_run or target_fresh or command_fresh,
                "enabled": enable[i] if enable_fresh else None,
                "active": command_fresh and any(v != 0 for v in actual_command[2:]),
            })
        state = {"sample_id": self.snapshot["sample_id"] + 1,
                 "dry_run": dry_run,
                 "simulation_time": now - self.started_at, "joints": joints,
                 "output_enabled": enable[0] if enable_fresh else None,
                 "imu": read_imu(msg)}
        try:
            json.dumps(state, allow_nan=False)
        except ValueError:
            return
        with self.changed:
            self.snapshot = state
            self.last_capture = now
            # Keep a fixed schedule rather than adding callback jitter to each period.
            slots = int((now - self.next_capture) / self.period) + 1
            self.next_capture += slots * self.period
            self.changed.notify_all()
        if self.recorder is not None and self.recorder.recording:
            self.recorder.add_state(state)

    def on_target(self, msg):
        if len(msg.motor_cmd) < len(self.names):
            return
        target = [tuple(float(getattr(m, k)) for k in ("q", "dq", "kp", "kd", "tau"))
                  for m in msg.motor_cmd[:len(self.names)]]
        if not all(math.isfinite(v) for motor in target for v in motor):
            return
        with self.lock:
            self.target = target
            self.target_at = time.monotonic()

    def on_enable(self, msg):
        if len(msg.motor_cmd) < len(self.names):
            return
        modes = [int(m.mode) for m in msg.motor_cmd[:len(self.names)]]
        if any(mode not in (0, 1) for mode in modes):
            return
        with self.lock:
            self.enable = [mode == 1 for mode in modes]
            self.enable_at = time.monotonic()

    def on_dry_target(self, msg):
        if len(msg.motor_cmd) < len(self.names):
            return
        target = [tuple(float(getattr(m, k)) for k in ("q", "dq", "kp", "kd", "tau"))
                  for m in msg.motor_cmd[:len(self.names)]]
        if not all(math.isfinite(v) for motor in target for v in motor):
            return
        with self.lock:
            self.dry_target = target
            self.dry_target_at = time.monotonic()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def close(self):
        self.stopping.set()
        with self.changed:
            self.changed.notify_all()
        with self.camera_lock:
            self.camera_lock.notify_all()
        if self.thread.is_alive():
            self.server.shutdown()
            self.thread.join(timeout=2)
        self.server.server_close()
        if self.recorder is not None and self.recorder.recording:
            self.stop_recording()
        if self.camera_source is not None:
            self.camera_source.close()


def main():
    parser = argparse.ArgumentParser(description="G1 29DOF robot data web: joints/IMU + camera + recording")
    parser.add_argument("--domain", type=int, default=1)
    parser.add_argument("--interface", default="lo")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--rate", type=float, default=50,
                        help="DDS sampling limit in Hz (default: 50)")
    parser.add_argument("--source", choices=("none", "realsense", "sim", "mock"), default="none",
                        help="Camera source: realsense device, sim injection (used by sim.py), or mock")
    camera_options = parser.add_mutually_exclusive_group()
    camera_options.add_argument("--profile", help="Task name or profile JSON (uses camera.config)")
    camera_options.add_argument("--camera-config", help="Camera module JSON (profiles/configs/cameras/*.json)")
    parser.add_argument("--record", action="store_true", help="Start recording immediately")
    parser.add_argument("--record-dir", default="logs/recordings", help="Recording root directory")
    args = parser.parse_args()
    if args.domain < 0 or not 1 <= args.port <= 65535 or not math.isfinite(args.rate) or args.rate <= 0:
        parser.error("domain must be nonnegative; rate must be positive; port must be 1..65535")
    camera_config = args.camera_config
    if args.source != "none" and args.profile:
        try:
            camera_config = profile_camera(args.profile)
        except (OSError, ValueError) as error:
            parser.error(str(error))
    camera_source = None
    camera_module = None
    try:
        if args.source == "realsense":
            camera_source = RealSenseSource(camera_config or DEFAULT_CAMERA_CONFIG)
            camera_module = camera_source.module
        elif args.source == "mock":
            camera_source = MockSource(camera_config)
            camera_module = camera_source.module
        elif args.source == "sim":
            camera_source = SimSource()
            if camera_config:
                camera_module = load_camera_module(camera_config)
                camera_source.set_module(camera_module)
    except (OSError, ValueError) as error:
        parser.error(str(error))

    monitor = JointMonitor(args.host, args.port, args.rate, camera_source=camera_source,
                           camera_module=camera_module, record_root=args.record_dir)
    try:
        monitor.start_subscribers(args.domain, args.interface)
        if camera_source is not None and hasattr(camera_source, "open"):
            camera_source.open()
        if args.record:
            monitor.start_recording()
        monitor.start()
        print(f"Waiting for G1 29DOF DDS data: domain={args.domain}, interface={args.interface}. Ctrl+C to exit.", flush=True)
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        for subscriber in monitor.subscribers:
            subscriber.Close()
        monitor.close()


if __name__ == "__main__":
    main()
