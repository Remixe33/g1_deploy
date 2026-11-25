"""录制器：PLY 点云序列、模型深度 PNG、时间戳/状态 CSV 和 meta.json。

同一录制代码路径服务仿真与实机；深度逐帧反投影为标准 PLY 二进制点云
（深度光轴系 x 右/y 下/z 前），CloudCompare/Open3D/PCL 直接可用。
"""
import csv
import json
import threading
import time
from pathlib import Path

import cv2
import numpy as np

class Recorder:
    """线程安全：add_frame（相机线程）与 add_state（DDS 线程）可并发调用。"""

    def __init__(self, out_dir, source_info, meta_extra=None, intrinsics=None):
        self.dir = Path(out_dir)
        (self.dir / "pointcloud").mkdir(parents=True, exist_ok=True)
        (self.dir / "model").mkdir(parents=True, exist_ok=True)
        self.source_info = dict(source_info)
        self.meta_extra = dict(meta_extra or {})
        self.intrinsics = dict(intrinsics or {})
        self.lock = threading.Lock()
        self.recording = True
        self.started_at = time.monotonic()
        self.wall_started = time.time()
        self.frames = 0
        self.model_frames = 0
        self.gaps = 0
        self.last_sequence = None
        self.state_rows = 0
        self._first_capture_at = None
        self._last_capture_at = None
        self._ply_seconds = 0.0
        self._ts_file = None
        self._ts_csv = None
        self._state_file = None
        self._state_csv = None
        self.meta_path = self.dir / "meta.json"

    def add_frame(self, frame):
        """保存实际收到的深度采样。"""
        captured = frame.capture_monotonic
        raw = np.asarray(frame.raw)
        with self.lock:
            if not self.recording or captured < self.started_at or frame.sequence == self.last_sequence:
                return
            if self._ts_csv is None:
                self._ts_file = open(self.dir / "timestamps.csv", "w", newline="", encoding="utf-8")
                self._ts_csv = csv.writer(self._ts_file)
                self._ts_csv.writerow(["index", "host_monotonic", "wall_epoch", "source_time", "sequence",
                                       "sequence_gap", "depth_min_m", "depth_max_m", "device_frame_number"])
            gap = self.last_sequence is not None and frame.sequence != self.last_sequence + 1
            self.gaps += int(gap)
            index = self.frames
            ply_start = time.monotonic()
            points = self._backproject(raw)
            self._write_ply(self.dir / "pointcloud" / f"{index:06d}.ply", points)
            self._ply_seconds += time.monotonic() - ply_start
            model = getattr(frame, "model", None)
            if model is not None:
                model_png = np.clip(np.nan_to_num(np.asarray(model), nan=0., posinf=1.) * 255,
                                    0, 255).astype(np.uint8)
                cv2.imwrite(str(self.dir / "model" / f"{index:06d}.png"), model_png)
                self.model_frames += 1
            finite = raw[np.isfinite(raw)]
            wall_epoch = self.wall_started + (captured - self.started_at)
            self._ts_csv.writerow([index, f"{captured:.6f}", f"{wall_epoch:.6f}",
                                   f"{frame.time:.6f}", frame.sequence, int(gap),
                                   f"{finite.min():.4f}" if finite.size else "",
                                   f"{finite.max():.4f}" if finite.size else "",
                                   getattr(frame, "device_frame_number", None)])
            self._ts_file.flush()
            self.last_sequence = frame.sequence
            if self._first_capture_at is None:
                self._first_capture_at = captured
            self._last_capture_at = captured
            self.frames += 1

    def add_state(self, snapshot):
        """snapshot: 监控快照 dict（joints 列表 + imu + 状态标志），CSV 按首行键展开。"""
        row = self._flatten_state(snapshot)
        with self.lock:
            if not self.recording:
                return
            if self._state_csv is None:
                self._state_file = open(self.dir / "state.csv", "w", newline="", encoding="utf-8")
                self._state_csv = csv.writer(self._state_file)
                self._state_csv.writerow(row.keys())
            self._state_csv.writerow(row.values())
            self.state_rows += 1

    def _backproject(self, raw):
        """针孔反投影到深度光轴系（x 右/y 下/z 前）；inf/nan/非正深度剔除。"""
        height, width = raw.shape[:2]
        fx = self.intrinsics.get("fx") or width / 2
        fy = self.intrinsics.get("fy") or height / 2
        ppx = self.intrinsics.get("ppx") or width / 2
        ppy = self.intrinsics.get("ppy") or height / 2
        valid = np.isfinite(raw) & (raw > 0)
        v, u = np.nonzero(valid)
        z = raw[v, u].astype(np.float32)
        points = np.empty((len(z), 3), dtype="<f4")
        points[:, 0] = (u - ppx) * z / fx
        points[:, 1] = (v - ppy) * z / fy
        points[:, 2] = z
        return points

    def _write_ply(self, path, points):
        header = ("ply\nformat binary_little_endian 1.0\n"
                  f"element vertex {len(points)}\n"
                  "property float x\nproperty float y\nproperty float z\nend_header\n")
        with open(path, "wb") as handle:
            handle.write(header.encode("ascii"))
            points.tofile(handle)

    @staticmethod
    def _flatten_state(snapshot):
        row = {"host_monotonic": f"{time.monotonic():.6f}", "wall_epoch": f"{time.time():.6f}",
               "sample_id": snapshot.get("sample_id"), "dry_run": snapshot.get("dry_run"),
               "output_enabled": snapshot.get("output_enabled"),
               "age_ms": snapshot.get("age_ms")}
        for key in ("rpy", "gyroscope", "accelerometer", "quaternion"):
            values = (snapshot.get("imu") or {}).get(key)
            for axis, value in enumerate(values or []):
                row[f"imu_{key}_{axis}"] = value
        torso = snapshot.get("torso_imu") or {}
        row["torso_fresh"] = torso.get("fresh")
        for key in ("rpy", "gyroscope", "accelerometer"):
            values = (torso.get("data") or {}).get(key)
            for axis, value in enumerate(values or []):
                row[f"torso_{key}_{axis}"] = value
        for joint in snapshot.get("joints", []):
            name = joint.get("name", f"joint{joint.get('index')}")
            for field in ("q", "dq", "tau", "q_target", "dq_target", "error"):
                row[f"{name}_{field}"] = joint.get(field)
        return row

    def stop(self, source_snapshot=None):
        with self.lock:
            if not self.recording:
                return {}
            stopped_at = time.monotonic()
            wall_stopped = time.time()
            self.recording = False
        source_info = source_snapshot() if source_snapshot is not None else None
        duration = stopped_at - self.started_at
        if self._ts_file is not None:
            self._ts_file.close()
        if self._state_file is not None:
            self._state_file.close()
        span = self._last_capture_at - self._first_capture_at if self.frames else 0.0
        meta = {"source": self.source_info, "duration_s": round(duration, 3),
                "frames": self.frames, "model_frames": self.model_frames,
                "sequence_gaps": self.gaps,
                "configured_fps": self.source_info.get("fps"),
                "saved_fps": round((self.frames - 1) / span, 3) if span > 0 else None,
                "first_capture_monotonic": self._first_capture_at,
                "last_capture_monotonic": self._last_capture_at,
                "stopped_host_monotonic": stopped_at,
                "timing": {"backproject_and_ply_seconds": round(self._ply_seconds, 6)},
                "pointcloud": {"format": "ply_binary_little_endian",
                               "frame": "depth_optical (x right, y down, z forward)",
                               "intrinsics": self.intrinsics},
                "state_rows": self.state_rows, "started_wall": self.wall_started,
                "stopped_wall": wall_stopped, "extra": self.meta_extra}
        if self.source_info.get("source") == "realsense" and source_info is not None:
            capture = {key: source_info[key] - self.source_info[key] for key in
                       ("received_frames", "device_frames_skipped", "device_counter_resets",
                        "capture_timeouts", "filter_frames")}
            capture["received_fps"] = round(capture["received_frames"] / duration, 3) if duration > 0 else None
            capture["filter_seconds"] = round(source_info["filter_seconds"] - self.source_info["filter_seconds"], 6)
            meta["capture"] = capture
        self.meta_path.write_text(json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")
        return meta
