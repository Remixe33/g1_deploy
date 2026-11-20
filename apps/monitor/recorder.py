"""录制器：RGB MP4 + 深度伪彩 MP4 + PLY 点云序列 + 时间戳/状态 CSV + meta.json。

同一录制代码路径服务仿真与实机；深度逐帧反投影为标准 PLY 二进制点云
（深度光轴系 x 右/y 下/z 前，xyz+RGB），CloudCompare/Open3D/PCL 直接可用。
"""
import csv
import json
import threading
import time
from pathlib import Path

import cv2
import numpy as np

DEPTH_RANGE_M = 2.5  # 伪彩归一化区间，与预览一致


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
        self._rgb_writer = None
        self._depth_writer = None
        self._ts_file = None
        self._ts_csv = None
        self._state_file = None
        self._state_csv = None
        self.meta_path = self.dir / "meta.json"

    def _writers(self, frame):
        size = (frame.width, frame.height)
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self._rgb_writer = cv2.VideoWriter(str(self.dir / "rgb.mp4"), fourcc, frame.fps, size)
        self._depth_writer = cv2.VideoWriter(str(self.dir / "depth_color.mp4"), fourcc, frame.fps, size)
        self._ts_file = open(self.dir / "timestamps.csv", "w", newline="", encoding="utf-8")
        self._ts_csv = csv.writer(self._ts_file)
        self._ts_csv.writerow(["index", "host_monotonic", "wall_epoch", "source_time", "sequence",
                               "sequence_gap", "depth_min_m", "depth_max_m"])

    def add_frame(self, frame):
        """frame: 带 raw/rgb/sequence/time/fps 的鸭子类型帧；RGB 为 RGB 顺序。
        按 sequence 去重：同一帧的重复投递（缓存帧）不入库。"""
        with self.lock:
            if self.last_sequence is not None and frame.sequence == self.last_sequence:
                return
        raw = np.asarray(frame.raw)
        height, width = raw.shape[:2]
        with self.lock:
            if not self.recording:
                return
            if self._rgb_writer is None:
                self._writers(frame)
            gap = self.last_sequence is not None and frame.sequence != self.last_sequence + 1
            self.gaps += int(gap)
            index = self.frames
            points, colors = self._backproject(raw, getattr(frame, "rgb", None))
            self._write_ply(self.dir / "pointcloud" / f"{index:06d}.ply", points, colors)
            model = getattr(frame, "model", None)
            if model is not None:
                model_png = np.clip(np.nan_to_num(np.asarray(model), nan=0., posinf=1.) * 255,
                                    0, 255).astype(np.uint8)
                cv2.imwrite(str(self.dir / "model" / f"{index:06d}.png"), model_png)
                self.model_frames += 1
            colorized = cv2.applyColorMap(
                np.rint((1 - np.clip(np.nan_to_num(raw / DEPTH_RANGE_M, posinf=1.), 0, 1)) * 255).astype(np.uint8),
                cv2.COLORMAP_TURBO)
            self._depth_writer.write(colorized)
            if frame.rgb is not None:
                self._rgb_writer.write(cv2.cvtColor(np.asarray(frame.rgb), cv2.COLOR_RGB2BGR))
            else:
                self._rgb_writer.write(np.zeros((height, width, 3), dtype=np.uint8))
            finite = raw[np.isfinite(raw)]
            self._ts_csv.writerow([index, f"{time.monotonic():.6f}", f"{time.time():.6f}",
                                   f"{frame.time:.6f}", frame.sequence, int(gap),
                                   f"{finite.min():.4f}" if finite.size else "",
                                   f"{finite.max():.4f}" if finite.size else ""])
            self._ts_file.flush()
            self.last_sequence = frame.sequence
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

    def _backproject(self, raw, rgb):
        """针孔反投影到深度光轴系（x 右/y 下/z 前）；inf/nan/非正深度剔除。"""
        height, width = raw.shape[:2]
        fx = self.intrinsics.get("fx") or width / 2
        fy = self.intrinsics.get("fy") or height / 2
        ppx = self.intrinsics.get("ppx") or width / 2
        ppy = self.intrinsics.get("ppy") or height / 2
        valid = np.isfinite(raw) & (raw > 0)
        v, u = np.nonzero(valid)
        z = raw[v, u].astype(np.float32)
        points = np.column_stack([(u - ppx) * z / fx, (v - ppy) * z / fy, z])
        colors = None
        if rgb is not None:
            rgb = np.asarray(rgb)
            if rgb.shape[:2] == raw.shape[:2]:
                colors = rgb[v, u]
        return points.astype(np.float32), colors

    def _write_ply(self, path, points, colors):
        count = len(points)
        if colors is None:
            dtype = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4")])
            properties = "property float x\nproperty float y\nproperty float z\n"
        else:
            dtype = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                              ("red", "u1"), ("green", "u1"), ("blue", "u1")])
            properties = ("property float x\nproperty float y\nproperty float z\n"
                          "property uchar red\nproperty uchar green\nproperty uchar blue\n")
        cloud = np.empty(count, dtype=dtype)
        cloud["x"], cloud["y"], cloud["z"] = points[:, 0], points[:, 1], points[:, 2]
        if colors is not None:
            cloud["red"], cloud["green"], cloud["blue"] = colors[:, 0], colors[:, 1], colors[:, 2]
        header = ("ply\nformat binary_little_endian 1.0\n"
                  f"element vertex {count}\n{properties}end_header\n")
        with open(path, "wb") as handle:
            handle.write(header.encode("ascii"))
            cloud.tofile(handle)

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

    def stop(self):
        with self.lock:
            if not self.recording:
                return {}
            self.recording = False
        duration = time.monotonic() - self.started_at
        if self._rgb_writer is not None:
            self._rgb_writer.release()
        if self._depth_writer is not None:
            self._depth_writer.release()
        if self._ts_file is not None:
            self._ts_file.close()
        if self._state_file is not None:
            self._state_file.close()
        meta = {"source": self.source_info, "duration_s": round(duration, 3),
                "frames": self.frames, "model_frames": self.model_frames,
                "sequence_gaps": self.gaps,
                "pointcloud": {"format": "ply_binary_little_endian",
                               "frame": "depth_optical (x right, y down, z forward)",
                               "intrinsics": self.intrinsics,
                               "rgb_alignment": "same-pixel mapping; nominal extrinsics, not pixel-accurate"},
                "state_rows": self.state_rows, "started_wall": self.wall_started,
                "stopped_wall": time.time(), "extra": self.meta_extra}
        self.meta_path.write_text(json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")
        return meta
