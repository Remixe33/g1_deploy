"""统一数据网页的相机源：实机 RealSense / 仿真注入 / mock 合成。

帧包含米制深度 raw、sequence、源时间 time、标称 fps 与主机单调时间 capture_monotonic。
"""
import json
import math
import sys
import threading
import time
import types
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from depth_stream import DepthStream


def load_camera_module(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _frame(raw, sequence, source_time, fps, capture_monotonic, **metadata):
    return types.SimpleNamespace(raw=raw, sequence=sequence, time=source_time, fps=fps,
                                 capture_monotonic=capture_monotonic, **metadata,
                                 height=raw.shape[0], width=raw.shape[1])


class SimSource:
    """仿真注入源：渲染在仿真主/查看器线程完成（deposit），本类只做信箱。

    窗格随相机模块 pipeline 推导：none → depth（door 原生）；parkour → raw+model。
    """

    def __init__(self):
        self._condition = threading.Condition()
        self._frame = None
        self._module = None
        self.stream = DepthStream.from_module({})
        self.panes = ["depth"]

    def set_module(self, module):
        self._module = module
        self.stream = DepthStream.from_module(module, simulation=True)
        pipeline = module.get("pipeline")
        self.panes = ["depth"] if pipeline == "none" else ["raw", "model"]

    def deposit(self, frame):
        received = time.monotonic()
        with self._condition:
            self._frame = _frame(frame.raw, frame.sequence, frame.time, frame.fps,
                                 received, model=frame.model)
            self._condition.notify_all()

    def read(self, timeout=1.0):
        with self._condition:
            if not self._condition.wait_for(lambda: self._frame is not None, timeout=timeout):
                return None
            frame = self._frame
            self._frame = None  # 等待下一次投递，避免监控线程忙等同一帧。
            return frame

    def intrinsics(self):
        """仿真渲染为标准针孔：由模块视场与仿真分辨率解析内参。"""
        module = self._module or {}
        width, height = self.stream.width, self.stream.height
        vfov = math.radians(float(module.get("vertical_fov_deg", 58.29)))
        hfov = 2.0 * math.atan(math.tan(vfov / 2) * width / height)
        return {"fx": (width / 2) / math.tan(hfov / 2), "fy": (height / 2) / math.tan(vfov / 2),
                "ppx": width / 2, "ppy": height / 2}

    def info(self):
        return {"source": "simulation", "width": self.stream.width, "height": self.stream.height,
                "fps": self.stream.fps,
                "panes": list(self.panes)}

    def close(self):
        pass


class MockSource:
    """无设备合成源：以软件节拍生成斜坡深度。"""

    def __init__(self, camera_json=None):
        module = load_camera_module(camera_json) if camera_json else {}
        self.stream = DepthStream.from_module(module)
        self.width, self.height, self.fps = self.stream.width, self.stream.height, self.stream.fps
        self.module = module
        self.sequence = 0
        self._deadline = time.monotonic()

    def read(self):
        self._deadline = max(self._deadline + 1.0 / self.fps, time.monotonic())
        time.sleep(max(0.0, self._deadline - time.monotonic()))
        received = time.monotonic()
        self.sequence += 1
        row = np.linspace(0.3, 2.5, self.width, dtype=np.float32)
        raw = np.tile(row, (self.height, 1))
        raw += float(self.sequence % 30) * 0.001  # 轻微变化，便于确认视频在动
        return _frame(raw, self.sequence, received, self.fps, received)

    def intrinsics(self):
        """合成针孔内参（与 realsense_web --mock 先例一致）。"""
        return {"fx": self.width / 2, "fy": self.height / 2,
                "ppx": self.width / 2, "ppy": self.height / 2}

    def info(self):
        return {"source": "mock", "width": self.width, "height": self.height, "fps": self.fps}

    def close(self):
        pass


class RealSenseSource:
    """pyrealsense2 采集：stream 取自相机模块，滤波链沿用独立预览；SDK 延迟导入。"""

    def intrinsics(self):
        """实机设备标定内参（open 前为 None 值占位）。"""
        intrinsics = dict(self._intrinsics or {})
        intrinsics.setdefault("fx", self.width / 2)
        intrinsics.setdefault("fy", self.height / 2)
        intrinsics.setdefault("ppx", self.width / 2)
        intrinsics.setdefault("ppy", self.height / 2)
        return intrinsics

    """pyrealsense2 采集：stream 取自相机模块，滤波链沿用独立预览；SDK 延迟导入。"""

    def __init__(self, camera_json):
        module = load_camera_module(camera_json)
        self.stream = DepthStream.from_module(module)
        if not self.stream.fps.is_integer():
            raise ValueError("RealSense depth FPS must be an integer supported by the device")
        self.width, self.height, self.fps = self.stream.width, self.stream.height, int(self.stream.fps)
        self.module = module
        self.sequence = 0
        self.serial = ""
        self._pipeline = None
        self._filters = None
        self._scale = 1.0
        self._intrinsics = None
        self._last_number = None
        self._first_received = None
        self._last_received = None
        self._device_frames_skipped = 0
        self._device_counter_resets = 0
        self._capture_timeouts = 0
        self._active_fps = None
        self._filter_frames = 0
        self._filter_seconds = 0.0
        self._filter_max_seconds = 0.0
        self.panes = ["depth"]

    def open(self):
        import pyrealsense2 as rs
        config = rs.config()
        if self.serial:
            config.enable_device(self.serial)
        config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        self._pipeline = rs.pipeline()
        profile = self._pipeline.start(config)
        depth_profile = profile.get_stream(rs.stream.depth).as_video_stream_profile()
        self._active_fps = depth_profile.fps()
        self._scale = profile.get_device().first_depth_sensor().get_depth_scale()
        intrinsics = depth_profile.get_intrinsics()
        self._intrinsics = {"fx": intrinsics.fx, "fy": intrinsics.fy,
                            "ppx": intrinsics.ppx, "ppy": intrinsics.ppy}
        # 滤波链只构建一次，时间滤波需要保留历史。
        self._filters = (rs.disparity_transform(True), rs.spatial_filter(), rs.temporal_filter(),
                         rs.disparity_transform(False), rs.hole_filling_filter(1))

    def capture(self, timeout_ms=1000):
        ready, sample = self._pipeline.try_wait_for_frames(timeout_ms)
        received = time.monotonic()
        if not ready:
            self._capture_timeouts += 1
            return None
        depth_frame = sample.get_depth_frame()
        if not depth_frame:
            return None
        number = depth_frame.get_frame_number()
        if self._last_number is not None and number == self._last_number:
            return None  # 同帧去重
        if self._last_number is not None:
            if number > self._last_number:
                self._device_frames_skipped += number - self._last_number - 1
            else:
                self._device_counter_resets += 1
        self._last_number = number
        if self._first_received is None:
            self._first_received = received
        self._last_received = received
        self.sequence += 1
        return depth_frame, received, number

    def read(self):
        captured = self.capture()
        if captured is None:
            return None
        depth_frame, received, number = captured
        sensor_time = float(depth_frame.get_timestamp()) * 0.001
        filter_start = time.monotonic()
        for stage in self._filters:
            depth_frame = stage.process(depth_frame)
        filter_seconds = time.monotonic() - filter_start
        self._filter_frames += 1
        self._filter_seconds += filter_seconds
        self._filter_max_seconds = max(self._filter_max_seconds, filter_seconds)
        raw = np.asanyarray(depth_frame.get_data()).astype(np.float32) * self._scale
        return _frame(raw, self.sequence, sensor_time, float(self.fps), received,
                      device_frame_number=number)

    def info(self):
        info = {"source": "realsense", "width": self.width, "height": self.height,
                "fps": self.fps, "depth_scale": self._scale,
                "intrinsics": self._intrinsics,
                "filters": "disparity+spatial+temporal+hole_filling"}
        span = (self._last_received - self._first_received) if self._last_received is not None else 0.0
        info.update(active_fps=self._active_fps, received_frames=self.sequence,
                    received_fps=round((self.sequence - 1) / span, 3) if span > 0 else None,
                    first_capture_monotonic=self._first_received, last_capture_monotonic=self._last_received,
                    device_frames_skipped=self._device_frames_skipped,
                    device_counter_resets=self._device_counter_resets,
                    capture_timeouts=self._capture_timeouts,
                    filter_frames=self._filter_frames, filter_seconds=round(self._filter_seconds, 6),
                    filter_max_ms=round(self._filter_max_seconds * 1000, 3))
        if self.serial:
            info["serial"] = self.serial
        return info

    def close(self):
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except RuntimeError:
                pass
            self._pipeline = None
