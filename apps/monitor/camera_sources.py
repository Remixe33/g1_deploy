"""统一数据网页的相机源：实机 RealSense / 仿真注入 / mock 合成。

帧为鸭子类型：需带 raw（米制深度 float32 HxW）、rgb（HxWx3 uint8 RGB 或 None）、
sequence（单调递增）、time（源时间戳秒）、fps 属性——apps/simulation 的 DepthFrame
天然满足，仿真注入无需转换。
"""
import json
import math
import threading
import time
import types
from pathlib import Path

import numpy as np


def load_camera_module(path):
    module = json.loads(Path(path).read_text())
    stream = module.get("stream", {})
    module.setdefault("_width", int(stream.get("width", 640)))
    module.setdefault("_height", int(stream.get("height", 480)))
    module.setdefault("_fps", float(stream.get("fps", 30)))
    return module


def _frame(raw, rgb, sequence, time, fps):
    return types.SimpleNamespace(raw=raw, rgb=rgb, sequence=sequence, time=time, fps=fps,
                                 height=raw.shape[0], width=raw.shape[1])


class SimSource:
    """仿真注入源：渲染在仿真主/查看器线程完成（deposit），本类只做信箱。"""

    def __init__(self):
        self._condition = threading.Condition()
        self._frame = None
        self._module = None

    def set_module(self, module):
        self._module = module

    def deposit(self, frame):
        """仿真主线程渲染后调用；DepthFrame 为冻结数据类，跨线程传递天然安全。"""
        with self._condition:
            self._frame = frame
            self._condition.notify_all()

    def read(self, timeout=1.0):
        with self._condition:
            if not self._condition.wait_for(lambda: self._frame is not None, timeout=timeout):
                return None
            return self._frame

    def intrinsics(self):
        """仿真渲染为标准针孔：由模块视场与仿真分辨率解析内参。"""
        module = self._module or {}
        stream = module.get("simulation_stream", module.get("stream", {}))
        width = int(stream.get("width", 640))
        height = int(stream.get("height", 480))
        vfov = math.radians(float(module.get("vertical_fov_deg", 58.29)))
        hfov = 2.0 * math.atan(math.tan(vfov / 2) * width / height)
        return {"fx": (width / 2) / math.tan(hfov / 2), "fy": (height / 2) / math.tan(vfov / 2),
                "ppx": width / 2, "ppy": height / 2}

    def info(self):
        module = self._module or {}
        stream = module.get("simulation_stream", module.get("stream", {}))
        return {"source": "simulation", "width": stream.get("width", 640),
                "height": stream.get("height", 480), "fps": stream.get("fps", 30)}

    def close(self):
        pass


class MockSource:
    """无设备合成源：斜坡米制深度 + 彩条 RGB，软件节拍；本机全流程验证用。"""

    def __init__(self, camera_json=None):
        module = load_camera_module(camera_json) if camera_json else {}
        stream = module.get("stream", {})
        self.width = int(stream.get("width", 640))
        self.height = int(stream.get("height", 480))
        self.fps = float(stream.get("fps", 30))
        self.module = module
        self.sequence = 0
        self._deadline = time.monotonic()

    def read(self):
        self._deadline = max(self._deadline + 1.0 / self.fps, time.monotonic())
        time.sleep(max(0.0, self._deadline - time.monotonic()))
        self.sequence += 1
        row = np.linspace(0.3, 2.5, self.width, dtype=np.float32)
        raw = np.tile(row, (self.height, 1))
        raw += float(self.sequence % 30) * 0.001  # 轻微变化，便于确认视频在动
        rgb = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        for band in range(6):
            rgb[:, band * self.width // 6:(band + 1) * self.width // 6, band % 3] = 40 + band * 40
        return _frame(raw, rgb, self.sequence, time.monotonic(), self.fps)

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
        stream = module.get("stream", {})
        self.width = int(stream.get("width", 480))
        self.height = int(stream.get("height", 270))
        self.fps = int(stream.get("fps", 60))
        self.module = module
        self.sequence = 0
        self.serial = ""
        self._pipeline = None
        self._filters = None
        self._scale = 1.0
        self._intrinsics = None
        self._last_number = None
        self._color_enabled = bool(module.get("rgb"))
        self._color_stream = module.get("rgb", {}).get("stream", {})

    def open(self):
        import pyrealsense2 as rs
        config = rs.config()
        if self.serial:
            config.enable_device(self.serial)
        config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        if self._color_enabled:
            color = self._color_stream
            config.enable_stream(rs.stream.color, int(color.get("width", 640)),
                                 int(color.get("height", 480)), rs.format.bgr8,
                                 int(color.get("fps", self.fps)))
        self._pipeline = rs.pipeline()
        profile = self._pipeline.start(config)
        depth_profile = profile.get_stream(rs.stream.depth).as_video_stream_profile()
        self._scale = profile.get_device().first_depth_sensor().get_depth_scale()
        intrinsics = depth_profile.get_intrinsics()
        self._intrinsics = {"fx": intrinsics.fx, "fy": intrinsics.fy,
                            "ppx": intrinsics.ppx, "ppy": intrinsics.ppy}
        # 滤波链只构建一次，时间滤波需要保留历史。
        self._filters = (rs.disparity_transform(True), rs.spatial_filter(), rs.temporal_filter(),
                         rs.disparity_transform(False), rs.hole_filling_filter(1))

    def read(self):
        sample = self._pipeline.wait_for_frames(1000)
        depth_frame = sample.get_depth_frame()
        if not depth_frame:
            return None
        number = depth_frame.get_frame_number()
        if self._last_number is not None and number == self._last_number:
            return None  # 同帧去重
        self._last_number = number
        for stage in self._filters:
            depth_frame = stage.process(depth_frame)
        received = time.monotonic()
        sensor_time = float(depth_frame.get_timestamp()) * 0.001
        raw = np.asanyarray(depth_frame.get_data()).astype(np.float32) * self._scale
        rgb = None
        if self._color_enabled:
            color_frame = sample.get_color_frame()
            if color_frame:
                rgb = np.asanyarray(color_frame.get_data())[:, :, ::-1].copy()  # SDK BGR → RGB
        self.sequence += 1
        return _frame(raw, rgb, self.sequence, sensor_time, float(self.fps))

    def info(self):
        info = {"source": "realsense", "width": self.width, "height": self.height,
                "fps": self.fps, "depth_scale": self._scale,
                "intrinsics": self._intrinsics,
                "filters": "disparity+spatial+temporal+hole_filling"}
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
