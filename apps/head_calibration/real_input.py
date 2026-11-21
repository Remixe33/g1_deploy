"""只读接收 G1 上肢状态和指定 RealSense 的原生 RGB/深度。"""

import threading
import time
from collections import deque
from contextlib import ExitStack

import cv2
import numpy as np

# 与 common/include/g1_joint_mapping.h 中的 G1-29 DDS 顺序一致。
ARM_JOINT_NAMES = tuple(
    f"{side}_{joint}_joint"
    for side in ("left", "right")
    for joint in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
                  "wrist_roll", "wrist_pitch", "wrist_yaw")
)


def _stream_request(config, key, defaults):
    request = config.get(key, {})
    if not isinstance(request, dict) or request.keys() - {"width", "height", "fps"}:
        raise ValueError(f"{key} 流配置只接受 width、height、fps")
    values = tuple(request.get(name, value) for name, value in zip(("width", "height", "fps"), defaults))
    if any(type(value) is not int or value <= 0 for value in values):
        raise ValueError(f"{key} 流的尺寸和帧率必须为正整数")
    return values


def _stream_profile(profile, intrinsics, format_name):
    fx, fy, cx, cy = (float(getattr(intrinsics, key)) for key in ("fx", "fy", "ppx", "ppy"))
    coeffs = [float(value) for value in intrinsics.coeffs]
    if min(fx, fy) <= 0 or not np.isfinite([fx, fy, cx, cy, *coeffs]).all():
        raise ValueError("RealSense 返回无效内参")
    matrix = [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]]
    return matrix, {
        "width": int(intrinsics.width), "height": int(intrinsics.height), "fps": int(profile.fps()),
        "format": format_name, "distortion_model": str(intrinsics.model), "coeffs": coeffs,
    }


def _rectification_map(rs, intrinsics, matrix):
    if intrinsics.model == rs.distortion.none:
        return None
    # 输出像素先沿理想针孔成像反投影，再用实际 SDK 畸变模型投影回原图。
    # 图像尺寸、内参和光轴都保持原 RGB profile，映射只在启动时生成一次。
    fx, fy, cx, cy = matrix[0][0], matrix[1][1], matrix[0][2], matrix[1][2]
    rays_x = (np.arange(intrinsics.width) - cx) / fx
    rays_y = (np.arange(intrinsics.height) - cy) / fy
    project = rs.rs2_project_point_to_pixel
    source_pixels = np.asarray([
        project(intrinsics, [x, y, 1.0]) for y in rays_y.tolist() for x in rays_x.tolist()
    ], dtype=np.float32).reshape(intrinsics.height, intrinsics.width, 2)
    if not np.isfinite(source_pixels).all():
        raise ValueError("RealSense RGB 去畸变映射包含无效坐标")
    return np.ascontiguousarray(source_pixels)


class RealInput:
    """进入上下文后开始采集，read() 不阻塞，无新 RGB 时返回 None。"""

    def __init__(self, config):
        self.interface = config.get("interface")
        self.serial = config.get("serial")
        if (not isinstance(self.interface, str) or not self.interface or self.interface == "lo"
                or self.interface.strip() != self.interface
                or not isinstance(self.serial, str) or not self.serial
                or self.serial.strip() != self.serial):
            raise ValueError("需填写实机 DDS 网卡和 RealSense 序列号")
        self.rgb_request = _stream_request(config, "rgb", (640, 480, 30))
        self.depth_request = _stream_request(config, "depth", (480, 270, 60))
        defaults = {"max_pair_s": 0.02, "window_s": 0.5, "max_span_rad": 0.002,
                    "max_speed_rad_s": 0.02, "max_frame_age_s": 0.15}
        pairing = config.get("pairing", {})
        if (not isinstance(pairing, dict) or pairing.keys() - defaults.keys()
                or any(type(value) not in (int, float) or not np.isfinite(value) or value <= 0
                       for value in pairing.values())):
            raise ValueError("实机静稳和时间配对参数须为有限正值")
        self.pairing = {**defaults, **pairing}
        if self.pairing["window_s"] <= self.pairing["max_pair_s"]:
            raise ValueError("静稳时间窗须大于关节配对容差")
        self.profile = None
        self._resources = None
        self._lock = threading.Lock()
        self._samples = deque(maxlen=4096)
        self._state_error = self._camera_error = None
        self._state_sequence = 0
        self._frame = None
        self._last_rgb = None

    def __enter__(self):
        import pyrealsense2 as rs
        from unitree_sdk2py.core.channel import (
            ChannelFactoryInitialize,
            ChannelSubscriber,
        )
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

        if self._resources is not None:
            raise RuntimeError("实机只读采集已启动")
        with self._lock:
            self._samples.clear()
            self._state_error = self._camera_error = self._frame = self._last_rgb = None
            self._state_sequence = 0
        config = rs.config()
        config.enable_device(self.serial)
        w, h, fps = self.rgb_request
        config.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
        w, h, fps = self.depth_request
        config.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
        pipeline = rs.pipeline()
        with ExitStack() as cleanup:
            active = pipeline.start(config, self._on_frameset)
            cleanup.callback(pipeline.stop)
            device = active.get_device()
            if device.get_info(rs.camera_info.serial_number) != self.serial:
                raise RuntimeError("RealSense 实际序列号与配置不符")
            rgb = active.get_stream(rs.stream.color).as_video_stream_profile()
            depth = active.get_stream(rs.stream.depth).as_video_stream_profile()
            rgb_intrinsics = rgb.get_intrinsics()
            K_rgb, rgb_profile = _stream_profile(rgb, rgb_intrinsics, "BGR8")
            K_depth, depth_profile = _stream_profile(depth, depth.get_intrinsics(), "Z16")
            if ((rgb_profile["width"], rgb_profile["height"], rgb_profile["fps"]) != self.rgb_request
                    or (depth_profile["width"], depth_profile["height"], depth_profile["fps"]) != self.depth_request
                    or rgb.format() != rs.format.bgr8 or depth.format() != rs.format.z16):
                raise RuntimeError("RealSense 实际原生流与配置不符")
            depth_scale = float(device.first_depth_sensor().get_depth_scale())
            extrinsics = depth.get_extrinsics_to(rgb)
            T_rgb_depth = np.eye(4)
            T_rgb_depth[:3, :3] = np.asarray(extrinsics.rotation, dtype=float).reshape(3, 3, order="F")
            T_rgb_depth[:3, 3] = extrinsics.translation
            rotation = T_rgb_depth[:3, :3]
            if (not np.isfinite(depth_scale) or depth_scale <= 0
                    or not np.isfinite(T_rgb_depth).all()
                    or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5, rtol=0.0)
                    or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-5, rtol=0.0)):
                raise ValueError("RealSense 返回无效深度单位或光学外参")
            self._rgb_map = _rectification_map(rs, rgb_intrinsics, K_rgb)
            self._K = np.asarray(K_rgb)
            self.profile = {
                "serial": self.serial, "K_rgb_raw": K_rgb, "K_rgb": K_rgb,
                "rgb_distortion": {"model": rgb_profile["distortion_model"], "coeffs": rgb_profile["coeffs"]},
                "rgb": {**rgb_profile, "rectification": "sdk_project_point_to_pixel", "rectified_distortion": "none"},
                "K_depth": K_depth, "depth": depth_profile,
                "depth_scale_m": depth_scale, "T_rgb_depth": T_rgb_depth.tolist(),
            }
            ChannelFactoryInitialize(0, self.interface)
            subscriber = ChannelSubscriber("rt/lowstate", LowState_)
            cleanup.callback(subscriber.Close)
            subscriber.Init(self._on_state, 1)
            self._resources = cleanup.pop_all()
        return self

    def __exit__(self, *exc_info):
        resources, self._resources = self._resources, None
        return resources.__exit__(*exc_info)

    def _on_frameset(self, frame):
        arrival = time.monotonic()
        frameset = frame.as_frameset()
        if not frameset:
            return
        rgb = frameset.get_color_frame()
        if not rgb:
            return
        frame_number = int(rgb.get_frame_number())
        timestamp = float(rgb.get_timestamp())
        domain = str(rgb.get_frame_timestamp_domain())
        with self._lock:
            if not np.isfinite(timestamp) or frame_number < 0 or timestamp < 0:
                self._camera_error = "RealSense RGB 帧缺少有效设备时间戳"
                return
            if self._last_rgb is not None:
                last_number, last_timestamp, last_domain = self._last_rgb
                if domain != last_domain or frame_number < last_number or timestamp < last_timestamp:
                    self._camera_error = "RealSense RGB 帧时钟变化，需重新启动采集"
                    return
                if frame_number == last_number:
                    return
            self._last_rgb = frame_number, timestamp, domain
            self._frame = (arrival, frameset, rgb, frame_number, timestamp, domain)

    def _on_state(self, message):
        arrival = time.monotonic()
        q = dq = tick = None
        error = None
        if getattr(message, "mode_machine", None) != 5:
            error = "未收到 G1-29 实机关节状态（mode_machine=5）"
        else:
            try:
                motors = message.motor_state
                q = np.asarray([motors[index].q for index in range(15, 29)], dtype=float)
                dq = np.asarray([motors[index].dq for index in range(15, 29)], dtype=float)
                tick = int(message.tick)
                if q.shape != (14,) or dq.shape != (14,) or not (np.isfinite(q).all() and np.isfinite(dq).all()):
                    error = "上肢关节位置或速度无效"
            except (AttributeError, IndexError, TypeError, ValueError, OverflowError):
                error = "上肢关节状态不完整"
        with self._lock:
            if error is not None:
                self._samples.clear()
                self._state_error = error
                return
            if self._samples and self._samples[-1][0] >= arrival:
                return
            self._samples.append((arrival, self._state_sequence, tick, q, dq))
            self._state_sequence += 1
            self._state_error = None

    def _pair(self, arrival, samples, state_error, capture):
        cfg = self.pairing
        if time.monotonic() - arrival > cfg["max_frame_age_s"]:
            return None, "RGB 帧已过期"
        if not samples:
            return None, state_error or "等待实机上肢关节反馈"
        times = np.array([row[0] for row in samples])
        # 采用同一主机时钟，选 RGB 到达时已收到的最近关节状态。
        end = int(np.searchsorted(times, arrival, side="right")) - 1
        if end < 0:
            return None, "RGB 到达前没有可配对的关节状态"
        start = int(np.searchsorted(times, arrival - cfg["window_s"], side="right")) - 1
        age = float(arrival - times[end])
        capture.update(pair_delta_s=age, state_host_arrival_s=float(times[end]),
                       state_sequence=samples[end][1], state_device_tick=samples[end][2])
        if age > cfg["max_pair_s"]:
            return None, "关节反馈与 RGB 间隔过大"
        if start < 0 or end <= start:
            return None, "关节反馈尚未覆盖静稳时间窗"
        max_gap = float(np.diff(times[start:end + 1]).max())
        if max_gap > cfg["max_pair_s"]:
            return None, "静稳时间窗内关节反馈不连续"
        rows = samples[start:end + 1]
        span = float(np.ptp([row[3] for row in rows], axis=0).max())
        speed = float(np.abs([row[4] for row in rows]).max())
        capture.update(stability_window_s=cfg["window_s"], max_state_gap_s=max_gap,
                       max_joint_span_rad=span, max_joint_speed_rad_s=speed,
                       stability_window_sequence_range=[rows[0][1], rows[-1][1]])
        if span > cfg["max_span_rad"] or speed > cfg["max_speed_rad_s"]:
            return None, "上肢尚未稳定"
        return rows[-1], None

    def read(self):
        if self._resources is None:
            raise RuntimeError("实机只读采集尚未启动")
        with self._lock:
            pending, self._frame = self._frame, None
            samples, state_error, camera_error = tuple(self._samples), self._state_error, self._camera_error
        if camera_error is not None:
            raise RuntimeError(camera_error)
        if pending is None:
            return None
        arrival, frameset, rgb, frame_number, timestamp, domain = pending
        raw_bgr = np.asanyarray(rgb.get_data()).copy()
        rgb_profile = self.profile["rgb"]
        if raw_bgr.dtype != np.uint8 or raw_bgr.shape != (rgb_profile["height"], rgb_profile["width"], 3):
            raise ValueError("RealSense RGB 帧数据与原生流配置不符")
        bgr = (raw_bgr.copy() if self._rgb_map is None else
               cv2.remap(raw_bgr, self._rgb_map, None, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT))
        capture = {
            "frame_number": frame_number, "device_timestamp_ms": timestamp, "device_clock_domain": domain,
            "host_arrival_s": arrival, "host_clock_domain": "time.monotonic",
            "host_stamp_source": "RealSense SDK frameset callback after synchronization",
            "state_device_clock_domain": "SDK2 LowState.tick (unit unspecified)",
            "stability_basis": "causal_trailing_host_arrival_window",
        }
        row, reason = self._pair(arrival, samples, state_error, capture)
        capture.update(stable=row is not None, stability_reason=reason)
        joints = velocities = None
        if row is not None:
            joints = dict(zip(ARM_JOINT_NAMES, row[3].tolist()))
            velocities = dict(zip(ARM_JOINT_NAMES, row[4].tolist()))

        native = frameset.get_depth_frame()
        depth_z16 = None
        if native:
            depth_z16 = np.asanyarray(native.get_data()).copy()
            depth_profile = self.profile["depth"]
            if depth_z16.dtype != np.uint16 or depth_z16.shape != (depth_profile["height"], depth_profile["width"]):
                raise ValueError("RealSense 深度帧数据与原生流配置不符")
            capture["depth_capture"] = {
                "frame_number": int(native.get_frame_number()),
                "device_timestamp_ms": float(native.get_timestamp()),
                "device_clock_domain": str(native.get_frame_timestamp_domain()),
            }

        return {"raw_bgr": raw_bgr, "bgr": bgr, "intrinsics_K": self._K.copy(), "joints": joints,
                "joint_velocities": velocities, "capture": capture, "depth_z16": depth_z16}
