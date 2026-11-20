"""Render independent depth and optional RGB views of the configured cameras."""
from dataclasses import replace
import json
import math
import time
from pathlib import Path

import mujoco
import numpy as np

from depth_frame import DepthFrame


active_camera = None


def horizontal_fov(vertical_fov_deg, width, height):
    return 2 * math.degrees(math.atan(math.tan(math.radians(vertical_fov_deg) / 2) * width / height))


def validate_supported_profile(module, kind, width, height, fps):
    supported = module.get("supported_streams", {}).get(kind)
    if not supported:
        return
    for profile in supported["profiles"]:
        if profile["width"] == width and profile["height"] == height and fps in profile["fps"]:
            return
    raise ValueError(f"Unsupported {kind} stream: {width}x{height}@{fps}")


def camera_id(model, name):
    identifier = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name)
    if identifier < 0:
        raise RuntimeError(f"Scene lacks the '{name}' camera element; regenerate the scene with a task profile")
    return identifier


class NativeDepthCamera:
    def __init__(self, model, data, camera_json_path):
        module = json.loads(Path(camera_json_path).read_text())
        self.fps = float(module["stream"]["fps"])
        self.vertical_fov_deg = float(module["vertical_fov_deg"])
        if not math.isfinite(self.fps) or self.fps <= 0 or not math.isfinite(self.vertical_fov_deg) or self.vertical_fov_deg <= 0:
            raise ValueError("Depth stream rate and FOV must be positive")
        simulation_stream = module.get("simulation_stream", {"width": 640, "height": 480})
        self.width = int(simulation_stream["width"])
        self.height = int(simulation_stream["height"])
        if self.width <= 0 or self.height <= 0:
            raise ValueError("Depth simulation dimensions must be positive")
        validate_supported_profile(module, "depth", self.width, self.height, self.fps)
        self.horizontal_fov_deg = horizontal_fov(self.vertical_fov_deg, self.width, self.height)
        self.model, self.data = model, data
        self.camera_id = camera_id(model, module.get("scene_camera", "depth_camera"))
        self.renderer = mujoco.Renderer(model, height=self.height, width=self.width)
        self.renderer.enable_depth_rendering()
        self.frame = None
        self.sequence = 0
        self._next = 0.0
        global active_camera
        active_camera = self

    def update(self):
        now = time.monotonic()
        if self.frame is None or now >= self._next:
            self._next = now + 1.0 / self.fps
            self.renderer.update_scene(self.data, camera=self.camera_id)
            depth = np.asarray(self.renderer.render()).copy()
            self.frame = DepthFrame(raw=depth, sequence=self.sequence, time=float(self.data.time), fps=self.fps)
            self.sequence += 1
        return self.frame

    def frustum_rays(self):
        origin = np.asarray(self.data.cam_xpos[self.camera_id])
        rotation = np.asarray(self.data.cam_xmat[self.camera_id]).reshape(3, 3)
        half_h = np.tan(np.deg2rad(self.horizontal_fov_deg) / 2)
        half_v = np.tan(np.deg2rad(self.vertical_fov_deg) / 2)
        corners = np.array([[+half_h, +half_v], [-half_h, +half_v],
                            [+half_h, -half_v], [-half_h, -half_v]])
        dirs = np.column_stack([corners[:, 0], corners[:, 1], -np.ones(len(corners))])
        dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
        return origin, np.ascontiguousarray(dirs @ rotation.T)

    def close(self):
        global active_camera
        self.renderer.close()
        if active_camera is self:
            active_camera = None


class NativeRgbCamera:
    def __init__(self, model, data, module):
        settings = module["rgb"]
        stream = settings["stream"]
        self.width = int(stream["width"])
        self.height = int(stream["height"])
        self.fps = float(stream["fps"])
        self.vertical_fov_deg = float(settings["vertical_fov_deg"])
        if not all(math.isfinite(value) and value > 0 for value in (self.width, self.height, self.fps, self.vertical_fov_deg)):
            raise ValueError("RGB stream dimensions, rate and FOV must be positive")
        validate_supported_profile(module, "rgb", self.width, self.height, self.fps)
        self.horizontal_fov_deg = horizontal_fov(self.vertical_fov_deg, self.width, self.height)
        self.data = data
        self.camera_id = camera_id(model, settings.get("scene_camera", "rgb_camera"))
        self.renderer = mujoco.Renderer(model, height=self.height, width=self.width)
        self.image = None
        self._next = 0.0

    def update(self):
        now = time.monotonic()
        if self.image is None or now >= self._next:
            self._next = now + 1.0 / self.fps
            self.renderer.update_scene(self.data, camera=self.camera_id)
            self.image = np.asarray(self.renderer.render()).copy()
        return self.image

    def close(self):
        self.renderer.close()


class MotorLimitAdapter:
    def __init__(self, model):
        self.actuators = np.arange(model.nu, dtype=int)
        self.limits = np.max(np.abs(model.actuator_ctrlrange), axis=1)


class NativeObservationPublisher:
    def __init__(self, model, data, port=None, config_path=None, camera_config=None,
                 frame_sink=None):
        if camera_config is None:
            raise ValueError("Native depth observations require a camera module")
        self.adapter = MotorLimitAdapter(model)
        self.camera = NativeDepthCamera(model, data, camera_config)
        module = json.loads(Path(camera_config).read_text())
        self.rgb = NativeRgbCamera(model, data, module) if module.get("rgb") else None
        self.frame_sink = frame_sink  # 主线程渲染后回调（如数据网页信箱），可为 None
        self._sink_sequence = None

    def update(self):
        pass

    def preview_frame(self):
        # MuJoCo's OpenGL renderer belongs to the viewer/main thread, not the physics thread.
        frame = self.camera.update()
        if frame is not None and self.rgb is not None:
            frame = replace(frame, rgb=self.rgb.update())
        if frame is not None and self.frame_sink is not None and frame.sequence != self._sink_sequence:
            self._sink_sequence = frame.sequence
            self.frame_sink(frame)
        return frame, 0.0 if frame is None else frame.time

    def close(self):
        if self.rgb is not None:
            self.rgb.close()
        self.camera.close()
