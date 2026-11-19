"""Display the raw and optional model outputs of the active depth pipeline."""
from contextlib import contextmanager
import sys
import time
from types import ModuleType


class DepthOverlay:
    def __init__(self, viewer, fov_debug=False, native=False):
        import cv2
        import mujoco
        import numpy as np

        self.viewer = viewer
        self.cv = cv2
        self.mujoco = mujoco
        self.np = np
        self.next_update = 0.0
        self.fov_debug = fov_debug
        self.native = native

    def draw_fov_edges(self):
        scene = getattr(self.viewer, "user_scn", None)
        if scene is None or len(scene.geoms) < 4:
            return
        np = self.np
        if self.native:
            from native_depth_camera import active_camera
            camera = active_camera
            if camera is None:
                return
            origin, dirs = camera.frustum_rays()
        else:
            from parkour_observation import active_adapter
            adapter = active_adapter
            if adapter is None:
                return
            rotation = adapter.data.xmat[adapter.torso].reshape(3, 3)
            origin = adapter.data.xpos[adapter.torso] + rotation @ adapter.offset
            corners = adapter.rays[[0, 63, 2240, 2303]]
            dirs = np.ascontiguousarray(corners @ rotation.T)
        with self.viewer.lock():
            for slot, direction in enumerate(dirs):
                direction = direction / np.linalg.norm(direction)
                half = 1.5
                helper = np.array([0.0, 0.0, 1.0])
                if abs(float(np.dot(helper, direction))) > 0.9:
                    helper = np.array([1.0, 0.0, 0.0])
                x_axis = np.cross(helper, direction)
                x_axis /= np.linalg.norm(x_axis)
                y_axis = np.cross(direction, x_axis)
                self.mujoco.mjv_initGeom(
                    scene.geoms[slot], self.mujoco.mjtGeom.mjGEOM_CAPSULE,
                    np.array([0.008, half, 0.0]), origin + direction * half,
                    np.column_stack([x_axis, y_axis, direction]).ravel(),
                    np.array([0.2, 1.0, 0.4, 0.35]))
            scene.ngeom = 4

    def draw_raw(self, frame):
        cv, np = self.cv, self.np
        viewport = self.viewer.viewport
        if viewport is None:
            return
        scale = min(480.0 / frame.width, 360.0 / frame.height,
                    viewport.width / (3.0 * frame.width), viewport.height / (2.0 * frame.height))
        width = int(frame.width * scale)
        height = int(frame.height * scale)
        if width < 32 or height < 32:
            return
        level = np.rint((1 - np.clip(np.nan_to_num(frame.raw / 2.5, posinf=1.), 0, 1)) * 255).astype(np.uint8)
        image = cv.applyColorMap(level, cv.COLORMAP_TURBO)
        image = cv.resize(image, (width, height), interpolation=cv.INTER_NEAREST)
        rate = f"@{frame.fps:g}" if frame.fps > 0 else ""
        cv.putText(image, f"Raw {frame.width}x{frame.height}{rate}", (8, 24),
                   cv.FONT_HERSHEY_SIMPLEX, min(.7, max(.35, width / 600)), (240, 240, 240), 1, cv.LINE_AA)
        rectangle = self.mujoco.MjrRect(viewport.left + viewport.width - width - 8,
                                        viewport.bottom + viewport.height - height - 8, width, height)
        self.viewer.set_images((rectangle, cv.cvtColor(image, cv.COLOR_BGR2RGB)))

    def draw_raw_and_rgb(self, frame):
        cv, np = self.cv, self.np
        viewport = self.viewer.viewport
        if viewport is None:
            return
        total_width = min(972, viewport.width * 3 // 4)
        tile_width = (total_width - 12) // 2
        scale = min(tile_width / frame.width, 360.0 / frame.height,
                    viewport.height / (2.0 * frame.height))
        tile_width = int(frame.width * scale)
        tile_height = int(frame.height * scale)
        if tile_width < 32 or tile_height < 32:
            return
        rgb = cv.resize(frame.rgb, (tile_width, tile_height), interpolation=cv.INTER_LINEAR)
        level = np.rint((1 - np.clip(np.nan_to_num(frame.raw / 2.5, posinf=1.), 0, 1)) * 255).astype(np.uint8)
        depth = cv.applyColorMap(level, cv.COLORMAP_TURBO)
        depth = cv.cvtColor(cv.resize(depth, (tile_width, tile_height), interpolation=cv.INTER_NEAREST),
                            cv.COLOR_BGR2RGB)
        canvas = np.full((tile_height + 30, 2 * tile_width + 12, 3), 24, dtype=np.uint8)
        canvas[26:26 + tile_height, 4:4 + tile_width] = rgb
        canvas[26:26 + tile_height, 8 + tile_width:8 + 2 * tile_width] = depth
        rate = f"@{frame.fps:g}" if frame.fps > 0 else ""
        for label, x in ((f"RGB {frame.rgb.shape[1]}x{frame.rgb.shape[0]}{rate}", 4),
                         (f"Depth {frame.width}x{frame.height}{rate}", 8 + tile_width)):
            font_scale = min(.65, max(.3, tile_width / 600))
            cv.putText(canvas, label, (x, 18), cv.FONT_HERSHEY_SIMPLEX, font_scale,
                       (240, 240, 240), 1, cv.LINE_AA)
        rectangle = self.mujoco.MjrRect(
            viewport.left + viewport.width - canvas.shape[1] - 8,
            viewport.bottom + viewport.height - canvas.shape[0] - 8, canvas.shape[1], canvas.shape[0])
        self.viewer.set_images((rectangle, canvas))

    def draw_raw_and_model(self, frame):
        cv, np = self.cv, self.np
        viewport = self.viewer.viewport
        if viewport is None:
            return
        width = min(960, viewport.width * 3 // 4,
                    2 * max(0, viewport.height // 2 - 30) * 64 // 36 + 12)
        tile_width = (width - 12) // 2
        tile_height = tile_width * 36 // 64
        height = tile_height + 30
        if width < 32 or height > viewport.height - 16:
            self.viewer.clear_images()
            return

        def color(values, no_hit=None):
            level = np.rint((1 - np.clip(values, 0, 1)) * 255).astype(np.uint8)
            pixels = cv.applyColorMap(level, cv.COLORMAP_TURBO)
            if no_hit is not None:
                pixels[no_hit] = 0
            return cv.resize(pixels, (tile_width, tile_height), interpolation=cv.INTER_NEAREST)

        raw_image = color(np.nan_to_num(frame.raw / 2.5, posinf=1.), ~np.isfinite(frame.raw))
        cv.rectangle(raw_image, (tile_width // 4, tile_height // 2),
                     (tile_width * 3 // 4 - 1, tile_height - 1), (255, 255, 255), 1)
        canvas = np.full((height, width, 3), 24, dtype=np.uint8)
        canvas[26:26 + tile_height, 4:4 + tile_width] = raw_image
        canvas[26:26 + tile_height, 8 + tile_width:8 + 2 * tile_width] = color(frame.model)
        for label, x in ((f"Raw {frame.height}x{frame.width}", 4),
                         (f"Model {frame.model.shape[0]}x{frame.model.shape[1]}", 8 + tile_width)):
            font_scale = min(.65, max(.3, tile_width / 600))
            cv.putText(canvas, label, (x, 18), cv.FONT_HERSHEY_SIMPLEX, font_scale,
                       (240, 240, 240), 1, cv.LINE_AA)
        rectangle = self.mujoco.MjrRect(
            viewport.left + viewport.width - width - 8,
            viewport.bottom + viewport.height - height - 8, width, height)
        self.viewer.set_images((rectangle, cv.cvtColor(canvas, cv.COLOR_BGR2RGB)))

    def update(self, frame, sim_time):
        if self.fov_debug:
            self.draw_fov_edges()
        now = time.monotonic()
        if frame is None or now < self.next_update or not self.viewer.is_running():
            return
        self.next_update = now + 1 / 15
        if frame.model is not None:
            self.draw_raw_and_model(frame)
        elif frame.rgb is not None:
            self.draw_raw_and_rgb(frame)
        else:
            self.draw_raw(frame)

    def close(self):
        if self.viewer.is_running():
            self.viewer.clear_images()


@contextmanager
def install_depth_overlay(fov_debug=False, native=False):
    import mujoco.viewer

    if not hasattr(mujoco.viewer.Handle, "set_images"):
        raise RuntimeError("Depth overlay requires a MuJoCo viewer with Handle.set_images (installed version is too old)")
    original_launch = mujoco.viewer.launch_passive
    previous_preview = sys.modules.get("depth_preview")
    viewers = []

    def launch_with_overlay(*args, **kwargs):
        viewer = original_launch(*args, **kwargs)
        viewers.append(viewer)
        return viewer

    def create_preview():
        if not viewers:
            raise RuntimeError("The MuJoCo viewer must be opened before the depth preview")
        return DepthOverlay(viewers[-1], fov_debug=fov_debug, native=native)

    preview = ModuleType("depth_preview")
    preview.DepthPreview = create_preview
    mujoco.viewer.launch_passive = launch_with_overlay
    sys.modules["depth_preview"] = preview
    try:
        yield
    finally:
        mujoco.viewer.launch_passive = original_launch
        if previous_preview is None:
            sys.modules.pop("depth_preview", None)
        else:
            sys.modules["depth_preview"] = previous_preview
