from contextlib import contextmanager
from functools import partial
from pathlib import Path
import sys
from types import ModuleType


PROJECT_ROOT = Path(__file__).resolve().parents[2]


@contextmanager
def install_observation_bridge(config, enabled, preview, native, camera_config, head_back,
                               frame_sink=None):
    config.PARKOUR_OBSERVATIONS = enabled
    config.PARKOUR_OBSERVATION_PORT = None
    config.PARKOUR_DEPTH_PREVIEW = preview
    if not enabled:
        yield
        return

    previous = sys.modules.get("parkour_observation")
    if native:
        from native_depth_camera import NativeObservationPublisher

        bridge = ModuleType("parkour_observation")
        bridge.ParkourObservationPublisher = partial(
            NativeObservationPublisher, camera_config=camera_config, frame_sink=frame_sink)
        sys.modules["parkour_observation"] = bridge
    else:
        import parkour_observation as bridge

        original = bridge.ParkourObservationPublisher

        class Publisher(original):
            def __init__(self, model, data, port=None, config_path=None):
                super().__init__(model, data, config_path=PROJECT_ROOT /
                                 "apps/parkour/config/g1_parkour.json")
                if head_back:
                    from terrain import apply_head_back_camera
                    apply_head_back_camera(self.adapter, head_back)

            def preview_frame(self):
                frame, sim_time = super().preview_frame()
                if frame is not None and frame_sink is not None:
                    frame_sink(frame)  # 数据网页信箱（主/查看器线程调用）
                return frame, sim_time

        bridge.ParkourObservationPublisher = Publisher
    try:
        yield
    finally:
        if native:
            if previous is None:
                sys.modules.pop("parkour_observation", None)
            else:
                sys.modules["parkour_observation"] = previous
        else:
            bridge.ParkourObservationPublisher = original
