from dataclasses import dataclass
import json
import math
from pathlib import Path


PROFILES_DIR = Path(__file__).resolve().parents[1] / "profiles"
DEFAULT_CAMERA_CONFIG = PROFILES_DIR / "configs/cameras/d435_rgbd.json"


def profile_camera(profile):
    path = Path(profile)
    if not path.is_absolute() and not path.exists():
        path = PROFILES_DIR / f"{profile}.json"
    profile_data = json.loads(path.read_text(encoding="utf-8"))
    camera = profile_data.get("camera", {}).get("config")
    if not camera:
        raise ValueError(f"Task profile has no camera.config: {path}")
    return path.parent / camera


@dataclass(frozen=True)
class DepthStream:
    width: int
    height: int
    fps: float

    @classmethod
    def from_module(cls, module, simulation=False):
        stream = dict(module.get("stream", {}))
        if simulation:
            stream.update(module.get("simulation_stream", {}))
        width, height, fps = stream.get("width", 640), stream.get("height", 480), stream.get("fps", 30)
        if type(width) is not int or type(height) is not int or min(width, height) <= 0:
            raise ValueError("Depth width and height must be positive integers")
        if type(fps) not in (int, float) or not math.isfinite(fps) or not 0 < fps <= 1e9:
            raise ValueError("Depth FPS must be positive and finite")
        return cls(width, height, float(fps))


class DepthCadence:
    """Keep the configured cadence through small delays; never catch up in bursts."""

    def __init__(self, stream):
        self.period_ns = round(1e9 / stream.fps)
        self.next_ns = 0

    def advance(self, now):
        following = self.next_ns + self.period_ns
        self.next_ns = following if now < following else now + self.period_ns
