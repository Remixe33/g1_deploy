from dataclasses import dataclass


@dataclass(frozen=True)
class DepthFrame:
    raw: object
    model: object = None
    sequence: int = 0
    time: float = 0.0
    fps: float = 0.0
    rgb: object = None

    @property
    def height(self):
        return self.raw.shape[0]

    @property
    def width(self):
        return self.raw.shape[1]
