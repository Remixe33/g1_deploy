"""Geometry checks for the calibration frame contract."""

import numpy as np


def validate_rigid(transform: np.ndarray, name: str = "transform") -> None:
    """Raise ``ValueError`` unless ``transform`` is a proper rigid transform."""
    transform = np.asarray(transform, dtype=float)
    if transform.shape != (4, 4):
        raise ValueError(f"{name}: expected 4x4, got {transform.shape}")
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9):
        raise ValueError(f"{name}: bottom row must be [0, 0, 0, 1]")
    rotation = transform[:3, :3]
    if not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-6):
        raise ValueError(f"{name}: rotation part is not orthonormal")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-6):
        raise ValueError(f"{name}: rotation determinant must be +1")
