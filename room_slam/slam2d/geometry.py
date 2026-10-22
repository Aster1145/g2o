"""Small 2D rigid-transform helpers.

A pose is a 3x3 homogeneous matrix in the plane::

    T = [ cos(t) -sin(t)  x ]
        [ sin(t)  cos(t)  y ]
        [   0       0     1 ]

Everything else in the project speaks this language, so converting to g2o's
SE2 (x, y, theta) happens in one place only.
"""

from __future__ import annotations

import math

import numpy as np


def se2(x: float, y: float, theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, x],
                     [s,  c, y],
                     [0.0, 0.0, 1.0]], dtype=float)


def translation(T: np.ndarray) -> np.ndarray:
    return T[:2, 2]


def xytheta(T: np.ndarray) -> tuple[float, float, float]:
    return float(T[0, 2]), float(T[1, 2]), math.atan2(T[1, 0], T[0, 0])


def heading(T: np.ndarray) -> float:
    return math.atan2(T[1, 0], T[0, 0])


def inverse(T: np.ndarray) -> np.ndarray:
    R = T[:2, :2]
    out = np.eye(3)
    out[:2, :2] = R.T
    out[:2, 2] = -R.T @ T[:2, 2]
    return out


def relative(T_from: np.ndarray, T_to: np.ndarray) -> np.ndarray:
    """Pose of ``T_to`` expressed in the frame of ``T_from``."""
    return inverse(T_from) @ T_to


def transform_points(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """Apply a pose to an (N, 2) array of points."""
    if len(pts) == 0:
        return pts
    return pts @ T[:2, :2].T + T[:2, 2]


def normalize_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def angle_diff(a: float, b: float) -> float:
    return normalize_angle(a - b)
