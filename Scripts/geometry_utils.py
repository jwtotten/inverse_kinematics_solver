import numpy as np
from typing import Tuple

_EPSILON = 1e-10


def segment_to_segment_distance(
    p0: np.ndarray,
    p1: np.ndarray,
    q0: np.ndarray,
    q1: np.ndarray,
) -> float:
    """Return the minimum Euclidean distance between two finite 3D line segments."""
    d1 = p1 - p0
    d2 = q1 - q0
    r = p0 - q0

    a = np.dot(d1, d1)
    e = np.dot(d2, d2)
    f = np.dot(d2, r)

    if a <= _EPSILON and e <= _EPSILON:
        return float(np.linalg.norm(r))

    if a <= _EPSILON:
        s = 0.0
        t = float(np.clip(f / e, 0.0, 1.0))
    else:
        c = np.dot(d1, r)
        if e <= _EPSILON:
            t = 0.0
            s = float(np.clip(-c / a, 0.0, 1.0))
        else:
            b = np.dot(d1, d2)
            denom = a * e - b * b
            if denom > _EPSILON:
                s = float(np.clip((b * f - c * e) / denom, 0.0, 1.0))
            else:
                s = 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t = 0.0
                s = float(np.clip(-c / a, 0.0, 1.0))
            elif t > 1.0:
                t = 1.0
                s = float(np.clip((b - c) / a, 0.0, 1.0))

    closest_on_ab = p0 + s * d1
    closest_on_cd = q0 + t * d2
    return float(np.linalg.norm(closest_on_ab - closest_on_cd))


def segments_are_crossing(
    p0: np.ndarray,
    p1: np.ndarray,
    q0: np.ndarray,
    q1: np.ndarray,
    threshold: float = 0.05,
) -> bool:
    """Return True if the two 3D segments are within threshold distance of each other."""
    return segment_to_segment_distance(p0, p1, q0, q1) < threshold


def build_segment_list(
    coordinates: list,
) -> list:
    """
    Convert [[hip],[knee],[foot]] coordinate list into [(hip_arr, knee_arr), (knee_arr, foot_arr)].
    Returns the femur segment and tibia segment as numpy array pairs.
    """
    hip = np.array(coordinates[0], dtype=float)
    knee = np.array(coordinates[1], dtype=float)
    foot = np.array(coordinates[2], dtype=float)
    return [(hip, knee), (knee, foot)]
