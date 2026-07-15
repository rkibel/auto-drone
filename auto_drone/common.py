from __future__ import annotations

from math import pi

def angle_delta(a: float, b: float) -> float:
    return (a - b + pi) % (2.0 * pi) - pi


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))
