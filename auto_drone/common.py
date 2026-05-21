from __future__ import annotations

from math import pi


UNKNOWN = -1
FREE = 0
OBSTACLE = 100


def angle_delta(a: float, b: float) -> float:
    return (a - b + pi) % (2.0 * pi) - pi


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def blend(base: tuple[int, int, int], overlay: tuple[int, int, int], alpha: float) -> tuple[int, int, int]:
    alpha = clamp(alpha, 0.0, 1.0)
    return tuple(int(base[i] * (1.0 - alpha) + overlay[i] * alpha) for i in range(3))
