"""Nav: pure follower guidance and future GNSS/Nav2 adapter. MOCK guidance only.

Input: astronaut 3D x/y in base_link, meters. Output: suggested Twist2.
Never send a suggested twist directly to a motor controller.
"""
from dataclasses import dataclass
from math import atan2, hypot, isfinite


@dataclass(frozen=True)
class Twist2:
    linear: float = 0.0
    angular: float = 0.0


def follow(x_m: float, y_m: float, *, desired_m=2.0, max_v=0.20, max_w=0.30,
           tolerance_m=0.15) -> Twist2:
    if not all(isfinite(v) for v in (x_m, y_m, desired_m, max_v, max_w, tolerance_m)):
        return Twist2()
    if desired_m <= 0 or max_v <= 0 or max_w <= 0 or x_m <= 0 or tolerance_m < 0:
        return Twist2()
    d = hypot(x_m, y_m)
    bearing = atan2(y_m, x_m)
    w = max(-max_w, min(max_w, 0.9 * bearing))
    v = max(0.0, min(max_v, 0.3 * (d - desired_m)))
    if d <= desired_m + tolerance_m:
        v = 0.0  # Inclusive distance deadband; never reverse when too close.
    if abs(bearing) > 0.5:  # rotate first for sharp direction changes
        v = 0.0
    return Twist2(v, w)


def gnss_goal_adapter(latitude: float, longitude: float):
    raise NotImplementedError('Connect existing GNSS/EKF and Nav2 goal code later')
