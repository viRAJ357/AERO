"""Pure geometry utilities for AERO object localization.

All functions are pure Python with no ROS dependency so they can be unit-tested
with pytest without a running ROS environment.
"""
import math
from typing import List, Optional, Tuple


def bearing_from_pixel(u: float, cx: float, fx: float) -> float:
    """Compute bearing (radians) of image column u.

    Positive angle is to the LEFT of the camera axis (right-handed convention).

    Args:
        u:  Column in image pixels.
        cx: Principal point x (pixels).
        fx: Focal length x (pixels).

    Returns:
        Bearing in radians.
    """
    return math.atan2(cx - u, fx)


def angular_half_width(
    u_left: float, u_right: float, cx: float, fx: float
) -> float:
    """Compute half the angular width of a bounding box.

    Args:
        u_left:  Left column of bounding box (pixels).
        u_right: Right column of bounding box (pixels).
        cx:      Principal point x (pixels).
        fx:      Focal length x (pixels).

    Returns:
        Half angular width in radians.
    """
    return abs(
        bearing_from_pixel(u_left, cx, fx) - bearing_from_pixel(u_right, cx, fx)
    ) / 2.0


def median_range(
    ranges: List[float],
    angle_min: float,
    angle_inc: float,
    center: float,
    half_width: float,
    rmin: float = 0.3,
    rmax: float = 4.0,
) -> Optional[float]:
    """Find the median LiDAR range within the angular window of a bounding box.

    Args:
        ranges:     Full list of LiDAR range values.
        angle_min:  Minimum angle of the scan (radians).
        angle_inc:  Angular increment per beam (radians).
        center:     Bearing to the centre of the bounding box (radians).
        half_width: Half the angular width of the bounding box (radians).
        rmin:       Minimum valid range (m).
        rmax:       Maximum valid range (m).

    Returns:
        Median range in metres, or None if fewer than 3 valid readings.
    """
    vals: List[float] = []
    for i, r in enumerate(ranges):
        a = angle_min + i * angle_inc
        # Wrap angular difference to [-pi, pi]
        diff = math.atan2(math.sin(a - center), math.cos(a - center))
        if abs(diff) <= half_width and math.isfinite(r) and rmin <= r <= rmax:
            vals.append(r)
    if len(vals) < 3:
        return None
    vals.sort()
    return vals[len(vals) // 2]


def range_spread(ranges: List[float], angle_min: float, angle_inc: float,
                center: float, half_width: float,
                rmin: float = 0.3, rmax: float = 4.0) -> Optional[float]:
    """Return the spread (max-min) of valid LiDAR ranges in the window.

    Returns None if fewer than 3 valid values.
    """
    vals: List[float] = []
    for i, r in enumerate(ranges):
        a = angle_min + i * angle_inc
        diff = math.atan2(math.sin(a - center), math.cos(a - center))
        if abs(diff) <= half_width and math.isfinite(r) and rmin <= r <= rmax:
            vals.append(r)
    if len(vals) < 3:
        return None
    return max(vals) - min(vals)


def pixel_to_map(
    r: float,
    bearing_cam: float,
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    cam_to_base_yaw: float = 0.0,
) -> Tuple[float, float]:
    """Project a range+bearing measurement to map frame.

    Args:
        r:               Range to object (m).
        bearing_cam:     Bearing from camera axis (radians, left-positive).
        robot_x:         Robot x in map frame (m).
        robot_y:         Robot y in map frame (m).
        robot_yaw:       Robot heading in map frame (radians).
        cam_to_base_yaw: Yaw offset camera→base_footprint (usually 0).

    Returns:
        (x, y) of the object in map frame.
    """
    # Bearing in robot/base frame (camera looks forward, left-handed → flip)
    bearing_base = -bearing_cam + cam_to_base_yaw
    # Bearing in map frame
    bearing_map = robot_yaw + bearing_base
    x = robot_x + r * math.cos(bearing_map)
    y = robot_y + r * math.sin(bearing_map)
    return x, y


def distance_2d(x1: float, y1: float, x2: float, y2: float) -> float:
    """Euclidean distance between two 2D points."""
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
