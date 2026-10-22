"""Unit tests for aero_perception.geometry_utils.

Tests all pure-math functions: bearing_from_pixel, angular_half_width,
median_range with wrap-around, and frame math.
"""
import math
import sys
import os
import pytest

# Add package directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'aero_perception')))
from aero_perception.geometry_utils import (
    bearing_from_pixel,
    angular_half_width,
    median_range,
    range_spread,
    pixel_to_map,
    distance_2d
)


def test_bearing_from_pixel():
    cx, fx = 320.0, 320.0
    # Optical center -> 0 bearing
    assert math.isclose(bearing_from_pixel(320.0, cx, fx), 0.0, abs_tol=1e-5)
    # Left of center (u < cx) -> positive bearing
    assert bearing_from_pixel(0.0, cx, fx) > 0.0
    # Right of center (u > cx) -> negative bearing
    assert bearing_from_pixel(640.0, cx, fx) < 0.0


def test_angular_half_width():
    cx, fx = 320.0, 320.0
    hw = angular_half_width(200.0, 440.0, cx, fx)
    assert hw > 0.0
    assert hw < math.pi / 2.0


def test_median_range():
    # 360 degree scan, 1 deg per beam
    angle_min = 0.0
    angle_inc = math.radians(1.0)
    center = 0.0
    half_width = math.radians(5.0)

    # Clean readings
    ranges = [2.0] * 360
    r = median_range(ranges, angle_min, angle_inc, center, half_width, rmin=0.3, rmax=4.0)
    assert r == 2.0

    # Noise tolerance: 3 valid readings with an outlier
    ranges[0] = 1.8
    ranges[1] = 2.0
    ranges[2] = 2.2
    ranges[3] = 0.1 # Below rmin -> discarded
    r = median_range(ranges, angle_min, angle_inc, center, half_width, rmin=0.3, rmax=4.0)
    assert r == 2.0


def test_range_spread():
    angle_min = 0.0
    angle_inc = math.radians(1.0)
    center = 0.0
    half_width = math.radians(2.0)
    ranges = [1.0, 1.2, 1.5] + [10.0] * 357
    spread = range_spread(ranges, angle_min, angle_inc, center, half_width)
    assert math.isclose(spread, 0.5, abs_tol=1e-5)


def test_pixel_to_map():
    # Robot at (0, 0) facing +X (yaw = 0)
    # Object straight ahead at 2m
    x, y = pixel_to_map(r=2.0, bearing_cam=0.0, robot_x=0.0, robot_y=0.0, robot_yaw=0.0)
    assert math.isclose(x, 2.0, abs_tol=1e-5)
    assert math.isclose(y, 0.0, abs_tol=1e-5)


def test_distance_2d():
    assert math.isclose(distance_2d(0, 0, 3, 4), 5.0, abs_tol=1e-5)
