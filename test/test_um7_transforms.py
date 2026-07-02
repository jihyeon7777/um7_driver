"""
Regression tests for the node's frame / orientation transforms.

Anchored to hardware-verified poses (see CLAUDE.md): a flat UM7 in NED reads
accel (0, 0, -1 g), and a physical clockwise-from-top rotation increases NED
yaw but must DECREASE ROS ENU yaw, since ENU yaw is counter-clockwise
positive. Importing um7_node pulls rclpy, which is present under colcon test.
"""

import math

import pytest

from um7_driver.um7_node import (
    _Q_FRD_TO_FLU, _Q_NED_TO_ENU, DEG_TO_RAD, quaternion_from_euler,
    quaternion_multiply)


def _enu_yaw_deg(roll, pitch, yaw):
    """Return the ROS ENU yaw (deg) for NED Euler angles given in degrees."""
    q = quaternion_from_euler(roll * DEG_TO_RAD, pitch * DEG_TO_RAD,
                              yaw * DEG_TO_RAD)
    q = quaternion_multiply(_Q_NED_TO_ENU, quaternion_multiply(q, _Q_FRD_TO_FLU))
    x, y, z, w = q
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def test_quaternion_is_normalized():
    """quaternion_from_euler returns a unit quaternion."""
    q = quaternion_from_euler(0.3, -0.2, 1.1)
    assert math.isclose(sum(c * c for c in q), 1.0, abs_tol=1e-9)


def test_level_ned_yaw_maps_to_enu_90_offset():
    """A level NED heading of 0 maps to ENU yaw +90 (ENU_yaw = 90 - NED_yaw)."""
    assert _enu_yaw_deg(0.0, 0.0, 0.0) == pytest.approx(90.0, abs=1e-6)


def test_cw_rotation_decreases_enu_yaw():
    """A physical CW-from-top rotation decreases ROS ENU yaw (captured poses)."""
    flat = _enu_yaw_deg(3.2, 0.08, 11.5)       # NED yaw +11.5
    rotated = _enu_yaw_deg(1.71, 0.69, 115.11)  # NED yaw +115.1 after CW turn
    delta = rotated - flat
    while delta > 180:
        delta -= 360
    while delta < -180:
        delta += 360
    assert delta < 0
    assert delta == pytest.approx(-103.6, abs=2.0)
