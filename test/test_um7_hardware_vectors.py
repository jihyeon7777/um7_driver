"""
Regression tests using real packets captured from a connected UM7.

These byte strings were captured over serial from a static UM7 and
cross-checked against physical invariants: a stationary accelerometer reads
~1 g total (PROC accel is in g, NOT m/s^2, despite the datasheet text) and
the calibrated magnetometer is unit-norm (~1.0). See CLAUDE.md.
"""

import math

import pytest

from um7_driver import um7_registers as reg
from um7_driver.um7_parser import Um7Parser

# HEALTH read response (addr 0x55). Bit 0 (GPS) is set: no GPS attached.
HEALTH_PACKET = bytes.fromhex('736e708055000000010227')

# Processed-data batch (addr 0x61, len 12): gyro/accel/mag PROC + times.
PROC_PACKET = bytes.fromhex(
    '736e70f0613ec6a29cbfa18352bf1e734b479018b63f165bdebea248943f46e6'
    'e0479018b6bf493aea3f203f8abceff8e8479018b31a94')

# Euler batch (addr 0x70, len 5) captured live in a tilted pose.
EULER_LIVE_PACKET = bytes.fromhex(
    '736e70d47038310cab2ea200000007001600020000479018b60649')


def test_health_packet():
    """The HEALTH read response decodes and shows the GPS bit set."""
    packet = Um7Parser().feed(HEALTH_PACKET)[0]
    assert packet.address == reg.DREG_HEALTH
    assert packet.values['health'] == 1
    assert packet.values['health'] & 0x1  # bit 0 = GPS timeout (no GPS)


def test_proc_batch_decodes_all_fields():
    """The PROC batch yields gyro, accel and mag for all three axes."""
    packet = Um7Parser().feed(PROC_PACKET)[0]
    assert packet.address == reg.DREG_GYRO_PROC_X
    assert packet.batch_length == 12
    for name in ('gyro_proc_x', 'gyro_proc_y', 'gyro_proc_z',
                 'accel_proc_x', 'accel_proc_y', 'accel_proc_z',
                 'mag_proc_x', 'mag_proc_y', 'mag_proc_z'):
        assert name in packet.values


def test_proc_accel_is_one_g_static():
    """A static accel reading is ~1 g in magnitude (PROC accel is in g)."""
    v = Um7Parser().feed(PROC_PACKET)[0].values
    magnitude = math.sqrt(v['accel_proc_x'] ** 2 + v['accel_proc_y'] ** 2
                          + v['accel_proc_z'] ** 2)
    assert magnitude == pytest.approx(1.0, abs=0.05)


def test_proc_mag_is_unit_norm():
    """The calibrated magnetometer vector has ~unit norm (dimensionless)."""
    v = Um7Parser().feed(PROC_PACKET)[0].values
    norm = math.sqrt(v['mag_proc_x'] ** 2 + v['mag_proc_y'] ** 2
                     + v['mag_proc_z'] ** 2)
    assert norm == pytest.approx(1.0, abs=0.05)


def test_live_euler_in_range():
    """The live Euler packet decodes to in-range angles."""
    v = Um7Parser().feed(EULER_LIVE_PACKET)[0].values
    assert -180.0 <= v['roll'] <= 180.0
    assert -90.0 <= v['pitch'] <= 90.0
    assert -180.0 <= v['yaw'] <= 180.0
