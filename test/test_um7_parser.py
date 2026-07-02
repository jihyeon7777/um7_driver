"""Regression tests for the ROS-free UM7 parser using verified vectors."""

import pytest

from um7_driver import um7_registers as reg
from um7_driver.um7_parser import build_command_packet, compute_checksum, Um7Parser

# Verified Euler batch packet from the datasheet / CLAUDE.md.
# 'snp' D4 70 <20 data bytes> 0C5F  -- checksum verified = 0x0C5F.
EULER_PACKET = bytes.fromhex(
    '736e70d470fff50078ec4c00000007fff2fff900004343bcf40c5f')


def test_checksum_matches_datasheet():
    """The sum of all preceding bytes equals the trailing checksum."""
    assert compute_checksum(EULER_PACKET[:-2]) == 0x0C5F


def test_single_euler_packet_framing():
    """A whole packet decodes to one batch packet at address 112, length 5."""
    packets = Um7Parser().feed(EULER_PACKET)
    assert len(packets) == 1
    packet = packets[0]
    assert packet.address == reg.DREG_EULER_PHI_THETA  # 112
    assert packet.is_batch is True
    assert packet.batch_length == 5


def test_euler_values():
    """Decoded angles, rates and time match the documented values."""
    packet = Um7Parser().feed(EULER_PACKET)[0]
    values = packet.values
    assert values['roll'] == pytest.approx(-11 / reg.EULER_ANGLE_DIVISOR)
    assert values['pitch'] == pytest.approx(120 / reg.EULER_ANGLE_DIVISOR)
    assert values['yaw'] == pytest.approx(-5044 / reg.EULER_ANGLE_DIVISOR)
    assert values['roll'] == pytest.approx(-0.12, abs=0.01)
    assert values['pitch'] == pytest.approx(1.32, abs=0.01)
    assert values['yaw'] == pytest.approx(-55.42, abs=0.01)
    assert values['roll_rate'] == pytest.approx(0.4375)
    assert values['pitch_rate'] == pytest.approx(-0.875)
    assert values['yaw_rate'] == pytest.approx(-0.4375)
    assert values['euler_time'] == pytest.approx(195.73, abs=0.05)


def test_split_feed_reassembles():
    """A packet split across two feeds still decodes exactly once."""
    parser = Um7Parser()
    assert parser.feed(EULER_PACKET[:9]) == []
    packets = parser.feed(EULER_PACKET[9:])
    assert len(packets) == 1
    assert packets[0].address == reg.DREG_EULER_PHI_THETA


def test_leading_garbage_is_skipped():
    """Junk bytes before the 'snp' header do not prevent decoding."""
    packets = Um7Parser().feed(b'\x00\xff\x12garbage' + EULER_PACKET)
    assert len(packets) == 1
    assert packets[0].batch_length == 5


def test_two_packets_back_to_back():
    """Two concatenated packets both decode from a single feed."""
    packets = Um7Parser().feed(EULER_PACKET + EULER_PACKET)
    assert len(packets) == 2


def test_bad_checksum_is_rejected():
    """A corrupted data byte fails the checksum and yields no packet."""
    corrupt = bytearray(EULER_PACKET)
    corrupt[5] ^= 0xFF  # flip a data byte, leave the checksum untouched
    parser = Um7Parser()
    assert parser.feed(bytes(corrupt)) == []
    assert parser.checksum_errors >= 1


def test_build_command_packet_bytes():
    """Command packets are 'snp' + PT(0) + address + 16-bit checksum."""
    assert build_command_packet(reg.CMD_ZERO_GYROS).hex() == '736e7000ad01fe'
    assert build_command_packet(reg.CMD_SET_MAG_REFERENCE).hex() == '736e7000b00201'


def test_command_packet_has_no_data():
    """A command packet round-trips as an addressed, data-less packet."""
    packet = Um7Parser().feed(build_command_packet(reg.CMD_ZERO_GYROS))[0]
    assert packet.address == reg.CMD_ZERO_GYROS
    assert packet.values == {}
    assert packet.raw_registers == {}
    assert packet.is_batch is False
    assert packet.command_failed is False
