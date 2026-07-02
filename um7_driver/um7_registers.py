"""
UM7 register map: addresses, encodings, and scale factors.

This module is the single source of truth for the UM7 binary protocol as
documented in ``UM7_Datasheet_v1-8_30.07.2018.pdf`` (Binary Packet Structure
and Register Overview). No magic numbers describing the protocol belong
anywhere else in the driver.

All multi-byte quantities are big-endian (MSB first). A register is 4 bytes.
When a 32-bit register packs two 16-bit values, the high half (bits 31:16)
comes first and the low half (bits 15:0) second -- there is NO word swap.
"""

from dataclasses import dataclass
from enum import Enum

# --- Packet framing ---------------------------------------------------------

START_SEQUENCE = b'snp'  # 's', 'n', 'p'

# Packet Type (PT) byte bit masks (datasheet Table 3/4).
PT_HAS_DATA = 0x80           # bit 7
PT_IS_BATCH = 0x40           # bit 6
PT_BATCH_LENGTH_MASK = 0x3C  # bits 5:2
PT_BATCH_LENGTH_SHIFT = 2
PT_HIDDEN = 0x02             # bit 1
PT_COMMAND_FAILED = 0x01     # bit 0

REGISTER_SIZE = 4  # bytes per register


def batch_length(pt: int) -> int:
    """Return the batch length (register count) encoded in a PT byte."""
    return (pt & PT_BATCH_LENGTH_MASK) >> PT_BATCH_LENGTH_SHIFT


# --- Scale factors (datasheet) ---------------------------------------------

EULER_ANGLE_DIVISOR = 91.02222   # int16 -> degrees
EULER_RATE_DIVISOR = 16.0        # int16 -> degrees/second
QUAT_DIVISOR = 29789.09091       # int16 -> unitless quaternion component


# --- Field encodings --------------------------------------------------------

class Encoding(Enum):
    """How the four register bytes decode into a physical value."""

    FLOAT32 = 'float32'  # IEEE-754, already a physical value
    INT16 = 'int16'      # signed 16-bit half, divide by ``divisor``
    RAW16 = 'raw16'      # signed 16-bit half, raw counts (no physical scale)
    UINT32 = 'uint32'    # whole register kept as unsigned int (bitfields)


class Half(Enum):
    """Which part of the 32-bit register a 16-bit field occupies."""

    HIGH = 'high'  # bits 31:16, transmitted first
    LOW = 'low'    # bits 15:0, transmitted second


@dataclass(frozen=True)
class Field:
    """One named physical value decoded from a register."""

    name: str
    encoding: Encoding
    divisor: float = 1.0
    half: 'Half | None' = None  # required for INT16/RAW16; None for whole reg


@dataclass(frozen=True)
class Register:
    """A UM7 data register: its address and the fields it decodes into."""

    address: int
    name: str
    fields: 'tuple[Field, ...]'


# --- Register addresses (datasheet Register Overview) -----------------------

DREG_HEALTH = 0x55               # 85
DREG_TEMPERATURE = 0x5F          # 95
DREG_TEMPERATURE_TIME = 0x60     # 96
DREG_GYRO_PROC_X = 0x61          # 97
DREG_GYRO_PROC_Y = 0x62          # 98
DREG_GYRO_PROC_Z = 0x63          # 99
DREG_GYRO_PROC_TIME = 0x64       # 100
DREG_ACCEL_PROC_X = 0x65         # 101
DREG_ACCEL_PROC_Y = 0x66         # 102
DREG_ACCEL_PROC_Z = 0x67         # 103
DREG_ACCEL_PROC_TIME = 0x68      # 104
DREG_MAG_PROC_X = 0x69           # 105
DREG_MAG_PROC_Y = 0x6A           # 106
DREG_MAG_PROC_Z = 0x6B           # 107
DREG_MAG_PROC_TIME = 0x6C        # 108
DREG_QUAT_AB = 0x6D              # 109
DREG_QUAT_CD = 0x6E              # 110
DREG_QUAT_TIME = 0x6F            # 111
DREG_EULER_PHI_THETA = 0x70      # 112
DREG_EULER_PSI = 0x71            # 113
DREG_EULER_PHI_THETA_DOT = 0x72  # 114
DREG_EULER_PSI_DOT = 0x73        # 115
DREG_EULER_TIME = 0x74           # 116

# --- Command addresses (no physical register; triggered with PT=0) ----------
# Sending 'snp' + PT(0x00) + address + checksum runs the command; the UM7
# replies COMMAND_COMPLETE (same address, PT=0) or COMMAND_FAILED (CF bit set).
CMD_ZERO_GYROS = 0xAD            # 173: zero the rate-gyro biases (keep still)
CMD_SET_MAG_REFERENCE = 0xB0     # 176: set current heading as magnetic north


def _reg(address: int, name: str, *fields: Field) -> Register:
    """Build a Register from an address, name, and field descriptors."""
    return Register(address, name, tuple(fields))


def _f32(name: str) -> Field:
    """Return a whole-register IEEE float32 field."""
    return Field(name, Encoding.FLOAT32)


def _i16(name: str, divisor: float, half: Half) -> Field:
    """Return a signed-16-bit field scaled by ``divisor``."""
    return Field(name, Encoding.INT16, divisor, half)


# Only data registers that this driver decodes. RAW gyro/accel/mag registers
# are intentionally omitted until their physical scale is confirmed on
# hardware (see CLAUDE.md); the driver publishes from the PROC registers.
_REGISTER_LIST = (
    _reg(DREG_HEALTH, 'health', Field('health', Encoding.UINT32)),
    _reg(DREG_TEMPERATURE, 'temperature', _f32('temperature')),
    _reg(DREG_TEMPERATURE_TIME, 'temperature_time', _f32('temperature_time')),
    _reg(DREG_GYRO_PROC_X, 'gyro_proc_x', _f32('gyro_proc_x')),
    _reg(DREG_GYRO_PROC_Y, 'gyro_proc_y', _f32('gyro_proc_y')),
    _reg(DREG_GYRO_PROC_Z, 'gyro_proc_z', _f32('gyro_proc_z')),
    _reg(DREG_GYRO_PROC_TIME, 'gyro_proc_time', _f32('gyro_proc_time')),
    _reg(DREG_ACCEL_PROC_X, 'accel_proc_x', _f32('accel_proc_x')),
    _reg(DREG_ACCEL_PROC_Y, 'accel_proc_y', _f32('accel_proc_y')),
    _reg(DREG_ACCEL_PROC_Z, 'accel_proc_z', _f32('accel_proc_z')),
    _reg(DREG_ACCEL_PROC_TIME, 'accel_proc_time', _f32('accel_proc_time')),
    _reg(DREG_MAG_PROC_X, 'mag_proc_x', _f32('mag_proc_x')),
    _reg(DREG_MAG_PROC_Y, 'mag_proc_y', _f32('mag_proc_y')),
    _reg(DREG_MAG_PROC_Z, 'mag_proc_z', _f32('mag_proc_z')),
    _reg(DREG_MAG_PROC_TIME, 'mag_proc_time', _f32('mag_proc_time')),
    _reg(DREG_QUAT_AB, 'quat_ab',
         _i16('quat_a', QUAT_DIVISOR, Half.HIGH),
         _i16('quat_b', QUAT_DIVISOR, Half.LOW)),
    _reg(DREG_QUAT_CD, 'quat_cd',
         _i16('quat_c', QUAT_DIVISOR, Half.HIGH),
         _i16('quat_d', QUAT_DIVISOR, Half.LOW)),
    _reg(DREG_QUAT_TIME, 'quat_time', _f32('quat_time')),
    _reg(DREG_EULER_PHI_THETA, 'euler_phi_theta',
         _i16('roll', EULER_ANGLE_DIVISOR, Half.HIGH),
         _i16('pitch', EULER_ANGLE_DIVISOR, Half.LOW)),
    _reg(DREG_EULER_PSI, 'euler_psi',
         _i16('yaw', EULER_ANGLE_DIVISOR, Half.HIGH)),
    _reg(DREG_EULER_PHI_THETA_DOT, 'euler_phi_theta_dot',
         _i16('roll_rate', EULER_RATE_DIVISOR, Half.HIGH),
         _i16('pitch_rate', EULER_RATE_DIVISOR, Half.LOW)),
    _reg(DREG_EULER_PSI_DOT, 'euler_psi_dot',
         _i16('yaw_rate', EULER_RATE_DIVISOR, Half.HIGH)),
    _reg(DREG_EULER_TIME, 'euler_time', _f32('euler_time')),
)

# Address -> Register lookup used by the parser.
REGISTERS = {register.address: register for register in _REGISTER_LIST}
