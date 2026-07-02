"""
Streaming decoder for the UM7 binary serial protocol (ROS-free).

Bytes in -> validated packets out -> named physical values. This module has
no rclpy or ROS message imports so it can be unit tested in isolation.

Packet layout (datasheet Binary Packet Structure):

    's' 'n' 'p' | PT | address | data (4 * batch_length bytes) | checksum (2)

The checksum is the unsigned 16-bit sum of every preceding byte, including
the ``snp`` header, transmitted big-endian.
"""

from dataclasses import dataclass, field
import struct

from um7_driver import um7_registers as reg


@dataclass
class Packet:
    """A validated UM7 packet decoded into named physical values."""

    address: int
    is_batch: bool
    batch_length: int
    values: 'dict[str, float]' = field(default_factory=dict)
    raw_registers: 'dict[int, bytes]' = field(default_factory=dict)
    command_failed: bool = False


def compute_checksum(data: bytes) -> int:
    """Return the UM7 checksum: unsigned 16-bit sum of ``data``."""
    return sum(data) & 0xFFFF


def build_command_packet(address: int) -> bytes:
    """Build a UM7 command packet (PT=0, no data) for a command address."""
    body = reg.START_SEQUENCE + bytes((0x00, address))
    return body + compute_checksum(body).to_bytes(2, 'big')


def decode_register(register: reg.Register, chunk: bytes) -> 'dict[str, float]':
    """Decode a 4-byte register ``chunk`` into its named field values."""
    values: 'dict[str, float]' = {}
    for f in register.fields:
        if f.encoding is reg.Encoding.FLOAT32:
            (values[f.name],) = struct.unpack('>f', chunk)
        elif f.encoding is reg.Encoding.UINT32:
            (values[f.name],) = struct.unpack('>I', chunk)
        else:  # INT16 or RAW16: a signed 16-bit half of the register
            half = chunk[0:2] if f.half is reg.Half.HIGH else chunk[2:4]
            (raw,) = struct.unpack('>h', half)
            if f.encoding is reg.Encoding.RAW16:
                values[f.name] = raw
            else:
                values[f.name] = raw / f.divisor
    return values


class Um7Parser:
    """
    Incremental parser that turns a UM7 byte stream into packets.

    Feed arbitrary chunks of serial data via :meth:`feed`; partial packets
    are buffered until the rest of their bytes arrive. Framing errors and bad
    checksums cause a one-byte resync so the stream recovers on its own.
    """

    def __init__(self) -> None:
        """Create an empty parser."""
        self._buffer = bytearray()
        self.checksum_errors = 0

    def feed(self, data: bytes) -> 'list[Packet]':
        """Append ``data`` and return every packet that is now complete."""
        self._buffer.extend(data)
        packets = []
        while True:
            result = self._extract_one()
            if result is None:
                break
            packets.append(result)
        return packets

    def _extract_one(self) -> 'Packet | None':
        """Pop and decode the next complete packet, or return None if none."""
        start = self._buffer.find(reg.START_SEQUENCE)
        if start < 0:
            # No header yet; keep the last two bytes in case 'sn'/'s' is split.
            if len(self._buffer) > 2:
                del self._buffer[:-2]
            return None
        if start > 0:
            del self._buffer[:start]  # discard junk before the header

        if len(self._buffer) < 4:
            return None  # need the PT byte to know the length

        pt = self._buffer[3]
        data_len = self._data_length(pt)
        if data_len is None:
            del self._buffer[:1]  # invalid PT (e.g. batch length 0); resync
            return self._extract_one()

        total_len = 5 + data_len + 2  # snp + pt + addr + data + checksum
        if len(self._buffer) < total_len:
            return None  # wait for the rest of the packet

        candidate = bytes(self._buffer[:total_len])
        received = int.from_bytes(candidate[-2:], 'big')
        if compute_checksum(candidate[:-2]) != received:
            self.checksum_errors += 1
            del self._buffer[:1]  # false header inside data; resync one byte
            return self._extract_one()

        del self._buffer[:total_len]
        return self._decode(candidate, pt, data_len)

    @staticmethod
    def _data_length(pt: int) -> 'int | None':
        """Return the data-section byte count for a PT byte, or None if bad."""
        if not pt & reg.PT_HAS_DATA:
            return 0
        if pt & reg.PT_IS_BATCH:
            count = reg.batch_length(pt)
            if count == 0:
                return None  # a batch must contain at least one register
            return count * reg.REGISTER_SIZE
        return reg.REGISTER_SIZE

    @staticmethod
    def _decode(candidate: bytes, pt: int, data_len: int) -> Packet:
        """Decode a checksum-validated packet into a :class:`Packet`."""
        address = candidate[4]
        is_batch = bool(pt & reg.PT_IS_BATCH)
        num_registers = data_len // reg.REGISTER_SIZE
        packet = Packet(address=address, is_batch=is_batch,
                        batch_length=num_registers,
                        command_failed=bool(pt & reg.PT_COMMAND_FAILED))
        data = candidate[5:5 + data_len]
        for i in range(num_registers):
            reg_address = address + i
            chunk = data[i * reg.REGISTER_SIZE:(i + 1) * reg.REGISTER_SIZE]
            packet.raw_registers[reg_address] = chunk
            register = reg.REGISTERS.get(reg_address)
            if register is not None:
                packet.values.update(decode_register(register, chunk))
        return packet
