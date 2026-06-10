"""ESP32 UART binary-frame parser — canonical Pi-side implementation.

Wire format (matches firmware/esp32/src/framing.cpp; canonical spec lives
in docs/UART_PROTOCOL.md):

    0       1       2       3       4..4+LEN-1     4+LEN
    ┌───────┬───────┬───────┬───────┬─────────────┬───────┐
    │ 0xA5  │ 0x5A  │ TYPE  │ LEN   │ PAYLOAD ... │ CRC8  │
    └───────┴───────┴───────┴───────┴─────────────┴───────┘
                              CRC covers TYPE + LEN + PAYLOAD

CRC-8: Dallas/Maxim, polynomial 0x07, init 0x00.

A near-identical decoder lives at firmware/esp32/tools/decode_serial.py
for ROS-free bench testing — keep the two in sync if the wire format
ever changes (it shouldn't; bump the second sync byte if it does).
"""

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional


SYNC0 = 0xA5
SYNC1 = 0x5A

# Must match MAX_PAYLOAD in firmware/esp32/src/framing.h. 64 lets LIDAR_FRAME
# carry a couple of LD14P scan packets (47 B each) per envelope.
MAX_PAYLOAD = 64


class FrameType(IntEnum):
    IMU         = 0x01  # 24 B: 6× float32 (ax, ay, az / gx, gy, gz)
    BUTTON      = 0x02  # 2 B:  uint8 id, uint8 state
    HEARTBEAT   = 0x03  # 4 B:  uint32 uptime_ms
    STATUS      = 0x04  # 2 B or 8 B (extended diag — see decode_status)
    # Bidirectional additions
    LIDAR_FRAME = 0x05  # N B:  raw LD14P bytes      (ESP32 → Pi)
    LIDAR_EN    = 0x06  # 1 B:  0=off, 1=on          (Pi → ESP32)
    LIDAR_ACK   = 0x07  # 1 B:  current motor state  (ESP32 → Pi)


class ButtonId(IntEnum):
    SHUTDOWN  = 0
    STARTSTOP = 1  # formerly RESET — same wire id (1), toggles the scanner stack
    SAVE      = 2


class ButtonState(IntEnum):
    RELEASED  = 0
    PRESSED   = 1
    LONGPRESS = 2


# Status-frame flag bits (matches firmware/esp32/src/config.h).
STATUS_BOOT     = 1 << 0
STATUS_IMU_OK   = 1 << 1
STATUS_IMU_DATA = 1 << 2


def crc8(data: bytes) -> int:
    """Dallas/Maxim CRC-8 (poly 0x07, init 0x00). Matches Linux `crc8`."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else ((crc << 1) & 0xFF)
    return crc


@dataclass
class ImuSample:
    ax: float  # m/s²
    ay: float
    az: float
    gx: float  # rad/s
    gy: float
    gz: float


@dataclass
class ButtonEvent:
    id: int
    state: int

    @property
    def name(self) -> str:
        return ButtonId(self.id).name if self.id in ButtonId._value2member_map_ else f"BTN_{self.id}"

    @property
    def state_name(self) -> str:
        return ButtonState(self.state).name if self.state in ButtonState._value2member_map_ else f"S_{self.state}"


@dataclass
class StatusFrame:
    """Parsed STATUS payload. Diagnostic fields are populated only for the
    extended 8-byte boot variant; otherwise they're None."""
    flags: int
    who_am_i: Optional[int] = None
    accel_cfg: Optional[int] = None
    gyro_cfg:  Optional[int] = None
    za_offset_before: Optional[int] = None
    za_offset_after:  Optional[int] = None

    @property
    def imu_ok(self) -> bool:
        return bool(self.flags & STATUS_IMU_OK)


# Decoded result type — (FrameType, payload-dataclass) or
# ("CRC_FAIL", (ftype:int, flen:int, expected_crc:int, got_crc:int))
DecodedFrame = tuple


def decode_imu(payload: bytes) -> ImuSample:
    ax, ay, az, gx, gy, gz = struct.unpack("<6f", payload)
    return ImuSample(ax, ay, az, gx, gy, gz)


def decode_button(payload: bytes) -> ButtonEvent:
    return ButtonEvent(id=payload[0], state=payload[1])


def decode_heartbeat(payload: bytes) -> int:
    return struct.unpack("<I", payload)[0]  # uptime_ms


def decode_status(payload: bytes) -> StatusFrame:
    if len(payload) == 2:
        return StatusFrame(flags=payload[0])
    if len(payload) == 8:
        za_b, za_a = struct.unpack("<hh", payload[4:8])
        return StatusFrame(
            flags=payload[0],
            who_am_i=payload[1],
            accel_cfg=payload[2],
            gyro_cfg=payload[3],
            za_offset_before=za_b,
            za_offset_after=za_a,
        )
    # Unrecognised length — surface raw flags only.
    return StatusFrame(flags=payload[0] if payload else 0)


_PAYLOAD_LENGTHS = {
    FrameType.IMU:         {24},
    FrameType.BUTTON:      {2},
    FrameType.HEARTBEAT:   {4},
    FrameType.STATUS:      {2, 8},
    FrameType.LIDAR_ACK:   {1},
    # LIDAR_FRAME accepts any non-empty payload up to MAX_PAYLOAD — chunks
    # are arbitrary cuts of the LD14P byte stream, not aligned to packets.
}


def encode_frame(ftype: int, payload: bytes = b"") -> bytes:
    """Build a wire frame: [SYNC0][SYNC1][TYPE][LEN][PAYLOAD][CRC8].

    Used for Pi → ESP32 commands (LIDAR_EN). Raises ValueError if `payload`
    is too large for the protocol.
    """
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(
            f"payload too large: {len(payload)} > {MAX_PAYLOAD}")
    scope = bytes([ftype, len(payload)]) + payload
    return bytes([SYNC0, SYNC1]) + scope + bytes([crc8(scope)])


def encode_lidar_en(enable: bool) -> bytes:
    """Pi → ESP32: set the LIDAR motor power state."""
    return encode_frame(int(FrameType.LIDAR_EN), bytes([1 if enable else 0]))


class FrameParser:
    """Stateful byte-feeder. Feed it bytes; collect frames as they complete.

    Resync algorithm (see docs/UART_PROTOCOL.md §5):
      * scan for SYNC0, then SYNC1
      * read TYPE, LEN; reject if LEN > MAX_PAYLOAD
      * accumulate PAYLOAD; verify CRC8

    On CRC failure the frame is reported via the result tuple ("CRC_FAIL", ...)
    and the parser hunts for the next sync immediately.
    """

    _HUNT0, _HUNT1, _TYPE, _LEN, _PAY, _CRC = range(6)

    def __init__(self) -> None:
        self._state = self._HUNT0
        self._ftype = 0
        self._flen  = 0
        self._buf   = bytearray()

    def feed(self, b: int):
        """Feed one byte. Returns a result tuple when a frame completes, else None."""
        st = self._state
        if st == self._HUNT0:
            if b == SYNC0:
                self._state = self._HUNT1
            return None
        if st == self._HUNT1:
            if b == SYNC1:
                self._state = self._TYPE
            elif b == SYNC0:
                pass  # consecutive 0xA5 — stay armed
            else:
                self._state = self._HUNT0
            return None
        if st == self._TYPE:
            self._ftype = b
            self._state = self._LEN
            return None
        if st == self._LEN:
            self._flen = b
            if b > MAX_PAYLOAD:
                self._state = self._HUNT0
                return None
            self._buf.clear()
            self._state = self._PAY if b > 0 else self._CRC
            return None
        if st == self._PAY:
            self._buf.append(b)
            if len(self._buf) == self._flen:
                self._state = self._CRC
            return None
        if st == self._CRC:
            ftype = self._ftype
            flen  = self._flen
            buf   = bytes(self._buf)
            self._state = self._HUNT0
            expected = crc8(bytes([ftype, flen]) + buf)
            if b != expected:
                return ("CRC_FAIL", (ftype, flen, expected, b))
            return self._dispatch(ftype, buf)
        # Unreachable
        self._state = self._HUNT0
        return None

    def feed_bytes(self, chunk: bytes):
        """Generator: yield each frame as bytes flow in."""
        for b in chunk:
            r = self.feed(b)
            if r is not None:
                yield r

    @staticmethod
    def _dispatch(ftype: int, payload: bytes):
        valid_lengths = _PAYLOAD_LENGTHS.get(FrameType(ftype) if ftype in FrameType._value2member_map_ else None)
        if valid_lengths is not None and len(payload) not in valid_lengths:
            return ("BAD_LEN", (ftype, len(payload), payload))
        if ftype == FrameType.IMU:
            return (FrameType.IMU, decode_imu(payload))
        if ftype == FrameType.BUTTON:
            return (FrameType.BUTTON, decode_button(payload))
        if ftype == FrameType.HEARTBEAT:
            return (FrameType.HEARTBEAT, decode_heartbeat(payload))
        if ftype == FrameType.STATUS:
            return (FrameType.STATUS, decode_status(payload))
        if ftype == FrameType.LIDAR_FRAME:
            # Pass raw bytes straight through — the pty writer doesn't care
            # about alignment, only ordering.
            return (FrameType.LIDAR_FRAME, payload)
        if ftype == FrameType.LIDAR_ACK:
            return (FrameType.LIDAR_ACK, bool(payload[0]))
        return ("UNKNOWN", (ftype, payload))
