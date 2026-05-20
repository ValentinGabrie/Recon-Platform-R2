"""Unit tests for the ESP32 UART frame parser (recon_hardware.framing).

The parser is the only piece of the bridge that can be unit-tested without
real hardware — the rest is rclpy plumbing. These tests cover:

  * CRC-8 polynomial correctness against a hand-computed reference
  * Round-trip encode → parse for IMU / BUTTON / HEARTBEAT / STATUS
  * Resync after byte loss (start mid-frame, garbage prefix)
  * Bad CRC → CRC_FAIL emission + parser keeps running
  * Extended 8-byte STATUS variant decoded into named fields
"""

import struct

import pytest

from recon_hardware.framing import (
    FrameParser,
    FrameType,
    ButtonId,
    ButtonState,
    crc8,
    decode_imu,
    decode_button,
    decode_heartbeat,
    decode_status,
)


def make_frame(ftype: int, payload: bytes) -> bytes:
    """Build a complete on-wire frame including sync bytes + CRC."""
    body = bytes([ftype, len(payload)]) + payload
    return bytes([0xA5, 0x5A]) + body + bytes([crc8(body)])


# ---- CRC8 ------------------------------------------------------------------

def test_crc8_known_vectors():
    # crc8(b"") = 0
    assert crc8(b"") == 0
    # Reference computed offline with the same polynomial (0x07, init 0)
    assert crc8(b"\x00") == 0x00
    assert crc8(b"\x01") == 0x07
    assert crc8(b"hello") == 0x92


def test_crc8_deterministic():
    a = bytes(range(24))
    assert crc8(a) == crc8(a)


# ---- Round trips -----------------------------------------------------------

def feed_all(frames: bytes):
    """Drain a parser, returning every emitted result."""
    p = FrameParser()
    out = list(p.feed_bytes(frames))
    return out


def test_imu_roundtrip():
    payload = struct.pack("<6f", 0.0, 0.0, 9.80665, 0.1, -0.2, 0.3)
    frame = make_frame(FrameType.IMU, payload)
    [result] = feed_all(frame)
    kind, sample = result
    assert kind == FrameType.IMU
    assert sample.az == pytest.approx(9.80665)
    assert sample.gx == pytest.approx(0.1)


def test_button_roundtrip():
    payload = bytes([ButtonId.SAVE, ButtonState.PRESSED])
    frame = make_frame(FrameType.BUTTON, payload)
    [result] = feed_all(frame)
    kind, evt = result
    assert kind == FrameType.BUTTON
    assert evt.name == "SAVE"
    assert evt.state_name == "PRESSED"


def test_heartbeat_roundtrip():
    payload = struct.pack("<I", 12345)
    frame = make_frame(FrameType.HEARTBEAT, payload)
    [result] = feed_all(frame)
    kind, uptime = result
    assert kind == FrameType.HEARTBEAT
    assert uptime == 12345


def test_status_short_roundtrip():
    payload = bytes([0x03, 0x00])  # BOOT | IMU_OK, reserved=0
    frame = make_frame(FrameType.STATUS, payload)
    [result] = feed_all(frame)
    kind, status = result
    assert kind == FrameType.STATUS
    assert status.flags == 0x03
    assert status.imu_ok is True
    # Extended diag fields stay None for the 2-byte variant.
    assert status.who_am_i is None


def test_status_diag_roundtrip():
    # [flags, who, accel_cfg, gyro_cfg, za_off_before (LE int16), za_off_after]
    payload = bytes([0x02, 0x68, 0x08, 0x08]) + struct.pack("<hh", 1544, 0)
    frame = make_frame(FrameType.STATUS, payload)
    [result] = feed_all(frame)
    kind, status = result
    assert kind == FrameType.STATUS
    assert status.who_am_i == 0x68
    assert status.accel_cfg == 0x08
    assert status.gyro_cfg == 0x08
    assert status.za_offset_before == 1544
    assert status.za_offset_after == 0


# ---- Resync + error handling ------------------------------------------------

def test_resync_after_garbage_prefix():
    """Junk before the first valid sync is silently discarded."""
    garbage = b"\x00\xff\x12\xa5\x34"  # an 0xA5 in the middle should not trigger
    payload = struct.pack("<I", 42)
    frame = garbage + make_frame(FrameType.HEARTBEAT, payload)
    results = feed_all(frame)
    # Exactly one heartbeat should come out.
    assert len(results) == 1
    assert results[0][0] == FrameType.HEARTBEAT
    assert results[0][1] == 42


def test_consecutive_syncs_still_parse():
    """0xA5 0xA5 0x5A ... — the parser must not lose the real sync."""
    payload = struct.pack("<I", 7)
    frame = b"\xa5" + make_frame(FrameType.HEARTBEAT, payload)
    results = feed_all(frame)
    assert len(results) == 1
    assert results[0][1] == 7


def test_bad_crc_emits_crc_fail_and_recovers():
    payload = struct.pack("<I", 99)
    frame = bytearray(make_frame(FrameType.HEARTBEAT, payload))
    frame[-1] ^= 0xFF  # corrupt the CRC byte
    # Append a good frame after to confirm the parser recovered.
    frame += make_frame(FrameType.HEARTBEAT, struct.pack("<I", 100))
    results = feed_all(bytes(frame))
    assert results[0][0] == "CRC_FAIL"
    assert results[1] == (FrameType.HEARTBEAT, 100)


def test_oversized_len_field_is_dropped():
    """LEN > MAX_PAYLOAD must not allocate a huge buffer; parser resyncs."""
    bad = bytes([0xA5, 0x5A, 0x01, 0xFF, 0x00, 0x00])
    # Followed by a valid frame to prove recovery.
    good = make_frame(FrameType.HEARTBEAT, struct.pack("<I", 5))
    results = feed_all(bad + good)
    # We may get nothing for the bad frame, or a stray junk discard;
    # critical thing is the good frame parses.
    assert results[-1] == (FrameType.HEARTBEAT, 5)


# ---- Streaming chunked input ------------------------------------------------

def test_chunked_feed():
    """Feeding bytes one at a time gives identical output to a single blob."""
    payload = struct.pack("<6f", 1, 2, 3, 4, 5, 6)
    frame = make_frame(FrameType.IMU, payload)
    p = FrameParser()
    results = []
    for b in frame:
        r = p.feed(b)
        if r is not None:
            results.append(r)
    assert len(results) == 1
    kind, sample = results[0]
    assert kind == FrameType.IMU
    assert sample.ax == 1.0 and sample.gz == 6.0
