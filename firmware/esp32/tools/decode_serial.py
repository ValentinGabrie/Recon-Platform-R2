#!/usr/bin/env python3
"""decode_serial.py — Bench-test decoder for the ESP32 I/O hub.

Reads the binary UART stream from the firmware in firmware/esp32/ and
prints decoded frames in human-readable form so you can verify wiring
before the H2.1 Pi-side ROS bridge exists.

Wire format reference: docs/UART_PROTOCOL.md

Dependencies:
    pip install pyserial

Usage:
    python3 decode_serial.py                 # /dev/ttyUSB0 @ 115200
    python3 decode_serial.py /dev/ttyACM0    # if your USB-Serial bridge enumerates as ACM
    python3 decode_serial.py --imu-rate 2    # decimate IMU prints to 2 Hz
    python3 decode_serial.py --no-color      # plain text (e.g. for piping to a log file)

Prints:
    IMU       a=( ax,  ay,  az) m/s²  g=( gx,  gy,  gz) rad/s   (rate-limited)
    BUTTON    <NAME> <STATE>                                     (immediate, per event)
    HEARTBEAT uptime = N.N s                                     (1 Hz)
    STATUS    flags = 0x.. [BOOT, IMU_OK, IMU_DATA]              (boot + on IMU error)
    CRC_FAIL  type=... len=... expected=... got=...              (any bad-CRC frame)

Ctrl+C prints a summary of frame counts before exiting — a clean run
should show CRC_FAIL = 0.
"""

import argparse
import struct
import sys
import time


# ----- Protocol constants -----------------------------------------------------

SYNC0 = 0xA5
SYNC1 = 0x5A

FRAME_IMU       = 0x01
FRAME_BUTTON    = 0x02
FRAME_HEARTBEAT = 0x03
FRAME_STATUS    = 0x04

FRAME_NAMES = {
    FRAME_IMU:       "IMU",
    FRAME_BUTTON:    "BUTTON",
    FRAME_HEARTBEAT: "HEARTBEAT",
    FRAME_STATUS:    "STATUS",
}

BUTTON_NAMES = {0: "SHUTDOWN", 1: "STARTSTOP", 2: "SAVE"}
STATE_NAMES  = {0: "RELEASED", 1: "PRESSED", 2: "LONGPRESS"}

STATUS_BITS = [
    (0x01, "BOOT"),
    (0x02, "IMU_OK"),
    (0x04, "IMU_DATA"),
]

MAX_PAYLOAD = 24  # current ceiling; must match firmware/esp32/src/framing.h


# ----- CRC8 (Dallas/Maxim, poly 0x07, init 0x00) ------------------------------

def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else ((crc << 1) & 0xFF)
    return crc


# ----- Payload decoders -------------------------------------------------------

def decode_imu(payload: bytes) -> str:
    ax, ay, az, gx, gy, gz = struct.unpack("<6f", payload)
    return (f"a=({ax:+6.2f}, {ay:+6.2f}, {az:+6.2f}) m/s²  "
            f"g=({gx:+6.3f}, {gy:+6.3f}, {gz:+6.3f}) rad/s")


def decode_button(payload: bytes) -> str:
    bid, state = payload[0], payload[1]
    return f"{BUTTON_NAMES.get(bid, f'?({bid})')} {STATE_NAMES.get(state, f'?({state})')}"


def decode_heartbeat(payload: bytes) -> str:
    (uptime,) = struct.unpack("<I", payload)
    return f"uptime = {uptime / 1000.0:.1f} s"


def decode_status(payload: bytes) -> str:
    flags = payload[0]
    set_bits = [name for mask, name in STATUS_BITS if flags & mask]
    base = f"flags = 0x{flags:02X} [{', '.join(set_bits) if set_bits else 'none'}]"
    # Extended boot-diagnostic STATUS (8 B) — see firmware/esp32/src/framing.h.
    if len(payload) == 8:
        who   = payload[1]
        a_cfg = payload[2]
        g_cfg = payload[3]
        za_b  = struct.unpack("<h", payload[4:6])[0]
        za_a  = struct.unpack("<h", payload[6:8])[0]
        return (f"{base}  who_am_i=0x{who:02X} "
                f"accel_cfg=0x{a_cfg:02X} (AFS_SEL={(a_cfg >> 3) & 0x03}) "
                f"gyro_cfg=0x{g_cfg:02X} (FS_SEL={(g_cfg >> 3) & 0x03}) "
                f"za_offset before={za_b} after={za_a}")
    return base


# Accept either the standard 2-byte STATUS payload OR the 8-byte extended
# boot-diagnostic flavour. None == accept any length up to MAX_PAYLOAD.
PAYLOAD_DECODERS = {
    FRAME_IMU:       (24,   decode_imu),
    FRAME_BUTTON:    (2,    decode_button),
    FRAME_HEARTBEAT: (4,    decode_heartbeat),
    FRAME_STATUS:    (None, decode_status),
}


# ----- Colour helpers ---------------------------------------------------------

class Colour:
    GREEN  = "\033[32m"
    YELLOW = "\033[33m"
    RED    = "\033[31m"
    CYAN   = "\033[36m"
    OFF    = "\033[0m"

    @classmethod
    def disable(cls):
        cls.GREEN = cls.YELLOW = cls.RED = cls.CYAN = cls.OFF = ""


FRAME_COLOUR = {
    "IMU":       lambda: Colour.CYAN,
    "BUTTON":    lambda: Colour.GREEN,
    "HEARTBEAT": lambda: Colour.YELLOW,
    "STATUS":    lambda: Colour.YELLOW,
    "CRC_FAIL":  lambda: Colour.RED,
}


def emit(label: str, msg: str) -> None:
    ts = time.strftime("%H:%M:%S")
    colour = FRAME_COLOUR.get(label, lambda: "")()
    print(f"[{ts}] {colour}{label:9}{Colour.OFF} {msg}", flush=True)


# ----- Frame parser (state machine — see docs/UART_PROTOCOL.md §5) -----------

class FrameParser:
    """Stateful byte-feeder. Call feed(byte_int); receive 0 or 1 frames."""

    HUNT0, HUNT1, READ_TYPE, READ_LEN, READ_PAYLOAD, READ_CRC = range(6)

    def __init__(self):
        self.state = self.HUNT0
        self.ftype = 0
        self.flen = 0
        self.buf = bytearray()

    def feed(self, b: int):
        """Return (kind, payload_or_err) when a frame completes, else None.

        kind is either the frame TYPE byte (int) or the string "CRC_FAIL".
        """
        if self.state == self.HUNT0:
            if b == SYNC0:
                self.state = self.HUNT1
            return None

        if self.state == self.HUNT1:
            if b == SYNC1:
                self.state = self.READ_TYPE
            elif b == SYNC0:
                pass  # consecutive 0xA5 — stay armed for 0x5A
            else:
                self.state = self.HUNT0
            return None

        if self.state == self.READ_TYPE:
            self.ftype = b
            self.state = self.READ_LEN
            return None

        if self.state == self.READ_LEN:
            self.flen = b
            if self.flen > MAX_PAYLOAD:
                self.state = self.HUNT0
                return None
            self.buf.clear()
            self.state = self.READ_PAYLOAD if self.flen > 0 else self.READ_CRC
            return None

        if self.state == self.READ_PAYLOAD:
            self.buf.append(b)
            if len(self.buf) == self.flen:
                self.state = self.READ_CRC
            return None

        if self.state == self.READ_CRC:
            expected = crc8(bytes([self.ftype, self.flen]) + bytes(self.buf))
            kind = self.ftype if b == expected else "CRC_FAIL"
            payload = (bytes(self.buf), expected, b) if kind == "CRC_FAIL" else bytes(self.buf)
            ftype_at_fail = self.ftype
            flen_at_fail = self.flen
            self.state = self.HUNT0
            if kind == "CRC_FAIL":
                return ("CRC_FAIL", (ftype_at_fail, flen_at_fail, expected, b))
            return (kind, payload)

        # Should never reach
        self.state = self.HUNT0
        return None


# ----- Main loop --------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("port", nargs="?", default="/dev/ttyUSB0",
                    help="Serial device (default /dev/ttyUSB0)")
    ap.add_argument("--baud", type=int, default=115200,
                    help="Baud rate (default 115200, matches firmware)")
    ap.add_argument("--imu-rate", type=float, default=10.0,
                    help="Max printed IMU samples per second (default 10; 0 = print all)")
    ap.add_argument("--no-color", action="store_true",
                    help="Disable ANSI colour escapes")
    args = ap.parse_args()

    if args.no_color or not sys.stdout.isatty():
        Colour.disable()

    try:
        import serial
    except ImportError:
        print("pyserial not installed. Run: pip install pyserial", file=sys.stderr)
        return 1

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1.0)
    except serial.SerialException as exc:
        print(f"Cannot open {args.port}: {exc}", file=sys.stderr)
        return 1

    print(f"Listening on {args.port} @ {args.baud} baud — Ctrl+C to quit\n",
          flush=True)

    parser = FrameParser()
    counts = {name: 0 for name in FRAME_NAMES.values()}
    counts["CRC_FAIL"] = 0
    last_imu_print = 0.0
    imu_period = 1.0 / args.imu_rate if args.imu_rate > 0 else 0.0

    try:
        while True:
            chunk = ser.read(64)  # read up to 64 bytes per iteration
            if not chunk:
                continue
            for b in chunk:
                result = parser.feed(b)
                if result is None:
                    continue
                kind, payload = result

                if kind == "CRC_FAIL":
                    counts["CRC_FAIL"] += 1
                    ftype, flen, expected, got = payload
                    emit("CRC_FAIL",
                         f"type=0x{ftype:02X} len={flen} "
                         f"expected=0x{expected:02X} got=0x{got:02X}")
                    continue

                name = FRAME_NAMES.get(kind, f"?0x{kind:02X}")
                counts[name] = counts.get(name, 0) + 1

                decoder = PAYLOAD_DECODERS.get(kind)
                if decoder is None:
                    emit(name, f"unknown frame type ({len(payload)} B payload)")
                    continue
                expected_len, fn = decoder
                # expected_len == None means "accept any length" (currently
                # used for STATUS, which has both a 2 B and an 8 B variant).
                if expected_len is not None and len(payload) != expected_len:
                    emit(name,
                         f"wrong payload length: got {len(payload)} B, "
                         f"expected {expected_len} B — {payload.hex()}")
                    continue

                if kind == FRAME_IMU and imu_period > 0:
                    now = time.time()
                    if now - last_imu_print < imu_period:
                        continue
                    last_imu_print = now

                emit(name, fn(payload))

    except KeyboardInterrupt:
        print("\n\n--- Frame counts ---", flush=True)
        for name in ("IMU", "BUTTON", "HEARTBEAT", "STATUS", "CRC_FAIL"):
            print(f"  {name:10} {counts.get(name, 0)}")
        return 0
    finally:
        try:
            ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
