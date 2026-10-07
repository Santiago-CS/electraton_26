#!/usr/bin/env python3
"""
Decode and measure the raw-IMU telemetry stream that main.c pushes
out of LPUART6.

The point of this is measurement, not display. It reports frames per second,
bytes per second, how much of the line rate that is, and - the number that
actually matters - how many frames were lost, worked out from gaps in the
sequence counter. A link that drops 3% of frames looks perfectly healthy if you
only count the ones that arrive.

Wire format (24 bytes, little-endian) - see the comment block in
main.c, which is the authority:

    0   u8   0xA5         sync
    1   u8   0x5A         sync
    2   u16  seq
    4   u32  t_us
    8   i16  accel[3]     raw LSB, X Y Z
    14  i16  gyro[3]      raw LSB, X Y Z
    20  i16  temp         raw LSB
    22  u16  crc          CRC-16/CCITT-FALSE over bytes 2..21

Usage:
    ./vtx_imu_decode.py --port /dev/ttyACM0
    ./vtx_imu_decode.py --port /dev/ttyACM0 --baud 115200 --print 200

NXP boards enumerate as /dev/ttyACMx on Linux, never /dev/ttyUSBx.
"""

import argparse
import struct
import sys
import time

FRAME_LEN = 24
SYNC = b"\xA5\x5A"

# Full-scale map at FS_SEL=1, confirmed empirically on this board - see
# sensors/icm_42686_p.h. Only used for the optional human-readable print; the
# measurement itself never needs it.
ACCEL_LSB_PER_G = 2048.0
GYRO_LSB_PER_DPS = 16.4
TEMP_SENS = 132.48
TEMP_OFFSET = 25.0


def crc16_ccitt_false(data: bytes) -> int:
    """Independent implementation. Checked against the standard vector:
    crc16_ccitt_false(b"123456789") == 0x29B1."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


assert crc16_ccitt_false(b"123456789") == 0x29B1, "CRC implementation is wrong"


def decode(frame: bytes):
    """Returns a dict, or None if the checksum does not match."""
    seq, t_us = struct.unpack_from("<HI", frame, 2)
    vals = struct.unpack_from("<7h", frame, 8)
    crc_rx = struct.unpack_from("<H", frame, 22)[0]
    if crc_rx != crc16_ccitt_false(frame[2:22]):
        return None
    return {
        "seq": seq,
        "t_us": t_us,
        "accel": vals[0:3],
        "gyro": vals[3:6],
        "temp": vals[6],
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True, help="e.g. /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--print", dest="every", type=int, default=0,
                    help="print every Nth decoded frame (0 = never)")
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="stop after this long (0 = run until interrupted)")
    args = ap.parse_args()

    try:
        import serial
    except ImportError:
        print("needs pyserial:  pip install pyserial", file=sys.stderr)
        return 1

    port = serial.Serial(args.port, args.baud, timeout=0.2)
    capacity = args.baud / 10.0  # 8N1 = 10 bits on the wire per byte

    buf = bytearray()
    frames = crc_err = resync = lost = 0
    bytes_seen = 0
    last_seq = None
    started = time.monotonic()
    window = started
    w_frames = w_bytes = w_lost = 0

    print(f"listening on {args.port} at {args.baud} ({capacity:.0f} B/s capacity)")
    print("ctrl-C to stop\n")

    try:
        while True:
            chunk = port.read(4096)
            if chunk:
                buf += chunk
                bytes_seen += len(chunk)
                w_bytes += len(chunk)

            # Resynchronise on the sync pair rather than assuming alignment:
            # the stream may be joined mid-frame.
            while True:
                at = buf.find(SYNC)
                if at < 0:
                    # keep one byte, a sync pair may straddle the boundary
                    if len(buf) > 1:
                        del buf[:-1]
                    break
                if at > 0:
                    del buf[:at]
                    resync += 1
                if len(buf) < FRAME_LEN:
                    break

                got = decode(bytes(buf[:FRAME_LEN]))
                if got is None:
                    # Bad checksum: drop just the sync byte, so a frame that
                    # begins inside this one can still be found.
                    crc_err += 1
                    del buf[:1]
                    continue

                del buf[:FRAME_LEN]
                frames += 1
                w_frames += 1

                if last_seq is not None:
                    gap = (got["seq"] - last_seq - 1) & 0xFFFF
                    if gap:
                        lost += gap
                        w_lost += gap
                last_seq = got["seq"]

                if args.every and frames % args.every == 0:
                    a = [v / ACCEL_LSB_PER_G for v in got["accel"]]
                    g = [v / GYRO_LSB_PER_DPS for v in got["gyro"]]
                    t = got["temp"] / TEMP_SENS + TEMP_OFFSET
                    print(f"  seq {got['seq']:5d}  t {got['t_us']/1e6:9.3f}s  "
                          f"a {a[0]:+6.3f} {a[1]:+6.3f} {a[2]:+6.3f} g  "
                          f"w {g[0]:+8.2f} {g[1]:+8.2f} {g[2]:+8.2f} dps  "
                          f"{t:5.1f} C")

            now = time.monotonic()
            if now - window >= 1.0:
                span = now - window
                expect = w_frames + w_lost
                lossp = (100.0 * w_lost / expect) if expect else 0.0
                print(f"{w_frames/span:7.1f} fps  {w_bytes/span:8.0f} B/s  "
                      f"{100.0*w_bytes/span/capacity:5.1f}% of line  "
                      f"lost {w_lost} ({lossp:.2f}%)  crc_err {crc_err}  resync {resync}")
                window, w_frames, w_bytes, w_lost = now, 0, 0, 0

            if args.seconds and (now - started) >= args.seconds:
                break
    except KeyboardInterrupt:
        pass

    span = max(time.monotonic() - started, 1e-9)
    expect = frames + lost
    print(f"\n{frames} frames in {span:.1f}s = {frames/span:.1f} fps, "
          f"{bytes_seen/span:.0f} B/s ({100.0*bytes_seen/span/capacity:.1f}% of line)")
    print(f"lost {lost} of {expect} expected "
          f"({100.0*lost/expect if expect else 0:.3f}%), "
          f"{crc_err} checksum errors, {resync} resyncs")
    return 0


if __name__ == "__main__":
    sys.exit(main())
