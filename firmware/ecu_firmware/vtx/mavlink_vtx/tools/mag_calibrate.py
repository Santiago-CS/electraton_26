#!/usr/bin/env python3
"""Magnetometer calibration for the VTX board: writes nav/mag_cal.h.

Collects the raw BMM350 counts the firmware sends in RAW_IMU over the VTX link
(UDP 14550, the same port the ground station uses - close it first, and make
sure wifibroadcast@gs is running), then fits the hard-iron offset and the
soft-iron matrix so that

    m_cal = W * (raw - offset)

lies on a unit sphere whatever way the board points.

Two fits, for the two ways the data can be collected:

  ellipsoid  (default) The board turned through every orientation - every
             face down at some point. Full 3-D correction. Do it on the bench,
             or better, with the board in the car if the car can be tilted.
  flat       The car driven slowly in two or three full circles on level
             ground. Only rotation about the vertical, so only the horizontal
             part can be fitted: X/Y offset and X/Y soft iron. Z gets no
             offset and the mean X/Y scale. This is the right one for a board
             already fixed in the car, which is where the calibration matters.

    ./mag_calibrate.py --mode flat --save run1.csv          # collect, Ctrl+C, fit
    ./mag_calibrate.py --mode flat --load run1.csv --write ../nav/mag_cal.h

Pure Python, no numpy: a 9x9 least-squares solve and a 3x3 eigen
decomposition are small enough to write out.
"""

import argparse
import math
import socket
import struct
import sys
import time

TELEM_PORT = 14550
MAV_STX_V2 = 0xFD
RAW_IMU_ID = 27
RAW_IMU_CRC_EXTRA = 144
FIRMWARE_SYS_COMP = (1, 1)  # the VTX's own mavfwd also sends RAW_IMU, as 1/250
SATURATED = (32767, -32768)


# ---------------------------------------------------------------- capture ---

def x25(data, crc=0xFFFF):
    for b in data:
        t = (b ^ (crc & 0xFF)) & 0xFF
        t = (t ^ (t << 4)) & 0xFF
        crc = ((crc >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4)) & 0xFFFF
    return crc


def extract_mag(buf):
    """Pull RAW_IMU magnetometer triples out of buf (a bytearray), consuming
    what it parses. Only frames from the firmware, only with a valid CRC."""
    out = []
    while True:
        at = buf.find(bytes([MAV_STX_V2]))
        if at < 0:
            buf.clear()
            return out
        del buf[:at]
        if len(buf) < 12:
            return out
        n = buf[1]
        # A stray 0xFD inside other data is common. Flags other than "signed"
        # mean it is not a header; without this check, one claiming 253 bytes
        # stalls the parser waiting for data that will never be a frame.
        if buf[2] & ~0x01 or buf[3] != 0:
            del buf[:1]
            continue
        total = 12 + n + (13 if buf[2] & 0x01 else 0)
        if len(buf) < total:
            if len(buf) > 2 * 280:    # longer than any frame could need: false start
                del buf[:1]
                continue
            return out
        msgid = buf[7] | (buf[8] << 8) | (buf[9] << 16)
        if msgid == RAW_IMU_ID and (buf[5], buf[6]) == FIRMWARE_SYS_COMP:
            crc, = struct.unpack_from("<H", buf, 10 + n)
            if crc == x25(bytes(buf[1:10 + n]) + bytes([RAW_IMU_CRC_EXTRA])):
                pl = bytes(buf[10:10 + n]).ljust(26, b"\x00")
                m = struct.unpack_from("<3h", pl, 20)
                if not any(v in SATURATED for v in m) and any(m):
                    out.append(m)
                del buf[:total]
                continue
            del buf[:1]  # bad CRC: not a frame start after all
            continue
        # Some other message. Its CRC_EXTRA is not known here, so it cannot be
        # verified - step one byte rather than trusting its length.
        del buf[:1]


def coverage(samples):
    """Horizontal sectors hit (of 12) and the spread of the vertical
    component, so you can see when you have turned enough."""
    if not samples:
        return 0, 0.0
    cx = sum(s[0] for s in samples) / len(samples)
    cy = sum(s[1] for s in samples) / len(samples)
    sectors = {int((math.degrees(math.atan2(s[1] - cy, s[0] - cx)) + 180) // 30) % 12
               for s in samples}
    zs = [s[2] for s in samples]
    norm = max(1.0, sum(math.sqrt(s[0] ** 2 + s[1] ** 2 + s[2] ** 2) for s in samples) / len(samples))
    return len(sectors), (max(zs) - min(zs)) / norm


def capture(port, seconds):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("0.0.0.0", port))
    except OSError as exc:
        sys.exit(f"cannot bind udp:{port} ({exc}) - close the ground station first")
    sock.settimeout(0.5)
    buf = bytearray()
    samples = []
    t0 = time.time()
    last = 0.0
    print("collecting - turn the board (or drive circles), Ctrl+C to fit")
    try:
        while seconds is None or time.time() - t0 < seconds:
            try:
                data, _ = sock.recvfrom(65535)
            except socket.timeout:
                data = b""
            buf += data
            samples.extend(extract_mag(buf))
            if time.time() - last >= 1.0:
                last = time.time()
                sec, zspan = coverage(samples)
                print(f"\r  {len(samples):6d} samples   heading sectors {sec:2d}/12   "
                      f"vertical spread {zspan:4.2f}  ", end="", flush=True)
    except KeyboardInterrupt:
        pass
    print()
    return samples


# -------------------------------------------------------------- linear algebra ---

def solve(a, b):
    """Gaussian elimination with partial pivoting; a is n x n, b length n."""
    n = len(b)
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(m[r][c]))
        if abs(m[p][c]) < 1e-12:
            raise ValueError("singular - not enough orientations in the data")
        m[c], m[p] = m[p], m[c]
        for r in range(n):
            if r != c:
                f = m[r][c] / m[c][c]
                for k in range(c, n + 1):
                    m[r][k] -= f * m[c][k]
    return [m[i][n] / m[i][i] for i in range(n)]


def least_squares(rows, rhs):
    n = len(rows[0])
    ata = [[sum(r[i] * r[j] for r in rows) for j in range(n)] for i in range(n)]
    atb = [sum(r[i] * y for r, y in zip(rows, rhs)) for i in range(n)]
    return solve(ata, atb)


def jacobi_eig(a, sweeps=50):
    """Eigenvalues and eigenvectors (columns of v) of a small symmetric matrix."""
    n = len(a)
    a = [row[:] for row in a]
    v = [[float(i == j) for j in range(n)] for i in range(n)]
    for _ in range(sweeps):
        off = sum(a[i][j] ** 2 for i in range(n) for j in range(n) if i != j)
        if off < 1e-24:
            break
        for p in range(n):
            for q in range(p + 1, n):
                if abs(a[p][q]) < 1e-30:
                    continue
                th = 0.5 * math.atan2(2 * a[p][q], a[q][q] - a[p][p])
                c, s = math.cos(th), math.sin(th)
                for k in range(n):
                    akp, akq = a[k][p], a[k][q]
                    a[k][p], a[k][q] = c * akp - s * akq, s * akp + c * akq
                for k in range(n):
                    apk, aqk = a[p][k], a[q][k]
                    a[p][k], a[q][k] = c * apk - s * aqk, s * apk + c * aqk
                for k in range(n):
                    vkp, vkq = v[k][p], v[k][q]
                    v[k][p], v[k][q] = c * vkp - s * vkq, s * vkp + c * vkq
    return [a[i][i] for i in range(n)], v


def sqrtm_sym(m):
    vals, vecs = jacobi_eig(m)
    if min(vals) <= 0:
        raise ValueError("fit is not an ellipsoid - turn the board through more orientations")
    n = len(m)
    return [[sum(vecs[i][k] * math.sqrt(vals[k]) * vecs[j][k] for k in range(n))
             for j in range(n)] for i in range(n)]


def matvec(m, x):
    return [sum(m[i][j] * x[j] for j in range(len(x))) for i in range(len(m))]


# ------------------------------------------------------------------- fits ---

def fit_ellipsoid(samples):
    """x^T M x + 2 u.x = 1, then offset = -M^-1 u and W = sqrtm(M / k)."""
    s = sum(math.sqrt(x * x + y * y + z * z) for x, y, z in samples) / len(samples)
    pts = [(x / s, y / s, z / s) for x, y, z in samples]
    rows = [[x * x, y * y, z * z, 2 * x * y, 2 * x * z, 2 * y * z, 2 * x, 2 * y, 2 * z]
            for x, y, z in pts]
    a, b, c, d, e, f, g, h, i = least_squares(rows, [1.0] * len(rows))
    m = [[a, d, e], [d, b, f], [e, f, c]]
    u = [g, h, i]
    centre = [-v for v in solve(m, u)]
    k = 1.0 + sum(centre[r] * sum(m[r][q] * centre[q] for q in range(3)) for r in range(3))
    w = sqrtm_sym([[m[r][q] / k for q in range(3)] for r in range(3)])
    return [v * s for v in centre], [[v / s for v in row] for row in w]


def fit_flat(samples):
    """2-D: x/y ellipse only. Z: no offset, the mean x/y scale."""
    s = sum(math.hypot(x, y) for x, y, _ in samples) / len(samples)
    pts = [(x / s, y / s) for x, y, _ in samples]
    rows = [[x * x, y * y, 2 * x * y, 2 * x, 2 * y] for x, y in pts]
    a, b, d, g, h = least_squares(rows, [1.0] * len(rows))
    m = [[a, d], [d, b]]
    centre = [-v for v in solve(m, [g, h])]
    k = 1.0 + sum(centre[r] * sum(m[r][q] * centre[q] for q in range(2)) for r in range(2))
    w2 = sqrtm_sym([[m[r][q] / k for q in range(2)] for r in range(2)])
    vals, _ = jacobi_eig([[m[r][q] / k for q in range(2)] for r in range(2)])
    wz = sum(math.sqrt(v) for v in vals) / 2.0
    w = [[w2[0][0] / s, w2[0][1] / s, 0.0], [w2[1][0] / s, w2[1][1] / s, 0.0], [0.0, 0.0, wz / s]]
    return [centre[0] * s, centre[1] * s, 0.0], w


def quality(samples, offset, w, flat):
    """Spread of the calibrated radius - horizontal only for a flat fit."""
    r = []
    for smp in samples:
        c = matvec(w, [smp[i] - offset[i] for i in range(3)])
        r.append(math.hypot(c[0], c[1]) if flat else math.sqrt(sum(v * v for v in c)))
    mean = sum(r) / len(r)
    return mean, math.sqrt(sum((v - mean) ** 2 for v in r) / len(r))


def header(offset, w, mode, n, spread):
    def f(v):
        txt = f"{v:.9g}"
        if not any(ch in txt for ch in ".eEn"):
            txt += ".0"            # "0f" is not a C float literal; "0.0f" is
        return txt + "f"

    rows = "\n".join("        {" + ", ".join(f(v) for v in row) + "},   \\" for row in w)
    return f"""/*
 * mag_cal.h - BMM350 calibration: raw counts -> calibrated field.
 *
 *     m_cal = MAG_CAL_W * (raw - MAG_CAL_OFFSET)
 *
 * Generated by tools/mag_calibrate.py, {mode} fit, {n} samples, calibrated
 * radius spread {spread * 100:.1f}%. Raw sensor axes, before k_mag_to_body in
 * main.c. Re-run the tool rather than editing these by hand.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _MAG_CAL_H_
#define _MAG_CAL_H_

#define MAG_CAL_OFFSET {{{", ".join(f(v) for v in offset)}}}

#define MAG_CAL_W                \\
    {{                            \\
{rows}
    }}

#endif /* _MAG_CAL_H_ */
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mode", choices=("ellipsoid", "flat"), default="ellipsoid")
    ap.add_argument("--port", type=int, default=TELEM_PORT)
    ap.add_argument("--seconds", type=float, default=None, help="stop after this long")
    ap.add_argument("--save", metavar="CSV", help="also save the raw samples")
    ap.add_argument("--load", metavar="CSV", help="fit saved samples instead of capturing")
    ap.add_argument("--write", metavar="HEADER", help="write the C header here")
    args = ap.parse_args()

    if args.load:
        with open(args.load) as fh:
            samples = [tuple(int(v) for v in line.split(",")) for line in fh if line.strip()]
    else:
        samples = capture(args.port, args.seconds)
    if args.save:
        with open(args.save, "w") as fh:
            fh.writelines(f"{x},{y},{z}\n" for x, y, z in samples)

    sectors, zspan = coverage(samples)
    print(f"{len(samples)} samples, {sectors}/12 heading sectors, vertical spread {zspan:.2f}")
    if len(samples) < 100 or sectors < 10:
        sys.exit("not enough coverage: turn through at least 10 of the 12 heading sectors")
    if args.mode == "ellipsoid" and zspan < 1.0:
        print("WARNING little vertical coverage for an ellipsoid fit - tilt the board "
              "every way, or use --mode flat for in-car circles")

    try:
        offset, w = (fit_flat if args.mode == "flat" else fit_ellipsoid)(samples)
    except ValueError as exc:
        sys.exit(f"fit failed: {exc}")

    mean, sd = quality(samples, offset, w, args.mode == "flat")
    print(f"offset {['%.1f' % v for v in offset]}")
    print(f"calibrated radius {mean:.3f} +/- {sd:.3f}  "
          f"({'good' if sd < 0.03 else 'marginal' if sd < 0.06 else 'poor - recollect'})")

    text = header(offset, w, args.mode, len(samples), sd)
    if args.write:
        with open(args.write, "w") as fh:
            fh.write(text)
        print(f"wrote {args.write} - rebuild and flash")
    else:
        print(text)


if __name__ == "__main__":
    main()
