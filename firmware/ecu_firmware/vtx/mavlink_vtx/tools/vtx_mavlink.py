"""MAVLink decoder for the VTX telemetry link, as structured data.

vtx_ground_station.py turns frames into text lines for a terminal pane; this
turns them into dicts for code that wants the numbers (vtx_dashboard.py). No
GUI or third-party dependencies.

Covers what is actually on the link: the M33 firmware's HEARTBEAT,
GPS_RAW_INT, RAW_IMU, ATTITUDE and NAMED_VALUE_FLOAT (sys 1 comp 1), the VTX
mavfwd's injected RAW_IMU temperature (sys 1 comp 250) and wfb-ng's
RADIO_STATUS (sys 3 comp 68). Anything else is still framed and counted, by id.

A frame is accepted only if its checksum verifies against the message's
CRC_EXTRA. An id with no known CRC_EXTRA cannot be verified, so it is accepted
only when its header is plausible and it ends exactly where another frame
starts - the same rule the ground station uses, for the same reason: mavfwd
splits frames across radio packets, and a lost packet leaves a frame tail
whose bytes can look like a header.
"""

import struct

STX_V1 = 0xFE
STX_V2 = 0xFD
MAX_MSGID = 0xFFFF   # every published dialect fits; a larger id is a misread header

# Raw sensor scaling, as configured by the firmware (sensors/icm_42686_p.h).
ACCEL_LSB_PER_G = 2048.0
GYRO_LSB_PER_DPS = 16.4

# Sensor -> body forward-right-down, k_imu_to_body in main.c. Keep in sync.
IMU_TO_BODY = ((1, 0, 0), (0, -1, 0), (0, 0, -1))

CRC_EXTRA = {
    0: 50,     # HEARTBEAT
    24: 24,    # GPS_RAW_INT
    26: 170,   # SCALED_IMU
    27: 144,   # RAW_IMU
    30: 39,    # ATTITUDE
    109: 185,  # RADIO_STATUS
    251: 170,  # NAMED_VALUE_FLOAT
}

MSG_NAMES = {
    0: "HEARTBEAT", 24: "GPS_RAW_INT", 26: "SCALED_IMU", 27: "RAW_IMU",
    30: "ATTITUDE", 109: "RADIO_STATUS", 251: "NAMED_VALUE_FLOAT",
}

GPS_FIX = {0: "no GPS", 1: "no fix", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK float", 6: "RTK"}


def x25(data, crc=0xFFFF):
    for b in data:
        t = (b ^ (crc & 0xFF)) & 0xFF
        t = (t ^ (t << 4)) & 0xFF
        crc = ((crc >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4)) & 0xFFFF
    return crc


def _body(v):
    return tuple(sum(IMU_TO_BODY[r][c] * v[c] for c in range(3)) for r in range(3))


def decode_payload(msgid, pl):
    """Fields of one message as a dict, or None for an id it does not know.
    MAVLink 2 trims trailing zero bytes, so payloads are padded back first."""
    if msgid == 0:
        pl = pl.ljust(9, b"\x00")
        return {"type": pl[4], "autopilot": pl[5], "base_mode": pl[6],
                "system_status": pl[7], "mavlink_version": pl[8]}
    if msgid == 24:
        pl = pl.ljust(30, b"\x00")
        t_us, lat, lon, alt = struct.unpack_from("<Qiii", pl, 0)
        eph, epv, vel, cog = struct.unpack_from("<4H", pl, 20)
        return {"time_usec": t_us, "lat": lat / 1e7, "lon": lon / 1e7, "alt_m": alt / 1000.0,
                "dop": None if eph == 0xFFFF else eph / 100.0,
                "speed_kmh": None if vel == 0xFFFF else vel * 0.036,
                "cog_deg": None if cog == 0xFFFF else cog / 100.0,
                "fix_type": pl[28], "sats": pl[29]}
    if msgid == 27:
        n = len(pl)
        pl = pl.ljust(29, b"\x00")
        t_us, = struct.unpack_from("<Q", pl, 0)
        raw = struct.unpack_from("<9h", pl, 8)
        d = {"time_usec": t_us, "acc_raw": raw[0:3], "gyro_raw": raw[3:6], "mag_raw": raw[6:9]}
        if n > 26:   # extension fields: id, temperature (cdegC)
            d["imu_id"] = pl[26]
            d["temp_c"] = struct.unpack_from("<h", pl, 27)[0] / 100.0
        d["all_zero"] = not any((t_us,) + raw)
        acc_s = tuple(v / ACCEL_LSB_PER_G for v in raw[0:3])
        gyr_s = tuple(v / GYRO_LSB_PER_DPS for v in raw[3:6])
        d["acc_body_g"] = _body(acc_s)
        d["gyro_body_dps"] = _body(gyr_s)
        return d
    if msgid == 26:
        pl = pl.ljust(22, b"\x00")
        t_ms, = struct.unpack_from("<I", pl, 0)
        v = struct.unpack_from("<9h", pl, 4)
        return {"time_boot_ms": t_ms, "acc_g": tuple(a / 1000.0 for a in v[0:3]),
                "gyro_dps": tuple(g * 0.05729578 for g in v[3:6]), "mag": v[6:9]}
    if msgid == 30:
        pl = pl.ljust(28, b"\x00")
        t_ms, r, p, y, rs, ps, ys = struct.unpack_from("<I6f", pl, 0)
        k = 57.29578
        return {"time_boot_ms": t_ms, "roll_deg": r * k, "pitch_deg": p * k, "yaw_deg": y * k,
                "rollspeed_dps": rs * k, "pitchspeed_dps": ps * k, "yawspeed_dps": ys * k}
    if msgid == 109:
        pl = pl.ljust(9, b"\x00")
        rxerr, fixed = struct.unpack_from("<HH", pl, 0)
        return {"rxerrors": rxerr, "fixed": fixed, "rssi": pl[4], "remrssi": pl[5],
                "txbuf": pl[6], "noise": pl[7], "remnoise": pl[8]}
    if msgid == 251:
        pl = pl.ljust(18, b"\x00")
        t_ms, value = struct.unpack_from("<If", pl, 0)
        name = pl[8:18].split(b"\x00", 1)[0].decode("ascii", "replace")
        return {"time_boot_ms": t_ms, "name": name, "value": value}
    return None


def summary(msgid, d):
    """One short human line for a decoded message, for tables and logs."""
    if d is None:
        return ""
    if msgid == 0:
        return f"type {d['type']}  state {d['system_status']}  v{d['mavlink_version']}"
    if msgid == 24:
        spd = "-" if d["speed_kmh"] is None else f"{d['speed_kmh']:.1f} km/h"
        return (f"{GPS_FIX.get(d['fix_type'], d['fix_type'])}  {d['sats']} sat  "
                f"{d['lat']:+.6f} {d['lon']:+.6f}  {spd}")
    if msgid == 27:
        if d["all_zero"]:
            return f"(sensors all zero)  temp {d.get('temp_c', 0):.1f} C"
        a = d["acc_body_g"]
        return f"acc {a[0]:+.2f} {a[1]:+.2f} {a[2]:+.2f} g  mag {d['mag_raw']}"
    if msgid == 30:
        return f"roll {d['roll_deg']:+.1f}  pitch {d['pitch_deg']:+.1f}  yaw {d['yaw_deg']:+.1f} deg"
    if msgid == 109:
        return f"rssi {d['rssi']}  noise {d['noise']}  rxerr {d['rxerrors']}  fixed {d['fixed']}"
    if msgid == 251:
        return f"{d['name']} = {d['value']:+.6g}"
    if msgid == 26:
        a = d["acc_g"]
        return f"acc {a[0]:+.2f} {a[1]:+.2f} {a[2]:+.2f} g"
    return ""


class Frame:
    __slots__ = ("version", "seq", "sysid", "compid", "msgid", "payload", "verified", "fields")

    def __init__(self, version, seq, sysid, compid, msgid, payload, verified):
        self.version, self.seq, self.sysid, self.compid = version, seq, sysid, compid
        self.msgid, self.payload, self.verified = msgid, payload, verified
        self.fields = decode_payload(msgid, payload) if verified else None

    @property
    def name(self):
        return MSG_NAMES.get(self.msgid, f"id {self.msgid}")


class MavlinkStream:
    """Feed it datagrams; get complete frames back. Keeps link statistics."""

    def __init__(self):
        self.buf = bytearray()
        self.frames = 0
        self.bytes = 0
        self.crc_errors = 0
        self.unparsed = 0
        self.lost = 0
        self.seq = {}        # (sysid, compid) -> last seq
        self.gaps = {}       # (sysid, compid) -> frames lost

    def feed(self, data):
        self.bytes += len(data)
        self.buf += data
        out = []
        while True:
            f = self._next()
            if f is None:
                break
            out.append(f)
        return out

    def _frame_len(self, b):
        if b[0] == STX_V2:
            if len(b) < 3:
                return None
            return 12 + b[1] + (13 if b[2] & 0x01 else 0)
        return 8 + b[1]

    def _next(self):
        b = self.buf
        while True:
            # Resync to the next start byte.
            i = 0
            while i < len(b) and b[i] not in (STX_V1, STX_V2):
                i += 1
            if i:
                self.unparsed += i
                del b[:i]
            if len(b) < 8:
                return None
            if b[0] == STX_V2 and (b[2] & ~0x01 or b[3] != 0):
                self.unparsed += 1     # flags MAVLink 2 does not define: not a header
                del b[:1]
                continue
            total = self._frame_len(b)
            if total is None or len(b) < total:
                if len(b) > 2 * 280:   # more than any frame needs: a false start
                    self.unparsed += 1
                    del b[:1]
                    continue
                return None

            if b[0] == STX_V2:
                n = b[1]
                seq, sysid, compid = b[4], b[5], b[6]
                msgid = b[7] | (b[8] << 8) | (b[9] << 16)
                payload = bytes(b[10:10 + n])
                crc_at, hdr = 10 + n, 1
                version = 2
            else:
                n = b[1]
                seq, sysid, compid, msgid = b[2], b[3], b[4], b[5]
                payload = bytes(b[6:6 + n])
                crc_at, hdr = 6 + n, 1
                version = 1

            extra = CRC_EXTRA.get(msgid)
            if extra is not None:
                crc, = struct.unpack_from("<H", b, crc_at)
                if crc != x25(bytes(b[hdr:crc_at]) + bytes([extra])):
                    self.crc_errors += 1
                    del b[:1]          # not a frame after all: step one byte
                    continue
                verified = True
            else:
                ends_clean = len(b) == total or b[total] in (STX_V1, STX_V2)
                if sysid == 0 or msgid > MAX_MSGID or not ends_clean:
                    self.unparsed += 1
                    del b[:1]
                    continue
                verified = False

            del b[:total]
            self.frames += 1
            key = (sysid, compid)
            prev = self.seq.get(key)
            if prev is not None:
                gap = (seq - prev - 1) & 0xFF
                if gap:
                    self.lost += gap
                    self.gaps[key] = self.gaps.get(key, 0) + gap
            self.seq[key] = seq
            return Frame(version, seq, sysid, compid, msgid, payload, verified)
