#!/usr/bin/env python3
"""
VTX ground station: live video on the left, telemetry terminal on the right.

Brings up the wfb-ng ground station, shows the H.265 video it puts on UDP 5600,
and decodes whatever the datalink puts on UDP 14550 into a scrolling pane.
Follows vtx_run_book.md - the runbook is the authority on the hardware steps
and this script only automates them.

    VTX camera -> majestic -> wfb_tx -- ch 161 --> RTL8812AU receiver
      -> wifibroadcast@gs -> UDP 5600 (video) + UDP 14550 (telemetry) -> here

WHAT THE RIGHT-HAND PANE SHOWS

The firmware on the i.MX93 M33 currently sends **raw IMU frames**, not MAVLink:
24-byte binary frames that main.c builds and tools/vtx_imu_decode.py
documents. So that is what you will see. The MAVLink v1/v2 decoder here is for
when mavlink_task starts emitting real MAVLink - both run at once and the pane
labels which it found, so you can tell at a glance what the link is carrying.

Anything that is neither is hex-dumped rather than hidden, because on a new
link "bytes are arriving but I cannot parse them" is the single most useful
thing to be able to see.

USAGE

    ./vtx_ground_station.py                  # start the gs, show video + telemetry
    ./vtx_ground_station.py --hw             # VA-API decode (less CPU)
    ./vtx_ground_station.py --codec h264     # if the VTX is set to H.264
    ./vtx_ground_station.py --no-gs          # do not touch the systemd service
    ./vtx_ground_station.py --no-video       # telemetry only

Starting the ground station needs root, so sudo will prompt once. The service
is only stopped again on exit if this script was the thing that started it.

REQUIREMENTS

    sudo apt install python3-gi gstreamer1.0-tools gstreamer1.0-plugins-base \
                     gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
                     gstreamer1.0-libav
    # optional but nicer, and required on Wayland:
    sudo apt install gstreamer1.0-gtk3
"""

import argparse
import os
import socket
import struct
import subprocess
import sys
import time

import gi

gi.require_version("Gtk", "3.0")
gi.require_version("Gst", "1.0")
from gi.repository import GLib, Gst, Gtk, Pango  # noqa: E402

# GstVideo is only needed for the X11 window-id embedding path, and it comes
# from a package that is not installed by default. Import it lazily so the
# script still runs without it - see choose_embedding().
try:
    gi.require_version("GstVideo", "1.0")
    from gi.repository import GstVideo  # noqa: E402
    HAVE_GSTVIDEO = True
except (ValueError, ImportError):
    GstVideo = None
    HAVE_GSTVIDEO = False

# Reuse the frame decoder rather than copy it. Importing it also runs its
# module-level CRC self-check against the standard 0x29B1 vector.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from vtx_imu_decode import (ACCEL_LSB_PER_G, FRAME_LEN, GYRO_LSB_PER_DPS,
                                SYNC, TEMP_OFFSET, TEMP_SENS, decode)
except ImportError:
    sys.exit("vtx_imu_decode.py must sit next to this script")

VIDEO_PORT = 5600
TELEM_PORT = 14550
WFB_CFG = "/etc/wifibroadcast.cfg"
GS_UNIT = "wifibroadcast@gs"
MAX_LINES = 3000

# Layout. The terminal pane is a fixed width; the video takes the rest.
WINDOW_W, WINDOW_H = 1500, 760
PANE_W = 580
VIDEO_MIN_W, VIDEO_MIN_H = 480, 270


# --------------------------------------------------------------------------
# MAVLink
#
# A minimal framer rather than a pymavlink dependency: this only needs to say
# what arrived, not interpret payloads. pymavlink is used purely to turn a
# message id into a name when it happens to be installed.
# --------------------------------------------------------------------------

MAVLINK_V1 = 0xFE
MAVLINK_V2 = 0xFD

# CRC_EXTRA per message - a hash of the message's field layout, mixed into the
# checksum so two ends that disagree about a message fail loudly instead of
# misreading each other. 109 was recovered by search from a real wfb-ng frame;
# the rest are from the common dialect and are what the firmware also uses, so
# agreement here does NOT prove them. QGroundControl showing the vehicle does.
MAV_CRC_EXTRA = {
    0: 50,     # HEARTBEAT
    24: 24,    # GPS_RAW_INT
    27: 144,   # RAW_IMU
    26: 170,   # SCALED_IMU
    30: 39,    # ATTITUDE
    109: 185,  # RADIO_STATUS  (confirmed against a captured frame)
}


def mav_crc(data, extra):
    """CRC-16/MCRF4XX (X.25). Verified against a captured wfb-ng frame."""
    crc = 0xFFFF
    for b in tuple(data) + (extra,):
        t = (b ^ (crc & 0xFF)) & 0xFF
        t = (t ^ (t << 4)) & 0xFF
        crc = ((crc >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4)) & 0xFFFF
    return crc


def mav_describe(msgid, pl):
    """Human-readable line for the messages we care about. MAVLink v2 truncates
    trailing zero bytes, so short payloads are padded back out first."""
    if msgid == 0 and len(pl) >= 9:
        pl = pl.ljust(9, b"\x00")
        typ, autop, _base, state, ver = pl[4], pl[5], pl[6], pl[7], pl[8]
        return f"HEARTBEAT type {typ} autopilot {autop} state {state} mavlink v{ver}"
    if msgid == 26:
        pl = pl.ljust(22, b"\x00")
        t_ms, = struct.unpack_from("<I", pl, 0)
        ax, ay, az, gx, gy, gz = struct.unpack_from("<6h", pl, 4)
        # back to degrees/s for reading: mrad/s / 1000 * 180/pi
        d = 57.29578 / 1000.0
        return (f"SCALED_IMU {t_ms / 1000.0:8.2f}s "
                f"a{ax / 1000.0:+6.2f}{ay / 1000.0:+6.2f}{az / 1000.0:+6.2f}g "
                f"w{gx * d:+7.1f}{gy * d:+7.1f}{gz * d:+7.1f}dps")
    if msgid == 24:
        pl = pl.ljust(30, b"\x00")
        lat, lon, alt = struct.unpack_from("<iii", pl, 8)
        eph, _epv, vel, cog = struct.unpack_from("<4H", pl, 20)
        fix, sats = pl[28], pl[29]
        fixname = {0: "noGPS", 1: "nofix", 2: "2D", 3: "3D", 4: "DGPS",
                   5: "RTKfl", 6: "RTK"}.get(fix, f"#{fix}")
        # eph is dilution of precision x100, not a distance; 65535 = unknown.
        dop = "  dop ?" if eph == 65535 else f"  dop {eph / 100.0:4.2f}"
        crs = "   cog   -" if cog == 65535 else f"   cog {cog / 100.0:5.1f}"
        return (f"GPS_RAW_INT {fixname:<5} {sats:2d}sv "
                f"{lat / 1e7:+11.7f} {lon / 1e7:+12.7f} {alt / 1000.0:7.1f}m"
                f"{dop}  {vel / 100.0:5.2f}m/s{crs}")
    if msgid == 27:
        n = len(pl)
        pl = pl.ljust(29, b"\x00")
        t_us, = struct.unpack_from("<Q", pl, 0)
        ax, ay, az, gx, gy, gz, mx, my, mz = struct.unpack_from("<9h", pl, 8)
        # MAVLink 2 extension fields: id (u8) at 26, temperature (i16, cdegC)
        # at 27. Shown because a sender can put its only real content there -
        # and a 29-byte payload that survived trailing-zero truncation proves
        # the last byte is non-zero.
        imu_id = pl[26]
        temp, = struct.unpack_from("<h", pl, 27)
        ext = ""
        if n > 26:
            ext = f" id {imu_id} temp " + ("-" if temp == 0 else f"{temp / 100.0:.2f}C")
        if not any((t_us, ax, ay, az, gx, gy, gz, mx, my, mz)):
            return f"RAW_IMU (all sensor fields zero){ext}"
        return (f"RAW_IMU {t_us / 1e6:10.2f}s a{ax:+7d}{ay:+7d}{az:+7d} "
                f"g{gx:+7d}{gy:+7d}{gz:+7d} m{mx:+6d}{my:+6d}{mz:+6d}{ext}")
    if msgid == 109:
        pl = pl.ljust(9, b"\x00")
        rxerr, fixed = struct.unpack_from("<HH", pl, 0)
        return (f"RADIO_STATUS rssi {pl[4]} remrssi {pl[5]} txbuf {pl[6]}% "
                f"noise {pl[7]} rxerr {rxerr} fixed {fixed}")
    return None

try:
    from pymavlink.dialects.v20 import common as _mav

    def mav_name(msgid):
        msg = _mav.mavlink_map.get(msgid)
        return msg.name if msg else f"id {msgid}"
except Exception:  # pymavlink absent, or a dialect import problem
    def mav_name(msgid):
        return f"id {msgid}"


# Every message id in every published dialect fits in 16 bits. The 24-bit
# field exists for headroom, so a larger id is a misread header, not a message.
MAV_MAX_MSGID = 0xFFFF

# Bytes that can start a frame of either format this tool understands.
FRAME_STARTS = (MAVLINK_V1, MAVLINK_V2, 0xA5)


def mav_header_plausible(buf):
    """Cheap sanity checks on a header, before trusting its length byte.

    0xFD and 0xFE occur constantly inside payloads, and once a lost datagram
    has left the parser misaligned, one of them will be taken for a frame
    start. Its "length" is then arbitrary - up to 255 - and accepting it
    swallows the real frames behind it, which then show up as sequence loss.
    Everything checked here is something no real sender produces.
    """
    if buf[0] == MAVLINK_V1:
        return buf[3] != 0                      # sysid 0 is broadcast, never a sender
    incompat, compat = buf[2], buf[3]
    sysid = buf[5]
    msgid = buf[7] | (buf[8] << 8) | (buf[9] << 16)
    return (sysid != 0
            and (incompat & ~0x01) == 0         # signing is the only defined flag
            and compat == 0                     # no compat flags are defined at all
            and msgid <= MAV_MAX_MSGID)


def mav_try_parse(buf):
    """Try to read one MAVLink frame from the front of buf.

    Returns (consumed, text, ident):

      (0, None, None)   not enough bytes yet - wait for more
      (1, None, None)   not a frame here - step past this byte
      (1, text, None)   shaped like a real frame but its checksum is wrong -
                        step past this byte, and report it, because that is
                        exactly how a wrong CRC_EXTRA shows itself
      (n, text, ident)  a frame of n bytes; ident is (sysid, compid, seq)

    A frame is believed in one of two ways. If its message id is one we know,
    the checksum decides - that is conclusive. If not, the header has to be
    plausible AND the frame has to end exactly where another frame starts, or
    at the end of what has arrived. Without that second rule an unknown id was
    accepted on faith, which is how "sys 0" senders with ids like 6051919
    (the bytes 4F 58 5C, ASCII "OX\\") made it into the sender list.
    """
    v1 = buf[0] == MAVLINK_V1
    hdr = 6 if v1 else 10

    if len(buf) < hdr:
        return 0, None, None
    if not mav_header_plausible(buf):
        return 1, None, None

    plen = buf[1]
    if v1:
        signed = False
        total = 6 + plen + 2
        seq, sysid, compid, msgid = buf[2], buf[3], buf[4], buf[5]
    else:
        signed = bool(buf[2] & 0x01)
        total = 10 + plen + 2 + (13 if signed else 0)
        seq, sysid, compid = buf[4], buf[5], buf[6]
        msgid = buf[7] | (buf[8] << 8) | (buf[9] << 16)

    if len(buf) < total:
        return 0, None, None

    payload = bytes(buf[hdr:hdr + plen])
    extra = MAV_CRC_EXTRA.get(msgid)
    tag = "MAVLINK1" if v1 else "MAVLINK2"

    if extra is not None:
        want = buf[hdr + plen] | (buf[hdr + plen + 1] << 8)
        if mav_crc(bytes(buf[1:hdr + plen]), extra) != want:
            return 1, (f"{tag} {mav_name(msgid):<22} BAD CHECKSUM "
                       f"(wrong CRC_EXTRA for id {msgid}?)"), None
        mark = " ok"
    else:
        if total < len(buf) and buf[total] not in FRAME_STARTS:
            return 1, None, None
        mark = ""

    pretty = mav_describe(msgid, payload)
    if pretty:
        text = f"{pretty}{mark}"
    else:
        text = (f"{tag} {mav_name(msgid):<22} "
                f"sys {sysid:3d} comp {compid:3d} seq {seq:3d} len {plen}"
                + ("  signed" if signed else ""))
    return total, text, (sysid, compid, seq)


def frame_valid_at(buf, i):
    """True if a frame that proves itself - by CRC - starts at buf[i].

    Used to decide whether a stalled parse is waiting on a real frame or on a
    payload byte that merely looks like a header.
    """
    view = buf[i:]
    if view[:2] == SYNC:
        return len(view) >= FRAME_LEN and decode(bytes(view[:FRAME_LEN])) is not None
    if view and view[0] in (MAVLINK_V1, MAVLINK_V2):
        used, text, ident = mav_try_parse(view)
        return used > 1 and ident is not None and text is not None and text.endswith(" ok")
    return False


# --------------------------------------------------------------------------
# Receiver bring-up
# --------------------------------------------------------------------------

def run(cmd, **kw):
    return subprocess.run(cmd, capture_output=True, text=True, **kw)


def preflight():
    """Check the things the runbook checks. Reports; never fatal - a warning
    that turns out not to matter is better than refusing to start."""
    notes = []

    ifaces = [ln.split()[0] for ln in run(["ip", "-br", "link"]).stdout.splitlines()
              if ln.startswith("wlx")]
    if not ifaces:
        notes.append("WARN  no wlx* interface - is the USB-C receiver plugged in?")
    else:
        for name in ifaces:
            drv = run(["ethtool", "-i", name]).stdout.splitlines()
            drv = drv[0].split(":", 1)[1].strip() if drv else "?"
            if drv == "rtl88xxau_wfb":
                notes.append(f"ok    {name}: driver {drv}")
            else:
                notes.append(f"WARN  {name}: driver {drv!r}, expected rtl88xxau_wfb")
                notes.append("      sudo modprobe -r rtw88_8812au && sudo modprobe 88XXau_wfb")
                notes.append("      then unplug and replug the receiver (runbook step 2)")

    if os.path.exists(WFB_CFG):
        chan = ""
        try:
            with open(WFB_CFG) as fh:
                for line in fh:
                    if line.strip().startswith("wifi_channel"):
                        chan = line.split("=", 1)[1].strip()
        except OSError:
            pass
        notes.append(f"ok    {WFB_CFG}: wifi_channel = {chan or '?'}")
    else:
        notes.append(f"WARN  {WFB_CFG} missing")

    notes.append("ok    /etc/gs.key present" if os.path.exists("/etc/gs.key")
                 else "WARN  /etc/gs.key missing - the link will not decrypt")
    return notes


def gs_active():
    return run(["systemctl", "is-active", GS_UNIT]).stdout.strip() == "active"


def gs_start():
    """Returns True if this call started it (so we should stop it on exit)."""
    if gs_active():
        print(f"{GS_UNIT} already running - leaving it alone")
        return False
    print(f"starting {GS_UNIT} (sudo may prompt)...")
    res = run(["sudo", "systemctl", "start", GS_UNIT])
    if res.returncode != 0:
        print(res.stderr.strip() or f"could not start {GS_UNIT}", file=sys.stderr)
        return False
    for _ in range(20):
        if gs_active():
            print(f"{GS_UNIT} active")
            return True
        time.sleep(0.25)
    print(f"WARN  {GS_UNIT} did not become active", file=sys.stderr)
    return False


def gs_stop():
    print(f"stopping {GS_UNIT}...")
    run(["sudo", "systemctl", "stop", GS_UNIT])


# --------------------------------------------------------------------------
# The window
# --------------------------------------------------------------------------

class GroundStation(Gtk.Window):
    def __init__(self, args, notes):
        super().__init__(title="VTX ground station")
        self.args = args
        self.set_default_size(WINDOW_W, WINDOW_H)
        self.connect("destroy", Gtk.main_quit)

        # Telemetry counters
        self.t_frames = self.t_bytes = self.t_lost = 0
        self.t_crc_err = self.t_unknown = 0
        self.last_seq = None
        self.mav_seq = {}    # (sysid, compid) -> last seq byte
        self.mav_seen = {}   # (sysid, compid) -> frames
        self.mav_gaps = {}   # (sysid, compid) -> missing frames
        self.imu_shown = 0
        self.rx_buf = bytearray()
        self.kinds = set()
        self.w_frames = self.w_bytes = self.w_lost = 0
        self.v_frames = 0
        self.w_vframes = 0
        self.window_t0 = time.monotonic()

        # A Paned rather than a Box. In a Box the pane widths follow what the
        # children ASK for, and a single-line status label asks for the full
        # width of its text - so every new sender in the status line widened
        # the right pane, squeezed the video down to a thumbnail, and grew the
        # window. A Paned with a set position holds both sides where they are:
        # resizing the window goes to the video, and nothing the telemetry
        # does can move the divider. It can still be dragged by hand.
        paned = Gtk.Paned(orientation=Gtk.Orientation.HORIZONTAL)
        paned.set_wide_handle(True)
        self.add(paned)

        # ---- left: video ----
        self.video_widget = None
        if args.video:
            frame = Gtk.Frame(label=f" video  udp:{VIDEO_PORT} ")
            self.video_holder = Gtk.Box()
            # A floor, so the picture can never be squeezed to nothing.
            self.video_holder.set_size_request(VIDEO_MIN_W, VIDEO_MIN_H)
            frame.add(self.video_holder)
            left = frame
        else:
            left = Gtk.Label(label="video disabled (--no-video)")
        paned.pack1(left, resize=True, shrink=False)

        # ---- right: the terminal pane ----
        right = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
        right.set_size_request(PANE_W, -1)
        paned.pack2(right, resize=False, shrink=False)
        paned.set_position(WINDOW_W - PANE_W)

        # Wraps, and is capped at a few lines, so its minimum width is a word
        # rather than its whole text and its height cannot creep either. That
        # is what stops the label from resizing anything.
        self.status = Gtk.Label(xalign=0.0)
        self.status.set_line_wrap(True)
        self.status.set_line_wrap_mode(Pango.WrapMode.WORD_CHAR)
        self.status.set_ellipsize(Pango.EllipsizeMode.END)
        self.status.set_lines(4)
        self.status.set_max_width_chars(1)   # let the pane width decide
        self.status.set_selectable(True)
        right.pack_start(self.status, False, False, 2)

        scroll = Gtk.ScrolledWindow()
        scroll.set_policy(Gtk.PolicyType.AUTOMATIC, Gtk.PolicyType.AUTOMATIC)
        self.view = Gtk.TextView()
        self.view.set_editable(False)
        self.view.set_cursor_visible(False)
        self.view.set_monospace(True)
        self.buf = self.view.get_buffer()
        scroll.add(self.view)
        right.pack_start(scroll, True, True, 0)
        self.scroll = scroll

        # Terminal look, without depending on a theme.
        css = Gtk.CssProvider()
        css.load_from_data(b"""
            textview, textview text { background:#101216; color:#d6dae0;
                                      font-family:monospace; font-size:10pt; }
            label { font-family:monospace; font-size:9pt; }
        """)
        Gtk.StyleContext.add_provider_for_screen(
            self.get_screen(), css, Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION)

        btns = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=4)
        clear = Gtk.Button(label="clear")
        clear.connect("clicked", lambda _b: self.buf.set_text(""))
        btns.pack_start(clear, False, False, 0)
        self.pause_btn = Gtk.ToggleButton(label="pause scroll")
        btns.pack_start(self.pause_btn, False, False, 0)
        self.raw_btn = Gtk.ToggleButton(label="hex")
        self.raw_btn.set_tooltip_text("also hex-dump every packet")
        btns.pack_start(self.raw_btn, False, False, 0)
        right.pack_start(btns, False, False, 0)

        for n in notes:
            self.log(n)
        self.log("")

        self.open_telemetry()
        if args.video:
            self.start_video()

        GLib.timeout_add(1000, self.tick)

    # ---- output ----

    def log(self, text):
        end = self.buf.get_end_iter()
        self.buf.insert(end, text + "\n")
        if self.buf.get_line_count() > MAX_LINES:
            start = self.buf.get_start_iter()
            cut = self.buf.get_iter_at_line(self.buf.get_line_count() - MAX_LINES)
            self.buf.delete(start, cut)
        if not self.pause_btn.get_active():
            adj = self.scroll.get_vadjustment()
            adj.set_value(adj.get_upper())

    # ---- telemetry ----

    def open_telemetry(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.sock.bind(("0.0.0.0", TELEM_PORT))
        except OSError as exc:
            self.log(f"WARN  cannot bind udp:{TELEM_PORT} ({exc})")
            self.log("      another listener has it - close QGroundControl?")
            return
        self.sock.setblocking(False)
        self.log(f"ok    listening for telemetry on udp:{TELEM_PORT}")
        GLib.io_add_watch(self.sock.fileno(), GLib.IO_IN, self.on_telemetry)

    def on_telemetry(self, _fd, _cond):
        while True:
            try:
                data, _addr = self.sock.recvfrom(65535)
            except BlockingIOError:
                break
            self.t_bytes += len(data)
            self.w_bytes += len(data)
            if self.raw_btn.get_active():
                self.log(f"  raw {len(data):4d}B  " + data[:32].hex(" "))
            self.rx_buf += data
            self.parse()
        return True

    def next_valid_imu(self, buf, start=1):
        """Index of the next CRC-valid IMU frame at or after start, or -1.

        The CRC is the only self-verifying structure in this stream, which
        makes it the right tool for telling the two formats apart when a byte
        could plausibly start either.
        """
        i = start
        while True:
            at = buf.find(SYNC, i)
            if at < 0 or at + FRAME_LEN > len(buf):
                return -1
            if decode(bytes(buf[at:at + FRAME_LEN])) is not None:
                return at
            i = at + 1

    def parse(self):
        """Pull every complete frame out of rx_buf, trying both formats."""
        buf = self.rx_buf
        while buf:
            # Our raw IMU frame
            if buf[:2] == SYNC:
                if len(buf) < FRAME_LEN:
                    break
                got = decode(bytes(buf[:FRAME_LEN]))
                if got is None:
                    self.t_crc_err += 1
                    del buf[:1]          # a real frame may start inside this one
                    continue
                del buf[:FRAME_LEN]
                self.on_imu(got)
                continue

            # MAVLink
            if buf[0] in (MAVLINK_V1, MAVLINK_V2):
                used, text, ident = mav_try_parse(buf)
                if used == 0:
                    # Not enough bytes for the frame this header claims. That
                    # is normal mid-stream, but if the "header" was really a
                    # payload byte its length is arbitrary and waiting would
                    # hold up every real frame behind it. If something that
                    # proves itself by CRC starts before this supposed frame
                    # could end, the magic byte was data - skip to that.
                    at = next((i for i in range(1, len(buf)) if frame_valid_at(buf, i)), -1)
                    if at > 0:
                        self.t_unknown += at
                        del buf[:at]
                        continue
                    break                # genuinely wait for the rest
                if used == 1:
                    del buf[:1]
                    if text:             # real-looking frame, bad checksum
                        self.t_crc_err += 1
                        self.log(text)
                    else:
                        self.t_unknown += 1
                    continue
                del buf[:used]
                if text:
                    self.t_frames += 1
                    self.w_frames += 1
                    self.kinds.add("MAVLink")
                    if ident is not None:
                        # MAVLink carries its own per-(sysid, compid) sequence
                        # byte. Tracking it is the only way to tell "the link
                        # is dropping frames" from "the sender slowed down" -
                        # without it the loss figure is always zero and means
                        # nothing.
                        sysid, compid, seq = ident
                        key = (sysid, compid)
                        prev = self.mav_seq.get(key)
                        if prev is not None:
                            gap = (seq - prev - 1) & 0xFF
                            if gap:
                                self.t_lost += gap
                                self.w_lost += gap
                                self.mav_gaps[key] = self.mav_gaps.get(key, 0) + gap
                        self.mav_seq[key] = seq
                        self.mav_seen[key] = self.mav_seen.get(key, 0) + 1
                    # 50 Hz of SCALED_IMU buries everything else, and appending
                    # that fast is real work for the main thread, which is also
                    # the one painting the video. Thin out the IMU lines only;
                    # every other message is rare and worth seeing.
                    if text.startswith(("SCALED_IMU", "RAW_IMU")):
                        self.imu_shown += 1
                        if self.imu_shown % self.args.every:
                            continue
                    self.log(text)
                continue

            # Neither: show it rather than hide it, one line per run of junk.
            junk = bytearray()
            while buf and buf[:2] != SYNC and buf[0] not in (MAVLINK_V1, MAVLINK_V2):
                junk.append(buf[0])
                del buf[:1]
            if junk:
                self.t_unknown += len(junk)
                self.log(f"  ?? {len(junk):3d} unparsed  " + bytes(junk[:24]).hex(" "))

    def on_imu(self, got):
        self.t_frames += 1
        self.w_frames += 1
        self.kinds.add("raw-IMU")
        if self.last_seq is not None:
            gap = (got["seq"] - self.last_seq - 1) & 0xFFFF
            if gap:
                self.t_lost += gap
                self.w_lost += gap
        self.last_seq = got["seq"]

        if self.t_frames % self.args.every:
            return
        a = [v / ACCEL_LSB_PER_G for v in got["accel"]]
        g = [v / GYRO_LSB_PER_DPS for v in got["gyro"]]
        t = got["temp"] / TEMP_SENS + TEMP_OFFSET
        self.log(f"IMU {got['seq']:5d} {got['t_us'] / 1e6:8.2f}s "
                 f"a{a[0]:+6.2f}{a[1]:+6.2f}{a[2]:+6.2f}g "
                 f"w{g[0]:+7.1f}{g[1]:+7.1f}{g[2]:+7.1f}dps {t:4.1f}C")

    # ---- video ----

    def start_video(self):
        if self.args.codec == "h265":
            enc, depay, parse = "H265", "rtph265depay", "h265parse"
            hw, sw = "vah265dec", "avdec_h265"
        else:
            enc, depay, parse = "H264", "rtph264depay", "h264parse"
            hw, sw = "vah264dec", "avdec_h264"

        # Hardware decode whenever the GPU offers it. The element is only
        # registered when a VA device actually supports the codec, so finding
        # it is a real capability check, not just "the plugin is installed".
        if self.args.decoder == "sw" or not Gst.ElementFactory.find(hw):
            dec = sw
        else:
            dec = hw

        jitter = f"rtpjitterbuffer latency={self.args.jitter} ! " if self.args.jitter else ""

        # Two changes from the runbook pipeline, both about not losing packets
        # on THIS machine:
        #
        #  buffer-size  udpsrc otherwise gets the kernel default, 208 KB here.
        #               A 1080p keyframe arrives as a burst that can approach
        #               that on its own, and anything that overflows is lost
        #               before the decoder ever sees it - which looks exactly
        #               like RF loss: grey, blocky, until the next keyframe.
        #               Capped by net.core.rmem_max, which is 5 MB here.
        #
        #  queue        udpsrc pushes synchronously, so if the decoder stalls
        #               for a moment the socket stops being read and fills.
        #               A queue gives the socket its own thread to drain into.
        pipe_core = (f"udpsrc port={VIDEO_PORT} buffer-size={self.args.sockbuf} "
                     f"caps=application/x-rtp,media=video,clock-rate=90000,"
                     f"encoding-name={enc} ! queue ! {jitter}{depay} ! {parse} ! "
                     f"{dec} ! videoconvert")

        # Three ways to get a picture, in order of preference:
        #
        #   widget  gtksink renders straight into a Gtk widget. The only path
        #           that also works on Wayland. Needs gstreamer1.0-gtk3.
        #   xid     hand xvimagesink an X11 window id. No extra GStreamer
        #           plugin, but needs the GstVideo typelib and X11.
        #   extern  neither available: run the runbook's own gst-launch
        #           pipeline in its own window. Not embedded, but it works
        #           with nothing installed beyond what the runbook already
        #           requires, which matters more than the layout.
        if Gst.ElementFactory.find("gtksink"):
            self.embed = "widget"
            sink = "gtksink name=vsink"
        elif HAVE_GSTVIDEO and Gst.ElementFactory.find("xvimagesink") and os.environ.get("DISPLAY"):
            self.embed = "xid"
            sink = "xvimagesink name=vsink force-aspect-ratio=true"
        else:
            self.embed = "extern"
            sink = None

        if self.embed == "extern":
            self.log("video: separate window (not embedded)")
            self.log("      for video inside this window, install either:")
            self.log("        sudo apt install gstreamer1.0-gtk3          (best, works on Wayland)")
            self.log("        sudo apt install gir1.2-gst-plugins-base-1.0 (X11 only)")
            self.video_holder.pack_start(
                Gtk.Label(label="video is in its own window\n\n"
                                "install gstreamer1.0-gtk3 to embed it here"),
                True, True, 0)
            cmd = ["gst-launch-1.0"] + pipe_core.split() + \
                  ["!", "autovideosink", "sync=false"]
            try:
                self.video_proc = subprocess.Popen(cmd)
            except OSError as exc:
                self.log(f"WARN  could not launch gst-launch-1.0: {exc}")
            return

        desc = f"{pipe_core} ! {sink} sync=false"
        self.log(f"video: {dec} ({'hardware' if dec == hw else 'software'}), "
                 f"{self.embed} embedding, socket {self.args.sockbuf // 1000} kB")

        try:
            self.pipeline = Gst.parse_launch(desc)
        except GLib.Error as exc:
            self.log(f"WARN  video pipeline failed to build: {exc.message}")
            self.log(f"      {desc}")
            return

        vsink = self.pipeline.get_by_name("vsink")

        if self.embed == "widget":
            self.video_widget = vsink.props.widget
            self.video_holder.pack_start(self.video_widget, True, True, 0)
            self.video_widget.show()
        else:
            self.video_widget = Gtk.DrawingArea()
            self.video_holder.pack_start(self.video_widget, True, True, 0)
            self.video_widget.show()
            self.video_widget.realize()
            xid = self.video_widget.get_window().get_xid()
            GstVideo.VideoOverlay.set_window_handle(vsink, xid)

        # Count decoded buffers for the FPS readout.
        pad = vsink.get_static_pad("sink")
        if pad:
            pad.add_probe(Gst.PadProbeType.BUFFER, self.on_video_buffer)

        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message::error", self.on_gst_error)
        bus.connect("message::warning", self.on_gst_warning)
        self.pipeline.set_state(Gst.State.PLAYING)

    def on_video_buffer(self, _pad, _info):
        self.v_frames += 1
        self.w_vframes += 1
        return Gst.PadProbeReturn.OK

    def on_gst_error(self, _bus, msg):
        err, dbg = msg.parse_error()
        self.log(f"GST ERROR {err.message}")
        if "Address already in use" in (dbg or "") or "Address already in use" in err.message:
            self.log(f"      something else holds udp:{VIDEO_PORT} - close QGroundControl")
        if dbg:
            self.log(f"      {dbg.splitlines()[0]}")

    def on_gst_warning(self, _bus, msg):
        err, _dbg = msg.parse_warning()
        self.log(f"GST WARN  {err.message}")

    # ---- once a second ----

    def tick(self):
        now = time.monotonic()
        span = max(now - self.window_t0, 1e-9)
        capacity = self.args.baud / 10.0

        tfps = self.w_frames / span
        tbps = self.w_bytes / span
        vfps = self.w_vframes / span
        expect = self.w_frames + self.w_lost
        lossp = (100.0 * self.w_lost / expect) if expect else 0.0

        if self.mav_seen:
            kinds = "  ".join(
                f"{s}/{c}:{n}" + (f" -{self.mav_gaps.get((s, c), 0)}"
                                  if self.mav_gaps.get((s, c)) else "")
                for (s, c), n in sorted(self.mav_seen.items()))
        else:
            kinds = "+".join(sorted(self.kinds)) if self.kinds else "nothing yet"
        vtxt = ("external" if getattr(self, "embed", None) == "extern"
                else ("off" if not self.args.video else f"{vfps:5.1f} fps"))
        self.status.set_markup(
            f"<b>video</b> {vtxt}   "
            f"<b>telem</b> {tfps:6.1f} fps  {tbps:7.0f} B/s "
            f"({100.0 * tbps / capacity:4.1f}% of {self.args.baud})\n"
            f"sys/comp:frames {GLib.markup_escape_text(kinds)}   "
            f"lost {self.w_lost} ({lossp:.2f}%)   "
            f"totals: {self.t_frames} frames, crc_err {self.t_crc_err}, "
            f"unparsed {self.t_unknown} B")

        self.window_t0 = now
        self.w_frames = self.w_bytes = self.w_lost = self.w_vframes = 0
        return True

    def shutdown(self):
        if getattr(self, "pipeline", None):
            self.pipeline.set_state(Gst.State.NULL)
        proc = getattr(self, "video_proc", None)
        if proc and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-gs", dest="gs", action="store_false",
                    help="do not start or stop wifibroadcast@gs")
    ap.add_argument("--keep-gs", action="store_true",
                    help="leave the ground station running on exit")
    ap.add_argument("--no-video", dest="video", action="store_false",
                    help="telemetry pane only")
    ap.add_argument("--codec", choices=("h265", "h264"), default="h265",
                    help="must match the VTX's majestic.yaml (default h265)")
    ap.add_argument("--decoder", choices=("auto", "hw", "sw"), default="auto",
                    help="auto (default) uses VA-API when the GPU supports the "
                         "codec, otherwise software")
    ap.add_argument("--hw", dest="decoder", action="store_const", const="hw",
                    help="same as --decoder hw")
    ap.add_argument("--sw", dest="decoder", action="store_const", const="sw",
                    help="same as --decoder sw")
    ap.add_argument("--sockbuf", type=int, default=4_000_000, metavar="BYTES",
                    help="UDP receive buffer for the video socket (default 4 MB; "
                         "the kernel caps it at net.core.rmem_max)")
    ap.add_argument("--jitter", type=int, default=0, metavar="MS",
                    help="insert rtpjitterbuffer with this latency "
                         "(default 0 = off, matching the runbook pipeline)")
    ap.add_argument("--every", type=int, default=10, metavar="N",
                    help="show every Nth IMU line, raw or MAVLink (default 10; "
                         "other messages are always shown). Loss accounting "
                         "always sees every frame.")
    ap.add_argument("--baud", type=int, default=115200,
                    help="link rate the telemetry percentage is measured against")
    args = ap.parse_args()
    if args.every < 1:
        args.every = 1

    Gst.init(None)

    notes = preflight()
    for n in notes:
        print(n)

    started_gs = gs_start() if args.gs else False

    win = GroundStation(args, notes)
    win.show_all()
    try:
        Gtk.main()
    except KeyboardInterrupt:
        pass
    finally:
        win.shutdown()
        if started_gs and not args.keep_gs:
            gs_stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
