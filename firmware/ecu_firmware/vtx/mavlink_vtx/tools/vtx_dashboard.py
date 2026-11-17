#!/usr/bin/env python3
"""VTX ground station + live telemetry dashboard - BE KEY | ITESO Racing Team.

One window for everything that comes back from the car over the VTX:

  Live      strip charts, GPS track coloured by speed, G-G diagram and gauge
            cards - the views of the post-session Telemetry Suite (branch
            tools/telemetry-gui-dashboard), fed live by the car's MAVLink
  Video     the camera, decoded on the GPU
  Messages  every MAVLink message received, its sender, rate and last value

Recording writes a CSV in the Telemetry Suite's column format to tools/logs,
so a session recorded here opens in that tool for lap analysis. Columns the
car has no sensor for (throttle, brake, battery temperature) are left empty.

It replaces vtx_ground_station.py for normal use: it runs the receiver checks
(shown in the status bar instead of a terminal), starts wifibroadcast@gs if
needed - polkit asks for the password in a desktop dialog - and stops it again
on exit if it started it.

    tools/.venv/bin/python tools/vtx_dashboard.py            # everything
    tools/.venv/bin/python tools/vtx_dashboard.py --record   # and record from the start

Telemetry comes from UDP 14550. If vtx_ground_station.py is running it owns
that port (and the video), so this falls back to the copy the ground station
forwards to 14560 - but run one or the other, not both, or they also compete
for the video on UDP 5600.

Needs PyQt6, pyqtgraph and numpy in tools/.venv, which must be created with
--system-site-packages for the GStreamer bindings - see tools/requirements.txt -
and, on X11, the system package libxcb-cursor0 (sudo apt install libxcb-cursor0).
"""

import argparse
import collections
import csv
import errno
import math
import os
import socket
import sys
import time

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import QRectF, QSocketNotifier, Qt, QTimer
from PyQt6.QtGui import QColor, QFont, QImage, QPainter
from PyQt6.QtWidgets import (QAbstractItemView, QApplication, QFrame, QGridLayout, QHBoxLayout,
                             QHeaderView, QLabel, QMainWindow, QPushButton, QTableWidget,
                             QStatusBar, QTableWidgetItem, QTabWidget, QToolBar, QVBoxLayout,
                             QWidget)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vtx_mavlink import GPS_FIX, MavlinkStream, summary  # noqa: E402
import wfb_gs  # noqa: E402

TELEM_PORT = 14550
VIDEO_PORT = 5600             # wfb-ng [gs_video] -> RTP H.265
FORWARD_PORT = 14560          # vtx_ground_station.py --forward default
LINK_BAUD = 115200            # M33 -> VTX UART, for the link-use figure
WINDOW_S = 60.0               # strip chart span
GG_TRAIL_S = 10.0
STALE_S = 2.0                 # a value older than this shows as "-"
RATE_WINDOW_S = 2.0           # message rates in the table
TRACK_MAX = 20000
FIRMWARE = (1, 1)             # the M33's sysid/compid; mavfwd is 1/250, wfb 3/68

# The Telemetry Suite's palette.
BG, PANEL, CARD, BORDER = "#0d0f12", "#111317", "#1a1d24", "#2a2e39"
CYAN, GREEN, RED, YELLOW, MAGENTA = "#00e5ff", "#00e676", "#ff1744", "#ffea00", "#e040fb"
ORANGE, BLUE, PURPLE, GREY = "#ff9100", "#00b0ff", "#b388ff", "#8b949e"

PMAC_STATE = {0: ("NO DATA (Linux)", RED), 1: ("NO CAN", ORANGE), 2: ("LIVE", GREEN)}
NMT_NAME = {5: "Operational", 127: "Pre-op", 4: "Stopped", 0: "boot-up", 255: "no heartbeat"}
MAG_STATE = {0: ("unverified", GREY), 1: ("trusted", GREEN), 2: ("DROPPED", ORANGE)}


# ------------------------------------------------------------------ data ---

class Series:
    """A rolling (t, value) series for one strip chart curve."""

    def __init__(self, maxlen=6000):
        self.t = collections.deque(maxlen=maxlen)
        self.v = collections.deque(maxlen=maxlen)

    def add(self, t, v):
        self.t.append(t)
        self.v.append(np.nan if v is None else v)

    def window(self, now, span):
        if not self.t:
            return np.empty(0), np.empty(0)
        t = np.fromiter(self.t, float)
        v = np.fromiter(self.v, float)
        keep = t >= now - span
        return t[keep] - now, v[keep]

    def clear(self):
        self.t.clear()
        self.v.clear()


class LiveState:
    """Everything the dashboard knows, updated frame by frame."""

    SERIES = ("speed", "rpm", "torque", "ibat", "ax", "ay", "tmot", "ths", "roll", "pitch")

    def __init__(self):
        self.t0 = time.monotonic()
        self.stream = MavlinkStream()
        self.series = {k: Series() for k in self.SERIES}
        self.reset()

    def now(self):
        return time.monotonic() - self.t0

    def reset(self):
        for s in self.series.values():
            s.clear()
        self.latest = {}          # key -> (value, t)
        self.track = collections.deque(maxlen=TRACK_MAX)   # (lat, lon, speed)
        self.gg = collections.deque(maxlen=2000)           # (t, ay, ax)
        self.inspector = {}       # row key -> dict
        self.t_last_rx = None
        self.rate = {"frames": 0.0, "bytes": 0.0}
        self._rate_mark = (self.now(), self.stream.frames, self.stream.bytes)

    def get(self, key, stale=STALE_S):
        """Latest value, or None if never seen or older than `stale`."""
        item = self.latest.get(key)
        if item is None or self.now() - item[1] > stale:
            return None
        return item[0]

    def _set(self, key, value, t):
        self.latest[key] = (value, t)

    def feed(self, data):
        t = self.now()
        self.t_last_rx = t
        for fr in self.stream.feed(data):
            self._ingest(fr, t)

    def _ingest(self, fr, t):
        key = (fr.sysid, fr.compid, fr.msgid)
        name = fr.name
        if fr.msgid == 251 and fr.fields:
            key += (fr.fields["name"],)
            name = f"NAMED_VALUE_FLOAT  {fr.fields['name']}"
        row = self.inspector.setdefault(key, {"name": name, "src": f"{fr.sysid}/{fr.compid}",
                                              "count": 0, "t": t, "text": "",
                                              "times": collections.deque(maxlen=512)})
        # Frames counted over the last RATE_WINDOW_S, not 1/dt averaged: several
        # frames often share one datagram, and averaging 1/dt then reads high.
        row["times"].append(t)
        row["count"] += 1
        row["t"] = t
        row["text"] = summary(fr.msgid, fr.fields) if fr.verified else "(unverified)"

        d = fr.fields
        if d is None:
            return
        if fr.msgid == 109:                                     # wfb-ng
            self._set("rssi", d["rssi"], t)
            self._set("noise", d["noise"], t)
            return
        if (fr.sysid, fr.compid) != FIRMWARE:
            return                                              # mavfwd's temperature
        if fr.msgid == 0:
            self._set("heartbeat", True, t)
        elif fr.msgid == 30:
            for k in ("roll_deg", "pitch_deg", "yaw_deg", "yawspeed_dps"):
                self._set(k, d[k], t)
            self.series["roll"].add(t, d["roll_deg"])
            self.series["pitch"].add(t, d["pitch_deg"])
        elif fr.msgid == 27 and not d["all_zero"]:
            ax, ay, _ = d["acc_body_g"]
            self._set("ax_g", ax, t)
            self._set("ay_g", ay, t)
            self.series["ax"].add(t, ax)
            self.series["ay"].add(t, ay)
            self.gg.append((t, ay, ax))
        elif fr.msgid == 24:
            for k in ("lat", "lon", "alt_m", "speed_kmh", "cog_deg", "fix_type", "sats", "dop"):
                self._set(k, d[k], t)
            self.series["speed"].add(t, d["speed_kmh"])
            if d["fix_type"] >= 2:
                self.track.append((d["lat"], d["lon"], d["speed_kmh"] or 0.0))
        elif fr.msgid == 251:
            n, v = d["name"], d["value"]
            self._set(n, v, t)
            series = {"MOT_RPM": "rpm", "MOT_TRQ": "torque", "BAT_A": "ibat",
                      "MOT_TEMP": "tmot", "HS_TEMP": "ths"}.get(n)
            if series:
                self.series[series].add(t, v * 100.0 if n == "MOT_TRQ" else v)

    def update_rates(self):
        t, f, b = self.now(), self.stream.frames, self.stream.bytes
        t0, f0, b0 = self._rate_mark
        if t - t0 >= 1.0:
            self.rate = {"frames": (f - f0) / (t - t0), "bytes": (b - b0) / (t - t0)}
            self._rate_mark = (t, f, b)


# ---------------------------------------------------------------- record ---

class Recorder:
    """CSV in the Telemetry Suite's data contract (core/mock_source.py), plus
    a few extra columns it ignores. No lap_number: the suite then splits laps
    itself from the GPS track."""

    COLUMNS = ["timestamp_ms", "distance_m", "lat", "lon", "speed_kmh", "ax_g", "ay_g",
               "yaw_rate_dps", "motor_rpm", "throttle_pct", "brake_bar", "motor_temp_c",
               "battery_temp_c", "pack_voltage_v", "pack_current_a",
               # extras
               "roll_deg", "pitch_deg", "yaw_deg", "torque_pu", "heatsink_temp_c",
               "gps_fix", "gps_sats"]

    def __init__(self, path):
        self.path = path
        self.fh = open(path, "w", newline="")
        self.w = csv.writer(self.fh)
        self.w.writerow(self.COLUMNS)
        self.t_start = None
        self.t_prev = None
        self.dist = 0.0
        self.rows = 0

    def write(self, st):
        t = st.now()
        if self.t_start is None:
            self.t_start = self.t_prev = t
        spd = st.get("speed_kmh")
        if spd is not None:
            self.dist += spd / 3.6 * (t - self.t_prev)
        self.t_prev = t

        def g(key, scale=1.0, nd=4):
            v = st.get(key)
            return "" if v is None else round(v * scale, nd)

        fix = st.get("fix_type")
        has_pos = fix is not None and fix >= 2
        self.w.writerow([
            int((t - self.t_start) * 1000), round(self.dist, 2),
            g("lat", nd=7) if has_pos else "", g("lon", nd=7) if has_pos else "",
            g("speed_kmh"), g("ax_g"), g("ay_g"), g("yawspeed_dps"), g("MOT_RPM"),
            "", "", g("MOT_TEMP"), "", g("CAP_V"), g("BAT_A"),
            g("roll_deg"), g("pitch_deg"), g("yaw_deg"), g("MOT_TRQ"), g("HS_TEMP"),
            "" if fix is None else fix, g("sats")])
        self.rows += 1

    def close(self):
        self.fh.close()



# ----------------------------------------------------------------- video ---

class VideoPipeline:
    """The VTX video, decoded for a Qt widget.

    The receive chain is vtx_ground_station.py's, including its latency fix:
    a one-frame leaky queue after the decoder, so a display that falls behind
    drops old frames instead of queueing them into seconds of delay. The GPU
    decodes (vah265dec); videoconvertscale turns that into a 1280x720 BGRx
    frame on 4 CPU threads; an appsink that only ever holds the newest frame
    hands it to Qt. Software decode is the fallback when VA-API is not there.

    Measured on this laptop (Radeon 780M), 1080p60 in, why it is built so:
      - Not vapostproc for the scale: with its output in system memory this
        driver returns all-zero frames - a black or green picture at full
        frame rate, which no error reports.
      - BGRx is Qt's own QImage.Format_RGB32. RGBA made every paint convert
        and alpha-blend each pixel.
      - The conversion runs in GStreamer's threads, so Python only copies a
        finished frame: 0.4 ms of the UI thread per frame at 60 fps.

    Needs only the core GStreamer bindings (gi + Gst), not GstVideo: the
    window-handle route to embed a sink needs a typelib this machine lacks.
    """

    def __init__(self, port, width, height, codec="h265", decoder="auto", sockbuf=4_000_000):
        import gi
        gi.require_version("Gst", "1.0")
        from gi.repository import Gst
        Gst.init(None)
        self.Gst = Gst

        if codec == "h265":
            enc, depay, parse, hw, sw = "H265", "rtph265depay", "h265parse", "vah265dec", "avdec_h265"
        else:
            enc, depay, parse, hw, sw = "H264", "rtph264depay", "h264parse", "vah264dec", "avdec_h264"
        size = f"width={width},height={height},pixel-aspect-ratio=1/1"
        latest = "queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream"
        use_hw = decoder != "sw" and bool(Gst.ElementFactory.find(hw))
        # videoconvertscale (GStreamer 1.22+) converts and scales in one
        # multithreaded pass; older GStreamer gets the two-element version.
        if Gst.ElementFactory.find("videoconvertscale"):
            convert = "videoconvertscale n-threads=4"
        else:
            convert = "videoconvert n-threads=4 ! videoscale"
        dec = hw if use_hw else sw
        tail = f"{dec} ! {latest} ! {convert} ! video/x-raw,format=BGRx,{size}"
        self.decoder = f"{dec} ({'hardware' if use_hw else 'software'})"
        self._data = None

        desc = (f"udpsrc port={port} buffer-size={sockbuf} "
                f"caps=application/x-rtp,media=video,clock-rate=90000,encoding-name={enc} "
                f"! queue ! {depay} ! {parse} ! {tail} "
                f"! appsink name=sink drop=true max-buffers=1 sync=false")
        self.pipeline = Gst.parse_launch(desc)
        self.sink = self.pipeline.get_by_name("sink")
        self.bus = self.pipeline.get_bus()
        self.pipeline.set_state(Gst.State.PLAYING)

    def pull(self):
        """The newest decoded frame as a QImage, or None if none since last time."""
        sample = self.sink.emit("try-pull-sample", 0)
        if sample is None:
            return None
        buf = sample.get_buffer()
        st = sample.get_caps().get_structure(0)
        w, h = st.get_value("width"), st.get_value("height")
        ok, info = buf.map(self.Gst.MapFlags.READ)
        if not ok:
            return None
        try:
            data = bytes(info.data)
        finally:
            buf.unmap(info)
        # Row stride from the buffer itself: decoders may pad rows. The QImage
        # points into `data`, kept alive here until the next frame replaces
        # it - by which time the view has been handed that next frame.
        self._data = data
        return QImage(data, w, h, len(data) // h, QImage.Format.Format_RGB32)

    def errors(self):
        """GStreamer errors and warnings since the last call, as text."""
        out = []
        Gst = self.Gst
        while True:
            msg = self.bus.pop_filtered(Gst.MessageType.ERROR | Gst.MessageType.WARNING)
            if msg is None:
                return out
            if msg.type == Gst.MessageType.ERROR:
                out.append("video error: " + msg.parse_error()[0].message)
            else:
                out.append("video warning: " + msg.parse_warning()[0].message)

    def stop(self):
        self.pipeline.set_state(self.Gst.State.NULL)


class VideoView(QWidget):
    """Paints the latest frame, letterboxed, with a small status overlay."""

    def __init__(self):
        super().__init__()
        self.setMinimumSize(320, 180)
        self.image = None
        self.t_frame = None
        self.fps = 0.0
        self._count = 0
        self._t_count = time.monotonic()
        self.status = "waiting for video on UDP 5600..."

    def set_frame(self, img):
        self.image = img
        self.t_frame = time.monotonic()
        self._count += 1
        self.update()

    def tick(self):
        now = time.monotonic()
        if now - self._t_count >= 1.0:
            self.fps = self._count / (now - self._t_count)
            self._count, self._t_count = 0, now
            self.update()

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.fillRect(self.rect(), QColor("#000000"))
        stale = self.t_frame is None or time.monotonic() - self.t_frame > STALE_S
        if self.image is not None:
            iw, ih = self.image.width(), self.image.height()
            scale = min(self.width() / iw, self.height() / ih)
            w, h = iw * scale, ih * scale
            target = QRectF((self.width() - w) / 2, (self.height() - h) / 2, w, h)
            p.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform)
            p.drawImage(target, self.image)
        p.setFont(QFont("monospace", 10))
        if stale:
            p.setPen(QColor(RED if self.image is not None else GREY))
            text = ("NO VIDEO - last frame frozen" if self.image is not None else self.status)
            p.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, text)
        p.setPen(QColor(CYAN))
        label = f"{self.fps:4.1f} fps   {self.status}"
        box = p.fontMetrics().boundingRect(label).adjusted(-6, -3, 6, 3).translated(12, 22)
        p.fillRect(box, QColor(0, 0, 0, 160))     # stays readable over a bright picture
        p.drawText(12, 22, label)
        p.end()

# ------------------------------------------------------------------- UI ---

def _plot_style(p, label, units=None):
    p.setLabel("left", label, units=units)
    # Otherwise pyqtgraph reads "g" as grams and shows "mg", and rpm as "krpm".
    p.getAxis("left").enableAutoSIPrefix(False)
    p.showGrid(x=True, y=True, alpha=0.3)
    p.setXRange(-WINDOW_S, 0, padding=0)
    p.setMouseEnabled(x=False, y=True)


class LiveStripCharts(QWidget):
    """The suite's strip charts, against time over the last WINDOW_S."""

    ROWS = [  # (title, units, [(series, colour, legend)])
        ("Speed", "km/h", [("speed", CYAN, "GPS")]),
        ("Motor", "rpm", [("rpm", "#ffffff", "RPM")]),
        ("Torque / Current", None, [("torque", GREEN, "Torque %"), ("ibat", RED, "I batt A")]),
        ("G-Force", "g", [("ax", YELLOW, "Ax (long)"), ("ay", MAGENTA, "Ay (lat)")]),
        ("Temp", "°C", [("tmot", ORANGE, "Motor"), ("ths", BLUE, "Heatsink")]),
        ("Attitude", "°", [("roll", PURPLE, "Roll"), ("pitch", "#80cbc4", "Pitch")]),
    ]

    def __init__(self):
        super().__init__()
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        self.win = pg.GraphicsLayoutWidget()
        self.win.setBackground(PANEL)
        lay.addWidget(self.win)
        self.curves = []
        first = None
        for r, (title, units, curves) in enumerate(self.ROWS):
            p = self.win.addPlot(row=r, col=0)
            _plot_style(p, title, units)
            if first is None:
                first = p
            else:
                p.setXLink(first)
            if len(curves) > 1:
                p.addLegend(offset=(6, 4), labelTextSize="8pt")
            for key, colour, legend in curves:
                c = p.plot(pen=pg.mkPen(colour, width=2), name=legend, connect="finite")
                self.curves.append((key, c))
            if r == len(self.ROWS) - 1:
                p.setLabel("bottom", "Time", units="s")

    def refresh(self, st):
        now = st.now()
        for key, curve in self.curves:
            t, v = st.series[key].window(now, WINDOW_S)
            curve.setData(t, v)


class LiveTrackMap(QWidget):
    """GPS trail coloured by speed, with the car's position."""

    def __init__(self):
        super().__init__()
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        self.plot = pg.PlotWidget(title="GPS track (colour = speed)")
        self.plot.setBackground(PANEL)
        self.plot.showGrid(x=True, y=True, alpha=0.2)
        self.plot.setAspectLocked(True)
        # Metres from the first fix of the session: x east, y north, equal scale.
        self.plot.setLabel("bottom", "East of first fix", units="m")
        self.plot.setLabel("left", "North of first fix", units="m")
        for ax in ("bottom", "left"):
            self.plot.getAxis(ax).enableAutoSIPrefix(False)
        lay.addWidget(self.plot)
        self.trail = pg.ScatterPlotItem(size=5, pen=None)
        self.car = pg.ScatterPlotItem(size=14, pen=pg.mkPen("w", width=2),
                                      brush=pg.mkBrush(255, 30, 30, 255))
        self.plot.addItem(self.trail)
        self.plot.addItem(self.car)
        self.waiting = pg.TextItem("waiting for a GPS fix...", color=GREY, anchor=(0.5, 0.5))
        self.plot.addItem(self.waiting)
        self.cmap = pg.colormap.get("turbo")
        self.origin = None
        self.n_drawn = -1

    def reset(self):
        self.origin = None
        self.n_drawn = -1
        self.trail.clear()
        self.car.clear()
        self.waiting.setVisible(True)

    def refresh(self, st):
        n = len(st.track)
        if n == 0 or n == self.n_drawn:
            return
        self.n_drawn = n
        self.waiting.setVisible(False)
        arr = np.array(st.track, dtype=float)
        if self.origin is None:
            self.origin = (arr[0, 0], arr[0, 1])
        lat0, lon0 = self.origin
        x = (arr[:, 1] - lon0) * 111139.0 * math.cos(math.radians(lat0))
        y = (arr[:, 0] - lat0) * 111139.0
        spd = arr[:, 2]
        top = max(float(spd.max()), 10.0)
        brushes = self.cmap.map(np.clip(spd / top, 0.0, 1.0), mode="qcolor")
        self.trail.setData(x=x, y=y, brush=brushes)
        self.car.setData([x[-1]], [y[-1]])


class LiveGGDiagram(QWidget):
    """The suite's G-G diagram: last GG_TRAIL_S of lateral vs longitudinal."""

    def __init__(self):
        super().__init__()
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        self.plot = pg.PlotWidget(title=f"G-G diagram (last {GG_TRAIL_S:.0f} s)")
        self.plot.setBackground(PANEL)
        self.plot.setAspectLocked(True)
        self.plot.setLabel("bottom", "Lateral (Ay)", units="g")
        self.plot.setLabel("left", "Longitudinal (Ax)", units="g")
        for ax in ("bottom", "left"):
            self.plot.getAxis(ax).enableAutoSIPrefix(False)
        self.plot.setXRange(-1.6, 1.6)
        self.plot.setYRange(-1.6, 1.6)
        lay.addWidget(self.plot)
        ang = np.linspace(0, 2 * np.pi, 120)
        for g in (0.5, 1.0, 1.5):
            self.plot.plot(g * np.cos(ang), g * np.sin(ang),
                           pen=pg.mkPen((80, 80, 80, 120), width=1, style=Qt.PenStyle.DashLine))
        self.plot.plot([-1.6, 1.6], [0, 0], pen=pg.mkPen((100, 100, 100, 150)))
        self.plot.plot([0, 0], [-1.6, 1.6], pen=pg.mkPen((100, 100, 100, 150)))
        self.trail = pg.ScatterPlotItem(size=4, brush=pg.mkBrush(0, 180, 255, 70), pen=None)
        self.dot = pg.ScatterPlotItem(size=14, brush=pg.mkBrush(255, 220, 0, 255),
                                      pen=pg.mkPen("w", width=2))
        self.plot.addItem(self.trail)
        self.plot.addItem(self.dot)

    def refresh(self, st):
        now = st.now()
        pts = [(ay, ax) for t, ay, ax in st.gg if t >= now - GG_TRAIL_S]
        if not pts:
            self.trail.clear()
            self.dot.clear()
            return
        a = np.array(pts)
        self.trail.setData(a[:, 0], a[:, 1])
        self.dot.setData([a[-1, 0]], [a[-1, 1]])


class GaugeCards(QWidget):
    """The suite's gauges panel, as live cards."""

    FIELDS = [  # key, title, colour
        ("speed", "GPS speed", CYAN), ("rpm", "Motor RPM", "#ffffff"),
        ("torque", "Torque", GREEN), ("ibat", "Battery current", RED),
        ("vbus", "Bus voltage", PURPLE), ("power", "Power", "#ffd740"),
        ("tmot", "Motor temp", ORANGE), ("ths", "Heatsink temp", BLUE),
        ("attitude", "Roll / Pitch", PURPLE), ("heading", "Heading", "#80cbc4"),
        ("gps", "GPS", CYAN), ("pmac", "Motor controller", GREEN),
        ("fault", "Fault", RED), ("mag", "Magnetometer", GREY),
        ("radio", "Radio RSSI", "#ffffff"), ("link", "Link", GREY),
    ]

    def __init__(self):
        super().__init__()
        self.setStyleSheet(f"""
            QFrame#Card {{ background-color: {CARD}; border-radius: 8px; border: 1px solid {BORDER}; }}
            QLabel#Title {{ font-size: 10px; color: {GREY}; }}
            QLabel#Value {{ font-size: 18px; font-weight: bold; }}
        """)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)
        card = QFrame()
        card.setObjectName("Card")
        grid = QGridLayout(card)
        grid.setSpacing(10)
        self.labels = {}
        for i, (key, title, colour) in enumerate(self.FIELDS):
            box = QVBoxLayout()
            box.setSpacing(1)
            t = QLabel(title.upper())
            t.setObjectName("Title")
            v = QLabel("-")
            v.setObjectName("Value")
            v.setStyleSheet(f"color: {colour};")
            box.addWidget(t)
            box.addWidget(v)
            grid.addLayout(box, i // 4, i % 4)
            self.labels[key] = (v, colour)
        lay.addWidget(card)

    def _put(self, key, text, colour=None):
        lbl, default = self.labels[key]
        lbl.setText(text)
        lbl.setStyleSheet(f"color: {colour or default};")

    def refresh(self, st):
        def fmt(key, f, scale=1.0):
            v = st.get(key)
            return "-" if v is None else f.format(v * scale)

        self._put("speed", fmt("speed_kmh", "{:.1f} km/h"))
        self._put("rpm", fmt("MOT_RPM", "{:.0f}"))
        self._put("torque", fmt("MOT_TRQ", "{:+.0f} %", 100.0))
        self._put("ibat", fmt("BAT_A", "{:+.1f} A"))
        self._put("vbus", fmt("CAP_V", "{:.1f} V"))
        v, i = st.get("CAP_V"), st.get("BAT_A")
        self._put("power", "-" if v is None or i is None else f"{v * i / 1000.0:+.2f} kW")
        self._put("tmot", fmt("MOT_TEMP", "{:.0f} °C"))
        self._put("ths", fmt("HS_TEMP", "{:.0f} °C"))
        r, p = st.get("roll_deg"), st.get("pitch_deg")
        self._put("attitude", "-" if r is None else f"{r:+.1f}° / {p:+.1f}°")
        yaw = st.get("yaw_deg")
        self._put("heading", "-" if yaw is None else f"{yaw % 360.0:.0f}°")

        fix, sats = st.get("fix_type"), st.get("sats")
        if fix is None:
            self._put("gps", "no data", RED)
        else:
            self._put("gps", f"{GPS_FIX.get(fix, fix)} · {sats} sat",
                      GREEN if fix >= 3 else (ORANGE if fix == 2 else RED))

        link = st.get("PMAC_LINK", stale=3.0)
        if link is None:
            self._put("pmac", "no status", GREY)
        else:
            text, colour = PMAC_STATE.get(int(link), (f"#{link:.0f}", GREY))
            nmt = st.get("PMAC_NMT", stale=3.0)
            if int(link) == 2 and nmt is not None:
                text += f" · {NMT_NAME.get(int(nmt), int(nmt))}"
            self._put("pmac", text, colour)
        flt = st.get("PMAC_FLT", stale=3.0)
        self._put("fault", "-" if flt is None else ("none" if int(flt) == 0 else f"0x{int(flt):04X}"),
                  GREEN if flt is not None and int(flt) == 0 else None)

        mag = st.get("MAG_STATE", stale=3.0)
        if mag is None:
            self._put("mag", "-")
        else:
            text, colour = MAG_STATE.get(int(mag), (f"#{mag:.0f}", GREY))
            err = st.get("MAG_ERR", stale=3.0)
            if err is not None and not math.isnan(err):
                text += f" · {err:+.0f}°"
            self._put("mag", text, colour)

        rssi = st.get("rssi", stale=3.0)
        self._put("radio", "-" if rssi is None else f"{rssi} (noise {st.get('noise', 3.0)})")

        age = None if st.t_last_rx is None else st.now() - st.t_last_rx
        if age is None:
            self._put("link", "waiting...", GREY)
        elif age > STALE_S:
            self._put("link", f"NO DATA {age:.0f} s", RED)
        else:
            self._put("link", f"{st.rate['frames']:.0f} msg/s", GREEN)


class MessagesTab(QWidget):
    """Every message type received: who sent it, how often, what it said."""

    HEAD = ["Message", "Source", "Received", "Hz", "Last value", "Age"]

    def __init__(self):
        super().__init__()
        lay = QVBoxLayout(self)
        lay.setContentsMargins(10, 10, 10, 10)
        self.totals = QLabel("")
        self.totals.setStyleSheet(f"color: {GREY}; font-size: 12px; padding: 4px;")
        lay.addWidget(self.totals)
        self.table = QTableWidget(0, len(self.HEAD))
        self.table.setHorizontalHeaderLabels(self.HEAD)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection)
        hh = self.table.horizontalHeader()
        for c in range(len(self.HEAD)):
            hh.setSectionResizeMode(c, QHeaderView.ResizeMode.ResizeToContents)
        hh.setSectionResizeMode(4, QHeaderView.ResizeMode.Stretch)
        self.table.setStyleSheet(f"""
            QTableWidget {{ background-color: {PANEL}; color: #e0e0e0; gridline-color: {BORDER};
                            font-family: monospace; font-size: 12px; }}
            QHeaderView::section {{ background-color: {CARD}; color: {CYAN}; font-weight: bold;
                                    padding: 4px; border: 1px solid {BORDER}; }}
        """)
        lay.addWidget(self.table)

    def refresh(self, st):
        s = st.stream
        lost_pct = 100.0 * s.lost / max(s.frames + s.lost, 1)
        use = 100.0 * st.rate["bytes"] / (LINK_BAUD / 10.0)
        self.totals.setText(
            f"Frames {s.frames}   ·   {st.rate['frames']:.0f} msg/s   ·   "
            f"{st.rate['bytes'] / 1000.0:.2f} kB/s (~{use:.0f}% of {LINK_BAUD} baud)   ·   "
            f"lost {s.lost} ({lost_pct:.2f}%)   ·   bad CRC {s.crc_errors}   ·   "
            f"unparsed bytes {s.unparsed}")
        rows = sorted(st.inspector.items(), key=lambda kv: (kv[1]["src"], kv[1]["name"]))
        self.table.setRowCount(len(rows))
        now = st.now()
        for r, (_, d) in enumerate(rows):
            age = now - d["t"]
            recent = sum(1 for x in d["times"] if x >= now - RATE_WINDOW_S)
            vals = [d["name"], d["src"], str(d["count"]),
                    f"{recent / RATE_WINDOW_S:.1f}", d["text"], f"{age:.1f} s"]
            for c, v in enumerate(vals):
                item = self.table.item(r, c)
                if item is None:
                    item = QTableWidgetItem()
                    self.table.setItem(r, c, item)
                item.setText(v)
                item.setForeground(pg.mkColor(RED if age > STALE_S else "#e0e0e0"))


class MainWindow(QMainWindow):
    def __init__(self, sock, source_text, args, notes):
        super().__init__()
        self.setWindowTitle("BE KEY RACING TEAM - Live Telemetry (VTX) | ITESO")
        self.resize(1650, 960)
        self.setStyleSheet(f"background-color: {BG}; color: #ffffff;")
        self.state = LiveState()
        self.sock = sock
        self.args = args
        self.recorder = None
        self.notes = notes
        self.video = None

        tb = QToolBar("Toolbar")
        tb.setMovable(False)
        tb.setStyleSheet(f"background-color: {CARD}; border-bottom: 2px solid {CYAN}; padding: 6px;")
        self.addToolBar(tb)
        team = QLabel("⚡ BE KEY | ITESO RACING TEAM  ")
        team.setStyleSheet(f"font-weight: bold; font-size: 13px; color: {CYAN}; letter-spacing: 1px;")
        tb.addWidget(team)
        src = QLabel(f"  📡 {source_text}  ")
        src.setStyleSheet(f"color: {GREY}; font-size: 12px;")
        tb.addWidget(src)
        self.btn_rec = QPushButton("⏺ Record CSV")
        self.btn_rec.setCheckable(True)
        self.btn_rec.clicked.connect(self._toggle_record)
        self._style_rec(False)
        tb.addWidget(self.btn_rec)
        btn_clear = QPushButton("🧹 Clear")
        btn_clear.setStyleSheet("background-color: #37474f; color: #e0e0e0; padding: 6px 12px;"
                                " margin-left: 8px; border-radius: 4px;")
        btn_clear.clicked.connect(self._clear)
        tb.addWidget(btn_clear)
        self.lbl_status = QLabel("")
        self.lbl_status.setStyleSheet(f"font-weight: bold; color: {GREEN}; margin-left: 14px; font-size: 12px;")
        tb.addWidget(self.lbl_status)

        self.tabs = QTabWidget()
        self.tabs.setStyleSheet(f"""
            QTabBar::tab {{ background: {CARD}; color: {GREY}; padding: 8px 24px; font-weight: bold;
                           border-top-left-radius: 6px; border-top-right-radius: 6px; margin-right: 4px; }}
            QTabBar::tab:selected {{ background: {CYAN}; color: #000; }}
        """)
        self.setCentralWidget(self.tabs)

        live = QWidget()
        h = QHBoxLayout(live)
        h.setContentsMargins(10, 10, 10, 10)
        h.setSpacing(10)
        self.strips = LiveStripCharts()
        h.addWidget(self.strips, stretch=6)
        right = QVBoxLayout()
        top = QHBoxLayout()
        self.map = LiveTrackMap()
        self.gg = LiveGGDiagram()
        top.addWidget(self.map, stretch=1)
        top.addWidget(self.gg, stretch=1)
        right.addLayout(top, stretch=1)
        self.cards = GaugeCards()
        right.addWidget(self.cards, stretch=1)
        h.addLayout(right, stretch=4)
        self.messages = MessagesTab()
        self.video_view = VideoView()
        self.tabs.addTab(live, "📡 Live")
        self.tabs.addTab(self.video_view, "🎥 Video")
        self.tabs.addTab(self.messages, "📨 Messages")

        # What the ground station's terminal pane used to say, in one line.
        self.status_bar = QStatusBar()
        self.status_bar.setStyleSheet(f"background-color: {CARD}; color: {GREY}; font-size: 11px;")
        self.setStatusBar(self.status_bar)
        self.lbl_system = QLabel("")
        self.status_bar.addWidget(self.lbl_system, 1)
        self.video_note = ""

        if args.video:
            try:
                w, h = (int(v) for v in args.video_size.lower().split("x"))
                self.video = VideoPipeline(args.video_port, w, h, args.codec, args.decoder)
                self.video_view.status = f"{self.video.decoder}, UDP {args.video_port}"
                self.video_note = f"video {self.video.decoder}"
            except Exception as exc:          # no GStreamer bindings, bad pipeline...
                self.video_view.status = f"video unavailable: {exc}"
                self.video_note = "video unavailable"
        else:
            self.video_view.status = "video off (--no-video)"
            self.video_note = "video off"
        self._update_system()

        self.notifier = QSocketNotifier(sock.fileno(), QSocketNotifier.Type.Read, self)
        self.notifier.activated.connect(self._on_readable)
        self._timer(50, self._refresh_fast)      # charts, gauges: 20 Hz
        self._timer(250, self._refresh_slow)     # map, table: 4 Hz
        self._timer(15, self._refresh_video)     # newest video frame, ~60 Hz
        self._timer(5000, self._update_system)   # wfb-ng service state
        self.rec_timer = self._timer(20, self._record_row, start=False)   # 50 Hz rows
        if args.record:
            self.btn_rec.setChecked(True)
            self._toggle_record(True)

    def _timer(self, ms, fn, start=True):
        t = QTimer(self)
        t.timeout.connect(fn)
        if start:
            t.start(ms)
        else:
            t.setInterval(ms)
        return t

    def _on_readable(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
            except BlockingIOError:
                break
            self.state.feed(data)

    def _refresh_fast(self):
        self.state.update_rates()
        self.strips.refresh(self.state)
        self.gg.refresh(self.state)
        self.cards.refresh(self.state)
        if self.recorder:
            self.lbl_status.setText(f"⏺ {os.path.basename(self.recorder.path)}  ·  "
                                    f"{self.recorder.rows} rows  ·  {self.recorder.dist:.0f} m")

    def _refresh_slow(self):
        self.map.refresh(self.state)
        self.messages.refresh(self.state)

    def _refresh_video(self):
        if self.video is None:
            return
        # Only convert frames while the tab is on screen; the appsink keeps
        # just the newest one, so nothing piles up while it is hidden.
        if self.video_view.isVisible():
            img = self.video.pull()
            if img is not None:
                self.video_view.set_frame(img)
            self.video_view.tick()
        for err in self.video.errors():
            self.video_view.status = err
            print(f"vtx_dashboard: {err}")

    def _update_system(self):
        parts = []
        if not self.args.no_gs:
            up = wfb_gs.gs_active()
            parts.append((f"{wfb_gs.GS_UNIT} {'active' if up else 'NOT RUNNING'}", GREEN if up else RED))
        for level, text in self.notes:
            parts.append((text, GREEN if level == "ok" else ORANGE))
        parts.append((self.video_note, GREY))
        self.lbl_system.setText("   ·   ".join(f'<span style="color:{c}">{t}</span>' for t, c in parts))

    def _style_rec(self, on):
        colour = "#b71c1c" if on else "#2979ff"
        self.btn_rec.setText("⏹ Stop recording" if on else "⏺ Record CSV")
        self.btn_rec.setStyleSheet(f"background-color: {colour}; color: white; padding: 6px 14px;"
                                   " font-weight: bold; border-radius: 4px; margin-left: 8px;")

    def _toggle_record(self, on):
        if on:
            os.makedirs(self.args.log_dir, exist_ok=True)
            path = os.path.join(self.args.log_dir, time.strftime("telemetry_%Y%m%d_%H%M%S.csv"))
            self.recorder = Recorder(path)
            self.rec_timer.start()
        else:
            self.rec_timer.stop()
            if self.recorder:
                self.recorder.close()
                self.lbl_status.setText(f"✔ saved {os.path.basename(self.recorder.path)}"
                                        f" ({self.recorder.rows} rows)")
                print(f"vtx_dashboard: saved {self.recorder.path}")
                self.recorder = None
        self._style_rec(on)

    def _record_row(self):
        if self.recorder:
            self.recorder.write(self.state)

    def _clear(self):
        self.state.reset()
        self.map.reset()

    def closeEvent(self, ev):
        if self.recorder:
            self.recorder.close()
            print(f"vtx_dashboard: saved {self.recorder.path}")
            self.recorder = None
        if self.video is not None:
            self.video.stop()
        super().closeEvent(ev)


# ------------------------------------------------------------------ main ---

def open_socket(port):
    """Bind the telemetry port. Deliberately without SO_REUSEADDR: with it on
    both sides, Linux lets a second program bind 14550 and silently steals the
    datagrams from the first. Without it, a busy port fails cleanly."""
    def bind(host, p):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind((host, p))
        s.setblocking(False)
        return s

    if port:
        return bind("0.0.0.0", port), f"UDP {port}"
    try:
        return bind("0.0.0.0", TELEM_PORT), f"UDP {TELEM_PORT} (direct from wfb-ng)"
    except OSError as exc:
        if exc.errno != errno.EADDRINUSE:
            raise
    return (bind("127.0.0.1", FORWARD_PORT),
            f"UDP {FORWARD_PORT} (copy from vtx_ground_station; {TELEM_PORT} is in use)")


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=0,
                    help=f"telemetry UDP port (default: {TELEM_PORT}, or {FORWARD_PORT} "
                         f"if vtx_ground_station.py already has {TELEM_PORT})")
    ap.add_argument("--no-video", dest="video", action="store_false", help="telemetry only")
    ap.add_argument("--video-port", type=int, default=VIDEO_PORT)
    ap.add_argument("--video-size", default="1280x720", metavar="WxH",
                    help="size the GPU scales the video to before display (default 1280x720)")
    ap.add_argument("--codec", choices=("h265", "h264"), default="h265",
                    help="must match the VTX's majestic.yaml (default h265)")
    ap.add_argument("--decoder", choices=("auto", "hw", "sw"), default="auto",
                    help="auto uses VA-API when the GPU has it, else software")
    ap.add_argument("--no-gs", action="store_true",
                    help=f"do not start or stop {wfb_gs.GS_UNIT}")
    ap.add_argument("--keep-gs", action="store_true",
                    help=f"leave {wfb_gs.GS_UNIT} running on exit if this started it")
    ap.add_argument("--record", action="store_true", help="start recording CSV immediately")
    ap.add_argument("--log-dir", default=os.path.join(here, "logs"),
                    help="where CSV recordings go (default tools/logs)")
    args = ap.parse_args()

    notes = [] if args.no_gs else wfb_gs.preflight()
    started_gs = False if args.no_gs else wfb_gs.gs_start(log=lambda m: print(f"vtx_dashboard: {m}"))

    try:
        sock, source = open_socket(args.port)
    except OSError as exc:
        sys.exit(f"cannot open the telemetry port: {exc}")
    print(f"vtx_dashboard: telemetry on {source}")

    pg.setConfigOptions(antialias=True)
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = MainWindow(sock, source, args, notes)
    win.show()
    rc = app.exec()
    if started_gs and not args.keep_gs:
        wfb_gs.gs_stop(log=lambda m: print(f"vtx_dashboard: {m}"))
    sys.exit(rc)


if __name__ == "__main__":
    main()
