"""Board layout for the IRT steering wheel controller.

    python tools/layout_steering.py place     # placement + fan-out -> build/placed.kicad_pcb + DSN
    (Freerouting)                              # build/board.dsn -> build/board.ses
    python tools/layout_steering.py finish    # import SES -> steering_wheel.kicad_pcb

Board: outline and mounting holes from docs/PCB_Volante_2026.DXF (about 139 x 84 mm,
mirror-symmetric), 4 layers (JLC04161H-7628): F.Cu signals, In1 GND plane, In2 +3V3 plane,
B.Cu signals. All parts on the top side.

Floorplan (board-relative mm, y down):
  * LCD module (108 x 60.88) centred on the mirror axis, parts underneath (<= 8 mm).
  * RF front end, RW612, flash and PSRAM: the hand-tuned block of the previous 160 x 90
    layout, moved as a rigid group (CORE offset) so U.FL J401 sits at the top-centre edge.
  * Power: the previous floorplan moved as a rigid group (PWR offset) so USB-C J101 sits in
    the bump at the bottom centre; battery J103 and power switch SW101 moved to the edge.
  * Encoder / button JST connectors in the side wings, outside the LCD, so cables can be
    plugged with the display fitted; LCD cable (J501), SWD, UART, RESET/ISP in the strip
    above it; NeoPixel output at the top of the right wing.
"""
import json
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(__file__))
from pcbgen import Board, Footprint, FootprintLib, rot_pt, uid, f, stackup_jlc08161h_7628  # noqa
from netinfo import load  # noqa: E402
from geom import Shape, SpatialIndex, distance  # noqa: E402
from sexp import QStr  # noqa: E402
from dxf_outline import load_outline  # noqa: E402
import board_rules  # noqa: E402
import fanout  # noqa: E402
import silk  # noqa: E402
import escape  # noqa: E402

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BUILD = os.path.join(HERE, "build")
OX, OY = 40.0, 30.0          # board origin on the page
DXF = os.path.join(HERE, "docs", "PCB_Volante_2026.DXF")
OUT = load_outline(DXF)      # board-relative outline, holes, mirror axis
W, H = OUT.width, OUT.height  # bounding box (138.9 x 83.8)
AXIS = OUT.axis_x             # mirror axis, x = 69.45
# LCDwiki MSP4031 module, 108 x 60.88, centred on the axis; its bottom edge sits 0.5 mm
# inside the lowest point of the bottom edge, which leaves a ~10 mm strip above it
LCD_W, LCD_H = 108.0, 60.88
LCD = (AXIS - LCD_W / 2, 12.8, AXIS + LCD_W / 2, 12.8 + LCD_H)
# rigid moves of the hand-placed blocks from the old 160 x 90 layout (old -> new coords)
CORE = (AXIS - 78.2, 0.3)     # U.FL J401 (old x 78.2) onto the axis, top edge
PWR = (AXIS - 62.0, 75.3 - 4.15 - 85.85)   # USB-C mouth (old y 85.85 + 4.15) onto the bump
HOLE_KEEPOUT = 3.5            # radius kept free around the mounting holes (screw head)
COPPER = ["F.Cu", "In1.Cu", "In2.Cu", "In3.Cu", "In4.Cu", "In5.Cu", "In6.Cu", "B.Cu"]
# 8-layer JLC08161H-7628: 4 routing layers, each against a ground plane.
SIGNAL_LAYERS = ["F.Cu", "In2.Cu", "In5.Cu", "B.Cu"]
PLANE_LAYERS = ["In1.Cu", "In3.Cu", "In4.Cu", "In6.Cu"]
# net -> the plane layers it owns. Only membership is used elsewhere, but the layer
# lists drive add_zones().
PLANE_NETS = {"GND": ["In1.Cu", "In4.Cu", "In6.Cu"], "+3V3": ["In3.Cu"]}
# the escape may leave the ball field on any inner signal layer
ESCAPE_LAYERS = ("In2.Cu", "In5.Cu")

# design rules (mm) -- JLCPCB 4-layer capability with margin
RULES = dict(clearance=0.1, track=0.127, via_d=0.4, via_drill=0.2, bga_via_d=0.35, bga_via_drill=0.2,
             edge=0.3)
BGA_VIA = (RULES["bga_via_d"], RULES["bga_via_drill"])
NETCLASS = {   # name: (track width, clearance, via d, via drill, [nets])
    "Power": (0.5, 0.2, 0.6, 0.3, ["VBUS", "VSYS", "VSYS_SW", "VBAT", "+5V", "/power/BAT_N",
                                   "LCD_VCC", "/power/BST_SW", "/power/BB_L1", "/power/BB_L2",
                                   "/hmi/VCC_BTN"]),
    "CorePower": (0.25, 0.1, 0.4, 0.2, ["+1V8", "VCORE", "/mcu_core/BUCK18_SW",
                                        "/mcu_core/BUCK11_SW", "/mcu_core/AVDD18_J15"]),
    "USB": (0.2, 0.15, 0.4, 0.2, ["USB_D_P", "USB_D_N"]),
}
RF_NETS = {"/rf/RF5G_SOC", "/rf/RF5G_FLT", "/rf/RF2G_SOC", "/rf/RF2G_FLT", "/rf/BLE_SOC",
           "/rf/BLE_FLT", "/rf/RFSW_OUT1", "/rf/RFSW_OUT2", "/rf/RFSW_IN", "/rf/DPX_LB",
           "/rf/ANT_COM", "/rf/ANT_FEED"}
RF_W = 0.34          # 50 ohm microstrip on 0.2104 mm 7628 prepreg over In1 GND
                     # (JLC08161H-7628 keeps the same F.Cu -> In1.Cu prepreg as the
                     #  4-layer JLC04161H-7628, so this width carries over: ~51.7 ohm)
ESCAPE_ARGS = dict(time_limit=1200, margin=3)
ESCAPE_LOCKED = False     # True: hand every escape to the autorouter as fixed wiring
# True: fix the inner-layer (In2/In5) escapes and leave only the B.Cu ones movable. This is
# what the 4-layer board did -- it had to, since Freerouting did not route In2 at all -- and
# it stops the autorouter unpicking the exactly-solved escape inside the ball field.
ESCAPE_LOCK_INNER = True


def A(x, y):
    """board-relative -> absolute page coordinates"""
    return OX + x, OY + y


_OFF = [0.0, 0.0]


class shift:
    """with shift(CORE): coordinates given to P / T / V / poly are those of the old layout,
    moved by the block offset."""

    def __init__(self, d):
        self.d = d

    def __enter__(self):
        self.prev = tuple(_OFF)
        _OFF[0], _OFF[1] = self.d

    def __exit__(self, *a):
        _OFF[0], _OFF[1] = self.prev


def S(x, y):
    return x + _OFF[0], y + _OFF[1]


def in_core(x, y):
    """old-layout coordinates of the core block -> new board-relative"""
    return x + CORE[0], y + CORE[1]


def edge_bottom(x):
    """y of the bottom board edge at board-relative x"""
    from shapely.geometry import LineString, Polygon
    g = LineString([(x, -1.0), (x, H + 1.0)]).intersection(Polygon(OUT.polygon))
    return g.bounds[3]


class Layout:
    def __init__(self):
        self.comps, self.nets, self.pin_net = load()
        dirs = {d[:-7]: os.path.join(HERE, "lib", "kicad10_fp", d)
                for d in os.listdir(os.path.join(HERE, "lib", "kicad10_fp"))}
        dirs["steering_wheel"] = os.path.join(HERE, "lib", "steering_wheel.pretty")
        self.lib = FootprintLib(dirs)
        self.b = Board("IRT steering wheel controller (RW612)")
        for ref, c in sorted(self.comps.items()):
            fp = Footprint(ref, c["footprint"], self.lib.load(c["footprint"]), c["value"])
            fp.path, fp.sheetname, fp.sheetfile = c["path"], c["sheetname"], c["sheetfile"]
            fp.dnp = c["dnp"]
            for k in ("Datasheet", "Description", "MPN", "Manufacturer", "LCSC"):
                if c["props"].get(k):
                    fp.props[k] = c["props"][k]
            for p in fp.pads:
                n = self.pin_net.get((ref, p.num))
                if n:
                    fp.pad_net[p.num] = n
            fp.placed = False
            self.b.add_fp(fp)
        # KiCad names every unconnected pin's net "unconnected-(REF-PIN-PadN)": give the
        # pads the same names so DRC's schematic-parity check matches (netlist exported by
        # kicad-cli from the schematic, see README)
        unc = os.path.join(BUILD, "kicad_netlist.net")
        if os.path.exists(unc):
            from sexp import parse as _parse, find as _find, find_all as _find_all
            doc = _parse(open(unc).read())
            for n in _find_all(_find(doc, "nets"), "net"):
                name = str(_find(n, "name")[1])
                if not name.startswith("unconnected-("):
                    continue
                for nd in _find_all(n, "node"):
                    ref, pin = str(_find(nd, "ref")[1]), str(_find(nd, "pin")[1])
                    fp = self.b.fps.get(ref)
                    if fp is not None:
                        for p in fp.pads_by_num(pin):
                            if p.num not in fp.pad_net:
                                # written to the file only: the routing tools keep
                                # treating these pads as unconnected
                                fp.pad_nc = getattr(fp, "pad_nc", {})
                                fp.pad_nc[p.num] = name
                                self.b.net(name)
        for n in self.nets:
            self.b.net(n)
        self.keepouts = []    # (x0,y0,x1,y1) board-relative areas closed to auto-placement
        self.obst = []
        self.obst_idx = SpatialIndex(1.0)
        # auto-placed courtyards stay 0.5 mm inside the outline and off the screw heads
        from shapely.geometry import Point, Polygon
        from shapely.prepared import prep
        region = Polygon([A(*p) for p in OUT.polygon]).buffer(-0.5)
        for hx, hy, _ in OUT.holes:
            region = region.difference(Point(*A(hx, hy)).buffer(HOLE_KEEPOUT))
        self.region = prep(region)

    # ------------------------------------------------------------------ helpers
    def fp(self, ref):
        return self.b.fps[ref]

    def P(self, ref, x, y, rot=0):
        fp = self.fp(ref)
        fp.place(*A(*S(x, y)), rot)
        fp.placed = True
        return fp

    def pad_pos(self, ref, num):
        fp = self.fp(ref)
        p = fp.pads_by_num(num)[0]
        return fp.pad_xy(p)

    def ball(self, name):
        return self.pad_pos("U201", name)

    def placed_boxes(self, skip=None):
        out = []
        for r, fp in self.b.fps.items():
            if fp.placed and r != skip:
                out.append((r, fp.courtyard()))
        return out

    # ------------------------------------------------------------------ auto placement
    def free(self, box, margin=0.12, skip=None):
        x0, y0, x1, y1 = box
        if x0 < OX + 0.5 or y0 < OY + 0.5 or x1 > OX + W - 0.5 or y1 > OY + H - 0.5:
            return False
        from shapely.geometry import box as sbox
        if not self.region.contains(sbox(x0, y0, x1, y1)):
            return False
        for r, b in self._boxes:
            if r == skip:
                continue
            if x0 < b[2] + margin and x1 > b[0] - margin and y0 < b[3] + margin and y1 > b[1] - margin:
                return False
        for k in self.keepouts:
            kx0, ky0, kx1, ky1 = A(k[0], k[1]) + A(k[2], k[3])
            if x0 < kx1 and x1 > kx0 and y0 < ky1 and y1 > ky0:
                return False
        if self.obst:
            s = Shape("box", (x0, y0, x1, y1), None, "x")
            for t in self.obst_idx.near(s, 1.0):
                if distance(s, t) < 0.05:
                    return False
        return True

    def add_obstacle(self, x1, y1, x2, y2, w):
        """pre-routed copper the auto-placer must keep courtyards off (absolute coords)"""
        s = Shape("seg", (x1, y1, x2, y2, w / 2), None, "x")
        self.obst.append(s)
        self.obst_idx.add(s)

    def auto_place(self, ref, target, pad_targets=None, rmax=25.0, step=0.25, rots=(0, 90, 180, 270),
                   margin=0.12):
        """Place ref as close as possible to target (absolute point, or a list of candidate
        centres), trying rotations. Score = distance from the centre + weighted distance of
        each pad to the nearest of its desired points
        (pad_targets: num -> (x, y[, w]) or ([(x, y), ...], w))."""
        fp = self.fp(ref)
        if ref in getattr(self, "given", {}):      # position known (post-routing tools)
            fp.place(*self.given[ref])
            fp.placed = True
            self._boxes.append((ref, fp.courtyard()))
            return fp
        centres = target if isinstance(target, list) else [target]
        norm = {}
        for k, t in (pad_targets or {}).items():
            if isinstance(t[0], (list, tuple)):
                norm[k] = (list(t[0]), t[1])
            else:
                norm[k] = ([(t[0], t[1])], t[2] if len(t) > 2 else 0.6)
        def est(c):
            return sum(w * min(math.hypot(c[0] - qx, c[1] - qy) for qx, qy in pts)
                       for pts, w in norm.values())
        centres = sorted(centres, key=est)
        best = None
        for tx, ty in centres:
            r = 0.0
            local = best
            while r <= rmax:
                n = max(1, int(2 * math.pi * r / step)) if r > 0 else 1
                for i in range(n):
                    a = 2 * math.pi * i / n
                    x = round((tx + r * math.cos(a)) / 0.05) * 0.05
                    y = round((ty + r * math.sin(a)) / 0.05) * 0.05
                    for rot in rots:
                        fp.place(x, y, rot)
                        if not self.free(fp.courtyard(), margin, skip=ref):
                            continue
                        score = r
                        for p in fp.pads:
                            t = norm.get(p.num)
                            if t:
                                px, py = fp.pad_xy(p)
                                score += t[1] * min(math.hypot(px - qx, py - qy) for qx, qy in t[0])
                        if local is None or score < local[0]:
                            local = (score, x, y, rot)
                if local is not None and r > local[0] + 0.01:
                    break
                r += step
            best = local
        if best is None:
            raise RuntimeError(f"no room for {ref} near {target}")
        _, x, y, rot = best
        fp.place(x, y, rot)
        fp.placed = True
        self._boxes.append((ref, fp.courtyard()))
        return fp

    def connected_points(self, ref, num, exclude=("GND",)):
        """Positions of placed pads on the same net (other parts)."""
        n = self.fp(ref).pad_net.get(num)
        if not n or n in exclude:
            return []
        pts = []
        for r2, p2 in self.nets.get(n, []):
            if r2 == ref:
                continue
            f2 = self.fp(r2)
            if f2.placed:
                pts.append(f2.pad_xy(f2.pads_by_num(p2)[0]))
        return pts


def place_power(L):
    """Power section floorplan (board-relative mm). Switcher loops follow the TI layout
    guidance: inductor on the L1/L2 (SW) pins, input cap on VIN/PGND, output caps on
    VOUT/PGND; charger BAT pins face the battery, OUT faces the power switch.
    Coordinates are those of the old 160 x 90 layout: call inside shift(PWR). The battery
    connector J103 and power switch SW101 are placed at the new board edge by build()."""
    P = L.P
    # USB-C input: CC resistors and ESD array right behind the receptacle
    P("J101", 62.0, 85.85, 0)
    P("U101", 62.0, 76.2, 90)
    P("R101", 60.3, 79.35, 90); P("R102", 63.75, 79.35, 90)
    P("C101", 57.5, 78.1, 90)
    P("TP101", 68.8, 79.2, 0)
    # charger (BQ24074) + main power switch (AO3401A)
    P("U102", 47.0, 78.5, 0)
    P("R104", 45.85, 75.4, 90); P("R106", 46.85, 75.4, 90)
    P("R105", 50.15, 76.9, 90)
    P("C103", 42.6, 78.9, 90)
    P("Q102", 52.6, 78.5, 0)
    P("C102", 51.3, 81.9, 0)
    P("TP103", 48.4, 83.4, 0); P("TP104", 55.6, 80.9, 0)
    # battery connector, DW01A + FS8205A protection, MAX17048 fuel gauge
    P("Q101", 22.5, 82.8, 0)
    P("U103", 29.8, 81.3, 0)
    P("U104", 19.0, 74.0, 0)
    P("TP102", 33.5, 78.2, 0)
    # 3.3 V buck-boost (TPS63001)
    P("U105", 40.0, 65.5, 0); P("L101", 35.4, 65.5, 90)
    P("C108", 38.5, 61.6, 90); P("C109", 41.0, 61.6, 90)
    P("C107", 39.3, 69.35, 0)
    P("C106", 43.3, 65.5, 0); P("R120", 42.82, 66.9, 90)
    # 5 V boost (TPS61023)
    P("U106", 52.0, 64.0, 0); P("L102", 52.0, 67.3, 0)
    P("C111", 48.9, 66.6, 90)
    P("C112", 52.6, 61.8, 180); P("C113", 52.6, 59.7, 180)
    P("TP105", 71.0, 80.0, 0); P("TP106", 55.2, 84.0, 0); P("TP107", 71.0, 83.0, 0)


def T(L, net, w, pts, layer="F.Cu"):
    """locked track polyline (board-relative points); F.Cu copper also blocks auto-placement"""
    for a, b_ in zip(pts, pts[1:]):
        t = L.b.track(layer, A(*S(*a)), A(*S(*b_)), w, net, locked=True)
        if layer == "F.Cu":
            L.add_obstacle(t["x1"], t["y1"], t["x2"], t["y2"], w + 0.1)


def V(L, net, x, y, d=0.6, drill=0.3):
    v = L.b.via(*A(*S(x, y)), d, drill, net, locked=True)
    L.add_obstacle(v["x"], v["y"], v["x"], v["y"], d + 0.1)
    return v


# battery connector: mouth towards the bottom-left ear, clear of Q101 and of the screw
J103_AT = (21.6, 71.0)


def route_power(L):
    """Hand-routed high-current / switching copper of the power section (old-layout
    coordinates: call inside shift(PWR))."""
    jx, jy = J103_AT[0] - PWR[0], J103_AT[1] - PWR[1]      # J103 in old-layout coordinates
    # ---- USB-C receptacle breakout (USB 2.0: A6/B6 = D+, A7/B7 = D-)
    T(L, "USB_D_P", 0.2, [(61.75, 81.5), (61.75, 80.3), (62.75, 80.3), (62.75, 81.5)])
    T(L, "USB_D_P", 0.2, [(61.75, 80.3), (61.05, 79.6), (61.05, 77.6)])          # -> U101.1
    for x in (61.25, 62.25):
        T(L, "USB_D_N", 0.2, [(x, 81.5), (x, 80.8)])
        V(L, "USB_D_N", x, 80.8, 0.4, 0.2)
    T(L, "USB_D_N", 0.2, [(61.25, 80.8), (62.25, 80.8), (62.95, 80.1), (62.95, 79.3)], "B.Cu")
    V(L, "USB_D_N", 62.95, 79.3, 0.4, 0.2)
    T(L, "USB_D_N", 0.2, [(62.95, 79.3), (62.95, 77.6)])                         # -> U101.3
    T(L, "/power/CC1", 0.2, [(60.75, 81.5), (60.75, 80.6), (60.3, 80.15)])
    T(L, "/power/CC2", 0.2, [(63.75, 81.5), (63.75, 79.9)])
    # VBUS: both receptacle VBUS pads joined on B.Cu, trunk to the charger IN pin
    T(L, "VBUS", 0.5, [(59.55, 81.5), (59.55, 80.3), (59.3, 80.05), (59.3, 74.3), (47.75, 74.3),
                       (47.75, 76.3)])
    T(L, "VBUS", 0.25, [(47.75, 76.3), (47.75, 77.0)])                          # U102.13 IN
    T(L, "VBUS", 0.5, [(64.45, 81.5), (64.45, 80.2), (64.7, 79.95), (64.7, 79.4), (68.8, 79.2)])
    for x in (59.3, 64.7):
        V(L, "VBUS", x, 79.4)
    T(L, "VBUS", 0.6, [(59.3, 79.4), (59.3, 78.4), (64.7, 78.4), (64.7, 79.4)], "B.Cu")
    T(L, "VBUS", 0.5, [(59.3, 79.4), (58.0, 79.4)])                              # C101
    T(L, "VBUS", 0.25, [(62.0, 74.6), (62.0, 73.9)])                             # U101.5 VBUS
    V(L, "VBUS", 62.0, 73.9, 0.4, 0.2)
    T(L, "VBUS", 0.3, [(62.0, 73.9), (62.0, 78.4)], "B.Cu")
    # ---- battery: J103 -> protection FETs (BAT-) and -> charger BAT (VBAT)
    T(L, "/power/BAT_N", 0.8, [(jx + 1.0, 82.8), (19.64, 82.8)])          # J103.2 -> Q101 S1
    T(L, "GND", 0.8, [(25.36, 82.8), (26.6, 82.8)])
    T(L, "GND", 0.8, [(26.6, 82.1), (26.6, 83.5)])
    V(L, "GND", 26.6, 82.1); V(L, "GND", 26.6, 83.5)
    T(L, "VBAT", 0.8, [(jx - 1.0, jy - 2.85), (jx - 1.0, 76.3), (44.3, 76.3), (44.3, 79.85),
                       (43.0, 79.85)])                                        # J103.1 -> C103
    T(L, "VBAT", 0.5, [(44.3, 78.5), (45.5, 78.5)])                              # U102.2/3 BAT
    T(L, "VBAT", 0.3, [(17.8, 74.0), (17.3, 74.0), (17.3, 76.0)])                # U104 CELL/VDD
    T(L, "VBAT", 0.5, [(33.5, 76.3), (33.5, 78.2)])                              # TP102
    # ---- charger output -> power switch -> VSYS_SW (B.Cu trunk to both regulators)
    T(L, "VSYS", 0.5, [(48.6, 78.5), (49.8, 78.5), (50.75, 79.45), (51.3, 79.45)])
    T(L, "VSYS", 0.5, [(51.5, 79.6), (50.6, 80.5), (50.6, 81.5)])                 # C102
    T(L, "/power/CHG_ILIM", 0.2, [(48.6, 77.75), (49.4, 77.75), (49.9, 77.41)])
    T(L, "VSYS_SW", 0.6, [(53.8, 78.5), (55.4, 78.5), (55.4, 77.6)])
    T(L, "VSYS_SW", 0.5, [(55.4, 78.5), (55.6, 80.9)])                           # TP104
    V(L, "VSYS_SW", 55.4, 78.5); V(L, "VSYS_SW", 55.4, 77.6)
    T(L, "VSYS_SW", 0.8, [(55.4, 77.6), (55.4, 70.6), (37.6, 70.6), (37.6, 69.8)], "B.Cu")
    V(L, "VSYS_SW", 50.8, 69.95); V(L, "VSYS_SW", 49.95, 70.6)
    T(L, "VSYS_SW", 0.8, [(50.8, 70.6), (50.8, 69.95)], "B.Cu")
    T(L, "VSYS_SW", 0.6, [(49.95, 70.6), (50.8, 69.95), (50.8, 69.0)])           # L102.1
    V(L, "VSYS_SW", 37.6, 69.8); V(L, "VSYS_SW", 37.6, 70.6)
    T(L, "VSYS_SW", 0.6, [(37.6, 70.6), (37.6, 69.8), (38.4, 69.35)])             # C107.1
    # ---- TPS63001: VIN bus, L1/L2 to the inductor, VOUT + FB at the output caps
    T(L, "VSYS_SW", 0.5, [(38.52, 69.0), (38.52, 67.85)])
    T(L, "VSYS_SW", 0.5, [(38.53, 66.6), (38.53, 67.85), (42.82, 67.85), (42.82, 67.4)])
    T(L, "VSYS_SW", 0.28, [(41.47, 66.6), (41.47, 67.85)])                       # EN = VIN
    T(L, "/power/BB_L1", 0.3, [(38.3, 66.0), (36.9, 66.0)])
    T(L, "/power/BB_L2", 0.3, [(38.3, 65.0), (36.9, 65.0)])
    T(L, "+3V3", 0.28, [(38.53, 64.4), (38.53, 62.8)])
    T(L, "+3V3", 0.2, [(41.47, 64.4), (41.3, 64.2), (41.3, 62.8)])                # FB
    T(L, "+3V3", 0.5, [(39.0, 62.55), (40.5, 62.55)])
    V(L, "+3V3", 39.75, 62.55)
    T(L, "/power/BB_VINA", 0.25, [(41.8, 65.5), (42.8, 65.5)])
    T(L, "/power/BB_VINA", 0.25, [(42.82, 66.39), (42.82, 65.6)])
    for a, b_ in (((38.8, 65.5), (39.4, 65.5)), ((41.2, 65.0), (40.6, 65.0)),
                  ((41.2, 66.0), (40.6, 66.0))):                                  # GND pins -> EP
        T(L, "GND", 0.25, [a, b_])
    # ---- TPS61023: VIN/SW to the inductor, VOUT to the output caps, GND through the
    #      channel between the pin rows (tight output loop)
    T(L, "VSYS_SW", 0.3, [(51.1, 64.55), (51.1, 65.6)])
    T(L, "/power/BST_SW", 0.3, [(52.9, 64.0), (53.45, 64.0), (53.45, 65.6)])
    T(L, "+5V", 0.35, [(52.9, 63.5), (53.3, 63.5), (53.3, 62.4)])
    T(L, "+5V", 0.5, [(53.55, 61.8), (53.55, 59.7)])
    T(L, "GND", 0.5, [(51.65, 61.8), (51.65, 59.7)])
    T(L, "GND", 0.3, [(51.65, 62.4), (52.0, 62.75), (52.0, 65.6)])
    T(L, "GND", 0.25, [(52.45, 64.5), (52.0, 64.75)])
    V(L, "GND", 52.0, 64.75, 0.4, 0.2); V(L, "GND", 52.0, 65.6, 0.4, 0.2)
    T(L, "VSYS_SW", 0.6, [(49.2, 67.375), (50.6, 67.375)])                       # C111.1
    # ---- RW612 internal bucks: switch node straight to the inductors, caps on the output
    #      (part of the core block, not the power block)
    with shift(CORE):
        T(L, "/mcu_core/BUCK18_SW", 0.3, [(76.5, 27.5), (76.5, 28.4), (74.62, 30.28), (74.62, 31.2)])
        T(L, "/mcu_core/BUCK11_SW", 0.3, [(77.5, 27.5), (77.5, 28.5), (76.9, 29.1), (76.9, 31.2)])
        T(L, "+1V8", 0.4, [(73.18, 32.4), (73.18, 33.4)])
        T(L, "VCORE", 0.4, [(78.02, 32.6), (78.02, 33.7)])


def route_bga_periphery(L):
    """Short, critical top-layer connections at the edge of the RW612 that an autorouter
    tends to box in: the 40 MHz crystal, the AVDD18 ball, the core-rail balls of the outer
    ring (tied to their neighbours or stubbed out), and the ISENSE bias resistor.
    Old-layout coordinates: call inside shift(CORE)."""
    # 40 MHz crystal (Y201 rotated so XTAL_IN faces H15)
    T(L, "/mcu_core/XTAL40_IN", 0.15, [(74.5, 24.0), (72.2, 24.0)])
    T(L, "/mcu_core/XTAL40_OUT", 0.15, [(74.5, 24.5), (73.3, 24.5), (72.75, 25.05), (72.75, 25.65),
                                        (70.6, 25.65), (70.6, 25.0)])
    # +1V8 K15-L15-M15-N15 column tie (keeps K15 from walling in J15), F15 onto F14
    T(L, "+1V8", 0.2, [(74.5, 23.0), (74.5, 21.5)])
    T(L, "+1V8", 0.2, [(74.5, 25.0), (75.0, 25.0)])
    # AVDD18 (J15) out between the +1V8 column and the crystal
    T(L, "/mcu_core/AVDD18_J15", 0.2, [(74.5, 23.5), (73.45, 23.5), (72.95, 23.0), (72.95, 21.8)])
    # VCORE outer balls: stub clear of the ball field
    T(L, "VCORE", 0.2, [(74.5, 26.0), (73.0, 26.0)])
    T(L, "VCORE", 0.2, [(81.5, 24.0), (82.7, 24.0)])        # H1 (G1 is depopulated)
    # ISENSE bias resistor at the ISENSE escape via (bottom-left corner of the escape region)
    L.P("R202", 73.8, 29.0, 180)
    T(L, "/mcu_core/ISENSE", 0.15, [(74.31, 29.0), (75.0, 29.0)])


BALL_RE = re.compile(r"\b([A-HJ-NPR](?:1[0-5]|[1-9]))\b")


def wing_slots():
    """JST PH connector slots in the side wings, outside the LCD: two columns of three per
    side, pin 1 at the bottom (rotation 90). Returns {name: (x, y)} board-relative."""
    # B4B-PH-K courtyard at rot 90: x -2.2 .. +3.3, y -8.45 .. +2.45 around pin 1
    col_l = {"A": 1.8 + 2.2, "B": 8.0 + 2.2}               # courtyards x 1.8-7.3 / 8.0-13.5
    rows = [25.5, 37.6, 49.7]                              # courtyard tops, 10.9 tall + 1.2 gap
    out = {}
    for c, x in col_l.items():
        for i, top in enumerate(rows):
            y = top + 8.45                                 # pin 1
            out[f"L{c}{i + 1}"] = (x, y)
            # mirror the courtyard (not the origin) about the axis
            x0 = x - 2.2
            out[f"R{c}{i + 1}"] = (2 * AXIS - (x0 + 5.5) + 2.2, y)
    return out


# which connector goes in which wing slot (L/R = side, A = outer column, B = inner)
WING = {"J507": "LA1", "J508": "LA2", "J509": "LA3",        # ABTN1-3
        "J510": "LB1", "J503": "LB2", "J505": "LB3",        # ABTN4, ENC1, BTN1
        "J511": "RA1", "J512": "RA2", "J513": "RA3",        # ABTN5-7
        "J514": "RB1", "J504": "RB2", "J506": "RB3"}        # ABTN8, ENC2, BTN2


def build(given=None):
    L = Layout()
    L.given = given or {}
    b = L.b
    # ---------------------------------------------------------------- mechanical (DXF)
    b.outline = [A(*p) for p in OUT.polygon]
    k = 0
    for seg in OUT.segments:
        stroke = [["stroke", ["width", "0.1"], ["type", "default"]], ["layer", QStr("Edge.Cuts")]]
        if seg[0] == "line":
            b.graphics.append(["gr_line", ["start", *map(f, A(*seg[1]))], ["end", *map(f, A(*seg[2]))],
                               *stroke, ["uuid", uid("edge", k)]])
            k += 1
        elif seg[0] == "arc":
            b.graphics.append(["gr_arc", ["start", *map(f, A(*seg[1]))], ["mid", *map(f, A(*seg[2]))],
                               ["end", *map(f, A(*seg[3]))], *stroke, ["uuid", uid("edge", k)]])
            k += 1
        else:
            for p0, p1 in zip(seg[1], seg[1][1:]):
                b.graphics.append(["gr_line", ["start", *map(f, A(*p0))], ["end", *map(f, A(*p1))],
                                   *stroke, ["uuid", uid("edge", k)]])
                k += 1
    # LCD module outline (MSP4031, 108 x 60.88) on User.Drawings
    lx0, ly0, lx1, ly1 = LCD
    for i, (s_, e) in enumerate((((lx0, ly0), (lx1, ly0)), ((lx1, ly0), (lx1, ly1)),
                                 ((lx1, ly1), (lx0, ly1)), ((lx0, ly1), (lx0, ly0)))):
        b.graphics.append(["gr_line", ["start", *map(f, A(*s_))], ["end", *map(f, A(*e))],
                           ["stroke", ["width", "0.15"], ["type", "dash"]], ["layer", QStr("Dwgs.User")],
                           ["uuid", uid("lcd", i)]])
    b.graphics.append(["gr_text", QStr("LCD MSP4031 outline (108 x 60.88), cable to J501 - verify position"),
                       ["at", *map(f, A(AXIS + 30.0, ly1 - 12.0)), "0"], ["layer", QStr("Dwgs.User")],
                       ["uuid", uid("lcdtxt")],
                       ["effects", ["font", ["size", "1.2", "1.2"], ["thickness", "0.15"]]]])
    b.graphics.append(["gr_text", QStr("Outline + holes: PCB_Volante_2026.DXF (3.0 mm holes as drawn)"),
                       ["at", *map(f, A(AXIS + 30.0, ly1 - 9.8)), "0"], ["layer", QStr("Dwgs.User")],
                       ["uuid", uid("dxftxt")],
                       ["effects", ["font", ["size", "1.2", "1.2"], ["thickness", "0.15"]]]])
    for i, (x, y, dia) in enumerate(OUT.holes):
        ref = f"H{i + 1}"
        # the DXF holes are 3.0 mm and only 1.3 mm from the edge at the top: plain NPTH,
        # no pad ring (a 6 mm ring would run off the board)
        fp = Footprint(ref, "MountingHole:MountingHole_3mm",
                       L.lib.load("MountingHole:MountingHole_3mm"), "MountingHole")
        fp.place(*A(x, y)); fp.placed = True
        fp.board_only = True
        b.add_fp(fp)

    # ---------------------------------------------------------------- fixed placements
    P = L.P
    with shift(CORE):
        # RW612 + RF front end (top centre)
        P("U201", 78, 24, 180)
        P("J401", 78.2, 3.3, 90)
        P("C414", 78.2, 6.9, 90)
        P("C413", 79.35, 7.4, 0); P("C415", 79.35, 6.25, 0); P("L403", 77.05, 6.25, 180)
        P("FL401", 78.2, 9.0, 180)
        P("C412", 78.75, 11.3, 90)
        P("U401", 78.75, 13.0, 90)
        P("C410", 77.0, 12.4, 180); P("C411", 80.5, 12.4, 0)   # signal pad towards U401
        # 5 GHz pi filter on x = 75
        P("C402", 73.85, 15.6, 180); P("L401", 75.0, 16.9, 90); P("C401", 73.85, 18.3, 180)
        # 2.4 GHz on x = 77.5
        P("C406", 77.5, 15.3, 90); P("C405", 76.35, 16.4, 180); P("C404", 77.5, 17.5, 90)
        P("C403", 76.35, 18.6, 180)
        # BLE on x = 80
        P("C409", 80.0, 15.3, 90); P("C408", 81.15, 16.4, 0); P("L402", 80.0, 17.5, 90)
        P("C407", 81.15, 18.6, 0)
        # memory
        P("U301", 96.0, 24.0, 0)
        P("U302", 88.5, 33.5, 0)
        # clocks next to the balls
        P("Y201", 71.3, 24.3, 180)     # XTAL_IN pad next to H15, XTAL_OUT around the bottom
        P("Y202", 81.2, 31.1, 0)
        P("L201", 73.9, 31.7, 180)     # SW pad towards ball A11
        P("L202", 77.3, 31.9, 0)
        # core buck output caps right at their inductors
        P("C216", 73.18, 34.4, 270)
        P("C234", 78.02, 34.7, 270)

    # LCD cable connector (JST PH 14-pin, 1:1 to the module header): strip above the LCD,
    # left of the RF block, on the side where the module has its header
    P("J501", 24.5, 5.4, 0)
    # NeoPixel output: top of the right wing, bulk cap next to the connector, buffer
    # under the LCD corner
    P("J502", 127.6, 20.6, 0)
    P("C504", 129.3, 13.9, 0)
    P("U501", 120.5, 21.5, 0)
    # strip above the LCD, right of the RF block: debug and boot
    P("J201", 80.5, 7.0, 0)          # SWD
    P("SW201", 91.0, 6.5, 0)         # RESET
    P("SW202", 101.0, 6.5, 0)        # ISP
    P("J202", 108.5, 6.5, 90)        # UART
    P("D301", 113.0, 10.6, 0)        # user LED, visible with the LCD fitted
    # expansion header: top of the left wing, above the connector columns
    P("J515", 11.0, 9.7, 0)
    # wings: encoder and button connectors
    slots = wing_slots()
    for ref, slot in WING.items():
        P(ref, *slots[slot], 90)
    # active-button expander + fused 3.3 V feed: under the LCD, right of centre
    P("U502", 104.0, 44.0, 0)
    P("F501", 104.0, 52.0, 0)

    # power section: the old floorplan as one rigid block (see place_power / route_power)
    with shift(PWR):
        place_power(L)
        P("D101", 40.5, 88.3, 0); P("D102", 44.5, 88.3, 0)
    P("J103", *J103_AT, 0)
    # optional pack-NTC header (DNP) in the left ear next to the battery connector
    P("J102", 28.3, 75.0, 90)
    # power switch at the bottom edge, rotated to the edge, actuator flush with it
    xs = 39.45
    m = (edge_bottom(xs + 3.0) - edge_bottom(xs - 3.0)) / 6.0
    th = math.atan2(-m, 1.0)
    tip = 3.4                      # courtyard edge on the actuator side (local +y)
    ex, ey = xs, edge_bottom(xs)
    P("SW101", ex - tip * math.sin(th), ey - tip * math.cos(th), math.degrees(th))
    with shift(PWR):
        route_power(L)
    with shift(CORE):
        route_bga_periphery(L)

    # keep the RF corridor, the BGA escape halo and the LCD socket row free of auto-placed parts
    L.keepouts += [(*in_core(72.8, 1.0), *in_core(83.2, 20.6)),        # RF front end
                   (*in_core(73.0, 19.3), *in_core(83.0, 28.7))]       # BGA pads + 0.8 mm

    L._boxes = L.placed_boxes()

    # ---------------------------------------------------------------- auto placement of passives
    ball_xy = {}

    def note_ball(ref):
        note = L.comps[ref]["props"].get("Note", "")
        m = BALL_RE.findall(note)
        return m[0] if m else None

    # 1) RW612 decoupling (notes name the ball)
    decaps = []
    for ref, c in L.comps.items():
        if ref.startswith("C2") and not L.fp(ref).placed:
            bname = note_ball(ref)
            if bname:
                decaps.append((ref, bname))
    # bulk caps first so they get the close spots near their inductor/ball
    order = sorted(decaps, key=lambda t: (-L.fp(t[0]).courtyard()[2] + L.fp(t[0]).courtyard()[0], t[0]))
    for ref, bname in order:
        bx, by = L.ball(bname)
        cx, cy = L.fp("U201").x, L.fp("U201").y
        # push the target radially out of the BGA halo
        dx, dy = bx - cx, by - cy
        if abs(dx) >= abs(dy):
            tx, ty = cx + math.copysign(5.8, dx if dx else 1), by
        else:
            tx, ty = bx, cy + math.copysign(5.8, dy if dy else 1)
        L.auto_place(ref, (tx, ty), rots=(0, 90), margin=0.1)

    # 2) everything else: near the pads it connects to inside its own schematic block,
    #    else near anything it connects to, else near the block's main part
    group_of = {r: c["group"] for r, c in L.comps.items()}

    def anchor(group):
        best = None
        for r, g in group_of.items():
            fp = L.fp(r)
            if g == group and fp.placed and (best is None or len(fp.pads) > len(best.pads)):
                best = fp
        return (best.x, best.y) if best else None

    def mean(v):
        return (sum(x for x, y in v) / len(v), sum(y for x, y in v) / len(v))

    def targets(ref):
        """(same-block placed pad positions per pad, other-block ones, has unplaced
        same-block neighbours)"""
        fp = L.fp(ref)
        per_same, per_other, waiting = {}, {}, False
        for p in fp.pads:
            n = fp.pad_net.get(p.num)
            if not n or n in ("GND", "+3V3"):
                continue
            for r2, p2 in L.nets.get(n, []):
                if r2 == ref:
                    continue
                f2 = L.fp(r2)
                same = group_of.get(r2) == group_of[ref]
                if not f2.placed:
                    waiting |= same
                    continue
                xy = f2.pad_xy(f2.pads_by_num(p2)[0])
                (per_same if same else per_other).setdefault(p.num, []).append(xy)
        return per_same, per_other, waiting

    pending = [r for r in sorted(L.comps) if not L.fp(r).placed]
    while pending:
        scored = []
        for ref in pending:
            per_same, per_other, waiting = targets(ref)
            ns = sum(len(v) for v in per_same.values())
            no = sum(len(v) for v in per_other.values())
            if ns:
                tier = (0, -ns)          # grow the block from what is already placed
            elif no and not waiting:
                tier = (1, -no)          # only talks to other blocks: sit next to them
            elif no:
                tier = (2, -no)          # wait for its own block first
            else:
                tier = (3, 0)
            scored.append((tier, ref, per_same, per_other))
        scored.sort()
        tier, ref, per_same, per_other = scored[0]
        if per_same:
            pts = [xy for v in per_same.values() for xy in v]
            pad_t = {k: (v, 0.6) for k, v in per_same.items()}
            # pads that only reach other blocks steer the rotation, not the position
            for k, v in per_other.items():
                if k not in pad_t:
                    pad_t[k] = (v, 0.12)
        elif per_other and tier[0] == 1:
            pts = [xy for v in per_other.values() for xy in v]
            pad_t = {k: (v, 0.6) for k, v in per_other.items()}
        else:
            a = anchor(group_of[ref])
            pts = [a] if a else [A(AXIS, 42.0)]
            pad_t = {k: (v, 0.12) for k, v in per_other.items()}
        m = mean(pts)
        # a net can join far-apart pads (e.g. on-board switch + external pads): also try
        # each distant target point as a search centre, nearest-point scoring decides
        centres = [m]
        for p in pts:
            if math.hypot(p[0] - m[0], p[1] - m[1]) > 3.0 and \
                    all(math.hypot(p[0] - c[0], p[1] - c[1]) > 2.0 for c in centres):
                centres.append(p)
        L.auto_place(ref, centres, pad_t)
        pending.remove(ref)
    return L


# ============================================================================ pre-routing
RF_CORRIDOR = (*in_core(72.6, 1.2), *in_core(83.4, 20.25))   # no F.Cu pour, no autorouting on F.Cu


def poly(L, net, w, pts, layer="F.Cu"):
    """Locked track polyline, board-relative points."""
    out = []
    for a, b_ in zip(pts, pts[1:]):
        out.append(L.b.track(layer, A(*S(*a)), A(*S(*b_)), w, net, locked=True))
    return out


def rf_route(L):
    """50 ohm microstrip (0.34 mm over In1 GND) from the RW612 RF balls through the
    matching networks, the RF switch and the diplexer to the U.FL."""
    W, NB, NS = RF_W, 0.2, 0.18       # line, BGA neck, SKY13323 neck
    # 5 GHz: R14 -> C401 (shunt) / L401 -> C402 (shunt) -> diplexer HB (FL401.4)
    poly(L, "/rf/RF5G_SOC", NB, [(75.0, 20.5), (75.0, 20.1)])
    poly(L, "/rf/RF5G_SOC", W, [(75.0, 20.1), (75.0, 17.22)])
    poly(L, "/rf/RF5G_SOC", W, [(74.17, 18.3), (75.0, 18.3)])
    poly(L, "/rf/RF5G_FLT", W, [(75.0, 16.58), (75.0, 12.75), (77.65, 10.1)])
    poly(L, "/rf/RF5G_FLT", W, [(74.17, 15.6), (75.0, 15.6)])
    poly(L, "/rf/RF5G_FLT", 0.3, [(77.65, 10.1), (77.65, 9.55)])
    # 2.4 GHz Wi-Fi: R9 -> C403 (DNP shunt) / C404 -> C405 (DNP shunt) / C406 -> SW OUT1
    poly(L, "/rf/RF2G_SOC", NB, [(77.5, 20.5), (77.5, 20.1)])
    poly(L, "/rf/RF2G_SOC", W, [(77.5, 20.1), (77.5, 17.82)])
    poly(L, "/rf/RF2G_SOC", W, [(76.67, 18.6), (77.5, 18.6)])
    poly(L, "/rf/RF2G_FLT", W, [(77.5, 17.18), (77.5, 15.62)])
    poly(L, "/rf/RF2G_FLT", W, [(76.67, 16.4), (77.5, 16.4)])
    poly(L, "/rf/RFSW_OUT1", W, [(77.5, 14.98), (77.5, 14.55), (78.0, 14.05)])
    poly(L, "/rf/RFSW_OUT1", NS, [(78.0, 14.05), (78.4, 13.65), (78.4, 13.45)])
    # BLE / 802.15.4: R4 -> C407 (shunt) / L402 -> C408 (shunt) / C409 -> SW OUT2
    poly(L, "/rf/BLE_SOC", NB, [(80.0, 20.5), (80.0, 20.1)])
    poly(L, "/rf/BLE_SOC", W, [(80.0, 20.1), (80.0, 17.82)])
    poly(L, "/rf/BLE_SOC", W, [(80.83, 18.6), (80.0, 18.6)])
    poly(L, "/rf/BLE_FLT", W, [(80.0, 17.18), (80.0, 15.62)])
    poly(L, "/rf/BLE_FLT", W, [(80.83, 16.4), (80.0, 16.4)])
    poly(L, "/rf/RFSW_OUT2", W, [(80.0, 14.98), (80.0, 14.55), (79.5, 14.05)])
    poly(L, "/rf/RFSW_OUT2", NS, [(79.5, 14.05), (79.1, 13.65), (79.1, 13.45)])
    # switch common -> DC block -> diplexer LB
    poly(L, "/rf/RFSW_IN", NS, [(78.75, 12.55), (78.75, 12.15)])
    poly(L, "/rf/RFSW_IN", W, [(78.75, 12.15), (78.75, 11.62)])
    poly(L, "/rf/DPX_LB", W, [(78.75, 10.98), (78.75, 10.1)])
    poly(L, "/rf/DPX_LB", 0.3, [(78.75, 10.1), (78.75, 9.55)])
    # diplexer common -> C414 (series, C413/C415/L403 DNP shunts) -> U.FL
    poly(L, "/rf/ANT_COM", 0.3, [(78.2, 8.45), (78.2, 8.0)])
    poly(L, "/rf/ANT_COM", W, [(78.2, 8.0), (78.2, 7.22)])
    poly(L, "/rf/ANT_COM", W, [(79.03, 7.4), (78.2, 7.4)])
    poly(L, "/rf/ANT_FEED", W, [(78.2, 6.58), (78.2, 4.82)])
    poly(L, "/rf/ANT_FEED", W, [(79.03, 6.25), (78.2, 6.25)])
    poly(L, "/rf/ANT_FEED", W, [(77.37, 6.25), (78.2, 6.25)])
    # RF switch control: bypass cap -> switch pin on top, via next to the cap
    poly(L, "/rf/RF_CTL0", 0.15, [(77.32, 12.4), (78.4, 12.55)])
    poly(L, "/rf/RF_CTL1", 0.15, [(80.18, 12.4), (79.1, 12.55)])
    for net, (x, y) in (("/rf/RF_CTL0", (77.32, 12.4)), ("/rf/RF_CTL1", (80.18, 12.4))):
        poly(L, net, 0.15, [(x, y), (x, y + 0.6)])
        L.b.via(*A(*S(x, y + 0.6)), 0.4, 0.2, net, locked=True)


def prerouting(L, log=print):
    b = L.b
    with shift(CORE):
        rf_route(L)
    cop = fanout.Copper(b, b.outline)
    os.makedirs(BUILD, exist_ok=True)
    _, _, used, pos = fanout.bga_fanout(b, cop, "U201", skip_nets=RF_NETS, log=log, via=BGA_VIA,
                                        cache_path=os.path.join(BUILD, "fanout_cache.json"))
    # escape of every dogbone via to 1.5 mm outside the ball array, on B.Cu / In2 / In5
    u = L.fp("U201")
    dest = {}
    for n in set(used.values()):
        pts = [L.fp(r).pad_xy(L.fp(r).pads_by_num(pn)[0]) for r, pn in L.nets.get(n, []) if r != "U201"]
        if pts:
            dest[n] = (sum(x for x, y in pts) / len(pts), sum(y for x, y in pts) / len(pts))
    # the exact escape solve takes minutes: cache it, keyed on its inputs
    import hashlib
    import json as _json
    def side(d):         # the escape only uses the destination to pick a side
        dx, dy = d[0] - u.x, d[1] - u.y
        return ("E" if dx > 0 else "W") if abs(dx) >= abs(dy) else ("S" if dy > 0 else "N")
    sig = hashlib.sha1(_json.dumps([sorted((list(k), v) for k, v in used.items()),
                                    sorted((n, side(d)) for n, d in dest.items()), ESCAPE_ARGS,
                                    list(ESCAPE_LAYERS), list(BGA_VIA)],
                                   sort_keys=True).encode()).hexdigest()
    cache = os.path.join(BUILD, "escape_cache.json")
    c = _json.load(open(cache)) if os.path.exists(cache) else {}
    if c.get("sig") == sig:
        trk, evias, L.escape_skipped = c["tracks"], c["vias"], c["skipped"]
        log(f"  escape: cached solution ({len(L.escape_skipped)} left to the autorouter)")
    else:
        trk, evias, L.escape_skipped = escape.solve(b, cop, pos, used, dest, (u.x, u.y), log=log,
                                                    inner_layers=ESCAPE_LAYERS, exit_via=BGA_VIA,
                                                    **ESCAPE_ARGS)
        os.makedirs(BUILD, exist_ok=True)
        _json.dump(dict(sig=sig, tracks=trk, vias=evias, skipped=[[list(v), n] for v, n in
                                                                   L.escape_skipped]),
                   open(cache, "w"))
    for t in trk:
        # with 8 layers every escape layer (B.Cu, In2, In5) is routable, so the escapes all
        # go to the autorouter as ordinary (movable) wiring: a warm start it may push or
        # re-route. On 4 layers the In2 ones had to stay fixed, since it did not route In2.
        t = dict(t, locked=(ESCAPE_LOCKED or
                            (ESCAPE_LOCK_INNER and t["layer"] != "B.Cu")))
        b.tracks.append(t)
        cop.add_track(t)
    for v in evias:
        b.vias.append(v)
        cop.add_via(v)
    # keep the ring around the BGA free for escape routing: plane vias of the
    # decoupling caps go on the far side of the caps
    bx = A(*in_core(74.5 - 2.0, 20.5 - 0.6)) + A(*in_core(81.5 + 2.0, 27.5 + 2.0))
    # keep radial corridors open beyond each escape end (through the decoupling ring).
    # Counted per layer, so an end on B.Cu and one on In2 at the same point stay distinct.
    corr = []
    ends = {}
    for t in trk:
        for x, y in ((t["x1"], t["y1"]), (t["x2"], t["y2"])):
            k = (t["layer"], round(x, 3), round(y, 3))
            ends[k] = ends.get(k, 0) + 1
    seen = set()
    for (_layer, x, y), cnt in ends.items():
        dx, dy = x - u.x, y - u.y
        if cnt != 1 or max(abs(dx), abs(dy)) < 4.9:
            continue                         # only track ends on the region boundary
        if (round(x, 3), round(y, 3)) in seen:
            continue                         # one corridor per point, whatever the layer
        seen.add((round(x, 3), round(y, 3)))
        if abs(dx) >= abs(dy):
            corr.append((x, y, x + 4.0 * (1 if dx > 0 else -1), y, 0.55, 3.0))
        else:
            corr.append((x, y, x, y + 4.0 * (1 if dy > 0 else -1), 0.55, 3.0))
    failed = fanout.plane_fanout(b, cop, skip_refs=("U201",), log=log, avoid=[bx + (1.5,)], avoid_segs=corr)
    if HOOKUP:
        hookup(L, cop, log=log)
    L.cop = cop
    return failed


# Pre-routing the core-supply islands looked attractive, but the locked top-layer links
# between the decoupling caps fence in the BGA perimeter: Freerouting left ~85 connections
# open with them against ~30 without. Off; the finishing router closes what is left.
HOOKUP = False
HOOKUP_NETS = {"+1V8": (0.25, 0.15, 0.127), "VCORE": (0.25, 0.15, 0.127),
               "/mcu_core/AVDD18_J15": (0.2, 0.15, 0.127)}


def hookup(L, cop, log=print, reach=10.0):
    """Join the copper islands of the RW612 core supplies near the BGA (ball groups, stubs,
    decoupling caps, inductor outputs) with short, locked maze routes, nearest pair first.
    These are the connections a person routes together with the fan-out; left to the
    autorouter they get boxed in by the escape wiring."""
    import finish_router as FR
    b = L.b
    u = L.fp("U201")
    r = FR.Router(L, list(b.tracks), list(b.vias), (OX, OY, OX + W, OY + H))
    # top layer only: B.Cu around the BGA belongs to the escape wiring
    r.add_keepout("B.Cu", Shape("box", (u.x - reach - 3, u.y - reach - 3, u.x + reach + 3, u.y + reach + 3),
                                None, "B.Cu"))
    # keep a straight top-layer exit open for every outer-ring signal ball that has no
    # escape yet (the autorouter takes those out on F.Cu)
    comps0 = FR.components(r.items)
    single = set()
    for n, cc in comps0.items():
        for c in cc.values():
            if len(c) == 1 and r.items[c[0]]["kind"] == "pad":
                single.add(r.items[c[0]]["ref"])
    n_exit = 0
    for p in u.pads:
        net = u.pad_net.get(p.num)
        if not net or net in HOOKUP_NETS or net in PLANE_NETS or ("U201", p.num) not in single:
            continue
        x, y = u.pad_xy(p)
        bxr, byr = x - OX - CORE[0], y - OY - CORE[1]      # old-layout coordinates
        d = None
        if abs(byr - 27.5) < 0.01:
            d = (0, 1)
        elif abs(byr - 20.5) < 0.01:
            d = (0, -1)
        elif abs(bxr - 81.5) < 0.01:
            d = (1, 0)
        elif abs(bxr - 74.5) < 0.01:
            d = (-1, 0)
        if d:
            r.add_keepout("F.Cu", Shape("seg", (x, y, x + 3.0 * d[0], y + 3.0 * d[1], 0.3), None, "F.Cu"))
            n_exit += 1
    log(f"  hookup: {n_exit} outer-ring ball exits kept open")

    def centre(comp):
        bbs = [FR.item_bbox(r.items[k]) for k in comp]
        return (min(q[0] for q in bbs) + max(q[2] for q in bbs)) / 2, \
               (min(q[1] for q in bbs) + max(q[3] for q in bbs)) / 2

    def gap(ca, cb):
        best = 1e9
        for ka in ca:
            sa = Shape(*r.items[ka]["shape"], None, "x")
            for kb in cb:
                best = min(best, distance(sa, Shape(*r.items[kb]["shape"], None, "x")))
        return best
    n_t = n_v = 0
    for net, widths in HOOKUP_NETS.items():
        dead = set()
        while True:
            cs = list(FR.components(r.items).get(net, {}).values())
            near = [c for c in cs if max(abs(centre(c)[0] - u.x), abs(centre(c)[1] - u.y)) <= reach]
            if len(near) < 2:
                break
            pairs = []
            for i, ca in enumerate(near):
                key = tuple(sorted(ca))
                if key in dead:
                    continue
                others = [c for j, c in enumerate(near) if j != i]
                pairs.append((min(gap(ca, cb) for cb in others), i, key))
            if not pairs:
                break
            pairs.sort()
            d, i, key = pairs[0]
            ca = near[i]
            cb = [k for j, c in enumerate(near) if j != i for k in c]
            res = None
            for w in widths:
                res = r.checked_route(net, ca, cb, w, via=(0.4, 0.2), via_cost=40, margins=(1.0, 2.0))
                if res is not None:
                    break
            if res is not None:
                length = sum(math.hypot(t["x2"] - t["x1"], t["y2"] - t["y1"]) for t in res[0])
                if length > 1.8 * d + 1.5:          # a detour: leave it to the autorouter
                    res = None
            if res is None:
                dead.add(key)
                continue
            t_new, v_new = res
            for t in t_new:
                t["locked"] = True
            for v in v_new:
                v["locked"] = True
            r.add(t_new, v_new)
            for t in t_new:
                cop.add_track(t)
            for v in v_new:
                cop.add_via(v)
            n_t += len(t_new)
            n_v += len(v_new)
        left = [c for c in FR.components(r.items).get(net, {}).values()
                if max(abs(centre(c)[0] - u.x), abs(centre(c)[1] - u.y)) <= reach]
        log(f"  hookup {net}: {len(left)} island(s) left near the BGA")
    b.tracks = list(b.tracks) + [t for t in r.new_tracks]
    b.vias = list(b.vias) + [v for v in r.new_vias]
    log(f"  hookup: {n_t} tracks, {n_v} vias")


def add_zones(L, signal_pours=True):
    """In1 = GND plane, In2 = +3V3 plane; optional GND pours on the outer layers (left out
    of the DSN handed to the autorouter, which would treat them as solid copper)."""
    b = L.b
    b.zones = [z for z in b.zones if False]
    pts = b.outline
    for ln in PLANE_NETS["GND"]:
        b.zones.append(dict(net="GND", layers=[ln], pts=pts, priority=0, clearance=0.2, min_w=0.2,
                            name=f"GND plane {ln.split(chr(46))[0]}"))
    for ln in PLANE_NETS["+3V3"]:
        b.zones.append(dict(net="+3V3", layers=[ln], pts=pts, priority=0, clearance=0.2, min_w=0.2,
                            name=f"+3V3 plane {ln.split(chr(46))[0]}"))
    if signal_pours:
        b.zones.append(dict(net="GND", layers=["F.Cu"], pts=pts, priority=0, clearance=0.2, min_w=0.2,
                            name="GND top"))
        b.zones.append(dict(net="GND", layers=["B.Cu"], pts=pts, priority=0, clearance=0.2, min_w=0.2,
                            name="GND bottom"))
    x0, y0, x1, y1 = RF_CORRIDOR
    b.zones.append(dict(keepout=dict(tracks="allowed", vias="allowed", pads="allowed",
                                     copperpour="not_allowed", footprints="allowed"),
                        layers=["F.Cu"], pts=[A(x0, y0), A(x1, y0), A(x1, y1), A(x0, y1)],
                        name="RF microstrip: no top pour"))


def build_from(board_path):
    """The layout model with the footprint positions taken from an existing board file, so
    the post-routing tools (finishing router, stitching) stay valid when the placement code
    has changed since that board was generated."""
    from sexp import parse, find, find_all
    doc = parse(open(board_path).read())
    pos = {}
    for node in find_all(doc, "footprint"):
        ref = None
        for p in find_all(node, "property"):
            if str(p[1]) == "Reference":
                ref = str(p[2])
        at = find(node, "at")
        if ref is None or at is None:
            continue
        pos[ref] = (float(at[1]), float(at[2]), float(at[3]) if len(at) > 3 else 0.0)
    L = build(given=pos)
    moved = 0
    for ref, (x, y, rot) in pos.items():
        if ref not in L.b.fps:
            continue
        fp = L.b.fps[ref]
        if abs(fp.x - x) > 1e-4 or abs(fp.y - y) > 1e-4 or abs((fp.rot - rot) % 360) > 1e-3:
            fp.place(x, y, rot)
            moved += 1
    if moved:
        print(f"build_from: {moved} footprints re-placed from {os.path.basename(board_path)}")
    return L


def write_pro(path):
    import json
    src = os.path.join(HERE, "steering_wheel.kicad_pro")
    pro = json.load(open(src))
    board_rules.apply(pro)
    pro["meta"]["filename"] = os.path.basename(path)
    with open(path, "w") as fh:
        json.dump(pro, fh, indent=2)
        fh.write("\n")


def save(L, name):
    os.makedirs(BUILD, exist_ok=True)
    path = os.path.join(BUILD, name + ".kicad_pcb")
    L.b.write(path, stackup_jlc08161h_7628())
    write_pro(os.path.join(BUILD, name + ".kicad_pro"))
    return path


if __name__ == "__main__" and "place" in sys.argv[1:]:
    # placement only: footprints, outline, holes, the locked pre-placed power/RF/BGA-edge
    # copper that belongs to the floorplan, silkscreen and plane zones
    L = build()
    silk.place_references(L.b)
    add_zones(L)
    print("placed:", save(L, "placed"))
elif __name__ == "__main__":
    L = build()
    save(L, "placed")
    prerouting(L)
    silk.place_references(L.b)
    add_zones(L, signal_pours=False)
    save(L, "prerouted_dsn")
    add_zones(L)
    save(L, "prerouted")
    print("tracks", len(L.b.tracks), "vias", len(L.b.vias))
