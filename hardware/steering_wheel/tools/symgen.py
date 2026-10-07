"""Generate the project symbol library (lib/steering_wheel.kicad_sym).

Symbols are written in the KiCad 9 library format (version 20241209), which KiCad 10
opens natively. Every IC symbol is a plain rectangle with pins on its sides; the RW612
is split into five units (GPIO 0-31, GPIO 32-63, system, RF, power).

Pin numbers for the RW612 come from rw612_balls.json, extracted from NXP's FRDM-RW612
schematic (docs/FRDM-RW612-SCH.pdf) and cross-checked against its net labels.
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from sexp import QStr, dumps  # noqa: E402

G = 2.54
FONT = ["effects", ["font", ["size", "1.27", "1.27"]]]


def f(v):
    s = f"{v:.3f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def prop(name, value, x, y, hide=False, justify=None):
    eff = ["effects", ["font", ["size", "1.27", "1.27"]]]
    if justify:
        eff.append(["justify", justify])
    if hide:
        eff.append(["hide", "yes"])
    return ["property", QStr(name), QStr(value), ["at", f(x), f(y), "0"], eff]


def pin(num, name, etype, x, y, angle, length=G):
    return ["pin", etype, "line", ["at", f(x), f(y), str(angle)], ["length", f(length)],
            ["name", QStr(name), FONT], ["number", QStr(num), FONT]]


def stacked_type(t, k):
    """Duplicate pins stacked on one spot: only the first keeps an output type,
    the rest become passive (avoids power_out <-> power_out ERC conflicts)."""
    if k > 0 and t in ("power_out", "output", "tri_state"):
        return "passive"
    return t


def text_w(s):
    return len(s) * 1.05


def ic_symbol(name, units, ref="U", footprint="", datasheet="", descr="", value=None,
              extra_props=None, pin_len=G):
    """units: list of dicts with keys left/right/top/bottom -> list of pin specs.

    A pin spec is (number, name, type) or a list of such tuples (stacked at one spot),
    or None for a one-grid gap.
    """
    sym = ["symbol", QStr(name), ["pin_names", ["offset", "1.016"]],
           ["exclude_from_sim", "no"], ["in_bom", "yes"], ["on_board", "yes"]]
    bodies = []
    top_y_max = 0
    for ui, u in enumerate(units, start=1):
        L, R = u.get("left", []), u.get("right", []),
        T, B = u.get("top", []), u.get("bottom", [])
        nrows = max(len(L), len(R), 1)
        ncols = max(len(T), len(B), 0)

        def names(side):
            out = []
            for p in side:
                if p is None:
                    continue
                p0 = p[0] if isinstance(p, list) else p
                out.append(p0[1])
            return out

        lw = max([text_w(n) for n in names(L)] + [0])
        rw = max([text_w(n) for n in names(R)] + [0])
        tw = max([text_w(n) for n in names(T) + names(B)] + [0])
        width = max(lw + rw + 4 * G, (ncols + 1) * G, tw * 0 + 6 * G)
        width = G * 2 * round(width / (2 * G) + 0.49)
        height_rows = nrows + 1
        top_extra = (tw + G) if T else 0
        bot_extra = (tw + G) if B else 0
        height = height_rows * G + top_extra + bot_extra
        height = G * 2 * round(height / (2 * G) + 0.49)
        x0, x1 = -width / 2, width / 2
        y1 = height / 2
        y0 = -height / 2
        # snap to grid
        y1 = G * round(y1 / G)
        y0 = y1 - height
        top_y_max = max(top_y_max, y1)
        body = ["symbol", QStr(f"{name}_{ui}_1"),
                ["rectangle", ["start", f(x0), f(y1)], ["end", f(x1), f(y0)],
                 ["stroke", ["width", "0.254"], ["type", "default"]],
                 ["fill", ["type", "background"]]]]
        # side pins
        def place_side(side, xpin, angle):
            y = y1 - G - top_extra
            y = G * round(y / G)
            for p in side:
                if p is not None:
                    for k, q in enumerate(p if isinstance(p, list) else [p]):
                        body.append(pin(q[0], q[1], stacked_type(q[2], k), xpin, y, angle, pin_len))
                y -= G

        place_side(L, x0 - pin_len, 0)
        place_side(R, x1 + pin_len, 180)

        def place_tb(side, ypin, angle):
            n = len(side)
            x = -G * ((n - 1) // 2)
            x = G * round(x / G)
            for p in side:
                if p is not None:
                    for k, q in enumerate(p if isinstance(p, list) else [p]):
                        body.append(pin(q[0], q[1], stacked_type(q[2], k), x, ypin, angle, pin_len))
                x += G

        place_tb(T, y1 + pin_len, 270)
        place_tb(B, y0 - pin_len, 90)
        if u.get("unit_name"):
            body.insert(2, ["unit_name", QStr(u["unit_name"])])
        bodies.append(body)
    sym.append(prop("Reference", ref, 0, top_y_max + 1.27, justify="bottom"))
    sym.append(prop("Value", value or name, 0, -(top_y_max + 1.27), justify="top"))
    sym.append(prop("Footprint", footprint, 0, 0, hide=True))
    sym.append(prop("Datasheet", datasheet, 0, 0, hide=True))
    sym.append(prop("Description", descr, 0, 0, hide=True))
    for k, v in (extra_props or {}).items():
        sym.append(prop(k, v, 0, 0, hide=True))
    sym.extend(bodies)
    sym.append(["embedded_fonts", "no"])
    return sym


def power_symbol(netname, ground=False):
    """Global power symbol (like power:+3V3 / power:GND) for project-specific rails."""
    s = ["symbol", QStr(netname), ["power"], ["pin_numbers", ["hide", "yes"]],
         ["pin_names", ["offset", "0"], ["hide", "yes"]],
         ["exclude_from_sim", "no"], ["in_bom", "yes"], ["on_board", "yes"],
         prop("Reference", "#PWR", 0, -3.81 if not ground else -6.35, hide=True),
         prop("Value", netname, 0, 3.556 if not ground else -3.81),
         prop("Footprint", "", 0, 0, hide=True),
         prop("Datasheet", "", 0, 0, hide=True),
         prop("Description", f'Power symbol creates a global label with name "{netname}"', 0, 0,
              hide=True),
         prop("ki_keywords", "global power", 0, 0, hide=True)]
    stroke = ["stroke", ["width", "0"], ["type", "default"]]
    fill = ["fill", ["type", "none"]]
    s.append(["symbol", QStr(f"{netname}_0_1"),
              ["polyline", ["pts", ["xy", "-0.762", "1.27"], ["xy", "0", "2.54"]], stroke, fill],
              ["polyline", ["pts", ["xy", "0", "2.54"], ["xy", "0.762", "1.27"]], stroke, fill],
              ["polyline", ["pts", ["xy", "0", "0"], ["xy", "0", "2.54"]], stroke, fill]])
    s.append(["symbol", QStr(f"{netname}_1_1"),
              ["pin", "power_in", "line", ["at", "0", "0", "90"], ["length", "0"],
               ["name", QStr(netname), FONT], ["number", QStr("1"), FONT]]])
    s.append(["embedded_fonts", "no"])
    return s


def switch_push():
    stroke = ["stroke", ["width", "0"], ["type", "default"]]
    fill = ["fill", ["type", "none"]]
    s = ["symbol", QStr("SW_Push"), ["pin_numbers", ["hide", "yes"]],
         ["pin_names", ["offset", "1.016"], ["hide", "yes"]],
         ["exclude_from_sim", "no"], ["in_bom", "yes"], ["on_board", "yes"],
         prop("Reference", "SW", 1.27, 2.54, justify="left"),
         prop("Value", "SW_Push", 0, -1.524),
         prop("Footprint", "", 0, 5.08, hide=True),
         prop("Datasheet", "~", 0, 5.08, hide=True),
         prop("Description", "Push button switch, normally open, two pins", 0, 0, hide=True),
         ["symbol", QStr("SW_Push_0_1"),
          ["circle", ["center", "-2.032", "0"], ["radius", "0.508"], ["stroke", ["width", "0"], ["type", "default"]], fill],
          ["circle", ["center", "2.032", "0"], ["radius", "0.508"], ["stroke", ["width", "0"], ["type", "default"]], fill],
          ["polyline", ["pts", ["xy", "0", "1.27"], ["xy", "0", "3.048"]], stroke, fill],
          ["polyline", ["pts", ["xy", "2.54", "1.27"], ["xy", "-2.54", "1.27"]], stroke, fill]],
         ["symbol", QStr("SW_Push_1_1"),
          pin("1", "1", "passive", -5.08, 0, 0), pin("2", "2", "passive", 5.08, 0, 180)],
         ["embedded_fonts", "no"]]
    return s


def switch_spdt():
    stroke = ["stroke", ["width", "0"], ["type", "default"]]
    fill = ["fill", ["type", "none"]]
    s = ["symbol", QStr("SW_SPDT"), ["pin_names", ["offset", "0"], ["hide", "yes"]],
         ["exclude_from_sim", "no"], ["in_bom", "yes"], ["on_board", "yes"],
         prop("Reference", "SW", 0, 5.08, justify="bottom"),
         prop("Value", "SW_SPDT", 0, -5.08, justify="top"),
         prop("Footprint", "", 0, 0, hide=True),
         prop("Datasheet", "~", 0, 0, hide=True),
         prop("Description", "Switch, single pole double throw (pin 2 = common)", 0, 0, hide=True),
         ["symbol", QStr("SW_SPDT_0_1"),
          ["circle", ["center", "-2.032", "0"], ["radius", "0.508"], stroke, fill],
          ["circle", ["center", "2.032", "2.54"], ["radius", "0.508"], stroke, fill],
          ["circle", ["center", "2.032", "-2.54"], ["radius", "0.508"], stroke, fill],
          ["polyline", ["pts", ["xy", "-1.524", "0.254"], ["xy", "1.778", "2.286"]], stroke, fill]],
         ["symbol", QStr("SW_SPDT_1_1"),
          pin("1", "A", "passive", 5.08, 2.54, 180), pin("2", "B", "passive", -5.08, 0, 0),
          pin("3", "C", "passive", 5.08, -2.54, 180)],
         ["embedded_fonts", "no"]]
    return s


def P(num, name, t="passive"):
    return (num, name, t)


def rw612_symbol(balls_json):
    d = json.load(open(balls_json))["pins"]
    by_name = {}
    for ball, nm in d.items():
        by_name.setdefault(nm, []).append(ball)

    def g(n):
        return P(by_name[f"GPIO[{n}]"][0], f"GPIO_{n}", "bidirectional")

    def stack(name, t):
        balls = sorted(by_name[name], key=lambda b: (b[0], int(b[1:])))
        return [P(b, name, t) for b in balls]

    u_gpio_a = {"unit_name": "GPIO 0-31",
                "left": [g(i) for i in range(0, 16)],
                "right": [g(i) for i in range(16, 32)]}
    u_gpio_b = {"unit_name": "GPIO 32-63",
                "left": [g(i) for i in range(32, 48)],
                "right": [g(i) for i in range(48, 64)]}
    u_sys = {"unit_name": "System / USB",
             "left": [P(by_name["XTAL_IN"][0], "XTAL_IN", "input"),
                      P(by_name["XTAL_OUT"][0], "XTAL_OUT", "output"), None,
                      P(by_name["PDN"][0], "PDN", "input"),
                      P(by_name["ISENSE"][0], "ISENSE", "passive"), None,
                      P(by_name["EXT_REQ"][0], "EXT_REQ/BOOT0", "bidirectional"),
                      P(by_name["EXT_GNT"][0], "EXT_GNT/BOOT1", "bidirectional"),
                      P(by_name["EXT_PRI"][0], "EXT_PRI/BOOT2", "bidirectional"),
                      P(by_name["EXT_FREQ/WCI-2_SOUT"][0], "EXT_FREQ/BOOT3", "bidirectional")],
             "right": [P(by_name["USB_DP"][0], "USB_DP", "bidirectional"),
                       P(by_name["USB_DM"][0], "USB_DM", "bidirectional"), None,
                       P(by_name["USB_ID"][0], "USB_ID", "input"),
                       P(by_name["USB_VBUS"][0], "USB_VBUS", "input")]}
    u_rf = {"unit_name": "RF",
            "left": [P(by_name["RF_CNTL0"][0], "RF_CNTL0", "bidirectional"),
                     P(by_name["RF_CNTL1"][0], "RF_CNTL1", "output"),
                     P(by_name["RF_CNTL2"][0], "RF_CNTL2", "bidirectional"),
                     P(by_name["RF_CNTL3"][0], "RF_CNTL3", "output")],
            "right": [P(by_name["RF_TR_2"][0], "RF_TR_2G4", "passive"),
                      P(by_name["RF_TR_5"][0], "RF_TR_5G", "passive"),
                      P(by_name["BRF_ANT"][0], "BRF_ANT", "passive")]}
    avdd18 = [b for b in by_name["AVDD18"] if b != "J15"]
    avdd18 = sorted(avdd18, key=lambda b: (b[0], int(b[1:])))
    u_pwr = {"unit_name": "Power",
             "left": [stack("VBAT", "power_in"), stack("VPA", "power_in"),
                      P("B8", "AVDD33_USB", "power_in"), None,
                      P("K1", "VIO_1", "power_in"), P("F1", "VIO_2", "power_in"),
                      P("A5", "VIO_3_AON", "power_in"), P("D1", "VIO_4_FLASH", "power_in"),
                      P("A4", "VIO_5_PSRAM", "power_in"), P("B15", "VIO_6", "power_in"),
                      P("R6", "VIO_RF", "power_in"), None,
                      P("A11", "BUCK18_VOUT", "power_out"), P("C11", "BUCK18_SENSE", "input"),
                      P("A9", "BUCK11_VOUT", "power_out"), P("B9", "BUCK11_SENSE", "input")],
             "right": [stack("VCORE", "power_in"), None,
                       P("B12", "VDD18", "power_in"), P("A7", "AVDD18_USB", "power_in"),
                       [P(b, "AVDD18", "power_in") for b in avdd18],
                       P("J15", "AVDD18_FILT", "power_in")],
             "bottom": [stack("GND", "power_in")]}
    # sanity: every ball used exactly once
    used = []
    for u in (u_gpio_a, u_gpio_b, u_sys, u_rf, u_pwr):
        for side in ("left", "right", "top", "bottom"):
            for p in u.get(side, []):
                if p is None:
                    continue
                for q in (p if isinstance(p, list) else [p]):
                    used.append(q[0])
    assert len(used) == len(set(used)) == 145, (len(used), len(set(used)))
    assert set(used) == set(d), set(d) ^ set(used)
    return ic_symbol(
        "RW612ET", [u_gpio_a, u_gpio_b, u_sys, u_rf, u_pwr], ref="U",
        footprint="steering_wheel:NXP_TFBGA145_8x8mm_P0.5mm",
        datasheet="https://www.nxp.com/products/RW612",
        descr="NXP RW612 wireless MCU (Wi-Fi 6 + BLE 5.4 + 802.15.4), TFBGA145 8x8 mm, 0.5 mm pitch",
        extra_props={"MPN": "RW612ETA2I"})


def build(out_path, balls_json):
    lib = ["kicad_symbol_lib", ["version", "20241209"], ["generator", QStr("steering_wheel_symgen")],
           ["generator_version", QStr("9.0")]]
    syms = []
    syms.append(rw612_symbol(balls_json))
    syms.append(ic_symbol(
        "BQ24074RGT", [{"left": [P("13", "IN", "power_in"), None, P("4", "~{CE}", "input"),
                                 P("6", "EN1", "input"), P("5", "EN2", "input"), None,
                                 P("16", "ISET", "passive"), P("12", "ILIM", "passive"),
                                 P("14", "TMR", "passive"), P("15", "ITERM", "passive")],
                        "right": [[P("10", "OUT", "power_out"), P("11", "OUT", "power_out")], None,
                                  [P("2", "BAT", "power_out"), P("3", "BAT", "power_out")], None,
                                  P("1", "TS", "input"), None,
                                  P("7", "~{PGOOD}", "open_collector"),
                                  P("9", "~{CHG}", "open_collector")],
                        "bottom": [[P("8", "VSS", "power_in"), P("17", "VSS", "power_in")]]}],
        footprint="Package_DFN_QFN:VQFN-16-1EP_3x3mm_P0.5mm_EP1.68x1.68mm",
        datasheet="https://www.ti.com/lit/ds/symlink/bq24074.pdf",
        descr="1.5 A single-cell Li-ion charger with dynamic power-path management, VQFN-16"))
    syms.append(ic_symbol(
        "MAX17048G+T10", [{"left": [P("3", "VDD", "power_in"), P("2", "CELL", "input"), None,
                                    P("6", "QSTRT", "input"), P("1", "CTG", "passive")],
                           "right": [P("8", "SDA", "bidirectional"), P("7", "SCL", "input"), None,
                                     P("5", "~{ALRT}", "open_collector")],
                           "bottom": [[P("4", "GND", "power_in"), P("9", "EP", "passive")]]}],
        footprint="Package_DFN_QFN:TDFN-8-1EP_2x2mm_P0.5mm_EP0.8x1.2mm",
        datasheet="https://www.analog.com/media/en/technical-documentation/data-sheets/MAX17048-MAX17049.pdf",
        descr="1-cell ModelGauge fuel gauge, I2C 0x36, TDFN-8 2x2"))
    syms.append(ic_symbol(
        "DW01A", [{"left": [P("5", "VCC", "power_in"), None, P("2", "CS", "input"),
                            P("4", "TD", "passive")],
                   "right": [P("1", "OD", "output"), P("3", "OC", "output")],
                   "bottom": [P("6", "GND", "power_in")]}],
        footprint="Package_TO_SOT_SMD:SOT-23-6",
        datasheet="https://www.lcsc.com/datasheet/lcsc_datasheet_1810101017_Fortune-Semicon-DW01A_C14213.pdf",
        descr="One-cell Li-ion protection IC (drives external dual N-MOSFET)"))
    syms.append(ic_symbol(
        "FS8205A", [{"left": [[P("2", "S1", "passive"), P("3", "S1", "passive")],
                              P("4", "G1", "input"), None,
                              P("5", "G2", "input"), [P("6", "S2", "passive"), P("7", "S2", "passive")]],
                     "right": [[P("1", "D12", "passive"), P("8", "D12", "passive")]]}],
        ref="Q", footprint="Package_SO:TSSOP-8_4.4x3mm_P0.65mm",
        datasheet="https://www.lcsc.com/datasheet/lcsc_datasheet_1810101017_Fortune-Semicon-FS8205A_C32254.pdf",
        descr="Dual N-channel MOSFET, common drain, 20 V 6 A, TSSOP-8 (battery protection switch)"))
    syms.append(ic_symbol(
        "TPS63001", [{"left": [P("5", "VIN", "power_in"), P("8", "VINA", "power_in"), None,
                               P("6", "EN", "input"), P("7", "PS/SYNC", "input"), None,
                               P("4", "L1", "passive")],
                      "right": [P("1", "VOUT", "power_out"), P("10", "FB", "input"), None, None, None,
                                None, P("2", "L2", "passive")],
                      "bottom": [P("9", "GND", "power_in"), [P("3", "PGND", "power_in"),
                                                             P("11", "PAD", "passive")]]}],
        footprint="Package_SON:VSON-10-1EP_3x3mm_P0.5mm_EP1.65x2.4mm",
        datasheet="https://www.ti.com/lit/ds/symlink/tps63001.pdf",
        descr="Buck-boost converter, fixed 3.3 V, 1.7 A switch, VSON-10 3x3"))
    syms.append(ic_symbol(
        "TPS61023", [{"left": [P("3", "VIN", "power_in"), P("2", "EN", "input"), None,
                               P("5", "SW", "passive")],
                      "right": [P("6", "VOUT", "power_out"), P("1", "FB", "input")],
                      "bottom": [P("4", "GND", "power_in")]}],
        footprint="Package_TO_SOT_SMD:SOT-563",
        datasheet="https://www.ti.com/lit/ds/symlink/tps61023.pdf",
        descr="Synchronous boost converter, 3.7 A valley limit, true output disconnect, SOT-563"))
    syms.append(ic_symbol(
        "USBLC6-2SC6", [{"left": [P("1", "I/O1", "passive"), P("3", "I/O2", "passive")],
                         "right": [P("6", "I/O1", "passive"), P("4", "I/O2", "passive")],
                         "top": [P("5", "VBUS", "passive")],
                         "bottom": [P("2", "GND", "passive")]}],
        footprint="Package_TO_SOT_SMD:SOT-23-6",
        datasheet="https://www.st.com/resource/en/datasheet/usblc6-2.pdf",
        descr="Low-capacitance ESD protection for USB 2.0 data lines + VBUS, SOT-23-6"))
    syms.append(ic_symbol(
        "74AHCT1G125", [{"left": [P("2", "A", "input"), P("1", "~{OE}", "input")],
                         "right": [P("4", "Y", "tri_state")],
                         "top": [P("5", "VCC", "power_in")],
                         "bottom": [P("3", "GND", "power_in")]}],
        footprint="Package_TO_SOT_SMD:SOT-23-5",
        datasheet="https://www.ti.com/lit/ds/symlink/sn74ahct1g125.pdf",
        descr="Single bus buffer, TTL-level input (3.3 V -> 5 V level shift), SOT-23-5"))
    syms.append(ic_symbol(
        "W25Q512JVFIQ", [{"left": [P("7", "~{CS}", "input"), P("16", "CLK", "input"), None,
                                   P("3", "~{RESET}", "input")],
                          "right": [P("15", "DI/IO0", "bidirectional"), P("8", "DO/IO1", "bidirectional"),
                                    P("9", "~{WP}/IO2", "bidirectional"),
                                    P("1", "~{HOLD}/IO3", "bidirectional")],
                          "top": [P("2", "VCC", "power_in")],
                          "bottom": [P("10", "GND", "power_in")],
                          }],
        footprint="Package_SO:SOIC-16W_7.5x10.3mm_P1.27mm",
        datasheet="https://www.winbond.com/hq/product/code-storage-flash-memory/serial-nor-flash/",
        descr="512 Mbit quad-SPI NOR flash, SOIC-16 300 mil (same part as FRDM-RW612)"))
    syms.append(ic_symbol(
        "APS6404L-3SQN-SN", [{"left": [P("1", "~{CE}", "input"), P("6", "SCLK", "input")],
                              "right": [P("5", "SI/SIO0", "bidirectional"),
                                        P("2", "SO/SIO1", "bidirectional"),
                                        P("3", "SIO2", "bidirectional"), P("7", "SIO3", "bidirectional")],
                              "top": [P("8", "VDD", "power_in")],
                              "bottom": [P("4", "VSS", "power_in")]}],
        footprint="Package_SO:SOIC-8_3.9x4.9mm_P1.27mm",
        datasheet="https://www.apmemory.com/",
        descr="64 Mbit quad-SPI PSRAM, 3.3 V, SOP-8 (same part as FRDM-RW612)"))
    syms.append(ic_symbol(
        "SKY13323-378LF", [{"left": [P("1", "OUTPUT1", "passive"), None, P("3", "OUTPUT2", "passive")],
                            "right": [P("5", "INPUT", "passive"), None, P("6", "VCTL1", "input"),
                                      P("4", "VCTL2", "input")],
                            "bottom": [[P("2", "GND", "passive"), P("7", "EPAD", "passive")]]}],
        footprint="steering_wheel:Skyworks_MLPD-6-1EP_1x1mm",
        datasheet="https://www.skyworksinc.com/products/switches/sky13323-378lf",
        descr="0.1-3 GHz GaAs SPDT switch (Wi-Fi 2.4 GHz / BLE-15.4 TDM), 6-pin MLPD"))
    syms.append(ic_symbol(
        "DPX165950DT-8085D1", [{"left": [P("4", "HB", "passive"), None, P("6", "LB", "passive")],
                                "right": [P("2", "COM", "passive")],
                                "bottom": [[P("1", "GND", "passive"), P("3", "GND", "passive"),
                                            P("5", "GND", "passive")]]}],
        ref="FL", footprint="steering_wheel:TDK_DPX165950DT_1608_6T",
        datasheet="https://product.tdk.com/en/search/rf/rf/diplexer/",
        descr="TDK multilayer diplexer 2.4 GHz / 5 GHz, 1.6x0.8 mm (same as FRDM-RW612)"))
    syms.append(switch_push())
    syms.append(switch_spdt())
    for rail in ["VBAT", "VSYS", "VSYS_SW", "VCORE", "LCD_VCC"]:
        syms.append(power_symbol(rail))
    lib.extend(syms)
    with open(out_path, "w") as fh:
        fh.write(dumps(lib) + "\n")
    return [s[1] for s in syms]


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(here)
    names = build(os.path.join(root, "lib", "steering_wheel.kicad_sym"),
                  os.path.join(root, "lib", "rw612_balls.json"))
    print("wrote", len(names), "symbols:", ", ".join(names))
