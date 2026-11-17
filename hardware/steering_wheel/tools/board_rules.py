"""Board design rules + net classes for the steering wheel board, written into the
KiCad project file (.kicad_pro). Numbers target JLCPCB 8-layer (JLC08161H-7628, 1.6 mm)
with margin: 0.1 mm min clearance, 0.1 mm min track, 0.35/0.2 mm vias (BGA fan-out only).
The BGA drill is 0.2 mm, not 0.15 mm: through 1.6 mm that would be a 10.7:1 aspect ratio,
over JLCPCB's 10:1 limit. 0.2 mm gives 8:1.

    python tools/board_rules.py steering_wheel.kicad_pro
"""
import json
import sys

# name: (clearance, track, via_d, via_drill, patterns)
NETCLASSES = {
    "Default": (0.1, 0.127, 0.4, 0.2, []),
    "Power": (0.15, 0.4, 0.6, 0.3, ["VBUS", "VSYS", "VSYS_SW", "VBAT", "+5V", "LCD_VCC", "/power/BAT_N",
                                     "/power/BST_SW", "/power/BB_L1", "/power/BB_L2", "/hmi/VCC_BTN"]),
    "CorePower": (0.1, 0.15, 0.4, 0.2, ["+1V8", "VCORE", "/mcu_core/BUCK18_SW", "/mcu_core/BUCK11_SW",
                                        "/mcu_core/AVDD18_J15"]),
    "USB": (0.1, 0.2, 0.4, 0.2, ["USB_D_P", "USB_D_N"]),
    "RF": (0.12, 0.34, 0.4, 0.2, ["/rf/RF5G_SOC", "/rf/RF5G_FLT", "/rf/RF2G_SOC", "/rf/RF2G_FLT",
                                   "/rf/BLE_SOC", "/rf/BLE_FLT", "/rf/RFSW_OUT1", "/rf/RFSW_OUT2",
                                   "/rf/RFSW_IN", "/rf/DPX_LB", "/rf/ANT_COM", "/rf/ANT_FEED"]),
}

RULES = {
    "allow_blind_buried_vias": False,
    "allow_microvias": False,
    "max_error": 0.005,
    "min_clearance": 0.1,
    "min_connection": 0.0,
    "min_copper_edge_clearance": 0.3,
    "min_groove_width": 0.0,
    "min_hole_clearance": 0.15,
    "min_hole_to_hole": 0.25,
    "min_microvia_diameter": 0.2,
    "min_microvia_drill": 0.1,
    "min_resolved_spokes": 1,
    "min_silk_clearance": 0.0,
    "min_text_height": 0.6,
    "min_text_thickness": 0.08,
    "min_through_hole_diameter": 0.2,
    "min_track_width": 0.1,
    "min_via_annular_width": 0.075,
    "min_via_diameter": 0.35,
    "solder_mask_to_copper_clearance": 0.0,
    "use_height_for_length_calcs": True,
}


def netclass(name, clearance, track, via_d, via_drill, priority):
    return {
        "bus_width": 12, "clearance": clearance, "diff_pair_gap": 0.15, "diff_pair_via_gap": 0.25,
        "diff_pair_width": 0.2, "line_style": 0, "microvia_diameter": 0.25, "microvia_drill": 0.1,
        "name": name, "pcb_color": "rgba(0, 0, 0, 0.000)", "priority": priority,
        "schematic_color": "rgba(0, 0, 0, 0.000)", "track_width": track, "via_diameter": via_d,
        "via_drill": via_drill, "wire_width": 6,
    }


def apply(pro):
    ds = pro.setdefault("board", {}).setdefault("design_settings", {})
    ds.setdefault("rules", {}).update(RULES)
    ds["track_widths"] = [0.0, 0.127, 0.15, 0.2, 0.25, 0.34, 0.5, 0.8]
    ds["via_dimensions"] = [{"diameter": 0.0, "drill": 0.0}, {"diameter": 0.3, "drill": 0.15},
                            {"diameter": 0.4, "drill": 0.2}, {"diameter": 0.6, "drill": 0.3}]
    ds["diff_pair_dimensions"] = [{"gap": 0.0, "via_gap": 0.0, "width": 0.0},
                                  {"gap": 0.15, "via_gap": 0.25, "width": 0.2}]
    d = ds.setdefault("defaults", {})
    d.setdefault("zones", {})["min_clearance"] = 0.2
    sev = ds.setdefault("rule_severities", {})
    # the footprint-library comparison needs the user's own KiCad library tables; keep it
    # a warning rather than hiding it
    sev.setdefault("lib_footprint_mismatch", "warning")
    ns = pro.setdefault("net_settings", {})
    classes, patterns = [], []
    for i, (name, (cl, tw, vd, vdr, pats)) in enumerate(NETCLASSES.items()):
        classes.append(netclass(name, cl, tw, vd, vdr, 2147483647 if name == "Default" else i))
        patterns += [{"netclass": name, "pattern": p} for p in pats]
    ns["classes"] = classes
    ns["netclass_patterns"] = patterns
    ns["netclass_assignments"] = None
    ns.setdefault("meta", {"version": 4})
    return pro


def netclass_of(net):
    for name, (_, _, _, _, pats) in NETCLASSES.items():
        if net in pats:
            return name
    return "Default"


if __name__ == "__main__":
    p = sys.argv[1]
    pro = json.load(open(p))
    apply(pro)
    with open(p, "w") as fh:
        json.dump(pro, fh, indent=2)
        fh.write("\n")
