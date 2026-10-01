#!/usr/bin/env python3
"""IRT steering-wheel controller -- SKiDL design (single source of truth).

    python steering_wheel.py        # writes netlist, KiCad 10 schematic, BOM

Blocks
  1. power     USB-C in, BQ24074 charger + power path, 1S protection, MAX17048 gauge,
               soft power switch, TPS63001 3.3 V buck-boost, TPS61023 5 V boost
  2. mcu_core  RW612 supplies/decoupling, internal-buck inductors, 40 MHz + 32 kHz
               crystals, reset/ISP, SWD + UART headers
  3. mcu_io    RW612 GPIO banks, QSPI flash, PSRAM, user LED
  4. rf        FRDM-RW612 RF front end (Wi-Fi 2.4/5 GHz + BLE) to a U.FL connector
  5. hmi       4.0" ST7796S LCD + FT6336U touch header, NeoPixel output, JST PH headers
               for 2 off-board EC11 encoders, 8 active buttons (on a TCA9555 I/O
               expander) and 2 passive buttons, expansion header

The RW612 section follows NXP's FRDM-RW612 rev V2 schematic (same flash, PSRAM,
crystals, strap and RF parts, same GPIO for the LCD, I2C, UART and SWD) so that the
MCUXpresso SDK / Zephyr FRDM-RW612 board support runs with minimal changes.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "tools"))

from skidl import KICAD9, POWER, Net, Part, lib_search_paths, set_default_tool, ERC  # noqa: E402
from skidl import generate_netlist  # noqa: E402
import builtins  # noqa: E402  (skidl publishes default_circuit / NC as builtins)

set_default_tool(KICAD9)
lib_search_paths[KICAD9] = [os.path.join(HERE, "lib"), os.path.join(HERE, "lib", "std")]

SW = "steering_wheel"

# ---------------------------------------------------------------- footprints
R0201 = "Resistor_SMD:R_0201_0603Metric"
R0402 = "Resistor_SMD:R_0402_1005Metric"
R0603 = "Resistor_SMD:R_0603_1608Metric"
C0201 = "Capacitor_SMD:C_0201_0603Metric"
C0402 = "Capacitor_SMD:C_0402_1005Metric"
C0603 = "Capacitor_SMD:C_0603_1608Metric"
C0805 = "Capacitor_SMD:C_0805_2012Metric"
L0201 = "Inductor_SMD:L_0201_0603Metric"
LED0603 = "LED_SMD:LED_0603_1608Metric"
TP = "TestPoint:TestPoint_Pad_D1.0mm"

# ---------------------------------------------------------------- sheet / group context
SHEETS = [  # (file stem, title)
    ("power", "Power: USB-C, charger, battery, regulators"),
    ("mcu_core", "RW612 core: supplies, clocks, reset, debug"),
    ("mcu_io", "RW612 I/O, QSPI flash, PSRAM"),
    ("rf", "RF front end (Wi-Fi 2.4/5 GHz + BLE) and U.FL"),
    ("hmi", "HMI: LCD + touch, NeoPixel, encoder and button connectors"),
]
_ctx = {"sheet": None, "group": None}


class block:
    """with block('power', 'Charger'):  -> parts created inside get these tags."""

    def __init__(self, sheet, group):
        self.sheet, self.group = sheet, group

    def __enter__(self):
        _ctx["sheet"], _ctx["group"] = self.sheet, self.group

    def __exit__(self, *a):
        _ctx["sheet"], _ctx["group"] = None, None


_order = [0]


def mk(lib, name, value=None, fp=None, mpn=None, mfr=None, dnp=False, note=None, lcsc=None,
       **pins):
    kw = {"footprint": fp} if fp else {}
    p = Part(lib, name, value=value or name, tag=f"p{_order[0]:04d}", **kw)
    p.sw_sheet = _ctx["sheet"]
    p.sw_group = _ctx["group"]
    p.sw_order = _order[0]
    _order[0] += 1
    p.sw_dnp = dnp
    p.sw_props = {}
    if mpn:
        p.sw_props["MPN"] = mpn
    if mfr:
        p.sw_props["Manufacturer"] = mfr
    if lcsc:
        p.sw_props["LCSC"] = lcsc
    if note:
        p.sw_props["Note"] = note
    for pin_id, net in pins.items():
        connect(p, pin_id, net)
    return p


def connect(p, pin_id, net):
    pin_id = str(pin_id).lstrip("_")
    pins = p[pin_id]
    pins = pins if isinstance(pins, list) else [pins]
    for pn in pins:
        if net is None:
            pn += builtins.default_circuit.NC
        else:
            pn += N(net) if isinstance(net, str) else net


_nets = {}


def N(name):
    if name not in _nets:
        _nets[name] = Net(name)
    return _nets[name]


def R(value, a, b, fp=R0402, **kw):
    return mk("Device", "R", value, fp, _1=a, _2=b, **kw)


def C(value, a, b, fp=C0402, **kw):
    return mk("Device", "C", value, fp, _1=a, _2=b, **kw)


def L(value, a, b, fp, **kw):
    return mk("Device", "L", value, fp, _1=a, _2=b, **kw)


def LED(color, anode, cathode, **kw):
    return mk("Device", "LED", color, LED0603, _2=anode, _1=cathode, **kw)


FLAG_NETS = []


def flag(net):
    """Mark a net as driven: skidl ERC (drive=POWER) and a KiCad PWR_FLAG symbol on the
    schematic (added by the exporter; virtual, not on the board / in the netlist)."""
    N(net).drive = POWER
    FLAG_NETS.append((net, _ctx["sheet"]))


def testpoint(net, name):
    return mk("Connector", "TestPoint", name, TP, _1=net)


# =====================================================================================
# 1. POWER
# =====================================================================================
with block("power", "USB-C input (5 V sink, USB 2.0 data)"):
    usb = mk("Connector", "USB_C_Receptacle_USB2.0_16P", "USB-C",
             "Connector_USB:USB_C_Receptacle_HRO_TYPE-C-31-M-12",
             mpn="TYPE-C-31-M-12", lcsc="C165948", mfr="Korean Hroparts",
             VBUS="VBUS", GND="GND", SHIELD="GND", CC1="CC1", CC2="CC2",
             SBU1=None, SBU2=None)
    connect(usb, "A6", "USB_D_P"); connect(usb, "B6", "USB_D_P")
    connect(usb, "A7", "USB_D_N"); connect(usb, "B7", "USB_D_N")
    R("5.1k", "CC1", "GND", note="Rd: advertises USB-C sink")
    R("5.1k", "CC2", "GND", note="Rd: advertises USB-C sink")
    mk(SW, "USBLC6-2SC6", "USBLC6-2SC6", mpn="USBLC6-2SC6", lcsc="C7519", mfr="STMicroelectronics",
       _1="USB_D_P", _6="USB_D_P", _3="USB_D_N", _4="USB_D_N", _5="VBUS", _2="GND")
    C("10u 25V", "VBUS", "GND", C0805)
    R("1k", "VBUS", "USB_VBUS_SNS", note="limits current into RW612 USB_VBUS when the board is off")
    flag("VBUS")
    flag("GND")

with block("power", "Charger + power path (BQ24074, 1 A charge, 1.4 A input limit)"):
    mk(SW, "BQ24074RGT", "BQ24074RGTR", mpn="BQ24074RGTR", lcsc="C54313", mfr="Texas Instruments",
       IN="VBUS", OUT="VSYS", BAT="VBAT", VSS="GND", _4="GND", EN1="GND", EN2="VSYS",
       ISET="CHG_ISET", ILIM="CHG_ILIM", TMR="CHG_TMR", ITERM=None, TS="BAT_TS",
       _7="CHG_PG_N", _9="CHG_STAT_N")
    R("887 1%", "CHG_ISET", "GND", note="ICHG = 890/R = 1.0 A (590R -> 1.5 A)")
    R("1.13k 1%", "CHG_ILIM", "GND", note="IIN_MAX = 1610/R = 1.42 A")
    R("68k", "CHG_TMR", "GND", note="safety timer: fast charge ~9 h (4000 mAh)")
    R("10k", "BAT_TS", "GND", note="TS bias; remove if an NTC is fitted to J(NTC)")
    mk("Connector", "Conn_01x02_Pin", "BATT_NTC", "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical",
       dnp=True, note="optional 10k NTC (103AT) from battery pack", _1="BAT_TS", _2="GND")
    C("10u 10V", "VSYS", "GND", C0805)
    C("10u 10V", "VBAT", "GND", C0805)
    R("1k", "VSYS", "LED_PG_A")
    LED("green", "LED_PG_A", "CHG_PG_N", mpn="LED green 0603", note="USB power good")
    R("1k", "VSYS", "LED_CHG_A")
    LED("orange", "LED_CHG_A", "CHG_STAT_N", mpn="LED orange 0603", note="charging (off = full)")
    R("10k", "CHG_PG_N", "MCU_CHG_PG_N", note="series isolation to MCU")
    R("10k", "CHG_STAT_N", "MCU_CHG_STAT_N", note="series isolation to MCU")
    R("100k", "MCU_CHG_PG_N", "+3V3")
    R("100k", "MCU_CHG_STAT_N", "+3V3")

with block("power", "1S Li-ion battery 3.7 V 4000 mAh + protection"):
    mk("Connector", "Conn_01x02_Pin", "BATTERY",
       "Connector_JST:JST_PH_S2B-PH-SM4-TB_1x02-1MP_P2.00mm_Horizontal",
       mpn="S2B-PH-SM4-TB", lcsc="C295747", mfr="JST", note="pin 1 = BAT+, pin 2 = BAT-; check pack polarity!",
       _1="VBAT", _2="BAT_N")
    mk(SW, "DW01A", "DW01A", mpn="DW01A", lcsc="C351410", mfr="Fortune Semiconductor",
       VCC="PROT_VCC", GND="BAT_N", OD="PROT_OD", OC="PROT_OC", CS="PROT_CS", TD=None)
    R("100", "VBAT", "PROT_VCC")
    C("100n", "PROT_VCC", "BAT_N")
    R("1k", "GND", "PROT_CS")
    mk(SW, "FS8205A", "FS8205A", mpn="FS8205A", lcsc="C16052", mfr="Fortune Semiconductor",
       S1="BAT_N", G1="PROT_OD", G2="PROT_OC", S2="GND", D12=None)
    flag("PROT_VCC")
    flag("BAT_N")

with block("power", "Fuel gauge (MAX17048, I2C 0x36)"):
    mk(SW, "MAX17048G+T10", "MAX17048G+T10", mpn="MAX17048G+T10", lcsc="C2682616", mfr="Analog Devices",
       VDD="VBAT", CELL="VBAT", GND="GND", EP="GND", CTG="GND", QSTRT="GND",
       SDA="PM_SDA", SCL="PM_SCL", _5="FG_ALRT_N")
    C("100n", "VBAT", "GND")
    R("10k", "FG_ALRT_N", "+3V3")
    R("4.7k", "PM_SDA", "+3V3", note="PM I2C bus (FC0): gauge + expansion, independent of LCD power")
    R("4.7k", "PM_SCL", "+3V3")

with block("power", "Main power switch (charging still works when OFF)"):
    mk(SW, "SW_SPDT", "PWR", "Button_Switch_SMD:SW_SPDT_PCM12", mpn="PCM12SMTR", lcsc="C221841", mfr="C&K",
       note="ON = pins 1-2 closed", _2="PWR_SW_G", _1="GND", _3=None)
    mk("Transistor_FET", "Q_PMOS_GSD", "AO3401A", "Package_TO_SOT_SMD:SOT-23",
       mpn="AO3401A", lcsc="C15127", mfr="Alpha & Omega", G="PWR_SW_G", S="VSYS", D="VSYS_SW")
    R("100k", "VSYS", "PWR_SW_G")
    flag("VSYS_SW")

with block("power", "3.3 V buck-boost (TPS63001)"):
    mk(SW, "TPS63001", "TPS63001DRCR", mpn="TPS63001DRCR", lcsc="C28060", mfr="Texas Instruments",
       VIN="VSYS_SW", VINA="BB_VINA", EN="VSYS_SW", _7="GND", L1="BB_L1", L2="BB_L2",
       VOUT="+3V3", FB="+3V3", GND="GND", PGND="GND", PAD="GND")
    L("2.2u", "BB_L1", "BB_L2", "Inductor_SMD:L_Wuerth_MAPI-3015", mpn="74438335022", mfr="Wurth")
    R("100", "VSYS_SW", "BB_VINA")
    C("100n", "BB_VINA", "GND")
    C("10u 10V", "VSYS_SW", "GND", C0603)
    C("22u 6.3V", "+3V3", "GND", C0805)
    C("22u 6.3V", "+3V3", "GND", C0805)
    C("100n", "+3V3", "GND")
    flag("BB_VINA")

with block("power", "5 V boost for LCD + NeoPixels (TPS61023, MCU enable)"):
    mk(SW, "TPS61023", "TPS61023DRLR", mpn="TPS61023DRLR", lcsc="C919459", mfr="Texas Instruments",
       VIN="VSYS_SW", EN="BOOST_EN", SW="BST_SW", VOUT="+5V", FB="BST_FB", GND="GND")
    L("1u", "VSYS_SW", "BST_SW", "Inductor_SMD:L_Wuerth_MAPI-4020", mpn="74438357010", mfr="Wurth")
    R("732k 1%", "+5V", "BST_FB")
    R("100k 1%", "BST_FB", "GND")
    R("100k", "BOOST_EN", "GND", note="5 V off until firmware enables it")
    C("10u 10V", "VSYS_SW", "GND", C0603)
    C("22u 10V", "+5V", "GND", C0805)
    C("22u 10V", "+5V", "GND", C0805)

with block("power", "Test points"):
    for net in ["VBUS", "VBAT", "VSYS", "VSYS_SW", "+3V3", "+5V", "GND"]:
        testpoint(net, net)

# =====================================================================================
# 2./3./4. RW612
# =====================================================================================
with block("mcu_core", "RW612 supplies"):
    u = mk(SW, "RW612ET", "RW612ETA2I", mpn="RW612ETA2I", mfr="NXP")
    # which sheet each unit is drawn on (unit numbers from lib: 1..5)
    u.sw_unit_sheets = {1: "mcu_io", 2: "mcu_io", 3: "mcu_core", 4: "rf", 5: "mcu_core"}
    u.sw_unit_groups = {1: "RW612 GPIO", 2: "RW612 GPIO", 3: "Clocks, reset, boot",
                        4: "RW612 RF control + strap", 5: "RW612 supplies"}
    for nm in ["VBAT", "VPA", "AVDD33_USB", "VIO_1", "VIO_2", "VIO_3_AON", "VIO_4_FLASH",
               "VIO_5_PSRAM", "VIO_6", "VIO_RF"]:
        connect(u, nm, "+3V3")
    connect(u, "BUCK18_VOUT", "BUCK18_SW"); connect(u, "BUCK18_SENSE", "+1V8")
    connect(u, "BUCK11_VOUT", "BUCK11_SW"); connect(u, "BUCK11_SENSE", "VCORE")
    connect(u, "VCORE", "VCORE")
    connect(u, "VDD18", "+1V8"); connect(u, "AVDD18_USB", "+1V8"); connect(u, "AVDD18", "+1V8")
    connect(u, "AVDD18_FILT", "AVDD18_J15")
    connect(u, "GND", "GND")
    L("1u", "BUCK18_SW", "+1V8", "Inductor_SMD:L_Wuerth_MAPI-2010", mpn="74438323010", mfr="Wurth",
      note="internal 1.8 V buck")
    L("1u", "BUCK11_SW", "VCORE", "Inductor_SMD:L_Wuerth_MAPI-2010", mpn="74438323010", mfr="Wurth",
      note="internal 1.1 V core buck")
    flag("+1V8")
    flag("VCORE")

with block("mcu_core", "Decoupling: VBAT / VPA / VIO (3.3 V)"):
    C("22u 10V", "+3V3", "GND", C0603, note="VBAT A10/B10")
    C("100n", "+3V3", "GND", C0201, note="VBAT B10")
    C("100n", "+3V3", "GND", C0201, note="VBAT A10")
    C("10u 6.3V", "+3V3", "GND", C0402, note="VPA R15")
    C("100p", "+3V3", "GND", C0201, note="VPA R15")
    C("10u 6.3V", "+3V3", "GND", C0402, note="VPA R8")
    C("100p", "+3V3", "GND", C0201, note="VPA R8")
    C("100n", "+3V3", "GND", C0201, note="AVDD33_USB B8")
    for ball in ["K1 VIO_1", "F1 VIO_2", "A5 VIO_3", "D1 VIO_4", "A4 VIO_5", "B15 VIO_6", "R6 VIO_RF"]:
        C("100n", "+3V3", "GND", C0201, note=ball)

with block("mcu_core", "Decoupling: 1.8 V and VCORE"):
    C("22u 10V", "+1V8", "GND", C0603, note="1V8 buck output")
    C("10u 6.3V", "+1V8", "GND", C0402, note="AVDD18 M15 bulk")
    for ball in ["B12 VDD18", "A7 AVDD18_USB", "F14", "F15", "M15", "N15", "R10", "R12", "R11",
                 "R2", "R1", "P1", "K15", "L15"]:
        C("100n", "+1V8", "GND", C0201, note=ball)
    R("2.0 1%", "+1V8", "AVDD18_J15", note="AVDD18 J15 filter (FRDM R90)")
    C("2.2u", "AVDD18_J15", "GND", C0402)
    C("100n", "AVDD18_J15", "GND", C0201)
    flag("AVDD18_J15")
    C("22u 10V", "VCORE", "GND", C0603, note="core buck output")
    C("10u 6.3V", "VCORE", "GND", C0402)
    for ball in ["D15", "D4", "H1", "K10"]:
        C("100n", "VCORE", "GND", C0201, note=f"VCORE {ball}")
        C("330p", "VCORE", "GND", C0201, note=f"VCORE {ball}")

with block("mcu_core", "Clocks, reset, boot"):
    connect(u, "XTAL_IN", "XTAL40_IN"); connect(u, "XTAL_OUT", "XTAL40_OUT")
    connect(u, "PDN", "RESET_N"); connect(u, "ISENSE", "ISENSE")
    connect(u, "EXT_REQ/BOOT0", "ISP_N")
    connect(u, "EXT_GNT/BOOT1", None); connect(u, "EXT_PRI/BOOT2", None)
    connect(u, "EXT_FREQ/BOOT3", None)
    connect(u, "USB_DP", "USB_D_P"); connect(u, "USB_DM", "USB_D_N")
    connect(u, "USB_ID", None); connect(u, "USB_VBUS", "USB_VBUS_SNS")
    mk("Device", "Crystal_GND24", "40MHz", "Crystal:Crystal_SMD_2016-4Pin_2.0x1.6mm",
       mpn="8Q40070007", mfr="TXC", note="same crystal as FRDM-RW612, no load caps",
       _1="XTAL40_IN", _3="XTAL40_OUT", _2="GND", _4="GND")
    mk("Device", "Crystal", "32.768kHz", "Crystal:Crystal_SMD_3215-2Pin_3.2x1.5mm",
       mpn="830009678", mfr="Wurth", note="on GPIO22/23 like FRDM-RW612",
       _1="XTAL32K_IN", _2="XTAL32K_OUT")
    C("DNP", "XTAL32K_IN", "GND", C0402, dnp=True)
    C("DNP", "XTAL32K_OUT", "GND", C0402, dnp=True)
    R("6.04k 1%", "ISENSE", "GND", note="RW612 bias reference")
    R("51k", "RESET_N", "+3V3")
    C("100n", "RESET_N", "GND")
    mk(SW, "SW_Push", "RESET", "Button_Switch_SMD:SW_SPST_TL3342", mpn="TL3342F160QG",
       mfr="E-Switch", _1="RESET_N", _2="GND")
    R("100k", "ISP_N", "+1V8", note="CONFIG_HOST_BOOT[0]: 1 = boot flash")
    mk(SW, "SW_Push", "ISP", "Button_Switch_SMD:SW_SPST_TL3342", mpn="TL3342F160QG",
       mfr="E-Switch", note="hold + reset = ISP boot (UART/USB)", _1="ISP_N", _2="GND")

with block("mcu_core", "Debug (SWD) and console UART"):
    mk("Connector", "Conn_ARM_JTAG_SWD_10", "SWD",
       "Connector_PinHeader_1.27mm:PinHeader_2x05_P1.27mm_Vertical_SMD",
       mpn="Cortex 10-pin 1.27 mm", _1="+3V3", _2="SWDIO", _3="GND", _5="GND", _4="SWCLK",
       _6=None, _7=None, _8=None, _9="GND", _10="RESET_N")
    mk("Connector", "Conn_01x03_Pin", "UART", "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical",
       note="FC3 console: 1=TX(out) 2=RX(in) 3=GND", _1="UART_TX", _2="UART_RX", _3="GND")

GPIO = {  # RW612 GPIO -> net (None = not connected)
    0: "PM_SDA", 1: "LCD_BL_PWM", 2: "ENC1_A", 3: "ENC1_B", 4: "ENC1_SW", 5: "PM_SCL",
    6: None, 7: None, 8: None, 9: "NEOPIXEL_DATA", 10: None, 11: "BTN1_N",
    12: "LED_USER_N", 13: "SWCLK", 14: "SWDIO", 15: None, 16: "CTP_SDA", 17: "CTP_SCL",
    18: "CTP_INT_N", 19: "CTP_RST_N", 20: "EXP_GPIO20", 21: "EXP_GPIO21",
    22: "XTAL32K_IN", 23: "XTAL32K_OUT", 24: "UART_RX", 25: "BTN2_N", 26: "UART_TX",
    27: None, 28: "FLASH_CS_N", 29: None, 30: "FLASH_IO0", 31: "FLASH_IO1",
    32: "FLASH_IO2", 33: "FLASH_IO3", 34: "FLASH_CLK", 35: "PSRAM_CLK", 36: "PSRAM_CS_N",
    37: None, 38: "PSRAM_IO0", 39: "PSRAM_IO1", 40: "PSRAM_IO2", 41: "PSRAM_IO3",
    42: None, 43: None, 44: "LCD_RST_N", 45: None, 46: "LCD_MOSI", 47: "LCD_DC",
    48: "LCD_SCK", 49: "LCD_CS_N", 50: "ENC2_A", 51: "ENC2_B", 52: "ENC2_SW",
    53: None, 54: None, 55: "MCU_CHG_STAT_N", 56: "MCU_CHG_PG_N", 57: "FG_ALRT_N",
    58: "BOOST_EN", 59: None, 60: None, 61: None, 62: None,
    63: "BTN_INT_N",   # corner ball A15: escapes on the top layer, no fan-out via needed
}
with block("mcu_io", "RW612 GPIO"):
    for n, net in GPIO.items():
        connect(u, f"GPIO_{n}", net)

with block("mcu_io", "QSPI flash (FlexSPI port A, boot)"):
    mk(SW, "W25Q512JVFIQ", "W25Q512JVFIQ", mpn="W25Q512JVFIQ", lcsc="C2962011", mfr="Winbond",
       _7="FLASH_CS_N", CLK="FLASH_CLK", _15="FLASH_IO0", _8="FLASH_IO1", _9="FLASH_IO2",
       _1="FLASH_IO3", _3="FLASH_RST_N", VCC="+3V3", GND="GND")
    R("51k", "FLASH_CS_N", "+3V3")
    R("51k", "FLASH_IO2", "+3V3")
    R("51k", "FLASH_IO3", "+3V3")
    R("51k", "FLASH_RST_N", "+3V3")
    C("100n", "+3V3", "GND")

with block("mcu_io", "PSRAM (FlexSPI port B)"):
    mk(SW, "APS6404L-3SQN-SN", "APS6404L-3SQN-SN", mpn="APS6404L-3SQN-SN", lcsc="C3028887", mfr="AP Memory",
       _1="PSRAM_CS_N", SCLK="PSRAM_CLK", _5="PSRAM_IO0", _2="PSRAM_IO1", SIO2="PSRAM_IO2",
       SIO3="PSRAM_IO3", VDD="+3V3", VSS="GND")
    R("10k", "PSRAM_CS_N", "+3V3")
    C("100n", "+3V3", "GND")
    C("4.7u", "+3V3", "GND", C0402)

with block("mcu_io", "User LED (GPIO12, active low)"):
    LED("green", "+3V3", "LED_USER_K")
    R("470", "LED_USER_K", "LED_USER_N")

with block("rf", "RW612 RF control + strap"):
    connect(u, "RF_CNTL0", "RF_CTL0")
    connect(u, "RF_CNTL1", "RF_CTL1")
    connect(u, "RF_CNTL2", "RF_CTL2_STRAP")
    connect(u, "RF_CNTL3", None)
    connect(u, "RF_TR_2G4", "RF2G_SOC")
    connect(u, "RF_TR_5G", "RF5G_SOC")
    connect(u, "BRF_ANT", "BLE_SOC")
    R("4.7k", "RF_CTL2_STRAP", "GND", note="strap CON[11]=0: debug port = SWD")

with block("rf", "5 GHz Wi-Fi filter"):
    C("0.3p", "RF5G_SOC", "GND", C0201, note="RF C0G, place per NXP layout")
    L("1.6nH", "RF5G_SOC", "RF5G_FLT", L0201, mpn="LQP03TN1N6B02D", mfr="Murata")
    C("0.3p", "RF5G_FLT", "GND", C0201)

with block("rf", "2.4 GHz Wi-Fi + BLE/15.4 switch (SKY13323)"):
    C("DNP", "RF2G_SOC", "GND", C0201, dnp=True)
    C("10p", "RF2G_SOC", "RF2G_FLT", C0201)
    C("DNP", "RF2G_FLT", "GND", C0201, dnp=True)
    C("10p", "RF2G_FLT", "RFSW_OUT1", C0201)
    C("1.6p", "BLE_SOC", "GND", C0201)
    L("3.3nH", "BLE_SOC", "BLE_FLT", L0201, mpn="LQP03TN3N3B02D", mfr="Murata")
    C("1.6p", "BLE_FLT", "GND", C0201)
    C("10p", "BLE_FLT", "RFSW_OUT2", C0201)
    mk(SW, "SKY13323-378LF", "SKY13323-378LF", mpn="SKY13323-378LF", lcsc="C150867", mfr="Skyworks",
       OUTPUT1="RFSW_OUT1", OUTPUT2="RFSW_OUT2", INPUT="RFSW_IN", VCTL1="RF_CTL0",
       VCTL2="RF_CTL1", GND="GND", EPAD="GND")
    C("100p", "RF_CTL0", "GND", C0201)
    C("100p", "RF_CTL1", "GND", C0201)
    C("10p", "RFSW_IN", "DPX_LB", C0201)

with block("rf", "Diplexer + antenna port"):
    mk(SW, "DPX165950DT-8085D1", "DPX165950DT-8085D1", mpn="DPX165950DT-8085D1", mfr="TDK",
       HB="RF5G_FLT", LB="DPX_LB", COM="ANT_COM", GND="GND")
    C("DNP", "ANT_COM", "GND", C0201, dnp=True, note="antenna match (shunt)")
    C("5.6p", "ANT_COM", "ANT_FEED", C0201, note="antenna match (series)")
    C("DNP", "ANT_FEED", "GND", C0201, dnp=True, note="antenna match (shunt)")
    L("DNP", "ANT_FEED", "GND", L0201, dnp=True, note="antenna match (shunt)")
    mk("Connector", "Conn_Coaxial", "U.FL", "Connector_Coaxial:U.FL_Hirose_U.FL-R-SMT-1_Vertical",
       mpn="U.FL-R-SMT-1(10)", lcsc="C88373", mfr="Hirose", note="dual-band 2.4/5 GHz antenna",
       In="ANT_FEED", Ext="GND")

# =====================================================================================
# 5. HMI
# =====================================================================================
with block("hmi", "4.0in SPI LCD ST7796S + FT6336U touch (LCDwiki MSP4031)"):
    # cable to the module: JST PH 14-pin, wired 1:1 to the module's 14-pin header
    mk("Connector", "Conn_01x14_Pin", "LCD_MSP4031",
       "Connector_JST:JST_PH_B14B-PH-K_1x14_P2.00mm_Vertical",
       mpn="B14B-PH-K-S(LF)(SN)", lcsc="C158024", mfr="JST",
       note="1 VCC 2 GND 3 CS 4 RST 5 DC 6 MOSI 7 SCK 8 LED 9 MISO 10 SCL 11 CTP_RST 12 SDA 13 INT 14 SD_CS"
            " (1:1 cable to the MSP4031 header)",
       _1="LCD_VCC", _2="GND", _3="LCD_CS_N", _4="LCD_RST_N", _5="LCD_DC", _6="LCD_MOSI_R",
       _7="LCD_SCK_R", _8="LCD_BL", _9=None, _10="CTP_SCL", _11="CTP_RST_N", _12="CTP_SDA",
       _13="CTP_INT_N", _14=None)
    R("0", "+5V", "LCD_VCC", R0603, note="LCD VCC = 5 V (default)")
    R("0", "+3V3", "LCD_VCC", R0603, dnp=True, note="alt: LCD VCC = 3.3 V")
    C("10u 10V", "LCD_VCC", "GND", C0603)
    C("100n", "LCD_VCC", "GND")
    R("22", "LCD_SCK", "LCD_SCK_R")
    R("22", "LCD_MOSI", "LCD_MOSI_R")
    R("100", "LCD_BL_PWM", "LCD_BL")
    R("100k", "LCD_BL", "GND", note="backlight off at reset")
    R("4.7k", "CTP_SDA", "+3V3", note="touch I2C bus (FC2, FT6336U 0x38)")
    R("4.7k", "CTP_SCL", "+3V3")
    R("10k", "CTP_INT_N", "+3V3")
    R("10k", "CTP_RST_N", "+3V3")
    flag("LCD_VCC")

with block("hmi", "NeoPixel strip (9x WS2812B, 5 V)"):
    mk(SW, "74AHCT1G125", "SN74AHCT1G125DBVR", mpn="SN74AHCT1G125DBVR", lcsc="C7484", mfr="Texas Instruments",
       A="NEOPIXEL_DATA", _1="GND", VCC="+5V", GND="GND", Y="NEO_DOUT")
    C("100n", "+5V", "GND")
    R("100k", "NEOPIXEL_DATA", "GND")
    R("330", "NEO_DOUT", "NEO_DIN")
    mk("Device", "C_Polarized", "220u 6.3V", "Capacitor_SMD:CP_Elec_6.3x5.4",
       _1="+5V", _2="GND", note="bulk at strip connector")
    mk("Connector", "Conn_01x03_Pin", "NEOPIXEL", "Connector_JST:JST_XH_B3B-XH-A_1x03_P2.50mm_Vertical",
       mpn="B3B-XH-A", mfr="JST", note="1=+5V 2=DIN 3=GND", _1="+5V", _2="NEO_DIN", _3="GND")

# Off-board controls on the wheel face, all on JST PH 2.0 mm vertical headers.
JST_PH2 = dict(fp="Connector_JST:JST_PH_B2B-PH-K_1x02_P2.00mm_Vertical", mpn="B2B-PH-K-S(LF)(SN)",
               lcsc="C131337", mfr="JST")
JST_PH4 = dict(fp="Connector_JST:JST_PH_B4B-PH-K_1x04_P2.00mm_Vertical", mpn="B4B-PH-K-S(LF)(SN)",
               lcsc="C131334", mfr="JST")

for k in (1, 2):
    with block("hmi", f"Rotary encoder {k} (off-board EC11 with push switch)"):
        mk("Connector", "Conn_01x04_Pin", f"ENC{k}", **JST_PH4,
           note="1=A 2=B 3=SW 4=GND (EC11 common C and switch return both to GND)",
           _1=f"ENC{k}_A_RAW", _2=f"ENC{k}_B_RAW", _3=f"ENC{k}_SW_RAW", _4="GND")
        R("10k", f"ENC{k}_A_RAW", "+3V3")
        R("10k", f"ENC{k}_B_RAW", "+3V3")
        R("10k", f"ENC{k}_SW_RAW", "+3V3")
        R("10k", f"ENC{k}_A_RAW", f"ENC{k}_A")
        R("10k", f"ENC{k}_B_RAW", f"ENC{k}_B")
        R("1k", f"ENC{k}_SW_RAW", f"ENC{k}_SW")
        C("10n", f"ENC{k}_A", "GND")
        C("10n", f"ENC{k}_B", "GND")
        C("100n", f"ENC{k}_SW", "GND")

with block("hmi", "Passive buttons (off-board switch to GND)"):
    for k in (1, 2):
        mk("Connector", "Conn_01x02_Pin", f"BTN{k}", **JST_PH2, note="1=switch 2=GND",
           _1=f"BTN{k}_RAW", _2="GND")
        R("10k", f"BTN{k}_RAW", "+3V3")
        R("1k", f"BTN{k}_RAW", f"BTN{k}_N")
        C("100n", f"BTN{k}_N", "GND")

# Active buttons: VCC / in (MCU -> button, e.g. its LED) / out (button -> MCU) / GND.
# All 16 lines sit on one TCA9555 on the PM I2C bus (0x20): port 0 drives the 'in' lines,
# port 1 reads the 'out' lines (internal 100k pull-ups), ~INT tells the RW612 on any change.
with block("hmi", "Active buttons: TCA9555 I/O expander (PM I2C 0x20) + fused 3.3 V"):
    xp = mk("Interface_Expansion", "TCA9555PWR", "TCA9555PWR", mpn="TCA9555PWR", lcsc="C465732",
            mfr="Texas Instruments", VCC="+3V3", GND="GND", SCL="PM_SCL", SDA="PM_SDA",
            A0="GND", A1="GND", A2="GND", _1="BTN_INT_N")
    for k in range(1, 9):
        # by pin number: SKiDL also reads a name like "P15" as pin number 15
        connect(xp, str(3 + k), f"ABTN{k}_IN_X")      # P0.0-P0.7 = pins 4-11
        connect(xp, str(12 + k), f"ABTN{k}_OUT_X")    # P1.0-P1.7 = pins 13-20
    C("100n", "+3V3", "GND")
    R("10k", "BTN_INT_N", "+3V3", note="~INT is open drain")
    mk("Device", "Polyfuse", "0.2A", "Fuse:Fuse_1206_3216Metric", mpn="1206L020YR", lcsc="C207035",
       mfr="Littelfuse", note="0.2 A hold: a shorted button cable cannot pull down the MCU's 3.3 V",
       _1="+3V3", _2="VCC_BTN")
    C("10u 10V", "VCC_BTN", "GND", C0603)
    flag("VCC_BTN")

for k in range(1, 9):
    with block("hmi", f"Active button {k}"):
        mk("Connector", "Conn_01x04_Pin", f"ABTN{k}", **JST_PH4, note="1=VCC 2=in 3=out 4=GND",
           _1="VCC_BTN", _2=f"ABTN{k}_IN", _3=f"ABTN{k}_OUT", _4="GND")
        R("100", f"ABTN{k}_IN_X", f"ABTN{k}_IN", note="limits current into a shorted cable")
        R("1k", f"ABTN{k}_OUT", f"ABTN{k}_OUT_X")
        C("10n", f"ABTN{k}_OUT_X", "GND")

with block("hmi", "Expansion header"):
    mk("Connector", "Conn_01x06_Pin", "EXPANSION", "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical",
       note="1=3V3 2=GND 3=PM_SDA 4=PM_SCL 5=GPIO20 6=GPIO21",
       _1="+3V3", _2="GND", _3="PM_SDA", _4="PM_SCL", _5="EXP_GPIO20", _6="EXP_GPIO21")


# =====================================================================================
# annotate + outputs
# =====================================================================================
PREFIX_FIX = {"#FLG": "#FLG"}


def annotate():
    """Reference designators: sheet index * 100 + running number (R101, U201 ...)."""
    order = {s: i + 1 for i, (s, _) in enumerate(SHEETS)}
    counters = {}
    flags = 0
    parts = sorted(builtins.default_circuit.parts, key=lambda p: (order[p.sw_sheet], p.sw_order))
    for p in parts:
        prefix = "".join(ch for ch in p.ref_prefix if not ch.isdigit()) or "X"
        key = (order[p.sw_sheet], prefix)
        counters[key] = counters.get(key, 0) + 1
        p.ref = f"{prefix}{order[p.sw_sheet]}{counters[key]:02d}"


def main():
    annotate()
    ERC()
    generate_netlist(file_=os.path.join(HERE, "steering_wheel.net"))
    from kicad_sch_export import export_project
    export_project(builtins.default_circuit, HERE, "steering_wheel", SHEETS,
                   title="IRT steering wheel controller (RW612)", flag_nets=FLAG_NETS)
    from bom import write_bom
    rows = write_bom(builtins.default_circuit, os.path.join(HERE, "bom.csv"))
    print(f"schematic: {len(SHEETS)} sheets, {len(builtins.default_circuit.parts)} parts, "
          f"{len(rows)} BOM lines -> steering_wheel.kicad_pro / bom.csv")


if __name__ == "__main__":
    main()
