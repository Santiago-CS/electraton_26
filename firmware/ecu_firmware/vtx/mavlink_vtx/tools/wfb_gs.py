"""wfb-ng ground-station service helpers, shared by vtx_ground_station.py and
vtx_dashboard.py. No GUI dependencies.

The checks follow vtx_run_book.md: the USB-C receiver on its wfb driver, the
channel config, the link key. They report and never refuse to start - a
warning that turns out not to matter is better than a tool that will not open.
"""

import os
import subprocess
import sys
import time

WFB_CFG = "/etc/wifibroadcast.cfg"
GS_UNIT = "wifibroadcast@gs"


def run(cmd, **kw):
    return subprocess.run(cmd, capture_output=True, text=True, **kw)


def preflight():
    """List of ("ok"|"WARN", text) lines."""
    notes = []

    ifaces = [ln.split()[0] for ln in run(["ip", "-br", "link"]).stdout.splitlines()
              if ln.startswith("wlx")]
    if not ifaces:
        notes.append(("WARN", "no wlx* interface - is the USB-C receiver plugged in?"))
    for name in ifaces:
        drv = run(["ethtool", "-i", name]).stdout.splitlines()
        drv = drv[0].split(":", 1)[1].strip() if drv else "?"
        if drv == "rtl88xxau_wfb":
            notes.append(("ok", f"{name}: driver {drv}"))
        else:
            notes.append(("WARN", f"{name}: driver {drv!r}, expected rtl88xxau_wfb - "
                                  "sudo modprobe -r rtw88_8812au && sudo modprobe 88XXau_wfb, "
                                  "then replug the receiver (runbook step 2)"))

    if os.path.exists(WFB_CFG):
        chan = ""
        try:
            with open(WFB_CFG) as fh:
                for line in fh:
                    if line.strip().startswith("wifi_channel"):
                        chan = line.split("=", 1)[1].split("#", 1)[0].strip()
        except OSError:
            pass
        notes.append(("ok", f"channel {chan or '?'}"))
    else:
        notes.append(("WARN", f"{WFB_CFG} missing"))

    notes.append(("ok", "/etc/gs.key present") if os.path.exists("/etc/gs.key")
                 else ("WARN", "/etc/gs.key missing - the link will not decrypt"))
    return notes


def gs_active():
    return run(["systemctl", "is-active", GS_UNIT]).stdout.strip() == "active"


def _wait_active():
    for _ in range(20):
        if gs_active():
            return True
        time.sleep(0.25)
    return False


def gs_start(log=print):
    """Start the service if it is not running. Returns True only if this call
    started it, so the caller knows to stop it again on exit.

    Plain `systemctl start` first: polkit then asks for the password in a
    desktop dialog, which works when the program was not started from a
    terminal. sudo is the fallback, and only when there is a terminal for it
    to prompt on."""
    if gs_active():
        log(f"{GS_UNIT} already running - leaving it alone")
        return False
    log(f"starting {GS_UNIT}...")
    res = run(["systemctl", "start", GS_UNIT])
    if res.returncode != 0 and sys.stdin is not None and sys.stdin.isatty():
        log("(sudo may prompt)")
        res = subprocess.run(["sudo", "systemctl", "start", GS_UNIT], text=True)
    if res.returncode != 0:
        log(f"WARN  could not start {GS_UNIT}")
        return False
    if _wait_active():
        log(f"{GS_UNIT} active")
        return True
    log(f"WARN  {GS_UNIT} did not become active")
    return False


def gs_stop(log=print):
    log(f"stopping {GS_UNIT}...")
    if run(["systemctl", "stop", GS_UNIT]).returncode != 0 and sys.stdin is not None \
            and sys.stdin.isatty():
        subprocess.run(["sudo", "systemctl", "stop", GS_UNIT])
