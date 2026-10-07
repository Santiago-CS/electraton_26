# VTX MAVLink link test — status

**Date:** 2026-10-04
**Branch:** `ivansigala/gps`
**Stage:** working on hardware, validated over the real radio link. Open items
are listed at the end; none of them block using it.

---

## 1. Goal

Prove the MAVLink messaging path from the i.MX93 Cortex-M33 to the laptop:
M33 → LPUART3 → VTX → radio → wfb-ng → UDP 14550 → checker, with an uplink for
heartbeats and commands coming back the other way.

Out of scope, and untouched: RPMsg, the Linux side, real vehicle data, the full
telemetry application.

## 2. Hardware as tested

| Signal | LPUART | Pad | Goes to |
|---|---|---|---|
| Telemetry TX | LPUART3 | `GPIO_IO14` (ALT1) | VTX RX |
| Telemetry RX | LPUART3 | `GPIO_IO15` (ALT1) | VTX TX (pad pulled up) |
| Debug console | LPUART2 | `UART2_TXD/RXD` | USB-C, `/dev/ttyACM*` |

- Board: **NXP FRDM-IMX93**, kernel `6.18.20-2.0.0-gb096ce610e95`, reachable at
  `root@192.168.0.16`.
- Firmware is built for the SDK board target `mcimx93evk`; the resulting M33
  image runs on the FRDM board unchanged (confirmed in practice).
- Custom device tree `imx93-11x11-frdm-m33-lpuart3.dtb` releases
  `GPIO_IO14/15` to the M33. LPUART2 is free for the M33 to use as its console.
- Telemetry is **115200 8N1**. The console is deliberately a different port, so
  every byte on LPUART3 is a MAVLink frame and `PRINTF` can never contaminate
  the link.
- The VTX connects **directly** to the LPUART3 pins. No USB-UART adapter is in
  the path.
- Loaded onto the M33 with remoteproc.

## 3. What was built

### `vtx/mavlink_test/` — the deliverable

A single firmware target adapted from the NXP `lpuart_interrupt` driver
example. **No build flags**: the IDE's Build button produces the MAVLink test.

| File | Lines | Role |
|---|---|---|
| `lpuart_interrupt.c` | 345 | the test — schedule, field formulas, RX handling |
| `telem_uart.c` / `.h` | 203 | non-blocking LPUART3 transport: all-or-nothing frame writes, interrupt RX ring |
| `mavlink/mavlink.c` / `.h` | — | MAVLink 2 encoder, MAVLink 1/2 parser (provided, **unmodified**) |
| `tools/linktest_check.py` | 721 | laptop-side checker |
| `readme.md` | — | wiring, build, schedule, how to run the checker |

Changes to the imported example:

- `mcimx93evk/interrupt/app.h` — `DEMO_LPUART` and friends from LPUART2 to
  **LPUART3** (base, clock root, clock gate, IRQn, IRQ handler).
- `mcimx93evk/interrupt/pin_mux.c` — added LPUART3 on `GPIO_IO14`/`GPIO_IO15`,
  RX pulled up; kept the `UART2` console pins alongside.
- `mcimx93evk/interrupt/hardware_init.c` — added `BOARD_InitDebugConsole()`.
- `lpuart_interrupt.c` — the example's echo loop replaced by the MAVLink
  schedule; its blocking writes replaced by `telem_uart`'s rings.
- `CMakeLists.txt` — added `telem_uart.c` and `mavlink/mavlink.c` plus the
  `mavlink` include directory.
- Added a **1 kHz SysTick** as the millisecond time base. The example had none,
  and on this part `SystemCoreClockUpdate()` is an empty stub with
  `SystemCoreClock` hardcoded to 200 MHz, so the reload is taken from
  `CLOCK_GetIpFreq(kCLOCK_Root_M33)` and only falls back to the constant if that
  reads back implausible. **On hardware it reported exactly 200000000 Hz.**

Build output: `debug/mavlink_test_cm33.{elf,bin}` — 17588 B text (13.5% of
129928), 5960 B data (5.4% of 108 K), **zero warnings** with `-Werror` on.

### `vtx/mcimx93evk_lpuart_interrupt/` — test infrastructure

The earlier working directory. Keeps the host-side test rig that is not needed
to build or run the firmware but is what validated `mavlink.c` and the checker:

| File | Role |
|---|---|
| `Makefile` | `host-test`, `crosscheck`, `fuzz`, `m33-warn`, `sim-test` targets |
| `tests/host/mavlink_harness.c` | exposes the library over stdin/stdout; `parse` mode is also the fuzz entry point |
| `tests/host/mavlink_crosscheck.py` | 283 checks against pymavlink |
| `tests/host/linktest_sim.c` + `shim/` | compiles the real test logic for the host over a PTY, throttled to the line rate |
| `linktest.c`, `telem_uart.c/h` | the flag-driven earlier version of the firmware |

Safe to delete once you no longer want to re-run the host tests; nothing in
`mavlink_test/` depends on it.

## 4. What the firmware sends

`t` is the `time_boot_ms` carried in that same message, so every value can be
recomputed exactly from the message itself.

| Period | Message | Content |
|---|---|---|
| 1 s | `HEARTBEAT` | `MAV_TYPE_GROUND_ROVER`, `MAV_AUTOPILOT_GENERIC`, `MAV_STATE_ACTIVE`, v3 |
| 100 ms | `NAMED_VALUE_INT "cnt"` | `n`, 0 at boot, +1 per send |
| 100 ms | `NAMED_VALUE_FLOAT "rpm"` | `(t / 10) % 6000` |
| 100 ms | `NAMED_VALUE_FLOAT "torque"` | `((t / 100) % 200) - 50` |
| 200 ms | `GPS_RAW_INT` | `k` 0 at boot, +1 per send; `lat = 194326000 + (k % 1000) * 10`, `lon = -991332000 - (k % 1000) * 10`, `alt = 2240000`, `eph = 120`, `epv = 180`, `vel = k % 65535`, `cog = (k * 100) % 36000`, `fix = 3`, `sats = 10`, `time_usec = t * 1000` |
| 200 ms | `GLOBAL_POSITION_INT` | same `lat`/`lon`/`alt`, `relative_alt = 0`, `vx = vy = vz = 0`, `hdg = cog` |
| 1 s | `NAMED_VALUE_INT` ×6 | `rx_ok`, `rx_bad`, `rx_unk`, `tx_drop`, `gcs_hb`, `cmd_n` |

A `COMMAND_LONG` aimed at system 1 or 0 is echoed on the next value tick as
`cmd_id`, `cmd_p1`, `cmd_p7`. A frame that will not fit the TX ring is dropped
whole and counted in `tx_drop` — never sent in part — and nothing in the main
loop or the TX path blocks.

Setting `LINKTEST_BURST` to `1` at the top of `lpuart_interrupt.c` moves the
value messages to 50 Hz (~5 kB/s, under half the line rate) for a stress run.

## 5. Test results

### Host tests — PASS

| Stage | Result |
|---|---|
| Cross-check vs pymavlink | **ALL PASS**, 283 checks |
| Fuzz, 5 MB `/dev/urandom` | **clean** — no ASan/UBSan report, exit 0 |
| `mavlink.c` with M33 toolchain + project flags | **zero warnings**, also clean with `-Wextra` |

Coverage: every `pack_*` output compared byte-for-byte against pymavlink's own
packing (24 vectors incl. all-zero payloads exercising MAVLink 2 trailing-zero
truncation, INT32 extremes, `u64` max, NaN/±Inf/−0.0/denormals, names of length
0/3/10/14); `seq` increment and 255→0 wrap; buffer-capacity refusal; `pack_raw`;
decode of pymavlink-packed v1 **and** v2 frames field by field; and 24
robustness streams — bad CRC two ways, unknown ids, bad incompat flags, signed
frames, zero-length payloads, truncation zero-filling, STX inside a payload,
v1/v2 interleaving, resync. CRC_EXTRA seeds confirmed equal to pymavlink's.

### Checker self-validation — 13/13 mutations caught

A passing test proves nothing if it cannot fail, so 13 single-line mutations
were introduced into the firmware logic and all 13 were detected: `rpm` divisor,
`torque` offset, `gps.lat`, `gps.epv`, `gps.cog`, `gps.time_usec`, `gpi.hdg`,
`gpi.relative_alt`, `hb.type`, `cmd_p7` (as content errors); a skipped `cnt` (as
49.9% gaps); a 130 ms value period (as a 7.69 Hz rate failure); a suppressed
echo (as 0/5 commands echoed).

### Hardware over the radio — PASS, 123 s

| Check | Result |
|---|---|
| Content errors | **0** across 5775 messages |
| Lost value messages (`cnt` gaps) | **0** (0.000%) |
| Lost GPS messages (`k` gaps) | **0** (0.000%) |
| Frames pymavlink rejected | **0** |
| Rates | HEARTBEAT 0.99–1.01, values 10.11–10.32, GPS 4.91–5.11 Hz — all inside ±5%, 12 windows |
| Uplink | `gcs_hb` +85 of 121 sent (**70%** delivery) |
| Commands | **13 of 13 that reached the M33 echoed correctly**, within the 1 s deadline |
| Corrupt-frame resync | 10 injected, `rx_bad` +10, 12 commands still echoed afterwards |
| `tx_drop` | **0** |

The downlink is effectively lossless on the bench. The uplink is the weak
direction at ~70%, so 11 of 24 commands never arrived — `cmd_n` rose by exactly
13 and all 13 were echoed, which is radio loss, not firmware.

### M33 console after 505 s continuous

```
t=505008 n=5050 k=2525 ok=0 bad=13 unk=0 drop=0 hb=121 cmd=21 ovr=0
```

- **Zero schedule drift.** The value tick runs at `8 + 100n` ms (the 8 ms is the
  boot banner), which predicts exactly 5050 ticks at `t=505008`. `n=5050`.
- **`drop=0`, `ovr=0`** over the whole 8.4 minutes: no TX ring overflow, no
  LPUART3 receiver errors, no RX ring overruns. This is the part of
  `telem_uart.c` that could only be tested on the board.
- Counters reconcile exactly with the two checker runs: `hb` 36+85=121,
  `cmd` 8+13=21, `bad` 3+10=13.

## 6. Open items

Ordered by how much they matter.

1. **`seq gaps` row in the checker is misleading over UDP.** MAVLink's sequence
   field is 8-bit and laps every ~5 s at this message rate, so the reported
   25245 "gaps" are wraps, not loss. The trustworthy loss figures are the `cnt`
   and `k` gaps, which use 32-bit counters and both read 0. Should be fixed to
   count against expected message count, or dropped on UDP connections.
2. **`vel = k % 65535` wraps after ~3.6 hours** at 5 Hz. The checker recovers
   `k` from `vel`, so past that point it will see `k` jump backwards and report
   a content error. Harmless for 10-minute runs, would break a soak test.
3. **`rx_ok` semantics are an interpretation.** The three RX cases are treated
   as mutually exclusive — a foreign heartbeat bumps only `gcs_hb`, a targeted
   command only `cmd_n`, and `rx_ok` counts everything *else*. That is why
   `ok=0` on the console. If `rx_ok` should count every good frame instead
   (142 in the run above), it is one line in `handle_frame()`.
4. **Not yet run:** the full 600 s acceptance duration, and the 50 Hz
   `LINKTEST_BURST` variant on hardware.
5. **Strict parser test not possible over the radio.** `--inject-bad N` cannot
   prove the M33 saw exactly N corrupt frames, because the VTX or wfb-ng may
   discard them first (it happened to pass all 10 through in the last run). The
   strict result needs a direct serial tap on LPUART3. The equivalent guarantee
   is covered by the 283 host cross-checks and the 5 MB fuzz.
6. **Host test rig was written by the same author as the code it tests.** The
   task spec listed `mavlink_harness.c` and `mavlink_crosscheck.py` as already
   provided; they did not exist in the repo. Every expectation was sourced from
   pymavlink or the MAVLink 2 framing rules rather than from reading
   `mavlink.c`, and the mutation testing above is the mitigation, but it is
   weaker evidence than an independent test would be.

## 7. Reproducing

```bash
# firmware
export PATH="$HOME/.mcuxpressotools/ninja-1.13.2:$HOME/.mcuxpressotools/cmake-3.30.0-linux-x86_64/bin:$HOME/.mcuxpressotools/.mcux-venv-3.12/bin:$PATH"
export ARMGCC_DIR=/usr/local/mcuxpressoide-25.6.136/ide/tools
cd mavlink_test && cmake --preset debug -B debug && cmake --build debug
# -> debug/mavlink_test_cm33.elf, load with remoteproc

# laptop checker (close QGroundControl first: only one program can hold UDP 14550)
python3 -m venv .venv && .venv/bin/pip install pymavlink pyserial
.venv/bin/python tools/linktest_check.py --duration 600 --inject-bad 10

# host tests (in the other directory)
cd ../mcimx93evk_lpuart_interrupt && make host-test
```

Note: NXP boards and their USB-serial adapters enumerate as `/dev/ttyACM*` on
Ubuntu, never `/dev/ttyUSB*`, whatever the vendor docs say.
