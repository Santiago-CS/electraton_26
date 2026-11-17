# Sensor bring-up on the FRDM-i.MX93 Cortex-M33 — status

**Date:** 2026-10-06
**Branch:** `ivansigala/gps`
**Board:** FRDM-i.MX93 + NXP IO expansion board (`TMP_SENSOR_BOARD.pdf`)
**Build:** `mavlink_vtx`, MCUXpresso SDK 26.09.00-pvw2, armgcc, 86 KB text / 21 KB bss

---

## 1. Scope

Bring up the M33's peripheral drivers and the sensors on the expansion board,
as the foundation for the VTX telemetry application. Everything here runs on
the M33 only — the A55/Linux side, RPMsg and the MAVLink application itself are
out of scope and untouched.

A caution that runs through the whole document: the expansion board's schematic
is drawn for a **FRDM-IMX95**. The pad names on the 40-pin header carry over to
the i.MX93 unchanged, but the *peripheral numbering does not*. What the
schematic calls I2C5/I2C6/UART3/SPI4 is not what the i.MX93 calls them. Every
mapping below was re-derived from the i.MX93 IOMUXC tables and the FRDM
netlist, not taken from the drawing.

---

## 2. Working, verified on hardware

| Function | Port | Pads | Transport | Evidence |
|---|---|---|---|---|
| Debug console | LPUART2 | UART2_RXD/TXD | polled | all output in this project |
| IMU ICM-42686-P | LPSPI4 | GPIO_IO18–21 | **eDMA** ch2/3 | 1.002 g on Z, gyro bias ~0.5 dps |
| GPS u-blox M10 | LPUART3 | GPIO_IO14/15 | **eDMA** ch6/7 | **3D fix, NAV-PVT at 10 Hz**, sub-metre hAcc |
| VTX telemetry | LPUART6 | GPIO_IO04/05 | eDMA ch0/1 | **working end to end** — HEARTBEAT, SCALED_IMU 50 Hz, GPS_RAW_INT 10 Hz over RF |
| BMM350 magnetometer *(task disabled)* | LPI2C4 | GPIO_IO02/03 | eDMA ch4/5 | CHIP_ID 0x33 reads back; **data burst is a bus artifact** (§4.1) |
| BMP581 barometer *(task disabled)* | LPI2C3 | GPIO_IO00/01 | eDMA ch8/9 | **working** — 84.1 kPa, 30.3 °C, altitude matches the GPS |

### eDMA addressing — settled

A memory-to-memory test on both controllers showed **eDMA addresses M33 TCM at
the plain local address**. No `MEMORY_ConvertMemoryMapAddress()` translation is
needed anywhere, and the translation that was originally in the UART and SPI
drivers has been removed. This was blocking confidence in all three DMA
drivers; it is now closed.

### eDMA channel allocation (DMA4)

| Channels | Owner |
|---|---|
| 0, 1 | LPUART6 — VTX telemetry |
| 2, 3 | LPSPI4 — IMU |
| 4, 5 | LPI2C4 — magnetometer |
| 6, 7 | LPUART3 — GPS |
| 8, 9 | LPI2C3 — barometer |

All five ports are in WAKEUPMIX, so all of them must use **DMA4**. DMA3 (AONMIX)
can only reach LPUART1/2, LPSPI1/2 and LPI2C1/2. Each driver rejects a
mismatched pairing at init rather than configuring a channel that can never
trigger.

---

## 3. Built but not yet proven

| Item | State |
|---|---|
| **BMM350 magnetometer** | Driver + task complete. Device does not answer — see §4.1 |
| **RGPIO driver** | Rewritten for i.MX93, compiles and links, never exercised on a pin |
| **LPUART6 eDMA TX** | A banner is sent at startup but nothing reads that pin |
| **LPI2C eDMA** | Runs to completion, but has only ever carried failed transactions |

---

## 4. Open items

### 4.1 Both I2C sensors do not acknowledge — *blocking, hardware*

Applies to the **BMM350 on LPI2C4** and the **BMP581 on LPI2C3** equally: the
two buses have the same topology, the same MOSFET translators and the same
missing resistors, so they fail the same way and the fix is the same.

| Bus | Device | Addr | 3.3 V pull-ups (DNP) | 1.8 V pull-ups (fitted) |
|---|---|---|---|---|
| LPI2C4 (schematic I2C6) | BMM350 | 0x14 | **R58** SCL, **R54** SDA | R11, R20 |
| LPI2C3 (schematic I2C5) | BMP581 | 0x46 | **R57** SCL, **R55** SDA | R13, R22 |

#### RESOLVED: the pull-ups were the fault

Enabling `I2C_INTERNAL_PULLUP=1` (internal pad pull-ups, both buses at 100 kHz)
brought **both** devices to life. That is the diagnosis confirmed: nothing was
wrong with either sensor, either driver, or the level translators. The four
missing resistors were the whole problem.

**BMP581 — fully working.** Real, independently sensible data:

```
BARO: BMP581 ready, osr_p=8x osr_t=2x effective, odr valid
BARO 84107.65 Pa  30.285 C  alt 1543.33 m  (+0.00 m from start)
BARO 84107.56 Pa  30.287 C
BARO 84107.70 Pa  30.299 C
BARO 84107.85 Pa  30.309 C
```

Three independent reasons to believe it: the pressure jitters by ~0.1 Pa
reading to reading, which is sensor noise of the right magnitude; the
temperature creeps monotonically upward, which is self-heating; and `OSR_EFF`
read back the oversampling that was asked for with `odr_valid` set, so the
ODR/OSR pair is runnable.

The altitude also cross-checks against the GPS. The barometer says **1543 m**
against an ISA reference and the GPS says **1618 m**. That 75 m gap is not an
error in either — it is the sea-level reference. Solving for the reference that
reconciles them gives **1022.6 hPa**, a perfectly ordinary high-pressure day
against ISA's 1013.25. This is exactly the caveat documented on
`bmp581_altitude_cm()`: absolute altitude is only as good as the reference,
relative change is trustworthy.

The barometer is not immune either, just much less affected — roughly **1 bad
read in 125**, which shows up as an all-ones field:

```
BARO 262143.98 Pa  0.883 C  alt -8788.09 m   <- 0xFFFFFF pressure
BARO: read failed, status=902 (1 of 251 reads)
```

`bmp581_read_data()` now range-checks its result against the part's operating
window (30–125 kPa, −45 to +90 °C) and returns `kStatus_Fail` for anything
outside, because the sensor physically cannot report that — it means the
transfer was corrupted. The raw fields are still written so a caller can
inspect them, and the task prints them. This is ordinary driver validation
rather than a workaround, but on this bus it is load-bearing: without it a
single glitch puts a −8788 m outlier into the altitude output.

**BMM350 — talking, but the data is still a bus artifact.** `CHIP_ID` now reads
back `0x33`, so the part is alive and at 0x14 — the first time that has been
reproducible. But the data burst is not real:

```
MAG raw x  8355711  y  8355711  z  8355711   t  8355711
```

`8355711` is `0x7F7F7F`: **every byte of the 14-byte burst reads 0x7F**, and all
four fields are identical, which no real sensor produces. `0b11111111` sampled
one bit late reads as `0b01111111`, so this is all-ones data with the first bit
after each ACK still recovering from being pulled low — the textbook weak
-pull-up signature. Note the short 3-byte `CHIP_ID` read succeeds while the
14-byte burst does not, which is consistent: the longer the read, the less
margin there is.

So the magnetometer is now blocked on edge rate rather than on connectivity,
and the remedy is the same one: **fit R54/R58 and R55/R57.** This flag was only
ever a diagnostic and it has now done its job.

#### Earlier evidence: a known-good device on the same bus was also silent

The barometer bus scan finds **nothing at all — not even 0x42**.

That matters because the **u-blox GPS is a known-good, known-powered I2C device
already wired to that bus**. The Pixhawk connector carries its DDC on pins 4
and 5, which are the same GPIO_IO00/IO01 pair, and the module is demonstrably
alive — it is streaming NAV-PVT over UART from the other pins of that same
connector at the same moment. If the bus were electrically functional, 0x42
would ACK whether or not the BMP581 ever answered.

So the diagnosis is no longer "the barometer does not reply". It is **this bus
cannot carry I2C at all**, which is exactly what four unpopulated pull-ups
predict, and it is now established without depending on any assumption about
the BMP581 itself.

#### What the error codes are saying

All three codes seen across runs are signatures of the same fault, and the
variation between them is the floating lines settling differently:

| Code | Name | What it means here |
|---|---|---|
| 900 | `kStatus_LPI2C_Busy` | lines read low at idle, so no START was ever issued |
| 902 | `kStatus_LPI2C_Nak` | address clocked out, nothing acknowledged |
| **905** | `kStatus_LPI2C_ArbitrationLost` | **master released a line expecting it to rise and read it back low** |

905 is the clearest of the three: reading back a level you did not drive is
precisely what no pull-up looks like. The magnetometer moved from 902 to 905
between runs with no change to that code path, which is itself evidence the bus
is floating rather than mis-driven.

#### BMM350 detail

Current symptom: `init failed (902)` = `kStatus_LPI2C_Nak`, and a full bus scan
(0x08–0x77) finds nothing.

This is progress — it was previously `900` (`kStatus_LPI2C_Busy`), meaning the
LPI2C saw the lines held low and never issued a START. Getting a NAK means the
master is now clocking the bus properly and nothing is replying.

What is known:

- The expansion board's **3.3 V-side I2C pull-ups are DNP from the factory**:
  **R54** (SDA) and **R58** (SCL), 2.2 kΩ, on the magnetometer bus. Only the
  1.8 V side (R20/R11) is fitted. A MOSFET level translator needs both.
- The FRDM side has no pull-ups on these pins either — GPIO_IO02/IO03 run
  through 0 Ω links (R2668/R2672) straight to the header, nothing else on the net.
- The schematic gives the address as **0x14** (`0x15 if ADSEL is pulled-up`).
  The scan covers both; neither answers.

#### BMP581 detail

Same story on its own bus. Two things make this one slightly easier to
diagnose:

- The **GPS DDC shares this bus** — the Pixhawk connector carries it on pins 4
  and 5, which are this same GPIO_IO00/IO01 pair. A default-configured u-blox
  module answers at **0x42**, and `baro_task` labels it in the scan output. So
  the scan distinguishes two cases cleanly: finding 0x42 but not 0x46 points at
  the sensor board or the BMP581; finding **neither** points at the pull-ups.
- There is a **solder-free way to try it**: `BARO_I2C_INTERNAL_PULLUP` in
  `pin_mux.c` enables the pads' internal pull-ups. They are weak — tens of kΩ
  against the 2.2k the board intends — which is why this bus is configured for
  100 kHz rather than 400 kHz. It is a diagnostic, not a fix: if the part comes
  up this way, that proves the device and the translator are fine and the only
  thing missing is R55/R57.

Next steps, in order:

0. **`I2C_SENSORS_ENABLED` is 0** in `main.c` — `mag_task` and
   `baro_task` are compiled out until the resistors are in. Set it back to 1
   afterwards; nothing else changes.
1. **Fit R54/R58 (magnetometer) and R55/R57 (barometer), 2.2k to 3V3.** This is
   now the only outstanding item for both sensors — the diagnostic above has
   already proven everything else in the chain works.
2. **Then remove `I2C_INTERNAL_PULLUP=1` from `CMakeLists.txt`** and restore
   400 kHz. Leaving it on masks the real fix with a marginal bus.
3. Re-check the magnetometer burst. If `0x7F7F7F` persists with proper
   pull-ups, the next suspect is the part sitting in suspend rather than normal
   mode — read back `PMU_CMD` and the status register to tell those apart.
4. Measure SDA and SCL at rest on P11 pins 3 and 5. Around 1.1 V means only the
   1.8 V side is pulling, through the translator's body diode.
3. Check the **GPIO_IO pad voltage domain**. The expansion board assumes 3.3 V
   on the host side of its translator; if the FRDM drives 1.8 V there, the
   translator is pointed the wrong way and resistors will not fix it.
4. Check **VCC_SENS** (the board's 1.8 V LDO) is actually up — the magnetometer
   is powered from it, and a NAK with a working bus is consistent with an
   unpowered device.

### 4.2 GPS — working, verified on hardware

Resolved. A 3D fix with 14 satellites, NAV-PVT sustained at exactly 10 Hz, and
the configuration accepted in full:

```
GPS:   rate      ACK (24 bytes)
GPS:   nav-pvt   ACK (17 bytes)
GPS:   nmea-off  ACK (42 bytes)
GPS 2026-10-07 03:38:09 fix=1 sats=14 VALID  UBX 10 Hz  [pvt=17 gga=1 rmc=1 csum=0/0 ovf=0]
    lat +20.7378618  lon -103.4255571  alt 1616247 mm  spd 90 mm/s
    hAcc 938 mm  vAcc 1747 mm  pDOP 2.13
```

Three things this confirms beyond "it runs":

1. **The `UBX_KEY_*` constants are correct** for the installed firmware. All
   three VALSET steps were ACK'd, which was the one genuinely unverified part
   of the UBX work. No longer an open risk.
2. **Both parsers independently agree.** The last NMEA sentence before the
   switchover read `2044.27189,N 10325.53336,W` — 20.7378648 N, 103.4255560 W.
   The first NAV-PVT read 20.7378635 N, 103.4255561 W. They agree to **15 cm
   N/S and 1 cm E/W**, which is the real position noise between two samples a
   second apart, not a decoding difference. Two independently written parsers
   landing on the same coordinate is good evidence both are right.
3. **The receive path has margin.** `ovf=0` and `csum=0/0` throughout: no ring
   overflow, no checksum errors in either format. 10 Hz NAV-PVT is 1000 B/s,
   **8.7% of the 115200 link**, so the rate ceiling is the receiver's
   navigation engine, never the wire.

`gga`/`rmc` frozen at 1 while `pvt` climbs by exactly 10 per second is the
`nmea-off` step working as intended.

Note the first report line said `7 Hz` — the configuration completed partway
into that reporting second, so only 7 frames fell inside it. Not a glitch.

#### Going faster than 10 Hz — `GPS_SINGLE_GNSS_25HZ`

10 Hz is the M10's multi-GNSS maximum and is the default. The 25 Hz path is
implemented behind a flag, **off by default**: set `GPS_SINGLE_GNSS_25HZ` in
`sensors/gps_m10.h`, or uncomment the `mcux_add_configuration` block in
`CMakeLists.txt`.

Turning it on adds a fourth configuration step, **emitted first**, that
switches Galileo, BeiDou and GLONASS off and leaves GPS, SBAS and QZSS on —
those three share the GPS L1 C/A signal and do not count against the
single-GNSS rate limit, so keeping them is free satellite count. Order matters:
asking for a 40 ms period while three constellations are still enabled is a
request the receiver is entitled to refuse. The step carries a 500 ms settle
wait, because changing constellations restarts the GNSS subsystem and it has to
re-acquire.

The builder **refuses** a sub-100 ms period when `single_gnss` is false rather
than emitting frames that would be rejected or, worse, silently clamped.

| | nav period | wire | satellites seen | link use |
|---|---|---|---|---|
| Default, multi-GNSS | 100 ms (10 Hz) | 1000 B/s | 14 at pDOP 2.13 *(measured)* | 8.7% |
| `GPS_SINGLE_GNSS_25HZ=1` | 40 ms (25 Hz) | 2500 B/s | roughly half, worse DOP *(not yet measured)* | 21.7% |

**Default answer is still no.** 2.5× the rate for a materially worse fix is a
bad trade unless something downstream genuinely needs 40 ms updates. The
`UBX_KEY_SIGNAL_*` constants this path uses are also in the not-yet-verified
group — if any of them is wrong you will see `GPS:   gnss      NAK` and the
receiver will stay multi-GNSS, which is a safe failure.

### 4.3 Software items

| Item | Notes |
|---|---|
| **BMM350 register values unverified** | PMU commands, ODR encodings and the `UPD_OAE` ordering are transcribed from the datasheet / Bosch API. `CHIP_ID == 0x33` is the checkpoint: if that reads back and data is still wrong, the fault is in the config sequence, not the bus. |
| **BMP581 ODR codes partly unverified** | The register addresses, field layout and output scaling are transcribed from the datasheet and are checked by the host tests. Four of the `BMP581_ODR_*` codes are marked `verify` in `bmp581_reg.h` — only `BMP581_ODR_50HZ` is on the default path, and `OSR_EFF` is read back after init so a bad ODR shows up as `odr_valid == false` rather than as silently wrong data. |
| **BMM350 reports raw counts** | Microtesla needs Bosch's OTP compensation (offset, sensitivity, TCO/TCS, cross-axis). Not implemented — inventing coefficients would give plausible wrong numbers. |
| **ICM-42686-P full-scale map** | Confirmed empirically: 2048 LSB/g at `FS_SEL=1` ⇒ ±16 g, the -42686 mapping. The `ACCEL_UI_FS_*` macros in `icm_42686_p_reg.h` still follow the **-42688** map and are not used by the driver. Worth cleaning up. |
| **`.rodata` as a DMA source** | All three DMA drivers refuse a buffer in code TCM (0x0FFE0000). Whether eDMA can actually reach that aperture was never tested — the guard is conservative, not a measured limit. |
| **`CONFIG_I2C_RETRY_TIMES=0`** | The SDK documents 0 as "wait forever". The blocking LPI2C path can spin indefinitely on a stuck line. Set it non-zero. |
| **GPS receive is chunked, not continuous** | 64-byte eDMA chunks in an 8-deep ring, re-armed from the ISR, and the task also consumes the *partial* chunk via `uart_rx_dma_count()` — without that the tail of each burst waits a whole navigation period. Worst-case added latency is now one `GPS_POLL_MS` tick. A true circular-TCD receive driven by the LPUART idle-line interrupt would remove even that. |
| **GPS rate ceiling** | `GPS_NAV_PERIOD_MS` is 100 ms (10 Hz), the M10's multi-GNSS maximum. 25 Hz is reachable but only by restricting the receiver to one constellation, which trades fix quality for rate. Bandwidth is not the limit: NAV-PVT at 25 Hz is 2500 B/s against 11520 B/s at 115200. |
| **Status codes printed raw** | `900`/`902` rather than a name. A small decoder would save a lookup each time. |

---

## 5. Hardware gotchas found along the way

Things that cost time and are not obvious from any single document:

- **GPIO_IO04 drives the RGB LED.** The pad carrying LPUART6_TX also runs
  through R2627 (1 kΩ) to the gate of Q3, one of the three FETs sinking LED1.
  UART traffic flickers it and an idle-high TX leaves it lit. Harmless
  electrically; just do not use that LED as a signal any more.
- **`BOARD_BootClockRUN()` configures no peripheral clock roots on this board.**
  It only brings up the PLLs. Every port's root and gate must be set by hand.
  `LPSPI_MasterInit` and `LPI2C_MasterInit` ungate themselves (`*_CLOCKS` is
  defined); **`LPTMR_Init` does not** — that one is per-peripheral, not a rule.
- **GPIO interrupt vectors are not named.** `startup_MIMX9352_cm33.S` leaves
  them as `ReservedNN_IRQHandler` (NN = IRQn + 16). Defining `GPIO2_0_IRQHandler`
  compiles, links, and never fires. See the table in `gpio_driver.c`.
- **The expansion board's bus numbering is i.MX95.** Its I2C6 is the i.MX93's
  **LPI2C4**; its I2C5 is LPI2C3 (the barometer). Its UART3 is the i.MX93's
  **LPUART3** — that one happens to match.
- **The schematic calls the IMU an ICM42688**, but the fitted part behaves as a
  **-42686** (±16 g at `FS_SEL=1`). Trust the measurement.
- **No I2C pull-ups anywhere by default** — see §4.1.

---

## 6. Driver bugs found and fixed

Recorded because several were silent and would have cost the same time again:

| Bug | Symptom | Fix |
|---|---|---|
| `LPUART_GetDefaultConfig()` called *after* setting fields | TX never enabled, no output at all | Call it first |
| LPTMR clock root never configured | Hard hang inside `LPTMR_Init` | Set root + gate in the driver |
| **Peripheral IRQ left at priority 0** | **Whole system froze silently** | `NVIC_SetPriority(..., 5)` on the LPUART/LPSPI/LPI2C lines, not just the eDMA channels |
| **Stale eDMA channel flags across a warm restart** | `ASSERT ERROR " handle != NULL "` in `EDMA_HandleIRQ` on the second and later `remoteproc start`, right after the first I2C init | `remoteproc` resets the M33 but not DMA4, and on this SoC `EDMA_Init()`'s eDMA4 path only writes `MP_CSR` — it never clears per-channel state. The SDK dispatcher calls `EDMA_HandleIRQ(s_EDMAHandle[inst][ch])` whenever `CH_INT` is set, with no NULL check, and `EDMA_CreateHandle()` enables the NVIC line for the whole channel *pair* — so a flag left by the previous run fires before the second handle of the pair exists. All three drivers now clear `DONE`/`ERROR`/`INT` and disable the request on both channels **before** creating either handle. Per-channel, not a module reset: DMA4 is shared with the A55. |
| **eDMA *channel* IRQ left at priority 0 in `uart_driver.c`** | **Whole system froze silently** on the GPS task's first completed chunk | Same bug, other half: `spi_driver.c`/`i2c_driver.c` set both the channel and peripheral lines, `uart_driver.c` only set the peripheral one. Latent until a UART eDMA callback first called `...FromISR()` |
| DMA completion status discarded | Every transfer reported success, including NAKs | Carry `completionStatus` out to the caller |
| `PRINTF_ADVANCED_ENABLE` undefined | `%d` printed negatives as unsigned | Define it in `CMakeLists.txt` |
| IMU gyro start-up not waited out | First sample was `0x8000` on all gyro axes | 50 ms settle after `PWR_MGMT0` |
| GPS receive re-armed from the task | RX stalled after 9 chunks | Re-arm in the ISR |
| GPS ping-pong too shallow for 115200 | Torn buffers: `fix=0 sats=0` beside `VALID`, frozen clock, longitude in the wrong hemisphere | 8-buffer ring + task notification instead of polling |
| GPS drained only *completed* chunks | Tail of every burst stranded for a full navigation period | Consume the partial chunk from the eDMA progress count |
| LPSPI instance array sized by `FSL_FEATURE_SOC_LPSPI_COUNT` | Out-of-bounds write for LPSPI8 | SDK's instance lookup returns 1..8, size the array accordingly |

The **priority-0 freeze** has now happened twice, on two different halves of
the same problem, and is the one worth remembering. `LPxxx_TransferCreateHandleEDMA()`
enables the *peripheral's own* interrupt in addition to the eDMA channels', at
reset priority 0. That ISR dispatches the completion callback, so any
`...FromISR()` call from it trips FreeRTOS's `configASSERT`, which is
`taskDISABLE_INTERRUPTS(); for(;;);` — no message, no output, everything stops.

---

## 7. Layout

```
mavlink_vtx/
├── drivers/
│   ├── uart_driver.{c,h}     LPUART1-8, eDMA or interrupt, VTX + GPS defaults
│   ├── spi_driver.{c,h}      LPSPI1-8 master, eDMA or blocking
│   ├── i2c_driver.{c,h}      LPI2C1-8 master, eDMA or blocking, bus scan
│   ├── lptmr_driver.{c,h}    LPTMR1/2 periodic with callback (kept, not in the build)
│   └── gpio_driver.{c,h}     RGPIO GPIO1-4, per-pin interrupt callbacks
├── sensors/
│   ├── icm_42686_p.{c,h}     IMU over SPI, burst read, scaled output
│   ├── bmm350.{c,h}          Magnetometer over I2C, raw counts
│   ├── bmp581.{c,h}          BMP581 barometer, real units + altitude
│   ├── bmp581_reg.h          BMP581 register map
│   ├── gps_m10.{c,h}         u-blox M10: NMEA + UBX demux, transport-agnostic
│   └── gps_ubx.{c,h}         UBX framing, CFG-VALSET, NAV-PVT decode
├── mcimx93evk/freertos_hello/
│   ├── pin_mux.c             all pad muxing
│   ├── hardware_init.c       clock roots and gates
│   └── app.h                 port assignments
├── comms/
│   └── mavlink_min.{c,h}     hand-rolled MAVLink v2 encoder
├── tools/
│   ├── vtx_imu_decode.py     serial decoder + link rate/loss meter
│   └── vtx_ground_station.py wfb-ng bring-up, video + telemetry window
└── main.c                    tasks: imu (+ MAVLink to the VTX), gps; mag/baro gated off
```

### Build

```bash
export PATH="$HOME/.mcuxpressotools/ninja-1.13.2:$HOME/.mcuxpressotools/cmake-3.30.0-linux-x86_64/bin:$HOME/.mcuxpressotools/.mcux-venv-3.12/bin:$PATH"
export ARMGCC_DIR=/usr/local/mcuxpressoide-25.6.136/ide/tools
cmake --preset debug && cmake --build debug
```

Compiles under `-Wall -Werror`; the drivers and sensors are additionally clean
under `-Wextra -Wshadow -Wconversion`.

### Test

`sensors/gps_m10.c` and `sensors/gps_ubx.c` are transport-independent and are
exercised on the host:

- **NMEA**, 23 checks: both hemispheres, negative altitude, bad checksum,
  mid-sentence resync, byte-at-a-time delivery.
- **UBX**, 63 checks: NAV-PVT field decode, checksum rejection, ACK/NAK
  matching (including an ACK for an unrelated message not satisfying the wait),
  oversize frames not desynchronising the parser, NMEA/UBX demultiplexed on one
  stream, frames split across `feed()` calls, and the `CFG-VALSET` builder
  compared byte-for-byte against a reference built independently in Python —
  both the 10 Hz multi-GNSS and 25 Hz single-GNSS step sequences, including
  step order, settle timings, and that a sub-100 ms period without
  `single_gnss` is refused rather than clamped.

- **BMP581**, 51 checks, run against a fake register file through minimal host
  shims for `fsl_common.h`/`i2c_driver.h` so the real decode path executes:
  every configuration register the driver writes compared against the
  datasheet field layout, CHIP_ID and NVM rejection paths, a bus NAK
  propagating rather than being swallowed, pressure/temperature scaling against
  Python-computed vectors, 24-bit sign extension at `0x7FFFFF`/`0x800000`/
  `0xFFFFFF`, rejection of the exact corrupt-transfer pattern seen on hardware
  (all-ones pressure) with the in-range values either side of it still
  accepted, and the altitude formula.

All passing. The Python reference is what makes the UBX checksum, the BMP581
scaling and the altitude maths trustworthy — testing them against themselves
would prove nothing.

Incidental cross-check worth recording: 83.5 kPa computes to **1602 m**, and
the GPS independently reports **1616 m** at this location. Two unrelated
sensors and two unrelated code paths agreeing to ~14 m is good evidence the
altitude formula is right.

---

## 7a. IMU → VTX raw telemetry

`imu_task` now streams raw IMU samples out of LPUART6 as 24-byte binary frames
and prints a throughput summary once a second.

Binary, not ASCII, deliberately: the same sample as text is ~40 bytes against
24, which at 200 Hz is the difference between 42% and 70% of a 115200 link —
that would be measuring the encoding rather than the link. Each frame carries a
**sequence number**, because without it the receiver cannot distinguish a
dropped frame from a slow one, and loss is the only figure worth having.

Frame: `A5 5A | seq u16 | t_us u32 | accel 3×i16 | gyro 3×i16 | temp i16 |
crc u16`, little-endian, CRC-16/CCITT-FALSE over bytes 2..21. Raw LSB counts,
so the frame does not depend on the configured full-scale range.

Timestamps come from the **DWT cycle counter**, not the tick: the FreeRTOS tick
is 5 ms here, which *is* the sample period at 200 Hz. `micros_init()` verifies
CYCCNT actually increments and falls back to ticks if not, printing which
source is in use — a silently wrong timebase would make every rate figure a lie.

Transmit is a queue of 8 frames with the TxIdle callback chaining straight into
the next from interrupt context, so there is no task round trip (and therefore
no 5 ms tick) between frames. Queue overruns drop the **newest** and are
counted; dropping the oldest would reorder sequence numbers, defeating the
receiver's loss detection.

Host side: `tools/vtx_imu_decode.py --port /dev/ttyACM0`. It resynchronises on
the sync pair, checks every CRC, and reports fps, B/s, % of line rate and loss
from sequence gaps. Its CRC is an independent implementation, verified against
the standard `0x29B1` check vector and cross-checked byte-for-byte against the
firmware's encoder.

| Rate | Payload | 115200 | 230400 | 921600 |
|---|---|---|---|---|
| 100 Hz | 2400 B/s | 21% | 10% | 3% |
| **200 Hz (current)** | 4800 B/s | **42%** | 21% | 5% |
| 400 Hz | 9600 B/s | 83% | 42% | 10% |
| 1000 Hz | 24000 B/s | *over* | 104% | 26% |

## 7b. Ground station

`tools/vtx_ground_station.py` automates vtx_run_book.md: runs the runbook's
preflight checks, starts `wifibroadcast@gs` if it is not already up, shows the
H.265 video from UDP 5600 on the left and decodes UDP 14550 into a scrolling
terminal pane on the right. It only stops the service again if it was the thing
that started it.

Ports come from `/etc/wifibroadcast.cfg`: `[gs_video]` sends to 5600 and
`[gs_mavlink]` to 14550, both `connect://127.0.0.1`, so the script binds and
listens on those.

The pane decodes **both** the firmware's raw-IMU frames and MAVLink v1/v2, and
labels which it found. Today the firmware sends raw IMU, not MAVLink; the
MAVLink path is there for when the firmware starts emitting it. Anything that
is neither is hex-dumped rather than hidden — on a new link, "bytes are
arriving but I cannot parse them" is the most useful thing to be able to see.

Video embedding has three paths, picked at runtime and logged:

| Path | Needs | Notes |
|---|---|---|
| `gtksink` widget | `gstreamer1.0-gtk3` | best; the only one that works on Wayland |
| X11 window id | `gir1.2-gst-plugins-base-1.0` | X11 only |
| separate window | nothing extra | runs the runbook's own `gst-launch` pipeline |

Neither optional package is installed on the laptop today, so it currently
takes the third path and says so, with the apt line to get embedding.

One parser bug worth recording, found by the host tests rather than on
hardware: `0xFE` and `0xFD` occur constantly inside IMU payload data
(accelerometer −2 encodes as `FE FF`), and during resync such a byte read as a
MAVLink header yields a bogus 255-byte length that stalls the parser
indefinitely. The fix uses the IMU frame's CRC as the disambiguator — the only
self-verifying structure in the stream — and prefers a CRC-valid IMU frame
starting before the claimed MAVLink frame could end.

## 7c. Why the ground station showed nothing from the board

The telemetry pane showed a steady 1 Hz of `MAVLINK2 id 109, sys 3, comp 68`
and nothing else. **That is wfb-ng's own RADIO_STATUS** - the ground station
reporting its own link quality (rssi 241, noise 214, txbuf 100%). The tunnel
was working and talking about itself; none of the M33's bytes were arriving.

The cause is that the VTX datalink is **not a transparent serial pipe**. It is
built to carry a flight controller's MAVLink, so it parses frames and discards
bytes that are not valid MAVLink. The custom 24-byte frame was well formed and
went out of LPUART6 cleanly - `sent` climbing, `drop=0 err=0` - and was then
dropped at the VTX.

So the firmware now speaks real MAVLink v2: `comms/mavlink_min.{c,h}`, about
250 lines, HEARTBEAT at 1 Hz and SCALED_IMU at 50 Hz, with GPS_RAW_INT and
ATTITUDE built and ready to wire. `VTX_STREAM_MAVLINK` flips back to the raw
frame, which is the experiment that proves the paragraph above.

**The encoder is verified against third-party data.** A real MAVLink frame was
captured from wfb-ng on UDP 14550, its CRC_EXTRA recovered by search (185 for
RADIO_STATUS, matching the dialect), and the encoder then rebuilds that exact
frame **byte for byte**, including MAVLink v2's trailing-zero truncation. So
the framing, the X.25 checksum and the truncation are not in doubt.

The remaining `CRC_EXTRA` constants (HEARTBEAT 50, GPS_RAW_INT 24,
SCALED_IMU 170, ATTITUDE 39) are from the common dialect and are **not**
confirmed on hardware. They fail safely: a wrong one means the receiver's
checksum will not match and the message is dropped, never misread. The ground
station validates them and prints `BAD CHECKSUM (wrong CRC_EXTRA for id N?)`.
QGroundControl showing the vehicle is the definitive check.

One unit trap worth recording: **SCALED_IMU carries angular rate in
millirad/s**, not millidegrees/s as the IMU driver produces. Nothing in the
field names says so and getting it wrong is out by 57x.

### Two firmware bugs found from that same run

| Bug | Symptom in the log | Fix |
|---|---|---|
| Sample period never changed with the ODR | banner said `10 Hz` after the ODR was set to 200 Hz | `IMU_SAMPLE_RATE_MS` lives in `sensors/icm_42686_p.h` and was still 100; now 5 |
| The startup banner's TxIdle counted as a frame | `sent=288` against `reads=280`, which is impossible | guard the TxIdle path on `s_vtx_tx_busy` - the banner also advanced the queue tail past an unfilled slot, putting one slot of stale bytes on the link |

## 7d. End-to-end confirmed

The whole chain is live:

```
M33 -> LPUART6 -> VTX -> wfb_tx -- ch161 --> receiver -> wifibroadcast@gs
    -> UDP 14550 -> ground station window
```

```
SCALED_IMU    18.11s a -0.03 -0.01 +1.00g w   +0.5   +1.0   +1.0dps ok
HEARTBEAT type 0 autopilot 0 state 4 mavlink v3 ok
```

`+1.00 g` on Z with the board flat, and a gyro bias of roughly +0.5/+1.0/+1.0
dps — the same bias measured over SPI earlier, so the numbers survived the
encoder, the radio and the decoder intact. Every frame validates its checksum,
which also confirms the `CRC_EXTRA` constants for HEARTBEAT and SCALED_IMU
were right.

### Two bugs the first good capture exposed

**`time_boot_ms` wrapped every 21.5 s.** Visible as `21.46s` followed by
`0.01s`. `micros()` returned `DWT->CYCCNT` divided down, and CYCCNT is 32 bits,
which at this board's ~200 MHz wraps in 21.47 s — matching the capture exactly.
A ground station is entitled to do something unhelpful with a clock that runs
backwards. Now accumulated into 64 bits. (The wrap period also pins
`SystemCoreClock` at 200 MHz: 2^32 / 21.46 s = 200.1 MHz.)

**The loss figure was meaningless for MAVLink.** The firmware reported 51 fps
while the ground station saw 45.7, and `lost` still read `0 (0.00%)` — because
sequence tracking only ever covered the raw-IMU frame format. MAVLink carries
its own per-`(sysid, compid)` sequence byte; the ground station now tracks it
per sender, so the status line reads `sys/comp:frames 1/1:3594  3/68:21 -2`
and that 10% gap can be attributed rather than guessed at.

### Something else is on the link

`MAVLINK2 id 27, sys 1, comp 250` arrives about every 1.2 s with a cleanly
incrementing sequence, so it is a real periodic sender and not corruption. id
27 is RAW_IMU and `comp 250` is not this firmware, which is `sys 1 comp 1`. The
ground station now decodes RAW_IMU, so the next capture will show what it
contains and probably who it is - the VTX's own telemetry is the first guess.
Worth settling, because it shares `sysid 1` with us and a GCS merges components
under one vehicle.

## 7e. GPS over the link, and video robustness

**GPS_RAW_INT is now sent**, at the GPS solution rate (10 Hz configured).
gps_task publishes a snapshot already in MAVLink units; imu_task transmits it,
so imu_task stays the only writer of the VTX queue and of the MAVLink sequence
counter. Three unit decisions, all made next to the GPS data:

- `eph` is **DOP × 100, not a distance** — the encoder previously named it
  `eph_cm`, which invited exactly the wrong value. NAV-PVT has PDOP but no
  HDOP, so PDOP is sent; it is always ≥ HDOP, so it over-states uncertainty
  rather than under-stating it. Not `hdop_1e2`: that came from GGA and is frozen
  once NMEA is switched off.
- Course over ground is sent as **unknown below 0.5 m/s** — on the bench it
  wanders through the whole compass.
- Fix type comes from the UBX `fixType`, because the shared struct folds it
  into GGA codes and loses 2-D vs 3-D.

MAVLink 2 extension fields (`h_acc` etc.) are deliberately not sent: they are
excluded from CRC_EXTRA, so a field-order mistake would land values in the
wrong fields without failing the checksum.

**Video: grey frames.** A grey picture with blocky patches that clears on its
own is the decoder running without a valid reference frame — packets were lost
before it, and it recovers at the next keyframe. Two places lose packets:

| Where | How to tell | Fix |
|---|---|---|
| RF, beyond what FEC 8/12 recovers | `wfb-cli gs` shows `lost` > 0 when it happens | lower bitrate, check distance (runbook: 1–2 m, closer overloads the receiver), stronger FEC |
| The laptop's socket | `lost` stays 0 while the picture greys | done — see below |

The ground station was reading video through a **208 KB** socket — the kernel
default — while udpsrc pushed synchronously into the decoder. A decoder stall
during a keyframe burst then overflows the socket and looks exactly like RF
loss. Now a **4 MB** socket (measured: `SO_RCVBUF` 212992 → 8000000, i.e. 4 MB
usable; `rmem_max` here is 5 MB), a `queue` so the socket drains on its own
thread, and **VA-API hardware decode by default** on the Radeon 780M with
`--sw` to force software.

Dropping the VTX to 30 fps helps the laptop-side case (half the decode work)
but barely helps RF loss — same bitrate, same packets on air. What shortens
how long a grey frame *lingers* is the keyframe interval, whichever cause.

**Open: `sys 1 comp 250` sending RAW_IMU** about every 1.2 s. Not this
firmware (`sys 1 comp 1`, never sends id 27). The ground station now validates
and decodes RAW_IMU, so its contents identify it.

## 7f. Confirmed on hardware, and what the next capture showed

**All MAVLink CRC_EXTRA constants in use are now hardware-confirmed** —
HEARTBEAT, SCALED_IMU and GPS_RAW_INT all decode `ok` at the ground station,
alongside wfb-ng's RADIO_STATUS. The GPS position, 3D fix, 12–15 satellites and
DOP 1.9–2.4 arrive at 10 Hz.

**Grey frames are gone** after the laptop-side changes (4 MB socket, queue,
VA-API decode). With nothing changed on the radio side, that places the cause
on the laptop: packets were being dropped by the viewer's 208 KB socket, not
lost over RF.

**The window resizing was the status label.** In a GTK Box, a single-line
label's minimum width is its whole text, and the per-sender list grows with
every sender — measured, the terminal pane's minimum went from 560 px to
1325 px with the text the session produced, squeezing the video to a thumbnail
and growing the window. Now a `Gtk.Paned` with a set divider, a fixed 580 px
pane, a 480×270 floor on the video, and a wrapping label capped at 4 lines:
measured minimum 580 → 580 for the same text.

**Garbage "senders".** The status line also listed `0/0`, `0/14`, `0/23`,
`0/59`… — frames with sysid 0 and ids like 6051919, whose id bytes are ASCII
(`4F 58 5C`, "OX\\"). Not real: sysid 0 is never a sender. They got through
because only message ids in the CRC table were checksum-validated; an unknown
id was accepted on faith once the parser had been misaligned by a lost
datagram, and each one swallowed the real frames behind it, inflating the loss
figure. Now an unknown id must have a plausible header (sysid ≠ 0, only the
signing flag, msgid ≤ 0xFFFF) *and* end exactly where another frame starts; a
known id is still decided by its checksum, and a checksum failure steps one byte
rather than discarding the claimed length. A stalled parse skips ahead to any
frame that proves itself by CRC.

### The `sys 1 comp 250` sender is the VTX's mavfwd — confirmed

From the VTX:

```
mavfwd --channels 8 --master /dev/ttyS2 --baudrate 115200 -p 100 -t -a 15
       --out 127.0.0.1:14550 --in 127.0.0.1:14551
```

Its RAW_IMU frames have every sensor field zero but arrive 29 bytes long — and
MAVLink 2 truncates trailing zeros, so a 29-byte payload proves the *last* byte
is non-zero. The only non-zero content is therefore in the extension fields
(`id`, `temperature`). mavfwd's own help confirms it:

```
-t --temp        Inject SoC temperature into telemetry
```

The ground station decodes the extension and prints
`RAW_IMU (all sensor fields zero) id 0 temp NN.NNC` — that is the VTX's SoC
temperature, not an IMU.

Benign. It shares `sysid 1` with this firmware, so a GCS shows the VTX
temperature under the same vehicle, which is a sensible place for it.

### Why the custom raw frame never arrived — also confirmed

Section 7c concluded the datalink filters non-MAVLink bytes, from the evidence
that MAVLink arrived over the same wire where the custom frame did not. The
help text states the mechanism outright:

```
-a --aggregate   Aggregate packets in frames (1 no aggregation,
                 0 no parsing only raw data forward)
```

The VTX runs `-a 15`: mavfwd **parses** the serial stream as MAVLink and packs
up to 15 whole messages per radio frame. Anything that does not parse as
MAVLink is dropped. Only `-a 0` forwards raw bytes. So a non-MAVLink payload
would need the datalink reconfigured; speaking MAVLink was the right fix.

**Correction.** An earlier version of this section claimed that, with
aggregation on, each datagram reaching the ground station carries only whole
messages. A later capture disproved it: the ground station received the
19-byte tail of one of this firmware's GPS_RAW_INT frames (sysid 1, compid 1,
msgid 24, a time_usec and latitude matching the surrounding frames) without its
first five bytes. A UDP datagram arrives whole or not at all, so the only way
to receive a frame's tail without its head is that the frame was split across
two radio packets and the packet carrying the head was lost. mavfwd's
aggregation does split messages across packets.

Two more mavfwd options worth knowing, neither enabled today:

- `-d --wfb` reports wfb link errors as MAVLink HUD messages (STATUSTEXT). The
  ground station does not decode STATUSTEXT yet.
- `-v --verbose` prints every packet mavfwd handles — the direct way to see
  what reaches the VTX's serial port. Both mean restarting mavfwd, which is part
  of `S98datalink`; per the runbook, restarting that over SSH drops the tunnel,
  so edit the startup config and reboot instead.

### Telemetry loss: a firmware bug, now fixed

The capture showed **4.1%** of this firmware's messages lost (`1/1:11697 -497`)
while mavfwd's injected temperature message, crossing the same radio, lost
0 of 177 (0.06% chance if it had faced the same loss). That correctly placed
the loss before mavfwd. This section then guessed the cause was UART overruns
on the VTX's `ttyS2`, and asked for `/proc/tty/driver/*` to be checked. That
guess was wrong. The VTX was fine: the driver showed no overrun, framing or
buffer errors (serial_core prints those counters only when non-zero), though
this vendor driver reports so little about itself that its silence alone would
not have been proof.

**The cause was in `vtx_uart_callback()`.** When the transmit queue moved to
variable-length MAVLink frames, `vtx_tx_kick()` was changed to send each slot's
real length, but the second send path, the one that chains the next frame from
the TxIdle interrupt, still sent a fixed `VTX_IMU_FRAME_BYTES` (24, the old raw
frame). Every chained frame went out cut short or padded:

| Frame | Real length | Sent as | Result at mavfwd |
|---|---|---|---|
| SCALED_IMU | 28 | 24 | checksum cut off, discarded |
| GPS_RAW_INT | 42 | 24 | last 18 bytes cut off, discarded |
| HEARTBEAT | 21 | 24 | 3 stale bytes appended |

A frame is chained when it is queued behind another in the same 5 ms pass. The
pass order is SCALED_IMU, HEARTBEAT, GPS_RAW_INT, so SCALED_IMU was never
chained. HEARTBEAT (every 200 ticks, a multiple of the divisor 4) sits at a
fixed phase and missed it this run. GPS_RAW_INT runs off the GPS's own clock,
drifting against the tick, and lands on a SCALED_IMU pass one time in four.

Predicted loss: 10 × ¼ = 2.5 of 61 frames/s = **4.10%**. Observed **4.08%**.
It also explains the rest: nothing from mavfwd or wfb-ng was lost; the loss came
in bursts, as the GPS phase drifted; the gaps in the log were GPS lines; and the
SCALED_IMU spacing never stretched.

The byte counter had the same stale constant, so the firmware's `B/s` and
`% of baud` report undercounted every frame since the MAVLink switch.

Both now use `s_vtx_tx_len[]`. A host test extracts the real
`vtx_queue_frame()`, `vtx_tx_kick()` and `vtx_uart_callback()` from
`main.c`, stubs the UART, and drives one pass's
SCALED_IMU + HEARTBEAT + GPS_RAW_INT through the queue. It checks each send's
length, the exact bytes on the wire, the byte counter, the startup-banner
guard, and that a full queue drops the newest frame without overwriting the slot
the DMA is reading. It passes on the fix and fails on the old line, at
HEARTBEAT 24 ≠ 21 and GPS 24 ≠ 42.

Expected after reflashing: loss on `1/1` near zero. The few lost radio packets
(the split GPS frame, 3 checksum errors) remain; they are the radio's share and
are small.

## 7g. Housekeeping

- `freertos_hello.c` (the name of the SDK example this grew from) is now
  `main.c`. The board directory `mcimx93evk/freertos_hello/` keeps its name: the
  `project()` call in `CMakeLists.txt` points the SDK at that path.
- The LPTMR1 10 Hz timer is gone, along with `mavlink_task`, which existed only
  to start it and read a tick count it never used. Despite its name it did no
  MAVLink; that lives in `imu_task`. `drivers/lptmr_driver.{c,h}` stay in the
  tree for reuse but are out of the build, so their interrupt handlers no longer
  land in the image.

## 8. Next session

1. Resolve the BMM350 bus — §4.1, steps 1–4. Blocking.
2. Set `CONFIG_I2C_RETRY_TIMES` non-zero. It matters more now that two tasks
   use I2C: the blocking path can spin forever on a stuck line.
3. Verify LPUART6 TX on a scope, so the VTX path is proven before the MAVLink
   application is layered on it. Note that LPUART3 TX is now proven — the GPS
   ACKs could not arrive otherwise — so the LPUART transmit path itself works.
4. If 25 Hz is ever wanted, enable `GPS_SINGLE_GNSS_25HZ` and check the
   `gnss` step ACKs — §4.2. Default answer: leave it off.
