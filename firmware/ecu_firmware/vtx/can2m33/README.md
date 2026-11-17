# can2m33 — motor controller telemetry from Linux to the M33

Reads the PMAC motor controller on `can0`, decodes it with the speaker's
`pmac.c`, and sends the values to the M33 over RPMsg. The M33 firmware
(`mavlink_vtx`) puts them on the VTX link as MAVLink, so they reach the ground
station with the IMU, attitude and GPS.

```
can0 ─▶ can2m33 (A55, Linux) ─▶ RPMsg "rpmsg-raw" ─▶ M33 comms/pmac_link.c
                                  /dev/rpmsgN            │
                                                         ▼
                              ground station ◀─ VTX ◀─ NAMED_VALUE_FLOAT
```

It only reads `can0`, never transmits, so it runs alongside the `speaker`
program: SocketCAN gives every reader its own copy of each frame.

## Files

| file | where | role |
|---|---|---|
| `can2m33.c` | here | the daemon |
| `pmac.c`, `pmac.h` | `../../speaker/` | PMAC CANopen decoder, shared with the speaker |
| `pmac_telem.h` | `../mavlink_vtx/comms/` | the RPMsg message, shared with the M33 firmware |

## Build and run on the board

The board has `gcc`. Copy the four files into one directory and build there:

```bash
# from the repo, on the laptop
scp vtx/can2m33/can2m33.c speaker/pmac.c speaker/pmac.h \
    vtx/mavlink_vtx/comms/pmac_telem.h root@<board>:/home/root/can2m33/

# on the board
cd /home/root/can2m33
gcc -O2 -Wall -Wextra -o can2m33 can2m33.c pmac.c
```

In the repo layout the same build is
`gcc -O2 -Wall -Wextra -I../mavlink_vtx/comms -I../../speaker -o can2m33 can2m33.c ../../speaker/pmac.c`.

Then:

```bash
ip link set can0 up type can bitrate 500000   # the PMAC: 500 kbit/s, node 1
modprobe rpmsg_char                           # gives /dev/rpmsgN for the channel
# start the M33 with mavlink_vtx (remoteproc), then:
./can2m33 -v
```

`-v` prints one status line a second. Options: `-i <if>`, `-n <node>`,
`-r <hz>` (default 10), `-d <dev>` to skip the channel search.

The order of starting does not matter. Until the M33 announces its channel the
daemon says it is waiting and looks again every second, and when the M33 is
stopped or restarted it notices the failed write and finds the new channel.

## The message

`pmac_telem_t` in `pmac_telem.h`: a 16-byte header (magic, version, sequence,
valid bits, CANopen state, fault) and nine floats in engineering units. A value
is only marked valid if it arrived in the last 0.5 s, so a dead bus shows as
no data rather than frozen numbers.

## On the ground

The M33 sends one value per 20 ms, round robin, about 5.5 Hz each, plus a
status every second:

| MAVLink name | value | from |
|---|---|---|
| `MOT_RPM` | motor speed, rpm | 606Ch, TPDO5 0x454 |
| `MOT_TRQ` | torque, fraction of rated | 2020h/6, TPDO4 0x271 |
| `BAT_A` | battery current, A | TPDO4 |
| `CAP_V` | DC link capacitor voltage, V | TPDO4 |
| `MOT_TEMP` | motor temperature, °C | TPDO3 0x411 |
| `HS_TEMP` | controller heatsink temperature, °C | TPDO4 |
| `IQ_A`, `ID_A` | motor currents, A | TPDO1 0x391 |
| `MOD_PCT` | modulation, % | TPDO2 0x330 |
| `PMAC_LINK` | 0 no data from Linux, 1 Linux up but no CAN, 2 live | 1 Hz |
| `PMAC_NMT` | CANopen state: 5 Operational, 127 Pre-op, 4 Stopped, 255 none | 1 Hz |
| `PMAC_FLT` | Sevcon fault id, 0 = none | 1 Hz |

About 1.4 kB/s, 12% of the 115200 link. `tools/vtx_ground_station.py` folds
them into one line a second:

```
PMAC LIVE  Operational  speed 1234 rpm  torque +0.512  Ibat 12.5 A  Vcap 48.5 V  ...
```

## Testing the RPMsg path alone: `can2m33_test`

`can2m33_test.c` sends the same messages as `can2m33` but fills them with a
simulated drive instead of CAN: 0 to 5000 rpm, cruise, regen back to 0, parked,
every 30 s. No CAN interface, no controller, no `pmac.c` - only the M33 and
the link. Run it instead of `can2m33`, not alongside it: both would write to
the same channel.

```bash
# on the board, next to pmac_telem.h
gcc -O2 -Wall -Wextra -o can2m33_test can2m33_test.c -lm
modprobe rpmsg_char
./m33_setup.sh mavlink_vtx_cm33.elf     # M33 with mavlink_vtx
ls /dev/rpmsg*                          # the channel: appears ~1 s after start
./can2m33_test
```

| run | the ground station shows |
|---|---|
| `./can2m33_test` | `PMAC LIVE  Operational  speed ... rpm` with the numbers moving |
| `./can2m33_test -n` | `PMAC Linux up, NO CAN` - message arriving, no valid values |
| `./can2m33_test -f 0x42` | as live, plus `FAULT 0x0042` |
| stop it | `PMAC NO DATA from Linux` within about a second |

If no `/dev/rpmsgN` appears, the M33 side never announced the channel: check
`dmesg` for the remoteproc boot and for `rpmsg-raw`, and that the M33 console
shows no `RPMSG: WARNING`.

## Testing the CAN decode without the controller

FlexCAN's internal loopback answers its own frames without driving the bus,
so fake PMAC frames can be injected on the board:

```bash
ip link set can0 down
ip link set can0 up type can bitrate 500000 loopback on
./can2m33 -v &
cansend can0 454#05D2040000        # TPDO5: speed 1234 rpm
cansend can0 271#080323C000F401    # TPDO4: 48.5 V, 35 C, 12.0 A, torque 0.5
cansend can0 701#05                # heartbeat: Operational
# afterwards
ip link set can0 down
ip link set can0 up type can bitrate 500000 loopback off
```

Values drop out of the message 0.5 s after the last frame, so send them in a
loop (`while :; do cansend ...; sleep 0.04; done`) to hold them.
