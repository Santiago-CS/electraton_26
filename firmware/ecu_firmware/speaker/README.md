# IRT Speaker — procedural engine sound for the FRDM i.MX93

Synthesised engine note for an electric drivetrain. Reads the PMAC motor
controller's CANopen feedback and plays a crossfaded, pitch-corrected sample
bank through the board's audio out.

**Just want to hear it?** On any Linux desktop:

```bash
sudo apt install build-essential libasound2-dev
gcc -O2 -o drive_kbd drive_kbd.c can_feed.c pmac.c -lasound -lm -lpthread
./drive_kbd
```

Hold ↑ to accelerate. Details in §3.

```
  Sevcon PMAC controller
        │ CANopen TPDOs, every 40 ms (606Ch velocity in 0x454, torque in 0x271)
        ▼
  pmac.c ── decodes all five TPDOs, SYNC, heartbeat, EMCY
        │
  can_feed.c ── reader thread ── smoothing ── virtual gearbox
        │                                   │
        │ engine rpm                        │ load 0..1
        ▼                                   ▼
  sampler.c ── pick bracketing rpm pair ── resample both to exact target pitch
             ── equal-power crossfade ── blend on/off-throttle layers
        │
        ▼
  ALSA (SAI codec) ── speaker
```

---

## 1. Files

| file | role |
|---|---|
| `sampler.c` | the board program `speaker`: sample bank, crossfade engine, ALSA output, command line (`--help`) |
| `pmac.c` / `pmac.h` | PMAC CANopen decoder: the TPDO map from the configuration spreadsheet, plus SYNC, heartbeat, EMCY and text/CSV output |
| `can_feed.c` / `can_feed.h` | SocketCAN reader thread feeding `pmac.c`, smoothing, virtual gearbox (`canfeed_map()`) |
| `drive_kbd.c` | keyboard test drive for a desktop terminal — simulated motor, real gearbox + sampler |
| `engine_sim.c` | standalone procedural synth, no samples. Fallback / reference |
| `samples/` | default pack, **sport** exhaust: 14 rpm points × 2 load layers + `manifest.txt` |

**Not in this folder yet:** `make_v8_pack.py` (the V8 pack generator),
`trim_loop.py` (cuts real recordings into loops), the alternative packs
`samples_straight/`, `samples_stock/` and `samples_soft/`, and the demo
renders `compare_tones.wav`, `demo_drive.wav` and `demo_mustang_v8.wav`. §8
and §9 describe how to use the two scripts; only `samples/` ships here, so
`V8_PACK` has nothing else to point at until they are added.

`engine_sim.c` is independent of everything else. Keep it — it's useful when
you want to hear a change without regenerating a pack, and it's the smaller
footprint option if you ever need to run this on an MCU instead.

---

## 2. Build matrix

```bash
sudo apt install libasound2-dev

# the board program: CAN feedback -> decode -> virtual engine -> ALSA
gcc -O2 -o speaker sampler.c can_feed.c pmac.c -lasound -lm -lpthread
./speaker                  # engine sound + PMAC decode printed live, on can0
./speaker -q               # engine sound only
./speaker --no-audio can0  # decode only - verify the bus before touching audio
./speaker --csv            # decode as CSV, one row per 40 ms cycle
./speaker -r capture.log   # replay a candump log: decode + gearbox, no sound
./speaker --sim            # bench: simulated throttle ramp, no controller
./speaker --help           # everything else (-v, -n node, -T rated Nm, -g volume)

# keyboard test drive on a desktop (arrow keys)
gcc -O2 -o drive_kbd drive_kbd.c can_feed.c pmac.c -lasound -lm -lpthread
./drive_kbd

# offline render to WAV, no CAN, no audio hardware
gcc -O2 -DOFFLINE -o sampler_off sampler.c -lm
./sampler_off demo.wav

# procedural synth, no samples
gcc -O2 -o engine_sim engine_sim.c -lasound -lm
```

All targets build clean under `-Wall -Wextra`, linked against real libasound.
Anything that links `can_feed.c` needs `pmac.c` too: the decoder lives there.

`sampler.c` has no `main()` when compiled with `-DSAMPLER_NO_MAIN`; that is how
`drive_kbd.c` reuses the sample bank and `render()` without copying them.

---

## 3. Keyboard test drive (desktop Linux)

Try the sound before the board and the controller are involved. `drive_kbd`
fakes only what the PMAC controller would report — motor rpm and torque — and
sends it through the **same** `canfeed_map()` gearbox and `render()` sampler the
car runs. What you hear here is what the car will play.

```bash
sudo apt install build-essential libasound2-dev
cd speaker
gcc -O2 -o drive_kbd drive_kbd.c can_feed.c pmac.c -lasound -lm -lpthread
./drive_kbd
```

| key | action |
|---|---|
| ↑ (hold) | full throttle |
| ↓ (hold) | brake — heavy regen, downshifts on the way down |
| → / ← | part-throttle lever ±10 %, stays set (cruise) |
| Space | lever back to 0 |
| `n` | neutral on/off: the engine free-revs, the car coasts |
| `+` / `-` | volume |
| `q` or Ctrl-C | quit |

Things to try: floor it from a standstill and listen to the 1→4 shifts; set
the lever to 30–40 % and notice it upshifts early like a gentle driver; lift at
speed for the overrun burble; `n` then tap ↑ for blips.

**How "hold" works.** A terminal only sends key presses and auto-repeats, never
a release. So a key counts as held for 0.6 s after the first press, which
covers the auto-repeat start delay, then for 0.12 s after each repeat. A single
tap gives a short blip; releasing a held key lets go within about 0.1 s. If your
auto-repeat delay is longer than 0.6 s (`xset q` shows it), raise `HOLD_FIRST`.

**Audio.** It opens ALSA `default`, which on Ubuntu or Fedora goes through
PipeWire or PulseAudio to your normal speakers. `ALSA_DEV=...` overrides this,
and `V8_PACK=...` points at a different sample pack:

```bash
./drive_kbd                            # sport (default)
V8_PACK=samples_straight ./drive_kbd   # sharpest
V8_PACK=samples_stock    ./drive_kbd
V8_PACK=samples_soft     ./drive_kbd   # the original, for comparison
```

**Simulated car.** The constants at the top of `drive_kbd.c` set how fast the
fake car is. Stock values: 0 to top speed in about 9 s, 1st gear lasts about
2 s. `K_ACC` is acceleration, `M_BASE`/`M_MAX` the motor's
constant-torque/constant-power corner and ceiling. None of this ships to the
car; tune `can_feed.c` for anything the car should inherit.

---

## 4. Audio setup on the board

```bash
aplay -l                         # find the output card,device
ALSA_DEV=plughw:0,0 ./speaker
```

On the FRDM i.MX93 the output is the MQS (`card 0: mqsaudio`), so
`plughw:0,0` is the one to try if `default` gives nothing.

Environment overrides: `ALSA_DEV`, `V8_PACK` (pack directory), `CAN_IF`,
`VOLUME`, and the `TONE_*` knobs in §4.1.

### 4.1 Brightness that tracks the engine

The packs are bright: in the stock pack the top octave sits only ~10 dB below
the midrange, partly the synthesised exhaust rasp itself and partly
linear-interpolation aliasing from `voice()` playing a loop above its own
pitch. On a small hard-mounted speaker, and especially on the i.MX93 MQS
output, that reads as a harsh fizz.

A flat cut is the wrong fix, though. That rasp **is** the character when the
engine is pulling; it only grates when it sits there unchanging at idle. Which
is also how a real engine behaves — muffled at idle, brightness arriving with
revs and load — so `render()` ends with a tone stage that *opens up* rather
than a filter that is always closed.

A brightness control `b` runs 0 at idle to 1 revving hard. rpm sets the
envelope (smoothstepped over `TONE_RPM_LO`..`TONE_RPM_HI`) and load scales it,
so overrun at 4000 rpm is not as bright as full throttle at 4000 rpm. `b` then
sweeps a high shelf and a low-pass between their idle and rev settings.

| variable | default | effect |
|---|---|---|
| `TONE_SHELF_HZ` | 2000 | shelf corner, Hz |
| `TONE_IDLE_DB` | -14 | shelf gain at `b`=0, dB |
| `TONE_REV_DB` | -2 | shelf gain at `b`=1, dB |
| `TONE_LPF_LO_HZ` | 5000 | low-pass corner at `b`=0, Hz |
| `TONE_LPF_HI_HZ` | 16000 | low-pass corner at `b`=1, Hz. `0` = open |
| `TONE_RPM_LO` | 1000 | rpm where it starts opening |
| `TONE_RPM_HI` | 4200 | rpm where it is fully open |
| `TONE_LOAD_W` | 0.5 | how much load darkens it, 0..1 |
| `TONE_TAU` | 0.12 | brightness smoothing, seconds |

It prints the resolved voicing at startup, and `drive_kbd` shows the live
shelf gain at the end of the status line, so you can watch it open as you
pull. Setting the older `TONE_SHELF_DB` / `TONE_LPF_HZ` pins both ends of the
sweep to one value, which gives back a static filter.

Measured against the original unfiltered voicing, at three fixed operating
points:

| band | idle (750 rpm) | cruise (2000, 60 %) | rev (5500, full) |
|---|---|---|---|
| 0–500 Hz | -0.0 dB | -0.0 dB | -0.0 dB |
| 1–2 kHz | -3.0 dB | -2.4 dB | -0.4 dB |
| 2–4 kHz | -10.1 dB | -8.4 dB | -1.5 dB |
| 4–7 kHz | -16.7 dB | -12.4 dB | -2.0 dB |
| 7–12 kHz | -25.1 dB | -17.8 dB | -2.1 dB |
| 12–21 kHz | -38.1 dB | -29.3 dB | -5.1 dB |
| overall RMS | -0.1 dB | -0.6 dB | -0.3 dB |

So idle loses the fizz, full throttle keeps essentially all of its character,
and the bass is untouched throughout. Cost is about 0.1 % of one i.MX93 core.

`TONE_TAU` matters more than it looks. A shift drops rpm by ~1500 in one
256-sample block; unsmoothed, the shelf would step ~15 dB instantly and click.
At the default it eases over ~300 ms instead.

Tuning by ear: `TONE_RPM_HI` is the main dial — lower it if the sound stays
dull too far up the rev range, raise it if the fizz comes back too early.
`TONE_IDLE_DB` sets how dark idle gets. Because the stage sits at the end of
`render()`, the CAN-driven `speaker` inherits all of it unchanged; this is a
voicing fix, not a keyboard-demo fix.

---

## 5. CAN feedback

### Bring up the bus

```bash
sudo ip link set can0 up type can bitrate 500000
ip -details -statistics link show can0
```

Check the bitrate against the controller's config. A mismatch shows as a
rising `bus-error` count, not as silence. The PMAC here runs 500 kbit/s,
node 1, with a SYNC every 40 ms.

If `ip link show type can` lists nothing, the kernel has no CAN interface at
all - the device tree does not enable one - and `./speaker` stops with
`cannot open can0`. `--sim` and `-r` still work.

### Verify the mapping before writing code

```bash
candump -td can0                 # timestamps, shows the TPDO rate
cansniffer can0                  # highlights bytes that change
```

Turn the motor slowly and watch which COB-ID has bytes moving monotonically.
Thirty seconds of this is worth more than any mapping document.

Two causes of "no frames at all":

- The controller must be in NMT **Operational** to transmit PDOs. If nothing
  else on the bus is NMT master: `cansend can0 000#0101` (start node 1). If a
  vehicle VCU is already master, stay listen-only and don't send it.
- If the TPDO transmission type is SYNC-driven (1–240) and there's no SYNC
  producer, it never fires. Add a SYNC producer, or set the TPDO to
  event-driven (type 255) with an event timer in `0x1800:05`.

### The decode

`pmac.c` holds the TPDO map from "CAN Open RD TPDO Configuration.xlsx", row
for row, checked against candump captures of the real controller: every TPDO
arrives with exactly the payload length the map implies. Byte offsets are not
written down - CANopen packs mapped objects in order, little-endian - so if
the controller is remapped, edit the rows of that TPDO and the rest follows.

| TPDO | COB-ID | contents |
|---|---|---|
| 1 | 0x391 | Id/Iq targets and actuals |
| 2 | 0x330 | Ud, Uq, modulation, inductance |
| 3 | 0x411 | field weakening limits, motor temperature |
| 4 | 0x271 | capacitor voltage, heatsink temperature, battery current, **torque** |
| 5 | 0x454 | digital inputs, **speed** |

Two of these drive the engine, chosen by `ENG_RPM_SIG` / `ENG_TRQ_SIG` at the
top of `can_feed.c`:

| object | drives | notes |
|---|---|---|
| `606Ch` velocity actual (TPDO5) | virtual engine rpm → pitch | rpm by default, but internal units if velocity scaling (`0x608F`, `0x6096`) is configured. Confirm against a tacho. |
| `2020h/6` actual torque (TPDO4) | load blend → timbre | thousandths of the rated torque in `2916h/1` |

Torque is a better load signal than a pedal would be: it captures current
limiting, thermal foldback and traction events for free, and negative torque
(regen) lands naturally on the overrun layer — engine braking and regen
braking are the same gesture.

### How `speaker` uses it

`main()` in `sampler.c` starts the reader thread (`canfeed_start()`), then
once per 256-sample audio block calls `canfeed_update()` for the engine rpm
and load, renders, and fades the output toward 1, 0.35 while a virtual shift
is in progress, or 0 when the feed is dead. All printing happens on the CAN
thread through a hook, never in the audio loop. `engine_step()` and
`read_pedal()` remain for `--sim`.

### Failsafe

Speed or torque older than 250 ms → `live` goes false → audio fades out over
~85 ms. Both have to be fresh: they arrive in different TPDOs, and a feed
that stayed live on torque alone would hold the pitch at the last speed. On a
vehicle this matters: a stuck engine note after a bus fault is worse than
silence, because it tells the driver the car is doing something it isn't.

---

## 6. Three things that will bite you

**Rate mismatch.** TPDOs arrive at 10–100 Hz - 25 Hz on this controller - and
the audio control loop runs at ~190 Hz. Raw frame values straight to pitch give
a zipper on every frame boundary. `canfeed_map()` runs a 60 ms one-pole
(`RPM_TAU`, `TRQ_TAU`) plus a 9000 rpm/s slew limit. If the TPDO rate drops,
lengthen the time constant rather than accepting the steps.

**Never read CAN from the audio path.** The blocking `read()` is on its own
thread. One blocking call in the ALSA loop is an instant xrun.

**A motor is not an engine.** The ME1302 pulls from zero and revs flat to
~5000 rpm. Map that straight to pitch and you get a CVT drone with no
character. The virtual 4-speed gearbox in `can_feed.c` does most of the
perceptual work. Set `MOTOR_MAX` to what the motor actually sees in your
drivetrain, not the datasheet ceiling, then tune `UPSHIFT[]` and `GRATIO[]`.

---

## 7. Tuning

| symptom | knob |
|---|---|
| free-revving in neutral too eager / lazy | `J` in `sampler.c` |
| keyboard demo car too fast / slow | `K_ACC` in `drive_kbd.c` |
| shifts at the wrong speed | `UPSHIFT[]` in `can_feed.c` (full-load shift points) |
| upshifts too early / late when driving gently | `SHIFT_LIGHT` — zero-load shift point as a fraction of `UPSHIFT[]` |
| rpm drop on shift too big/small | `GRATIO[]` ratio spacing |
| hunting between two gears | raise `DOWNSHIFT_HYST` toward 0.8 |
| pitch steps or warbles | lengthen the one-pole in `canfeed_map()` |
| too quiet / clipping | `-g` / `VOLUME` (0.1–2.0); the fixed gain is `1.75f` in the output loop of `main()` |
| sounds thin at low rpm | add rpm points below 1500 — that's where the ear is most sensitive |

---

## 8. Making other car sounds

This is the part worth understanding, because almost everything is one table.

### The general rule

For a 4-stroke, each cylinder fires once per **720°** of crank. The character
of an engine comes from *where in those 720° the events land*, and from
**which exhaust pipe each one goes down**. Cylinder count alone tells you very
little — a cross-plane and a flat-plane V8 have identical counts and sound
nothing alike.

Firing frequency, for reference:

```
f_fire = rpm/60 × cylinders/2        (4-stroke)
f_fire = rpm/60 × cylinders          (2-stroke)
```

### Edit two lists

In `make_v8_pack.py`, `ANG_R` and `ANG_L` are the crank angles at which each
bank fires, over one 720° cycle. Replace them and you have a different engine.

| engine | bank A | bank B | character |
|---|---|---|---|
| Inline-4 | `[0,180,360,540]` | *(none)* | even, buzzy, no burble |
| Inline-3 | `[0,240,480]` | *(none)* | thrummy, offbeat |
| Inline-6 | `[0,120,240,360,480,600]` | *(none)* | smooth, overlapping power strokes |
| **Cross-plane V8** (Mustang, Camaro) | `[0,180,450,630]` | `[90,270,360,540]` | **uneven per bank → the burble** |
| Flat-plane V8 (458, GT350) | `[0,180,360,540]` | `[90,270,450,630]` | both banks even → the scream |
| Flat-6 (911) | `[0,240,480]` | `[120,360,600]` | even overall, characteristic thrash |
| 45° V-twin (Harley) | `[0]` | `[315]` | the 315/405 gap → potato-potato |
| 90° V-twin (Ducati) | `[0]` | `[270]` | |

Cross-plane vs flat-plane V8 is the clearest demonstration: same eight
cylinders, same 90° global spacing, but the flat-plane crank splits them
evenly between the pipes. That single change is the whole difference between
a Mustang and a Ferrari.

### Non-4-stroke

Both `make_v8_pack.py` and `trim_loop.py` assume a 720° cycle:

```python
t_cycle = 120.0 / rpm_nominal        # 4-stroke, two revolutions
```

For a 2-stroke or a rotary the cycle is one revolution — change it to
`60.0 / rpm_nominal` and halve the angle values accordingly.

A 2-rotor 13B fires at roughly `[0,360]` / `[180,540]` on the eccentric shaft,
which is the same firing *frequency* as an inline-4 — but combustion lasts far
longer per event. Lengthen the pulse decay in `blowdown_kernel()` (the
`tau_d` fraction, try 3–4× longer) and you get the smooth wail instead of a
4-cylinder chop.

### Beyond firing order

The other half of the character is the exhaust itself. `make_v8_pack.py`
models it physically, per bank:

```
firing events -> blowdown pulse -> header runners -> tailpipe waveguide
   -> muffler -> sum banks -> wave steepening -> open-end radiation (d/dt)
```

The pipe resonances come out of the pipe lengths and do **not** move with rpm.
Everything is set by the `TONES` table and a few constants:

| want | change |
|---|---|
| sharper, crisper attack | shorter pulse decay in `blowdown_kernel()` (`0.040` → `0.030`) |
| raspier under load | raise `steep` for the tone |
| deeper, bigger car | longer tailpipes `L` (+0.5 m drops the resonances ~20%) |
| more muffled | lower `muff_fc`, raise `muff` |
| more ring / hollow "drone" | raise the magnitude of `R_END` toward −0.8 |
| lumpier idle, more gargle | spread `RUNNER_MS_R/L` further apart |
| less overrun crackle | `-DCRACKLE_RATE=3` when compiling (0 disables it) |

Why this sounds sharp where the v1 pack sounded soft: v1 fed single clicks
into narrow low resonators, which rings like a drum. The blowdown pulse rises
in ~0.1 ms (broadband edge), the waveguide keeps the transient intact, and the
open end radiates the *derivative* of the flow, +6 dB/octave. Measured on the
same full-throttle pull, spectral centroid went from ~870 Hz (v1) to ~1390 Hz
(sport) and ~1700 Hz (straight).

### Forced induction and EV

| layer | how |
|---|---|
| turbo spool | sine sweep 2–8 kHz, amplitude ∝ torque, lags rpm by ~300 ms |
| blow-off valve | one-shot noise burst, trigger on torque falling through zero |
| supercharger whine | tonal, locked to `rpm × pulley_ratio`, 2–4 kHz, narrow Q |
| EV / sci-fi | drop the impulse train entirely, use pure tones at fixed multiples of motor rpm (orders 12, 24, 36 — real inverter and gear whine) |

The EV route is the simplest of all: no firing pattern, just order-locked
sines. It's also the honest one if the goal is pedestrian warning rather than
entertainment — and if that's ever the target, check UNECE R138 / FMVSS 141,
which specify minimum levels and frequency content.

### Different car, same pipeline

1. Edit `ANG_R` / `ANG_L`, `TONES`, and `RPM_POINTS` in `make_v8_pack.py`
2. Regenerate: `python3 make_v8_pack.py --tone sport --out samples_mycar`
3. Retune `ENG_IDLE`, `ENG_REDLINE`, `UPSHIFT[]`, `GRATIO[]` in `can_feed.c`
4. Render offline and listen before you touch the board

Keep one pack per car in its own directory and select with `V8_PACK`. Nothing
in `sampler.c` is V8-specific — it only reads `manifest.txt`.

---

## 9. Using real recordings

### Sourcing

- **Freesound.org** — filter to CC0. Check the spectrogram for wind noise.
- **OpenGameArt.org** — CC0/CC-BY, game-oriented loops.
- **Pixabay Audio** — no-attribution licence, but it forbids redistributing the
  files as a sound library. Matters if you publish a pack.
- **VDrift**, **Stunt Rally 3** — rpm-layered engine sounds on GitHub, but
  **GPLv3**, which propagates to anything you ship with the assets.
- **Record it yourself.** For a specific car this is by far the best route. A
  clip-on lav taped 1 m behind the tailpipe, pointing *across* the flow rather
  than into it, beats anything you'll download.

Trademark note: car model names are manufacturer marks. Fine for internal work;
worth a lawyer's opinion before it goes on a product.

### Capturing

Each rpm point needs a **steady 3–4 second hold**, twice:

- **on-load** — in gear against the brakes, or on a dyno. Fat and raspy.
- **off-load** — coasting down through that rpm, throttle closed. Quieter,
  darker, with overrun burble.

8 points minimum, 12–16 better. Space them tighter down low: 700, 1000, 1300,
1700, 2200, 2800, 3500, 4300, 5100, 5900, 6600.

### Prep

```bash
ffmpeg -i onboard.mp4 -vn -ac 1 -ar 48000 -acodec pcm_s16le raw.wav
sox raw.wav trimmed.wav trim 12.4 4.0
python3 trim_loop.py trimmed.wav 3400 samples/3400_on.wav --cyl 8 --scan
```

`--scan` searches ±6% around your rpm guess for the cleanest loop period and
prints the corrected rpm. **Use that number in the manifest, not your guess** —
being 2% off means every crossfade through that point is permanently detuned.

```
# rpm    on.wav       off.wav          (must be ascending)
700.00   700_on.wav   700_off.wav
1200.00  1200_on.wav  1200_off.wav
```

Don't normalise every loop to the same peak — that flattens the dynamics. Aim
for idle sitting 12–15 dB below redline, and on-load ~6 dB above off-load at
the same rpm.

---

## 10. Why it sounds right

- **Per-loop resampling.** Both bracketing loops are stretched to the exact
  target rpm before crossfading. Skip this and you hear a pitch step at every
  sample point — the commonest giveaway in amateur engine audio.
- **Persistent phase.** Each loop keeps its playback position while silent, so
  fading back in never restarts mid-cycle and clicks.
- **Load is a separate axis from rpm.** 2500 rpm coasting and 2500 rpm pulling
  are different sounds at identical pitch. One-dimensional banks always sound
  like a toy.
- **Combustion scatter.** ±7 % per event on load, much more ragged on
  overrun, plus a fixed per-cylinder imbalance. Without it you get a synth
  buzz, not an engine.
- **Fixed pipe resonances.** The pipe doesn't change length with rpm.
  Pitch-shifting everything together sounds like tape speeding up — which is
  also why the pack uses 14 rpm points: less resampling, less smearing.
- **Live crackle.** Overrun pops are generated at random in `sampler.c`, not
  baked into loops, so they never repeat with the 1.2 s loop period.

---

## 11. Troubleshooting

| symptom | cause |
|---|---|
| `cannot open samples/manifest.txt` | run `make_v8_pack.py`, or set `V8_PACK` |
| `need 16-bit PCM` | re-encode: `ffmpeg -i in.wav -acodec pcm_s16le out.wav` |
| `speaker: cannot open can0` | no CAN interface (`ip link show type can`), or it is down - see §5 |
| silence, `ENGINE DEAD` in the decode | bus down, wrong bitrate, controller not Operational, no SYNC producer - or one of speed/torque missing |
| `ENGINE LIVE` but rpm stuck at 0 | speed TPDO remapped — compare `candump` with the map in `pmac.c` |
| rpm value wildly wrong | velocity scaling configured; it's in internal units, not rpm |
| clicking at regular intervals | loop isn't an integer number of cycles — rerun `trim_loop.py --scan` |
| pitch steps as it revs | resample ratio clamped: widen `RATIO_MIN`/`RATIO_MAX`, or add rpm points |
| xruns under load | raise `PERIOD`, or `chrt -f 70` |
| audio drops during shifts only | expected — that's the deliberate duck |
| `drive_kbd`: `stdin is not a tty` | run it straight from a terminal, not piped or from an IDE's run button |
| `drive_kbd`: tap of ↑ revs, holding doesn't keep it up | auto-repeat delay > 0.6 s or repeat off — `xset r on`, or raise `HOLD_FIRST` |
| `drive_kbd`: terminal left without echo after a crash | type `reset` and Enter |
| `open default: ...` on a desktop | no ALSA→PipeWire bridge: install `pipewire-alsa` (or `pulseaudio` alsa plugin) |

---

## 12. Known limitations

- All targets compile and link against libasound. `drive_kbd` was checked on
  a pseudo-terminal with simulated key auto-repeat into a paced null sink —
  not yet listened to on real speakers.
- The decode is checked against candump captures of the controller, but the
  engine rpm it produces has not been confirmed against a tachometer (see
  the `606Ch` note in §5).
- Mono only. Stereo would want two slightly different pipe detunes panned
  apart — cheap to add, meaningful improvement on a two-speaker install.
- No start/stop sounds. Cranking and shutdown are one-shots, not loops, and
  need a separate trigger path.
- The virtual gearbox is open-loop. It doesn't know about actual vehicle
  speed, so on a steep grade the shift points will feel early.
