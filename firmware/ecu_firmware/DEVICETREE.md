# FRDM-i.MX93 device trees — handing peripherals to the Cortex-M33

**Date:** 2026-10-05
**Branch:** `ivansigala/gps`
**Kernel:** linux-imx `6.18.20`, tag `lf-6.18.20-2.0.0` (commit `b096ce610`)
**Stage:** both DTBs build clean and are verified against the stock blob. Only
`-m33-hdr.dtb` is new; `-m33-lpuart3.dtb` is the one already in use by
[`vtx/STATUS.md`](vtx/STATUS.md).

---

## 1. Goal

Give the Cortex-M33 exclusive use of a set of peripherals on the FRDM-i.MX93 so
M33 firmware can drive them without Linux fighting for the same registers,
pads and clocks.

Two device trees, both derived from the stock `imx93-11x11-frdm.dts`:

| DTB | Hands over |
|---|---|
| `imx93-11x11-frdm-m33-lpuart3.dtb` | LPUART3 only (the VTX telemetry link) |
| `imx93-11x11-frdm-m33-hdr.dtb` | The whole 40-pin header: LPUART3/6/8, LPSPI3/4, LPI2C3/4, 7 GPIOs, plus the 3V3 and 5V rails |

They are alternatives, not a stack — boot one or the other.

## 2. What "handing over a peripheral" actually requires

A Linux device tree cannot enable a peripheral *on* the M33; the M33 firmware
does that. What the DT must do is release the block and leave it in a state the
M33 can use. Four things, and missing any one of them breaks it:

1. **`status = "disabled"`** on the node, so Linux never probes it.
2. **`init-on-array` on `&clk`** — the critical and least obvious one. Without
   it `clk_disable_unused()` shuts the peripheral's clock gate off at late
   initcall and the M33's peripheral goes dead a few seconds into boot.
3. **`assigned-clocks` / `assigned-clock-parents`** to park the root clock on a
   known source (24 MHz oscillator here) so the M33 does not have to guess the
   rate.
4. **A pinctrl group muxing the pads**, attached to a node Linux *does* own
   (`&lpuart1`, the A55 console). Linux programs IOMUXC on the M33's behalf
   before the M33 firmware starts.

This is NXP's own pattern — see `imx93-11x11-frdm-root.dts` in the BSP, which
does exactly this for LPUART2. Both of our DTBs follow it.

Points 2–4 are why `status = "disabled"` on its own is not enough, and why a
plain overlay that only flips status will appear to work and then fail.

### TRDC needs no change

Checked `imx-atf/plat/imx/imx93/trdc_config.h`:

```c
{ 0, 2, 0, MBC_BLK_ALL, 0, true }, /* MBC0 AIPS2 for M33 DID2 */
```

M33 (DID2) already has read/write on all of AIPS2 in wakeupmix, which contains
LPUART3–8, LPSPI3–8 and LPI2C3–8. No ATF rebuild needed for any of this.

## 3. Pad numbering on the 40-pin header (P11)

**The FRDM-i.MX93 P11 header is Raspberry-Pi compatible and the SoC pads map 1:1
onto BCM numbering: `GPIO_IO<n>` is header `GPIO<n>`.**

This is worth stating because it removes an ambiguity that otherwise makes pad
assignment guesswork. It was derived from the pin numbers NXP documents in
`imx93-11x11-frdm-lpuart.dts`:

| NXP comment | Pad | Header pin | BCM name at that pin |
|---|---|---|---|
| "Pin27--TX" | `GPIO_IO00` | 27 | BCM0 (ID_SD) |
| "Pin28--RX" | `GPIO_IO01` | 28 | BCM1 (ID_SC) |
| "Pin3--CTS" | `GPIO_IO02` | 3 | BCM2 (SDA1) |
| "Pin5--RTS" | `GPIO_IO03` | 5 | BCM3 (SCL1) |
| "Pin7--TX" | `GPIO_IO04` | 7 | BCM4 |
| "Pin29--RX" | `GPIO_IO05` | 29 | BCM5 |
| "Pin31--CTS" | `GPIO_IO06` | 31 | BCM6 |
| "Pin26--RTS" | `GPIO_IO07` | 26 | BCM7 |

Eight for eight. The peripheral layout corroborates it independently — the
i.MX93 alternate functions land exactly where a Pi expects them:

- `IO08..IO11` = LPSPI3 PCS0/SIN/SOUT/SCK → RPi **SPI0** (pins 24/21/19/23)
- `IO12/IO13` = TPM3_CH2/TPM4_CH2 → RPi **PWM0/PWM1** (pins 32/33)
- `IO14/IO15` = LPUART3 TX/RX → RPi **UART0** (pins 8/10)
- `IO16..IO21` = LPSPI4 PCS2/PCS1/PCS0/SIN/SOUT/SCK → RPi **SPI1**
- `IO02/IO03` = LPI2C4 SDA/SCL → RPi **I2C1**

> Physical pin numbers for `IO08` and above are inferred from standard RPi 40-pin
> numbering, not read out of an NXP document. The pad-to-BCM rule is confirmed;
> if you need a specific physical pin for wiring, check it against UM12181 once.

## 4. `imx93-11x11-frdm-m33-lpuart3.dtb`

The minimal one. Already in service for the VTX MAVLink link.

| Signal | Pad | Header pin |
|---|---|---|
| LPUART3 TX | `GPIO_IO14` (ALT1) | 8 |
| LPUART3 RX | `GPIO_IO15` (ALT1) | 10 |

Changes versus stock `imx93-11x11-frdm.dtb`:

- `&clk { init-on-array = <IMX93_CLK_LPUART3_GATE>; }`
- `&lpuart3` → `status = "disabled"`, `dmas`/`dma-names` deleted (srcid 18 rx / 17 tx)
- new `pinctrl_uart3` group, attached to `&lpuart1`
- LPUART3 root clock parented to `IMX93_CLK_24M`
- `&edma2 { dma-channel-mask = <0x0 0xffffffff>; }` — see §6

Nothing on the board is lost. Linux keeps its console on LPUART1 and the M33
keeps LPUART2 as its own console.

## 5. `imx93-11x11-frdm-m33-hdr.dtb`

The whole header. Pad assignment:

| Block | Pads | Header pins | Notes |
|---|---|---|---|
| LPUART3 | IO14, IO15 | 8, 10 | TX/RX |
| LPUART6 | IO04–IO07 | 7, 29, 31, 26 | TX/RX + CTS/RTS |
| LPUART8 | IO12, IO13 | 32, 33 | **TX/RX only** |
| LPSPI3 | IO08–IO11 | 24, 21, 19, 23 | PCS0, SIN, SOUT, SCK |
| LPSPI4 | IO18–IO21 | 12, 35, 38, 40 | PCS0, SIN, SOUT, SCK |
| LPI2C3 | IO00, IO01 | 27, 28 | SDA, SCL |
| LPI2C4 | IO02, IO03 | 3, 5 | SDA, SCL |
| GPIO2 | IO16, IO17, IO22, IO23, IO26 | 36, 11, 15, 16, 37 | plain GPIO |
| *CAN2 — Linux, not M33* | *IO25, IO27* | *22, 13* | *onboard transceiver* |

`IO24` (pin 18) is deliberately left untouched. The allocation covers
`IO00`–`IO27` with no overlap.

**`IO25`/`IO27` are NOT M33 GPIOs** — they carry CAN2, see §5.1. They were in
the original request but the board makes it impossible: those two pads feed the
FRDM's onboard CAN transceiver and are the only CAN2 route on this board.
`IO24`, `IO28` and `IO29` are free if you need substitute GPIOs.

### 5.1 CAN2 stays with Linux

`flexcan2` is left **enabled for Linux**, exactly as the stock board DTS has it:

```dts
/* no override -- base imx93-11x11-frdm.dts owns it */
pinctrl-0 = <&pinctrl_flexcan2>;      /* IO25 = CAN2_TX, IO27 = CAN2_RX */
pinctrl-1 = <&pinctrl_flexcan2_sleep>;
xceiver-supply = <&reg_can2_stby>;     /* PCAL6524 GPIO 23, active low */
status = "okay";
```

This costs `IO25`/`IO27` from the M33 GPIO list, and there is no way around it:

- CAN2's four pad options are `IO25/IO27`, `DAP_TDI/DAP_TDO` (taken by lpuart5 /
  Bluetooth), `ENET1_TD3/TD2` (Ethernet) and `SD2_DATA0/DATA1` (SD card). Only
  the first pair is free.
- `flexcan1` is **never referenced** by `imx93-11x11-frdm.dts` — no transceiver
  is wired to it on this board. Its pad options (`PDM_*`, `SAI1_TX*`) collide
  with the audio block anyway. It stays `disabled`.

Expect `can0` in Linux. The transceiver's standby line is handled by the
regulator, so no manual GPIO poking is needed.

### Two conflicts, resolved in favour of the request

The i.MX93 offers very few alternate-function choices for these blocks, so two
collisions were unavoidable:

- **LPUART8 has no flow control.** Its only CTS/RTS pads are `IO14`/`IO15`,
  which LPUART3 is using. TX/RX only.
- **LPSPI3 and LPSPI4 each get one chip select.** LPSPI3 PCS1 is `IO07`, taken
  by LPUART6 RTS. LPSPI4 PCS1/PCS2 are `IO17`/`IO16`, which are in the GPIO
  list. Use PCS0, or drive extra chip selects as GPIO.

### Clocks

All eight gates in `init-on-array`:

```
LPUART3_GATE  LPUART6_GATE  LPUART8_GATE  LPSPI3_GATE
LPSPI4_GATE   LPI2C3_GATE   LPI2C4_GATE   GPIO2_GATE
```

All seven root clocks parented to `IMX93_CLK_24M`. **This caps LPSPI SCK at
~12 MHz.** If a device on SPI needs more, reparent that root to a
`sys_pll_pfd*_div2` source — all these roots use the `LOW_SPEED_IO_SEL` mux, so
`osc_24m`, `sys_pll_pfd0_div2`, `sys_pll_pfd1_div2` and `sys_pll_pfd2_div2` are
the available parents.

### Header supply rails

The stock DTS declares `reg_vexp_3v3` (VEXP_3V3) and `reg_vexp_5v` (VEXP_5V)
but **gives them no consumers**, so the regulator core leaves both rails off and
the header is unpowered. Both now carry `regulator-always-on` and
`regulator-boot-on`:

- 3V3 → header pins 1, 17 — gated by PCAL6524 GPIO 2, `vin-supply = <&buck4>`
- 5V → header pins 2, 4 — gated by PCAL6524 GPIO 8

The PCAL6524 expander sits on **LPI2C2**, which Linux keeps. Handing LPI2C3 to
the M33 therefore does not strand the rails.

### What this costs on the Linux side

| Lost | Why |
|---|---|
| `tpm3`, `tpm4` — RGB LED PWM | Deleted. Channels are on `IO04`/`IO12`/`IO13`, needed by LPUART6 and LPUART8. |
| `spidev0` | LPSPI3 is disabled. |
| AP1302 camera + ISP I/O expander | Both were on LPI2C3, now disabled. The AP1302 capture path is gone. |

If any of these matter, drop the corresponding block from the request — the
conflicts are all pad-level and independent of each other.

## 6. DMA4 (eDMA2) channel split

Both DTBs reserve **DMA4 channels 32-63 for the Cortex-M33**:

```dts
&edma2 {
	dma-channel-mask = <0x00000000 0xffffffff>;
};
```

`&edma2` is `fsl,imx93-edma4` at `0x42000000`, 64 channels — the SDK's **DMA4**,
in WAKEUPMIX. Every peripheral handed over above lives in WAKEUPMIX, so they all
request on DMA4. (`&edma1` is DMA3, AONMIX, 31 channels — that is what the M33's
LPUART2 console would use.)

### Why the mask is required

Without it the M33 cannot own a DMA4 channel at all. Two mechanisms in
`drivers/dma/fsl-edma-main.c`:

1. **Linux wipes every channel at probe.** The per-channel init loop zeroes the
   channel CSR and clears `CH_MUX` for all 64 channels. Anything the M33 set up
   beforehand is clobbered.
2. **There is no statically safe channel.** `imx93_data4` sets
   `FSL_EDMA_DRV_HAS_CHMUX`, so `fsl_edma3_xlate()` grabs *any* free channel and
   writes the source ID into `CH_MUX`. Allocation is dynamic. Linux also
   requests a per-channel IRQ for all 64.

Set bits in `dma-channel-mask` are the channels Linux **skips entirely** — no
IRQ request, no CSR/`CH_MUX` write, no suspend/resume, not registered. Note this
is the **opposite polarity** to the `brcm,dma-channel-mask` binding of the same
name; confirmed from the code (`if (chan_masked & BIT(i)) continue;`) and from
`imx8qm-ss-audio.dtsi` using `<0>` to mean "reserve nothing".

### Why 32-63 is safe

Measured on the board running `-m33-hdr.dtb`, `/sys/kernel/debug/dmaengine/summary`
showed **zero** DMA4 channels in use (DMA3 had 8: lpi2c1, lpi2c2, lpuart1 console,
sai1). Scanning the blob, the only enabled Linux node that can ever request a
DMA4 channel is `lpuart5` (Bluetooth, srcid 22 rx / 21 tx) — **2 channels worst
case**. It reads as zero at runtime because `fsl_lpuart` only requests DMA when
the port is opened, and BT was not attached.

The allocator walks the channel list from the head and the list is built in
ascending index order (`vchan_init` → `list_add_tail`), so Linux always takes the
**lowest free** index. Its two channels land at ch0/ch1, nowhere near the
reserved block.

M33 worst case is 14 channels (7 blocks × 2 directions), so 32 is generous. To
change the split, the mask is a 64-bit value split low-word-first: ch16-63 would
be `<0x00000000 0xffffffff>` → `<0xffff0000 0xffffffff>`.

### Source IDs to program into `CHn_MUX`

These are DMA request source IDs, **not** channel numbers:

| Block | RX | TX |
|---|---|---|
| LPUART3 | 18 | 17 |
| LPUART6 | 88 | 87 |
| LPUART8 | 90 | 89 |
| LPSPI3 | 15 | 14 |
| LPSPI4 | 80 | 79 |
| LPI2C3 | 9 | 8 |
| LPI2C4 | 11 | 10 |
| *LPUART5 — Linux/BT, do not use* | *22* | *21* |

Channel *n*'s registers are at `0x42000000 + 0x10000 + n * 0x8000`, so ch32
starts at **`0x42110000`**.

### Caveat: DMA into M33 TCM

eDMA is bus master **DID7**. `imx-atf/plat/imx/imx93/trdc_config.h` grants DID7
access to AIPS2/AIPS3 (the peripherals — fine), but has **no DID7 entry for the
CM33 TCM blocks** in aonmix; only DID0/DID2/DID3 and "SoC masters DID10" for
system TCM. DMA4 writing into M33 TCM may therefore fault. Put DMA buffers in
DDR, or test TCM explicitly before relying on it.

## 7. Building

The source lives in the NXP kernel tree, which is **not** part of this repo:

```
~/projects/irt/imx-env/linux-imx/arch/arm64/boot/dts/freescale/
    imx93-11x11-frdm-m33-lpuart3.dts
    imx93-11x11-frdm-m33-hdr.dts
```

That tree is a shallow clone of `nxp-imx/linux-imx` at tag `lf-6.18.20-2.0.0`,
matching the tag on `imx-atf`, `imx-mkimage`, `imx-sm` and `uboot-imx`. Both
files are registered in `freescale/Makefile`.

### In-tree (needs `flex` and `bison`)

```bash
sudo apt install flex bison        # not currently installed on this host
cd ~/projects/irt/imx-env/linux-imx
make ARCH=arm64 freescale/imx93-11x11-frdm-m33-hdr.dtb
```

### Without them — `cpp` + system `dtc`

This is how both blobs were actually produced. Equivalent for a DTB-only build;
no cross-compiler needed either.

```bash
cd ~/projects/irt/imx-env/linux-imx
DTS=arch/arm64/boot/dts/freescale/imx93-11x11-frdm-m33-hdr

cpp -nostdinc -I include -I arch/arm64/boot/dts/freescale -I arch \
    -undef -D__DTS__ -x assembler-with-cpp $DTS.dts -o /tmp/hdr.pre
dtc -I dts -O dtb -@ -o $DTS.dtb /tmp/hdr.pre
```

`dtc 1.7.0` (Ubuntu `device-tree-compiler`) is what was used. `-@` emits
`__symbols__` so runtime overlays still apply.

Expect exactly one warning, which is pre-existing in the stock board DTS and
unrelated:

```
imx93-11x11-frdm.dts:604.10-615.6: Warning (graph_child_address):
  .../tcpc@50/connector/ports: graph node has single child node 'port@0'
```

### Deploying

Copy the blob to the board's boot partition and point U-Boot's `fdtfile` at it,
or overwrite the active DTB. The board is at `root@192.168.0.16` per
`vtx/STATUS.md`; the boot partition mount point is whatever that image uses —
check `mount | grep mmcblk` on the board rather than assuming.

Current copy in this directory:

```
imx93-11x11-frdm-m33-hdr.dtb   62621 bytes   md5 ea9825b93f66bf405276c89daf8ba30e
```

## 8. Verification performed

Mostly build-time. `-m33-hdr.dtb` **has** been booted once, which is where the
`/sys/kernel/debug/dmaengine/summary` output in §6 came from — that confirmed
the console, LPI2C1/2 and SAI1 come up normally and that DMA4 is idle. The
peripherals themselves have **not** been exercised from the M33, and the
`dma-channel-mask` added afterwards has **not** been booted.
(`-m33-lpuart3.dtb` has separately been in service, per `vtx/STATUS.md`.)

What was checked:

- Compiles with no new warnings.
- Decompiled and diffed against a stock `imx93-11x11-frdm.dtb` built the same
  way. After filtering phandle renumbering, the only deltas are the intended
  ones: 3 nodes flipped to `disabled`, `tpm3`/`tpm4` removed, 8 pin groups
  added, the regulator properties, and the `&lpuart1` / `&clk` additions.
- Every `fsl,pins` entry decoded back from the blob and matched against
  `imx93-pinfunc.h` — all 27 pad/mux pairs correct.
- Every clock ID in `init-on-array` and `assigned-clocks` decoded and checked
  against `include/dt-bindings/clock/imx93-clock.h`.
- `dma-channel-mask` decoded back from both blobs as `<0x0 0xffffffff>` =
  ch32-63, and cross-checked against the board's live
  `/sys/kernel/debug/dmaengine/summary`.
- Confirmed `osc_24m` is parent index 0 of `LOW_SPEED_IO_SEL` in
  `drivers/clk/imx/clk-imx93.c`, so the `assigned-clock-parents` cannot fail and
  take the A55 console down with it.

## 9. Open items

- **Zephyr has no nodes for most of this.**
  `nxp_zephyr/zephyr/dts/arm/nxp/imx/nxp_imx93_m33.dtsi` defines only
  `lpuart2: serial@44390000`. LPUART3/6/8, LPSPI3/4 and LPI2C3/4 do not exist
  for the M33 yet. For LPUART3 that means a `serial@42570000` node with
  interrupt 68 and `IMX_CCM_LPUART3_CLK`, plus pinctrl groups in
  `boards/nxp/frdm_imx93/frdm_imx93-pinctrl.dtsi`. Blocks M33 firmware using
  anything beyond LPUART2/LPUART3.
- **GPIO2 is a shared bank.** Linux still probes the controller, so do not
  request `IO16/17/22/23/25/26/27` from the Linux side while the M33 owns them.
  i.MX93 provides separate secure/non-secure GPIO apertures for exactly this
  split (TRDC exposes `GPIO2` and `GPIO2_In` as distinct blocks), so concurrent
  access to *different* pins is a supported design — just not to the same pin.
- **Header physical pin numbers for `IO08`+ are inferred**, see §3. Confirm
  against UM12181 before wiring anything irreversible.
- **LPSPI SCK capped at ~12 MHz** by the 24 MHz root, see §5.
- **Confirm the DMA4 mask on the next boot.** Masked channels are skipped
  before `vchan_init`, so they are never registered — `dmaengine/summary` should
  report `dma2 (42000000.dma-controller): number of channels: 32` instead of 64.
  If it still says 64 the property did not take and the M33 must not touch DMA4.
- **DMA into M33 TCM is unverified**, see §6. DDR buffers are the safe default.
