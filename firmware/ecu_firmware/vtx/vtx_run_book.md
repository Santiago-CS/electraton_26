# VTX video on the laptop: runbook

Use these steps to view live video from the Eachine Sphere Link VTX on the Dell G15 (Ubuntu 24.04), from a cold start.

```
VTX camera ─► majestic (H.265, 1080p60, 4 Mbit/s) ─► wfb_tx ── ch 161 ──► USB-C receiver (RTL8812AU)
   ─► wfb-ng ground station (wifibroadcast@gs) ─► UDP 127.0.0.1:5600 ─► gst-launch window
```

One-time setup is already done:
- patched `rtl8812au` driver installed with DKMS, Secure Boot key enrolled
- wfb-ng installed
- channel 161 set in `/etc/wifibroadcast.cfg`
- `/etc/gs.key` pairs with the VTX

## 0. Install the video tools (once)

```bash
sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
                 gstreamer1.0-plugins-bad gstreamer1.0-libav
gst-inspect-1.0 avdec_h265 | head -3     # must print the element, not "No such element"
```

## 1. Hardware

1. Plug the USB-C receiver into the laptop.
2. Screw the antenna onto the VTX **before** powering it. 800 mW into no antenna can damage the VTX's transmitter.
3. Power the VTX from **15 V, at least 1.5 A**, with the current limit at about 2 A. A weak supply makes the VTX reboot every ~30 s as soon as it starts transmitting.
4. Keep the VTX 1–2 m from the receiver. Closer than that, the 800 mW signal can overload the receiver.

## 2. Check the receiver driver

```bash
ip -br link | grep wlx                    # interface name, e.g. wlxc8fe0f574df1
ethtool -i wlxc8fe0f574df1 | head -1      # must say: driver: rtl88xxau_wfb
```

If it says `rtw88_8812au`, the kernel's own driver took the receiver. Swap it for the wfb driver:

```bash
sudo modprobe -r rtw88_8812au && sudo modprobe 88XXau_wfb
# unplug and replug the receiver, then check again
# if it comes back after a reboot: sudo update-initramfs -u, then reboot
```

## 3. Check the ground-station config

```bash
grep -E '^wifi_channel|^wifi_region' /etc/wifibroadcast.cfg   # wifi_channel = 161
grep -A2 '\[gs_video\]' /etc/wifibroadcast.cfg                # peer = 'connect://127.0.0.1:5600'
ls -l /etc/gs.key                                             # must exist
```

## 4. Start the ground station

```bash
sudo systemctl start wifibroadcast@gs
systemctl status wifibroadcast@gs --no-pager | head -5        # Active: active (running)
# optional: start it automatically at boot
sudo systemctl enable wifibroadcast@gs
```

## 5. Confirm the radio link

```bash
wfb-cli gs                 # Ctrl+C to exit
```

In the **RX: gs video** box, look for:

| Field | Healthy |
|---|---|
| `recv` | climbing, around 600–700 pkt/s |
| `udp` | about ⅔ of `recv` (FEC 8/12) |
| `sess` | `1` (the key pairs with the VTX) |
| `lost` | `0` |
| `Freq` | `5805` (channel 161) |
| `[ANT]` RSSI | stronger than about −70 dBm; on the bench, −20 to −40 |

In a second terminal you can follow the logs: `sudo journalctl -fu wifibroadcast@gs`. A healthy start logs `New session detected [video rx]`.

## 6. Show the video

Only one program can listen on UDP 5600 at a time, so close QGroundControl first.

**Basic viewer (software decode):**

```bash
gst-launch-1.0 udpsrc port=5600 \
  caps='application/x-rtp,media=video,clock-rate=90000,encoding-name=H265' \
  ! rtph265depay ! h265parse ! avdec_h265 ! videoconvert \
  ! autovideosink sync=false
```

**With an FPS overlay:**

```bash
gst-launch-1.0 udpsrc port=5600 \
  caps='application/x-rtp,media=video,clock-rate=90000,encoding-name=H265' \
  ! rtph265depay ! h265parse ! avdec_h265 ! videoconvert \
  ! fpsdisplaysink video-sink=autovideosink text-overlay=true sync=false
```

**View and record at the same time.** The `-e` makes Ctrl+C close the file cleanly.

```bash
gst-launch-1.0 -e udpsrc port=5600 \
  caps='application/x-rtp,media=video,clock-rate=90000,encoding-name=H265' \
  ! rtph265depay ! h265parse ! tee name=t \
  t. ! queue ! avdec_h265 ! videoconvert ! autovideosink sync=false \
  t. ! queue ! matroskamux ! filesink location=vtx_$(date +%Y%m%d_%H%M%S).mkv
```

**Hardware decode (optional, less CPU).** First check that the VA-API decoder exists:

```bash
gst-inspect-1.0 vah265dec | head -3
```

If it does, replace `avdec_h265 ! videoconvert` with `vah265dec ! videoconvert` in any pipeline above.

**If the VTX is set to H.264** instead of H.265, change `H265` to `H264` and `rtph265depay ! h265parse ! avdec_h265` to `rtph264depay ! h264parse ! avdec_h264`.

**QGroundControl instead of GStreamer:**
1. Open Application Settings → Video.
2. Set Source to `UDP h.265 Video Stream` and URL to `0.0.0.0:5600`.
3. Turn on Low Latency Mode.

## 7. Stop

```bash
# Ctrl+C in the gst-launch terminal, then:
sudo systemctl stop wifibroadcast@gs
```

Unplug the VTX power **before** removing its antenna.

## 8. Check or change the VTX video settings (optional, over the air)

The ground station also brings up a tunnel to the VTX:

```bash
ssh root@10.5.0.10                       # password 12345
grep -A12 '^video0:' /etc/majestic.yaml  # codec, size, fps, bitrate
cat /etc/wfb.conf                        # channel, txpower, mcs_index
```

After editing `/etc/majestic.yaml`, restart the video encoder with `/etc/init.d/S95majestic restart`.

Avoid restarting `S98datalink` over SSH: it drops the tunnel your session runs on. Use `reboot` instead and reconnect after about 40 s.

## 9. Troubleshooting

| Symptom | Likely cause | Check / fix |
|---|---|---|
| `recv` stays 0 and the journal shows nothing | VTX isn't transmitting, or is on another channel | Check VTX power: if its serial console shows `IPL` lines every ~30 s, it's browning out. On the VTX, `ps \| grep wfb_tx` must show `wfb_tx`. On both sides, check that the channel is 161 (`/etc/wfb.conf` on the VTX). |
| `recv` climbs but `sess` stays 0, or the journal shows decrypt/session errors | Key mismatch | On the laptop, `/etc/gs.key` must pair with the VTX's `/etc/drone.key`. |
| `udp` > 0 but no video window | Decoder missing, or wrong codec | Install `gstreamer1.0-libav`. Check `codec:` in the VTX's `majestic.yaml` and use the matching H.264/H.265 pipeline. |
| Gray or smeared picture for the first second | Waiting for the first full keyframe (GOP 60 = 1 s) | Normal; it clears by itself. |
| Blocky picture or freezes, with `lost` > 0 | Weak or overloaded RF | Adjust the distance (1–2 m on the bench), check antennas, look at RSSI in `wfb-cli`. |
| `Address already in use` from udpsrc | Another viewer is holding port 5600 | Close QGroundControl or the other `gst-launch`. |
| `driver: rtw88_8812au` | Kernel driver took the receiver | See step 2. |
| Laptop's own Wi-Fi shows only 2.4 GHz networks | wfb-ng set the Wi-Fi region to RU in `/etc/modprobe.d/wfb.conf` | Change `ieee80211_regdom=RU` to `MX`, then reboot. |
