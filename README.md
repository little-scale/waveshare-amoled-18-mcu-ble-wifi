# Waveshare AMOLED 1.8 Motion Controller — BLE / Wi-Fi

Single-screen motion firmware for dancers using Max over Wi-Fi/OSC, BLE, or both.
Hardware: Waveshare ESP32-S3-Touch-AMOLED-1.8 V2, 368 × 448.

This standalone repository contains the single-screen motion controller, derived from
[Micro Apps for ESP32-S3 Touch AMOLED 1.8](https://github.com/little-scale/micro-apps-for-esp32-s3-touch-amoled-1.8).
Microphone processing and the micro-app pages are absent.

## Quick start

1. Flash the firmware using the build instructions below.
2. The board starts in BLE mode and advertises as `device-xxxx`, using its hardware suffix.
3. Connect and subscribe to the IMU characteristic listed below. Each notification
   contains nine float32 values: acceleration, angular velocity, pitch, roll and yaw.
4. Tap the sphere to re-centre. Use the cog for settings; choose TRANSPORT to enable
   Wi-Fi/OSC or both transports, then configure NETWORK and OSC as needed.

Full message formats, UUIDs and control commands: [PROTOCOL.md](PROTOCOL.md).

## Interaction

A wire sphere with three rotating great circles and a distinctive marker shows orientation.
Solid marker = facing hemisphere; outlined marker = rear hemisphere.
Tap and release within the main display area to make the current pose the reference.
This resets the displayed and transmitted orientation together, including when the ball is hidden.
A tap must last less than 600 ms and travel less than 18 pixels.
Raw acceleration and angular velocity are not re-centred.

The top row contains a settings cog, Wi-Fi, BLE and battery indicators.
Tap the cog for settings or the Wi-Fi indicator for network selection.
The Wi-Fi glyph is bright when connected and muted otherwise.
Tap BLE for transport settings. Its glyph is bright when subscribed with sufficient
MTU; a small dot indicates a connection. The transport page shows detailed status.
An `IMU?` indicator appears if samples are unavailable.
There is no page-switch button.

## Settings

- **NETWORK:** on-device scan, network selection, password keyboard with SHOW/HIDE,
  connection and retry. No hotspot is needed.
- **OSC:** target IPv4 address, destination port and receive port.
- **DEVICE:** shared hostname, OSC device name and BLE advertised name. Defaults to `device-xxxx`
  using the hardware suffix. Saving a name restarts the board.
- **MOTION:** 50/100 Hz output cap, change-only/continuous mode, gyro calibration
  and re-centre. SAVE stores rate/mode; BACK discards edits.
- **TRANSPORT:** tap to cycle Wi-Fi/OSC, BLE, Both; select BLE 50/100 Hz.
  SAVE restarts if changing transport. BLE is the default transport at 50 Hz for
  boards without a saved preference; existing choices are retained. MOTION's rate
  controls OSC; its change/continuous setting applies to both.
  BLE-only disables Wi-Fi. Network selection then opens TRANSPORT so you can enable
  Wi-Fi before scanning; existing network credentials are retained.
- Settings close after 150 seconds without touch input.

Gyro calibration asks you to keep the board still for roughly 2 seconds. Motion
resets its collection; it fails after 6 seconds without a sufficient still period.
Calibration saves gyro bias only and does not change the reference pose.
Tap-to-recentre works immediately in any pose and does not require stillness.

Existing Wi-Fi, OSC and device-name settings are reused. This edition stores its
rate/mode and gyro bias under separate keys so old IMU-output-off settings and the
full edition's differently mapped calibration are not inherited.
Streaming is always enabled by default and has no output toggle.

## BLE

Select BLE or Both in TRANSPORT and save. Connect to the device's configured name,
then subscribe to its IMU characteristic. Each notification is **36 bytes: nine
little-endian IEEE-754 float32 values**, in the same order and units as OSC. There is
no header. Split each notification into nine consecutive groups of four bytes.
The minimum negotiated ATT MTU is 39 (the device requests 64); smaller MTUs produce
no motion notifications rather than truncated frames. The settings screen reports this.

- Service: `89689de0-1604-42c1-85ba-e286075c4488`
- IMU Notify: `2e5abe1b-4f44-429e-8b47-1526afbe343d`

See [PROTOCOL.md](PROTOCOL.md) for control writes, metadata and Max decoding details.
BLE and OSC have independent change baselines and send limits. Both use the latest
sample; successful BLE sends mean accepted by the stack, not acknowledged by Max.
Radio contention and central connection intervals can reduce delivered rates.

## OSC

Replace `<device>` with the configured name. Default destination: port 9000.
Default device receive port: 9001. All floats are OSC float32 and all integers int32.

| Direction | Address | Types | Arguments |
| --- | --- | --- | --- |
| Device → Max | `/<device>/imu0` | `fffffffff` | `ax ay az gx gy gz pitch roll yaw` |
| Max → device | `/<device>/foreground` | `iii` | RGB integers, each 0–255 |
| Max → device | `/<device>/background` | `iii` | RGB integers, each 0–255 |
| Max → device | `/<device>/display` | `i` | 0 hides the ball; 1 shows it |

Defaults are white foreground, black background and visible ball. Colours and
visibility are session settings and reset on reboot. Incoming commands are never echoed.
Display visibility affects only the ball region; the status row, settings, touch-to-recentre,
sensor fusion and outgoing motion continue. Receive single OSC messages, not OSC bundles.

Acceleration is measured in g, including gravity. Gyro values are bias-corrected
degrees/second. Pitch/roll/yaw are degrees relative to the last reference pose.
Pitch is -90…90; roll and yaw are -180…180. Euler angles have wrap points and a
singularity near ±90° pitch even though the internal quaternion remains continuous.
Max should unwrap angular values before differentiating them.

The motion edition applies a proper 90° coordinate rotation to both sensor vectors:
`x = sensor.y; y = -sensor.x; z = sensor.z`.
This deliberately replaces the full edition's reflected X/Y swap; do not assume
identical signs when reusing older Max mappings. Verify with simple single-axis
turns in the dancer's intended mounting orientation.

## Sampling and timing

The sensor worker targets up to 200 updates/second, subject to fresh sensor data.
The configured sensor ODR is approximately 224 Hz when both sensors are enabled.
Gyro range is ±1024°/s and acceleration range ±8 g.
The quaternion complementary filter integrates gyro and uses gravity for pitch/roll
correction, reducing accelerometer correction during large acceleration.
There is no magnetometer: yaw is relative and drifts. Re-centre as needed.

OSC defaults to change-only, capped at 100 messages/second (10 ms).
50 Hz is also selectable. Any one of these changes against the last successfully
sent frame triggers transmission of all nine values:
- acceleration: 0.01 g on any axis
- angular velocity: 0.5°/s on any axis
- orientation: 0.25° on any axis, using shortest angular difference

Full float values are transmitted without rounding. Gradual changes accumulate
against the last sent frame. First connection, reconnection, re-centring and routing
changes refresh the baseline. Change-only mode stays silent when unchanged.
Continuous mode sends the latest fresh sample at the selected cap.
There is no idle heartbeat and no timestamp argument; Max can time-stamp arrivals.
UDP delivery and exactly even arrival timing are not guaranteed.

Sampling, transmission and drawing run separately. A single latest-frame mailbox
prevents queued old data; display redraw is capped at about 30 Hz.
Sampling and stream tasks run on core 0, UI on the Arduino loop core. I2C operations
share a mutex; drawing does not hold that mutex.
No samples older than 100 ms are sent.
Settings screens keep motion running, although Wi-Fi scans/reconnection naturally
interrupt network delivery.

## Build, flash and tests

Use the scripts from this checkout:
```sh
./scripts/build.sh
./scripts/flash.sh /dev/cu.usbmodem...
```

Install Arduino CLI and the pinned ESP32 core once:

```sh
arduino-cli core update-index --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core install esp32:esp32@3.3.10 --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
```

Hardware libraries are bundled in `vendor/waveshare-v2`; see [third-party notices](THIRD_PARTY_NOTICES.md).
Partition layout remains 16 MB flash, approximately 3 MB app,
8 MB OPI PSRAM. The scripts flash with verification and do not erase saved settings.

Private provisioning is optional: copy `firmware/Provisioning.local.h.example` to
`firmware/Provisioning.local.h`. This private file is ignored by Git and is not
automatically copied from another checkout. Compiled provisioned binaries contain credentials.

Host tests exercise the actual fusion and stream gate:
```sh
c++ -std=c++17 -I tests/stubs -I firmware tests/motion_test.cpp -o /tmp/motion-test
/tmp/motion-test
```

GitHub Actions runs the host tests and compiles the firmware on pushes and pull requests.

## Source guide

| File | Responsibility |
| --- | --- |
| `firmware/firmware.ino` | Sensor/stream tasks, latest-frame mailbox, settings and remote commands |
| `firmware/ImuService.cpp`, `MotionMath.h` | Sensor acquisition, calibration, quaternion fusion and re-centre |
| `firmware/MotionStream.h` | Rate cap and change thresholds |
| `firmware/MotionPacket.h`, `BleTransport.cpp` | Binary float encoding, GATT subscription and control writes |
| `firmware/OscTransport.cpp` | OSC messages over UDP |
| `firmware/UserInterface.cpp` | Sphere, touch interaction, network keyboard and settings screens |
| `firmware/ConfigStore.cpp` | Saved preferences |
| `tests/motion_test.cpp` | Host tests for fusion, streams, byte encoding and command validation |

Distributed under the [GNU GPL v3](LICENSE); bundled libraries retain their own licences.
