# Motion edition protocol

Transport: BLE (default), Wi-Fi/OSC, or Both. Saved transport choices survive updates.
Single OSC messages only; no bundles.
All addresses start with the configured device name. Outgoing default port 9000,
incoming port 9001. Configure both and the target IPv4 address on the device.

## Output

`/<device>/imu0`, type tag `,fffffffff`

| Position | Value | Units |
| --- | --- | --- |
| 1–3 | ax, ay, az | g, includes gravity |
| 4–6 | gx, gy, gz | degrees/second, gyro bias corrected |
| 7–9 | pitch, roll, yaw | degrees relative to last re-centre |

Always on for enabled transports. OSC default cap 100 Hz, optionally 50 Hz;
BLE default cap 50 Hz, optionally 100 Hz. Change-only default compares with
the last successfully transmitted frame: 0.01 g, 0.5°/s or 0.25° on any axis.
All nine values are sent together when any threshold is exceeded, without rounding.
Optional continuous mode sends fresh frames at the cap. No stale sample backlog,
idle heartbeat, timestamps, movement triggers or additional outgoing messages.
First connection/reconnection, settings routing changes and re-centring refresh state.
Output continues when settings are open or the main visual is hidden.

## BLE GATT, motion protocol version 2

| Role | UUID | Properties |
| --- | --- | --- |
| Service | `89689de0-1604-42c1-85ba-e286075c4488` | Primary |
| IMU | `2e5abe1b-4f44-429e-8b47-1526afbe343d` | Notify |
| Control | `c2bbf0cb-9029-4925-abbc-3dd86492fc06` | Write, Write Without Response |
| Metadata | `fe9c3dcb-451c-4512-b845-a611ffa3280f` | Read |

Advertised name equals the configured device name. Connect, negotiate ATT MTU >=39
(device requests 64), and enable notifications on IMU's CCCD. Each notification is
exactly 36 bytes, no header, no timestamp, no fragmenting:

| Byte offsets (zero based) | Float values |
| --- | --- |
| 0–11 | ax, ay, az |
| 12–23 | gx, gy, gz |
| 24–35 | pitch, roll, yaw |

Each value occupies four bytes in **little-endian IEEE-754 binary32** order.
Examples: `00 00 80 3f` = 1.0; `00 00 80 bf` = -1.0; `00 00 00 00` = 0.0.
Values retain their float32 bit patterns; thresholds do not quantize the payload.
OSC handles its own standard network byte order; this BLE payload is little endian.
Version 2 metadata identifies this format. The full Micro Apps edition uses the
same UUIDs but a different header-bearing payload; do not reuse its offsets.

The supplied `4bf.maxpat` takes little-endian groups of four. Its normal-number
formula needs a special case for exponent zero to decode exact zero and subnormals
correctly (all-zero bytes should output exactly 0). No patch modification is bundled.

The central must negotiate sufficient MTU; otherwise notifications are withheld.
TRANSPORT shows connection, subscription and MTU status. A fresh frame is sent after
subscription/reconnection; BLE and OSC track separate change baselines. No idle heartbeat.
BLE rate is a cap, subject to central scheduling and radio capacity, not a delivery guarantee.

Control writes are short binary commands, **not OSC packets**:

| Bytes (decimal) | Action |
| --- | --- |
| `1 R G B` | Background, each channel 0–255 |
| `2 R G B` | Foreground, each channel 0–255 |
| `3 0` / `3 1` | Hide / show orientation ball |

Each write is one complete command of exactly four or two bytes. Invalid commands
are ignored. Accepted commands are applied on the UI task, with no echo. A bounded
16-command queue may drop writes if a sender floods it. Control writes do not require
an IMU subscription. This firmware does not require pairing/bonding; nearby clients
can connect and control it. Only one central connection is intended at a time.

## Input

| Endpoint | Tag | Payload |
| --- | --- | --- |
| `/<device>/foreground` | `,iii` | R G B, each integer 0–255 |
| `/<device>/background` | `,iii` | R G B, each integer 0–255 |
| `/<device>/display` | `,i` | 0 or 1 |

Invalid values, types, addresses and oversized packets are ignored.
Foreground/background change the palette, including the status and settings UI.
Display 0 clears just the orientation-ball area; status and settings remain visible.
Main-area taps still re-centre when hidden. Display 1 draws the current orientation.
No incoming command is echoed. Palette, visibility and reference pose reset on reboot.

## Orientation

Quaternion-based six-axis complementary fusion, with Euler output for Max.
Re-centre records the inverse current quaternion as the reference and applies it to
subsequent orientation. It does not alter acceleration or angular velocity.
Gyro calibration is separate and requires stillness.

Right-handed coordinate mapping: x=sensor.y, y=-sensor.x, z=sensor.z.
This differs from the full edition's legacy swap. Yaw has no absolute heading
reference and drifts. Euler wrap/singularities remain; unwrap in Max when needed.
