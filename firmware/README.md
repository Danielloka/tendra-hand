# Firmware

Low-level motor control running on the **ESP32-S3-N16R8**, built with **PlatformIO** (Arduino framework, C++17).
The PC sends joint targets in radians over **USB-C** (native USB serial). The ESP32 moves the motors there smoothly.

One code base builds firmware for two hands:

| Build env | Hand | Joints | Motors | Config |
|---|---|---|---|---|
| `tendra_s3` (default) | **v0** prototype (thumb + index) | 8 | 28BYJ-48 steppers + ULN2003 boards | `include/config.h` |
| `hand_v1_servo` | **v1** full hand | 20 | Feetech **SCS0009** smart servos on one bus, via the **FE-URT-1** | `include/config_v1.h` |

## Build, flash, talk to it

From this folder (`firmware/`), or use the PlatformIO buttons in VS Code:

```bash
pio run                          # build v0 (the default env)
pio run -e hand_v1_servo         # build v1
pio run -t upload                # flash v0 (ESP32 on USB-C); add -e hand_v1_servo for v1
pio device monitor               # type commands, see replies (Ctrl+C to quit)
```

If the upload can't connect: hold **BOOT**, tap **RESET**, release BOOT, and try again.

## How it's built

| File | Role |
|---|---|
| `include/config.h` | v0: **pin map**, joint names/limits (same order as the sim), speed/acceleration defaults |
| `include/config_v1.h` | v1: servo IDs, joint names/limits, calibration defaults, bus pins/baud, speed limits |
| `src/motor_driver.h` | **HAL interface**: one joint actuator, in radians, with optional feedback. The rest of the code only uses this. |
| `src/uln2003_stepper.*` | v0: 28BYJ-48 + ULN2003 driver (half-step), implements `MotorDriver` |
| `src/scs0009_servo.*` | v1: SCS0009 servo driver, implements `MotorDriver` |
| `src/scs_protocol.h` | v1: SCS packet building, checksum, reply parser, register map (pure C++) |
| `src/scs_bus.*` | v1: `ScsBus` — ping, read/write registers, sync write, feedback; over an abstract byte port |
| `src/scs_uart_port.h` | v1: the byte port on an ESP32 hardware UART |
| `src/servo_calibration.h` | v1: joint angle ↔ servo ticks (scale, zero, direction) |
| `src/motion_profile.h` | Smooth motion: accelerate, cruise, brake (trapezoidal profile), handles new targets mid-move. Used by both hands. |
| `src/main.cpp` | Safe startup, main loop, serial command protocol (v0/v1 picked by `TENDRA_HAND_V1`) |
| `test/host/` | Unit tests that run on the PC (motion profile; SCS protocol, bus, calibration and servo driver) |

## Serial protocol (text, one command per line)

The same protocol for both hands; only the number of joints *N* changes (v0: 8, v1: 20).
Joints are numbered **1–N = motors M1–MN** (v1: motor number = servo ID). Angles are in **radians**, and **positive = closing the hand**.

- v0 joints: `index_dip, index_pip, index_mcp_flex, index_mcp_abd, thumb_ip, thumb_mcp, thumb_cmc_flex, thumb_cmc_rot`
- v1 joints: `index_dip, index_pip, index_mcp_flex, index_mcp_abd, thumb_ip, thumb_mcp_flex, thumb_cmc_flex, thumb_cmc_rot, middle_dip, middle_pip, middle_mcp_flex, middle_mcp_abd, ring_dip, ring_pip, ring_mcp_flex, ring_mcp_abd, little_dip, little_pip, little_mcp_flex, little_mcp_abd`

| Command | Meaning | Reply |
|---|---|---|
| `P q1 … qN` | set all N targets | `OK` (v1: `OK offline <mask>` if some servos don't answer; those don't move) |
| `J i q` | set target of joint *i* | `OK` (v1: `ERR joint offline`) |
| `S` | status | `S q1 … qN moving_mask`. v0: counted steps. **v1: measured positions** (real → sim) |
| `F` | feedback, per joint `q,load,temp,volt` | `F 0.1234,12.5,31,5.0 …`; `nan,0,0,0` if the joint has no data (always on v0). *load* is % of max torque, positive = pulling toward closing; *temp* °C; *volt* V |
| `X` | stop all, smoothly | `OK` |
| `R` | release all: v0 coils off, **v1 torque off** (joints go limp) | `OK` |
| `Z` | current pose = zero | `OK` |
| `M i n` | raw move joint *i* by *n* native units, **ignores joint limits** (calibration). v0 steps (max ±4096), v1 servo ticks (max ±300, still kept inside 0–1023) | `OK` |
| `K i s` | set joint *i* scale in native units per joint radian (v0 steps/rad, v1 ticks/rad) | `OK` |
| `V i v a` | max velocity (rad/s) and acceleration (rad/s²) of joint *i* (`0` = all) | `OK` |
| `B` | **v1 only:** scan the bus (IDs 0–253, ~0.4 s) | `B 1 2 3 …` (IDs that answered) |
| `I` | firmware info + scales | v0: `I tendra-hand fw 0.1.0 joints=8 name:scale …`; v1: `I tendra-hand fw 0.3.0 hand=v1-servo joints=20 name:ticks_per_rad:zero_ticks:online …` |

Targets are clamped to the joint limits. v0: coils switch off after 1 s without motion. v1: servos keep holding (torque on) until `R`.

## v1: SCS0009 servos

### Wiring (FE-URT-1 as signal converter)

| ESP32-S3 | FE-URT-1 | Notes |
|---|---|---|
| GPIO **17** (UART1 TX) | TX | |
| GPIO **18** (UART1 RX) | RX | |
| GND | GND | common ground with the servo supply |

GPIO 17/18 are ordinary pins: not strapping pins (0, 3, 45, 46), not native USB (19/20), not UART0 (43/44), not flash/PSRAM (26–37), not the RGB LED (48). Change them in `config_v1.h`.
Some FE-URT-1 boards have TX/RX **silk-screened the wrong way round**: if `B` finds nothing, swap the two wires.
The FE-URT-1 switches the half-duplex bus direction by itself. If it echoes our own bytes back to RX, `ScsBus` skips the echo automatically.
Bus: **1,000,000 baud**, 8N1 (the SCS0009 default). Every servo needs a unique ID = its motor number (1–20); set IDs one servo at a time with Feetech's FD software over the FE-URT-1's USB port.
Power the servos from their own supply (SCS0009: 4–7.4 V). 20 servos under load draw several amps: size the supply and wiring for it, and keep the ESP32 on USB.

### How it moves

- Each servo has an **absolute position sensor** (0–1023 ticks over ~300°, ≈195.6 ticks/rad, centre 512). No homing is needed; the firmware always knows where each joint is.
- The firmware ramps each goal with the same trapezoidal profile as v0 (one tick at a time, ≤ 400 ticks/s) and streams the goals of all moving servos every **20 ms** in one **SYNC WRITE** packet. So even a big target jump becomes a smooth, speed-limited move.
- As a second limit, every goal carries the servo's **goal speed** register (`kServoGoalSpeed`, never 0, because 0 means "full speed").
- Feedback (position, load, voltage, temperature) is read from one servo every 2 ms, round robin (all 20 in ~40 ms). A servo that misses 3 replies counts as **offline** and is only retried once per second.

### Calibration: joint angle ↔ servo ticks

`ticks = zero_ticks + (invert ? -1 : 1) · ticks_per_rad · q`, with `ticks_per_rad = servo_per_joint · 195.57`.

- `servo_per_joint` = servo radians per joint radian = *r*<sub>joint drum</sub> / *r*<sub>spool</sub> (same tendon length both sides). Default **1.0** until measured; spool radius default 6 mm (`spool_radius_m`), equal to the 6 mm joint drums, so 1:1.
- `zero_ticks` = servo reading with the joint straight. Default 512 (servo centred when the tendon is tied).
- `invert` if more ticks opens the joint.

### v1 safety rules

- **Never moves on boot.** `setup()` broadcasts *torque off* to every servo, then switches each one off again and reads its position. Nothing moves until a `P`/`J`/`M` command.
- On the first command, a joint's goal is first set to **where it is now**, then torque comes on, then it ramps to the target. No jump.
- A servo that has **never answered cannot be switched on** (its position is unknown): `J`/`M` reply `ERR joint offline`.
- `R` = torque off for all (broadcast + each servo). `X` = smooth stop, holding position.
- The firmware only writes the SRAM registers (torque, goal position/time/speed). It never writes EPROM (ID, baud, angle limits).

### v1 first power-on checklist

1. Flash `hand_v1_servo` with **only USB connected**. Send `I`: 20 joints, all `:0` (offline).
2. Power the servos. Send `B`: it should list the IDs you connected. Send `I` again (online joints end in `:1`) and `F` to see positions, voltage and temperature.
3. Straighten the joints by hand (they are limp), then send `Z`, then `I`; copy the `zero_ticks` values into `config_v1.h`.
4. One joint at a time: `M i 30` (≈9° at the servo), then `M i -30`. Check that the right joint moves and **positive closes** it; otherwise set `invert = true` in `config_v1.h`.
5. Calibrate scale: `M i 150`, measure the joint angle, then `K i <150 / angle_in_rad>`. Write it into `config_v1.h` as `servo_per_joint = value / 195.57`.
6. Only then use `J i q` / `P ...`. Keep `R` ready (and a power switch).

## Safety rules (v0 steppers)

- At boot, all motor pins go LOW and **nothing moves until a command arrives**.
- **PSRAM must stay disabled**: GPIO 35–37 drive motors 6 and 7 (see `platformio.ini`).
- Steppers are open loop: the firmware *counts* steps. Straighten all joints by hand before power-up (or send `Z` after straightening).
- Stay near a power switch during first tests. A tendon that is too tight or a wrong direction can pull hard.

## First power-on checklist (v0, one motor at a time)

1. Flash with **only USB connected** (motor power off). Open the monitor and send `I`, which should list 8 joints.
2. Connect the motor power. Straighten all joints by hand, then send `Z`.
3. For each motor: send `M i 200` (a small move, ~18°), then `M i -200`.
   - Check that the right joint moves and that **positive closes** the joint. If it opens instead, set `invert = true` for that joint in `config.h`.
4. Calibrate scale: send `M i 1000`, measure the joint angle (in degrees), then `K i <1000 / angle_in_rad>`. Write the value down (it'll go into `config.h`).
5. Only then use `J i q` / `P ...` with real angles.

## Host tests

Run from `firmware/` (a throwaway C++ compiler from the `ziglang` Python package, no install needed):

```bash
uvx --from ziglang python -m ziglang c++ -std=c++17 -O2 -w -Isrc test/host/test_motion_profile.cpp -o .pio/test_motion_profile.exe && .pio/test_motion_profile.exe
uvx --from ziglang python -m ziglang c++ -std=c++17 -O2 -w -Isrc -Iinclude test/host/test_scs.cpp src/scs_bus.cpp src/scs0009_servo.cpp -o .pio/test_scs.exe && .pio/test_scs.exe
```

`test_scs.cpp` checks packet bytes and checksums against hand-computed values, the reply parser (noise, bad checksums), `ScsBus` against a fake servo bus with and without echo, angle ↔ tick conversion, and the servo driver's boot safety, ramp speed and limit clamping.

## Sources and licenses

The SCS code here is written from scratch (Apache-2.0, like the rest of the firmware). The protocol and register map were taken from Feetech's documentation and read (not copied) from the official [FTServo_Arduino](https://github.com/ftservo/FTServo_Arduino) library (`SCS.cpp`, `SCSCL.h/.cpp`). No third-party code is vendored.
