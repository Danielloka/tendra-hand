# Firmware

Low-level motor control running on the **ESP32-S3-N16R8**, built with **PlatformIO** (Arduino framework, C++17).
The PC sends joint targets in radians over **USB-C** (native USB serial). The ESP32 moves the motors there smoothly.

## Build, flash, talk to it

From this folder (`firmware/`), or use the PlatformIO buttons in VS Code:

```bash
pio run                 # build
pio run -t upload       # flash (ESP32 on USB-C)
pio device monitor      # type commands, see replies (Ctrl+C to quit)
```

If the upload can't connect: hold **BOOT**, tap **RESET**, release BOOT, and try again.

## How it's built

| File | Role |
|---|---|
| `include/config.h` | **Pin map**, joint names/limits (same order as the sim), speed/acceleration defaults |
| `src/motor_driver.h` | **HAL interface**: one joint actuator, in radians. The rest of the code only uses this. |
| `src/uln2003_stepper.*` | 28BYJ-48 + ULN2003 driver (half-step), implements `MotorDriver` |
| `src/motion_profile.h` | Smooth motion: accelerate, cruise, brake (trapezoidal profile), handles new targets mid-move |
| `src/main.cpp` | Safe startup, main loop, serial command protocol |
| `test/host/` | Unit tests for the motion profile that run on the PC |

Switching to SCS0009 servos = writing `scs0009_servo.*` that implements `MotorDriver`, then swapping the driver array in `main.cpp`.

## Serial protocol (v0.1, text, one command per line)

Joints are numbered **1–8 = motors M1–M8**: `index_dip, index_pip, index_mcp_flex, index_mcp_abd, thumb_ip, thumb_mcp, thumb_cmc_flex, thumb_cmc_rot`. Angles are in **radians**, and **positive = closing the hand**.

| Command | Meaning | Reply |
|---|---|---|
| `P q1 … q8` | set all 8 targets | `OK` |
| `J i q` | set target of joint *i* | `OK` |
| `S` | status | `S q1 … q8 moving_mask` |
| `X` | stop all, smoothly | `OK` |
| `R` | release all coils (joints go limp) | `OK` |
| `Z` | current pose = zero | `OK` |
| `M i n` | raw move joint *i* by *n* steps, **ignores limits** (calibration; max ±4096) | `OK` |
| `K i s` | set joint *i* scale in steps per radian (calibration) | `OK` |
| `V i v a` | max velocity (rad/s) and acceleration (rad/s²) of joint *i* (`0` = all) | `OK` |
| `I` | firmware info + scales | `I …` |

Targets are clamped to the joint limits. Coils switch off after 1 s without motion.

## Safety rules

- At boot, all motor pins go LOW and **nothing moves until a command arrives**.
- **PSRAM must stay disabled**: GPIO 35–37 drive motors 6 and 7 (see `platformio.ini`).
- Steppers are open loop: the firmware *counts* steps. Straighten all joints by hand before power-up (or send `Z` after straightening).
- Stay near a power switch during first tests. A tendon that is too tight or a wrong direction can pull hard.

## First power-on checklist (one motor at a time)

1. Flash with **only USB connected** (motor power off). Open the monitor and send `I`, which should list 8 joints.
2. Connect the motor power. Straighten all joints by hand, then send `Z`.
3. For each motor: send `M i 200` (a small move, ~18°), then `M i -200`.
   - Check that the right joint moves and that **positive closes** the joint. If it opens instead, set `invert = true` for that joint in `config.h`.
4. Calibrate scale: send `M i 1000`, measure the joint angle (in degrees), then `K i <1000 / angle_in_rad>`. Write the value down (it'll go into `config.h`).
5. Only then use `J i q` / `P ...` with real angles.
