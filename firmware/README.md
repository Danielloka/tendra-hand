# Firmware

Low-level motor control running on the **ESP32-S3-N16R8**, built with **PlatformIO** (Arduino framework, C++).

The PC sends joint targets over **USB (native USB CDC)**. The ESP32 turns them into smooth motor motion.

Design principle: a **hardware abstraction layer (HAL)**. Joint logic talks to a `MotorDriver` interface:
- `Uln2003StepperDriver`: current 28BYJ-48 steppers
- `Scs0009ServoDriver`: planned smart servos

Swapping motor hardware means swapping the driver, not rewriting the firmware.

⚠️ Keep **PSRAM disabled**: GPIO 35–37 are used for motors (see `CLAUDE.md`).
