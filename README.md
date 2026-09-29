# Tendra Hand

**An open-source, tendon-driven, 3D-printed robotic hand, built to match the dexterity of a human hand.**

**The ultimate goal:** a robot that can do what humans do (cook, do chores, use tools) just as well as a person. Not only picking things up, but understanding a task and carrying it out. The hand is the focus, because almost everything useful people do goes through their hands, and it is the hardest part to get right.

Tendra Hand starts there: a hardware + software project to build a humanoid robotic hand with the same degrees of freedom as a human hand, then the senses and AI to use it for real tasks, and to share *everything* openly: CAD files, printable parts, firmware, control software, simulation, AI models and research notes.

> 🚧 **Status: early prototype.** Phase 1: the V0 thumb and index finger are built (8 independently driven joints); focus is smooth, clean motion and a MuJoCo digital twin. Phase 2: **Tendra Hand V1**, the first full hand (5 fingers, 21 joints, smart servos), is designed and simulated, not built yet. See the [roadmap](docs/roadmap.md).

## How it works

```
┌──────────────────────────┐   USB-C (serial)   ┌───────────────────────┐        ┌──────────────┐
│ PC (Python)              │ ─────────────────▶ │ ESP32-S3              │ ─────▶ │ Motors       │
│ • high-level control     │    joint targets   │ • low-level motor     │        │ → tendons    │
│ • AI / learning          │ ◀───────────────── │   control (firmware)  │        │ → joints     │
│ • MuJoCo digital twin    │    joint states    │ • hardware abstraction│        └──────────────┘
└──────────────────────────┘                    └───────────────────────┘
```

- **Mechanics:** PLA/PETG printed skeleton (TPU fingertip pads planned). Each joint is pulled both ways by a tendon loop on its own motor.
- **Actuators:** 28BYJ-48 steppers + ULN2003 drivers today → Feetech **SCS0009** smart servos (serial bus, position feedback) next.
- **Controller:** ESP32-S3, firmware built with PlatformIO.
- **Simulation:** [MuJoCo](https://mujoco.org/), with a digital twin that mirrors and can drive the real hand.

### Current prototype: 8 DOF

| Finger | Joints |
|---|---|
| Index | MCP side-to-side, MCP bend, PIP, DIP |
| Thumb | base rotation, CMC bend, MCP, IP |

## Repository layout

| Folder | Contents |
|---|---|
| [`hardware/`](hardware/) | CAD (Fusion 360 + STEP), print files, electronics, robot description (URDF) |
| [`firmware/`](firmware/) | ESP32-S3 firmware (PlatformIO) |
| [`sim/`](sim/) | MuJoCo models, model converter, digital twin |
| [`software/`](software/) | PC-side Python: hand API, control, calibration, AI |
| [`docs/`](docs/) | Roadmap and guides |
| [`research/`](research/) | Research log, experiments, references |
| [`website/`](website/) | Project website source |
| [`media/`](media/) | Photos, videos, screenshots and diagrams of the project, with a catalog |

## Getting started

Build and setup guides will be added as each part matures. For now:
- Robot model: [`hardware/robot_description/`](hardware/robot_description/)
- Plan: [`docs/roadmap.md`](docs/roadmap.md)
- Progress and findings: [`research/log.md`](research/log.md)

## Contributing

The project is in an early stage, but ideas, issues and questions are welcome. Open an issue on GitHub.

## License

Tendra Hand is open source, with a license suited to each kind of work:

| What | License | File |
|---|---|---|
| Software: firmware, Python, simulation, AI code and models | [Apache-2.0](https://www.apache.org/licenses/LICENSE-2.0) | [`LICENSE`](LICENSE) |
| Hardware: CAD, print files, electronics, robot description (`hardware/`) | [CERN-OHL-S-2.0](https://ohwr.org/cern_ohl_s_v1.txt) | [`hardware/LICENSE`](hardware/LICENSE) |
| Documentation and research (`docs/`, `research/`, website content) | [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) | [`docs/LICENSE`](docs/LICENSE) |

In short: you may use, build, modify and sell Tendra Hand. If you distribute a modified *hardware* design, you must share its design files under the same license. Please credit the project.
