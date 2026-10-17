# ESP32-S3 Rover — Manual Control, LiDAR and ROS 2 Mapping

A personal robotics project combining an ESP32-S3 mobile platform, UDP motor commands, a Vorwerk/Kobold VR100 LiDAR stream, a Python/Tkinter control interface and optional ROS 2 Cartographer mapping. This repository documents both the implementation and the debugging lessons learned while integrating hardware and software.

> **Status:** Development project. Features are implemented in source, but behavior depends on the physical robot, network, LiDAR orientation and ROS 2 environment. Do not interpret the presence of a feature as a claim that every scenario is fully validated.

## Features in the current Python source

- Tkinter control interface for manual and autonomous operation
- UDP communication with ESP32 (`192.168.1.3` by default; motor port `4210`, LiDAR port `4211`)
- Live LiDAR visualization and conversion to ROS 2 `sensor_msgs/LaserScan`
- Optional Cartographer 2D mapping; map and TF integration
- Grid-based A* path planning for map goals
- Obstacle-aware goal navigation, steering correction, replanning and stuck-recovery logic
- Link-loss handling intended to stop the rover when configured conditions are met

## Repository layout

```text
esp32-autonomous-rover/
├── ubuntu/rover_manuel_otonom_harita.py
├── esp32/rover_firmware.ino
├── esp32/README.md
├── docs/
│   ├── architecture.md
│   ├── setup.md
│   ├── development-log.md
│   ├── known-limitations.md
│   └── test-results.md
├── tests/test_plan.md
├── media/README.md
├── requirements.txt
├── .gitignore
└── LICENSE-CHOICE.md
```

## Hardware (as currently described)

- ESP32-S3 board
- Two TB6612FNG motor-driver boards controlling four drive channels/motors
- Vorwerk Kobold VR100 LiDAR (not an RPLIDAR A1)
- Separate motor and logic power arrangements; verify all grounds and voltage limits before testing

The included firmware copy has only had its Wi-Fi SSID/password replaced with placeholders; no other source-code edits were intentionally made. Enter credentials locally and do not commit them. See [`esp32/README.md`](esp32/README.md).

## Setup (Ubuntu + ROS 2 Humble)

1. Install ROS 2 Humble and source its environment in the same terminal.
2. Install required ROS packages, including Cartographer ROS and map server where needed.
3. Install Python dependencies listed in `requirements.txt` (NumPy is used by the ROS integration; Pillow is optional for smoother map rendering).
4. Confirm the ESP32 IP, host network and UDP ports match the firmware. The Python source currently defaults to `192.168.1.3`.
5. Run `python3 ubuntu/rover_manuel_otonom_harita.py`.

The script attempts to keep non-ROS UI functionality available when ROS imports fail; mapping features require a correctly sourced ROS 2 environment and installed packages. Exact launch behavior must be verified on the target Ubuntu system.

## Safety

- Lift the wheels off the ground for initial motor-direction tests.
- Test the stop command and communication-loss behavior before autonomous runs.
- Keep a physical way to disconnect motor power.
- Never commit Wi-Fi passwords or private network details that you do not want public.
- LiDAR-only mapping has no wheel odometry or IMU input in the current Cartographer configuration; drift and pose jumps are known limitations.

## Development notes

The [`docs/development-log.md`](docs/development-log.md) records concrete engineering challenges and the diagnostic lessons behind them. It distinguishes observed symptoms from confirmed root causes rather than claiming every issue has a proven fix.

## Demo media

Add your own rover photo, interface screenshot and short clips/GIFs to `media/`. Do not add mock screenshots or claim unrecorded test results.
