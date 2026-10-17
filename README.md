# ESP32 Autonomous Rover — Manual Control, LiDAR, Mapping & Goal Navigation

A small differential-drive rover controlled from an Ubuntu desktop application. The project combines an ESP32-based motor-control layer with a Python/Tkinter interface, UDP communication, live LiDAR visualization, ROS 2 mapping, and grid-based path planning.

> **Project status:** The Ubuntu-side Python application is included. An ESP32 firmware template and credential example are included, but the template is reconstructed from the described protocol/pins and is **not guaranteed to match the original sketch line-for-line**. Review and test it against your original firmware before flashing. End-to-end hardware behavior is not yet independently verified.

## Features implemented in the Ubuntu application

- **Manual driving:** keyboard (`W/A/S/D` or arrow keys) and on-screen controls.
- **Autonomous reactive driving:** LiDAR-sector-based obstacle avoidance and a stuck/recovery state tracker.
- **Live LiDAR display:** receives packets over UDP, checks packet checksums, and assembles a 360-degree scan.
- **ROS 2 bridge:** publishes the scan on `/scan`, subscribes to `/map`, and reads the `map → base_link` transform.
- **2D mapping:** starts Cartographer and its occupancy-grid node from the interface.
- **Map-based goal navigation:** selects a goal on the map, computes a path using custom A* grid search, and drives along the planned path.
- **Map export:** invokes Nav2's `map_saver_cli` to save a map.
- **Connection safety behavior:** displays LiDAR link status and can stop motion when the stream is lost.

These are features visible in the Python source; they are not a claim that every feature has passed repeatable hardware tests.

## Architecture

```text
Vorwerk/Kobold-style LiDAR
        │ scan packets
        ▼
ESP32 ── UDP (LiDAR, port 4211) ──► Ubuntu Python/Tkinter application
  ▲                                      │
  │ UDP motor commands (port 4210)       ├── Live LiDAR display
  └──────────────────────────────────────┤
                                         ├── ROS 2 `/scan`
                                         │      └── Cartographer → `/map` + TF
                                         └── A* path planner → motor commands
```

The application currently binds the LiDAR listener to `0.0.0.0:4211` and sends motor commands to the ESP32 address configured in the Python file (`ESP32_IP`, default currently `192.168.1.3`) on UDP port `4210`. Configure this address for your own network before running it. The included firmware template documents the expected ports and commands, but compare it against your original sketch before relying on packet-level compatibility.

## Hardware

Add the exact components and wiring used in your final build here. At minimum, document:

- ESP32 board model
- Motor driver and motor count
- LiDAR model and power/data connections
- Battery and voltage regulation
- Wiring diagram, including common ground and driver enable/standby pins

Do not copy a wiring diagram from a different revision of the rover without checking it against the actual firmware and hardware.

## Software requirements

- Ubuntu 22.04 recommended for the ROS 2 Humble setup
- Python 3
- Tkinter (`python3-tk` on Ubuntu)
- NumPy (needed for mapping/path-planning features)
- Pillow is optional and used for smoother map rendering when available
- ROS 2 Humble, `cartographer_ros`, and `nav2_map_server` for mapping and map export

The interface can start without ROS 2, but the map tab will be unavailable. Goal navigation requires a live map and a valid robot pose from TF.

## Setup (Ubuntu)

1. Install system dependencies and set up ROS 2 Humble using the [official ROS 2 installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
2. Install the project-side packages:

   ```bash
   sudo apt update
   sudo apt install python3-tk python3-numpy python3-pil ros-humble-cartographer-ros ros-humble-nav2-map-server
   ```

   If `python3-pil` is unavailable for your setup, Pillow is optional for this application.
3. Open `ubuntu/rover_manuel_otonom_harita.py` and set `ESP32_IP` to the rover's current LAN IP address.
4. For the ESP32, copy `esp32/secrets.example.h` to `esp32/secrets.h` and set your local Wi-Fi credentials. Keep `secrets.h` out of Git. Review `esp32/rover_firmware_template.ino` against your original sketch before flashing. Make sure the LiDAR stream reaches Ubuntu on UDP `4211` and motor commands are accepted on UDP `4210`.
5. In a terminal, source ROS 2 and launch the app:

   ```bash
   source /opt/ros/humble/setup.bash
   python3 ubuntu/rover_manuel_otonom_harita.py
   ```

6. Start with the wheels lifted or the robot restrained. Test STOP, manual movement, and link-loss behavior before enabling autonomous movement.

Detailed setup notes and known limitations: [`docs/setup.md`](docs/setup.md).

## Controls

- `W/A/S/D` or arrow keys: manual movement
- `SPACE`: toggle reactive autonomous mode (when goal navigation/calibration is not active)
- Use the **Live LiDAR** tab to inspect scan reception.
- Use the **Map (Cartographer)** tab to start mapping, save a map, and select a navigation goal.

Exact behavior can depend on the firmware, sensor orientation, ROS 2 configuration, and the physical robot.

## Repository layout

```text
ubuntu/     Ubuntu control application
esp32/      ESP32 firmware template and local secrets example
docs/       Architecture, setup, and test checklist
media/      Real rover photos, UI screenshots, and demonstration clips/GIFs
```

## Validation and limitations

- The Python source passes a syntax compilation check.
- The ESP32 file is explicitly a template; compile it with the target board package and test safely before use.
- No claim of a completed end-to-end hardware test is made in this repository yet.
- The Cartographer configuration is LiDAR-only: the source sets `use_odometry = false` and does not use an IMU. Map/pose drift may occur, especially over longer runs or in repetitive/feature-poor spaces.
- Reliable mapping/navigation depends on correct LiDAR orientation, TF frame configuration, and the ESP32's motor/packet implementation.
- UDP is used on the local network; this is not an authenticated or encrypted control protocol. Use a trusted local network.

See [`docs/test-results.md`](docs/test-results.md) before treating any performance as verified.

## Demo media

Add only real captures from your own robot. Recommended clips:

1. Manual control from the interface.
2. Live LiDAR scan while rotating/moving the robot.
3. Mapping a small room.
4. Selecting a goal on the map and showing the robot's attempt to reach it.
5. A short close-up of the hardware and wiring.

Place optimized GIFs/screenshots in `media/` and link them here when ready. Avoid publishing Wi-Fi passwords, personal network details, or any third-party/team code. If a real Wi-Fi password was shared in chat or elsewhere, rotate it before publishing.

## License

No license has been selected in this starter repository. Choose a license before presenting the code as reusable open source; otherwise, default copyright restrictions apply.
