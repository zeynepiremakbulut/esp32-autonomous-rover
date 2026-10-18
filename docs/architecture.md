# Architecture

```text
Tkinter UI / autonomous controller (Ubuntu)
  ├─ UDP motor commands ───────────────> ESP32-S3 ──> TB6612FNG drivers ──> drive motors
  ├─ UDP LiDAR receiver <────────────── ESP32-S3 <── UART LiDAR (VR100)
  └─ ROS 2 bridge: /scan <── LiDAR data
       ├─ Cartographer: /map + TF
       └─ map goal -> grid planning (A*) -> local obstacle-aware control -> UDP motor commands
```

## Hardware power layout

The build used two LM2596 converter modules for separate loads: one for the TB6612FNG motor supply (about 6 V was the discussed target), and another for the VR100 LiDAR supply (about 3.8 V was discussed during bring-up). The ESP32-S3 was powered separately from a Baseus 10,000 mAh / 30 W USB power bank. The LiDAR voltage is not recorded here as a verified specification: measure the rail and confirm the sensor's required input voltage before powering it. Keep the rails separated as designed and verify grounding/wiring on the actual hardware.

## Important integration boundaries

- **Transport:** Python and firmware must agree on IP addresses, ports, command strings and packet framing.
- **Sensing:** the raw LiDAR packet parser must preserve packet boundaries/checksums and produce valid ranges/angles.
- **Coordinate frames:** the radar display's angular convention, physical sensor orientation and ROS scan convention must agree.
- **Mapping:** current configuration is LiDAR-only; it does not use wheel odometry or an IMU.
- **Navigation:** map planning and live obstacle handling are separate from reliable localization. A* can plan a path, but it cannot compensate for a drifting or jumping map pose by itself.
- **Safety:** the PC-side link-loss logic and ESP32-side 500 ms watchdog are separate protections and should be tested independently.
