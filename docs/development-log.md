# Development log: problems, debugging and lessons

This is a project-development record, not a claim that every root cause has been conclusively isolated. Symptoms explicitly discussed during development are marked **Observed/reported**; possible explanations are marked **Hypothesis to verify**. Add dates and evidence as new tests are run.

## 1. Map pose jumps or drifts after reaching a goal

**Observed/reported:** during goal navigation, the robot could reach the selected point and stop, then its displayed position would start to shift. At times, the map appeared to open up or show a newly entered room even though the robot was not intentionally moving; the issue sometimes disappeared and returned.

**Why it matters:** navigation uses the map pose as feedback. If localization shifts, the goal controller may believe the robot is somewhere else, even if the physical rover has not moved accordingly.

**Relevant implementation detail:** the Cartographer configuration is LiDAR-only (`use_odometry = false`), with no wheel odometry or IMU input. The source itself warns that drift may occur.

**Debugging approach to document/test:**
- Record `/scan`, `/map`, TF (`map -> odom -> base_link`) and `/tf` timestamps around the event.
- Compare the physical rover position with the displayed pose after the robot stops.
- Check whether stale scans, changing scan orientation, transform timing, or scan matching coincide with the jump.
- Repeat in a small, static room and save a short screen recording.

**Status:** limitation identified; exact root cause and a complete fix are not claimed.

## 2. Map orientation and LiDAR angle convention

**Observed in source comments:** the UI and ROS conversion use an angular convention that may need flipping if the map appears mirrored. The physical sensor's forward direction must be calibrated rather than assumed.

**Debugging lesson:** compare a known wall/obstacle in front of the physical rover with the radar plot and RViz scan. Change one angle/sign parameter at a time and record the result.

**Status:** configurable in source; final physical orientation still needs a documented validation run.

## 3. Low PWM / inconsistent motor response — code-level test item

**Status distinction:** this is included as a diagnostic item because the controller contains compensation logic; it is not presented here as a confirmed user-reported incident.

**Implementation evidence:** navigation includes a minimum PWM, a timed boost when commands are sent but movement is not detected, steering limits and stuck-recovery logic.

**Engineering reason:** small DC motors and gearboxes often have a dead zone where a PWM value is insufficient to overcome static friction. Left/right motors may not respond identically.

**Test to record:** measure the lowest reliable starting PWM for each side on the actual battery voltage and floor. Do not assume a boost proves movement; verify physically.

**Status:** compensation logic exists; calibrated motor thresholds and repeatable test results should be added.

## 4. Obstacle avoidance, replanning and getting stuck

**Implementation evidence:** the controller uses front/side LiDAR sectors, steering correction, lookahead, replanning limits and recovery states.

**Potential failure modes to test:** narrow passages, a goal behind the rover, an obstacle entering after planning, sensor dropouts, and recovery oscillation.

**Debugging lesson:** log the front/left/right minimum ranges, chosen steering, PWM values, replan count and recovery state when a run fails. A successful single run is not enough to claim robust autonomy.

## 5. Power distribution and LiDAR bring-up

**Setup recalled from the build:** the rover used two LM2596 DC-DC converter modules, not just a generic “separate power arrangement.” One was intended to supply the TB6612FNG motor rail at about 6 V from the series battery pack; the second was used/planned for a separate LiDAR supply. The LiDAR output around 3.8 V came up during troubleshooting, but that value must not be mistaken for a verified VR100 supply specification. The ESP32-S3 was powered separately from a Baseus 10,000 mAh / 30 W USB power bank.

**Bring-up context:** the LiDAR had been a troubleshooting focus; successful sensor startup and a stable scan should be documented only after observed. Before repeating tests, measure both LM2596 outputs with a multimeter, verify polarity, confirm the LiDAR's required input voltage from reliable hardware documentation, and check the ground/reference wiring. Never change the converter while it is connected to the sensor unless the adjustment procedure is safe for that setup.

**Status:** this documents the hardware arrangement and the values discussed during testing. It does not claim the LiDAR supply voltage or stable LiDAR operation has been conclusively validated.

## 6. LiDAR UDP packet parsing and freshness

**Firmware details shared during development:** the VR100-like stream uses 22-byte packets, `0xFA` packet starts and checksum validation; multiple packets are batched for UDP delivery. Python also tracks scan-point lifetime and warns on stale LiDAR data.

**What to verify:** packet loss, checksum failures, batch boundaries, scan rate, and whether the UI stops using old points after the stream disappears. Test the exact firmware and Python receiver together.

## 7. Link-loss safety and watchdogs

**Implementation details:** firmware motor watchdog is 500 ms; Wi-Fi reconnect logic is present. The Python source also has link-warning and stop-on-link-loss settings.

**Why this matters:** the host may freeze, the Wi-Fi may drop, or packets may stop while the last motor command remains active. Two independent stop paths are useful only if their behavior is verified.

**Test:** with wheels lifted, command a slow movement, then disable the PC sender or Wi-Fi. Measure whether motors stop and how quickly. Repeat for each failure mode and record results.

## 8. Startup and serial-pin risks

**Firmware detail shared during development:** Wi-Fi connection setup waits in a loop until connected, which can block startup indefinitely if credentials/network configuration are wrong. UART RX/TX are configured on GPIO 44/43; confirm these do not interfere with the board's USB serial/debug setup.

**Status:** identified review items; not claimed fixed.

## 9. Lessons learned

- A correct path planner does not guarantee reliable localization.
- Hardware and software should be debugged in layers: power, motors, Wi-Fi, LiDAR bytes, ROS scan, TF/map, then autonomous navigation.
- Tune one parameter at a time and save before/after evidence.
- Distinguish a feature implemented in code from a behavior validated repeatedly on hardware.
