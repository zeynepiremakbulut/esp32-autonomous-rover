# Known limitations and review items

- Power arrangement includes two LM2596 modules (motor-driver rail and separate LiDAR rail); actual output voltages and the LiDAR's required input voltage must be verified before further tests. The previously discussed ~3.8 V LiDAR setting is not a confirmed specification.
- Mapping uses LiDAR without wheel odometry or IMU; drift/pose jumps are possible.
- Physical LiDAR forward orientation and angle convention must be validated.
- Map pose jumps after goal arrival have been reported; root cause is not confirmed.
- Motor PWM thresholds and left/right response need calibration on the actual surface and battery condition.
- Wi-Fi startup may block indefinitely if the firmware waits forever for a connection.
- GPIO 43/44 UART use must be checked against board-specific USB/serial behavior.
- Python and firmware command parsing, UDP batching and checksum handling must be tested together.
- No quantitative success rate, maximum range, navigation accuracy or runtime is claimed until measured.
