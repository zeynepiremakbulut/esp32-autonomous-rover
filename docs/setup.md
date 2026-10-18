# Setup and bring-up

## Software

- Ubuntu 22.04 recommended for the ROS 2 Humble setup described by this project
- ROS 2 Humble environment sourced in the shell
- Cartographer ROS and `nav2-map-server` packages where used
- Python 3, NumPy, and optionally Pillow

Example package installation (confirm package names for your installed ROS distribution):

```bash
sudo apt update
sudo apt install ros-humble-cartographer-ros ros-humble-nav2-map-server python3-numpy python3-pil
source /opt/ros/humble/setup.bash
python3 ubuntu/rover.py
```

## Hardware power arrangement

The rover setup discussed during development used two LM2596 buck-converter modules for separate loads:

- **Motor drivers:** a 4-cell series battery pack was discussed as the input to an LM2596 adjusted to about 6 V for the TB6612FNG motor supply. Measure the converter output before connecting it.
- **LiDAR:** a second LM2596 was used/planned as a separate supply while trying to bring up the Vorwerk/Kobold VR100 LiDAR; an output around 3.8 V was discussed. This is a recorded troubleshooting value, **not confirmation that it is the correct operating voltage**. Verify the sensor's required supply and measure the actual output before reconnecting it.
- **ESP32-S3:** powered separately through a Baseus 10,000 mAh / 30 W USB power bank. Do not feed the motor-driver rail into the ESP32 3V3 pin.
- **Grounding:** confirm the required common ground between control electronics, motor drivers and signal interfaces, while keeping the power rails separate as intended. Check the exact board wiring before applying power.

These notes document the setup discussed, not a certified wiring diagram. The exact battery, converter outputs, LiDAR power requirement and wiring should be verified on the physical build.

## Network

The Python source defaults to ESP32 address `192.168.1.3`, motor UDP `4210`, LiDAR UDP `4211`, and binds the host receiver to `0.0.0.0`. Ensure the computer and ESP32 can reach each other on the same network and that the firmware uses matching ports. Avoid exposing this rover's control ports to untrusted networks.

## Safe bring-up order

1. Power the ESP32 with motor power disconnected. Verify Wi-Fi connection and IP.
2. Verify LiDAR packets arrive before enabling the motors.
3. Lift the rover so wheels cannot touch the ground; test each motor direction at low PWM.
4. Test `M:x`, the 500 ms command watchdog and Wi-Fi-loss behavior.
5. Check LiDAR orientation and whether the displayed front agrees with the physical front.
6. Only then test mapping, manual movement, and goal navigation in a clear area.

The exact motor power arrangement, voltage levels, pin mapping and serial behavior must be verified against the hardware actually assembled.
