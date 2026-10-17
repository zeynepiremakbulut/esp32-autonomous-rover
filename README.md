# ESP32-S3-Based Autonomous Ground Robot

This project covers computer-based control of an ESP32-S3-powered four-wheel ground robot, live LiDAR visualization, 2D mapping with ROS 2, and experiments with navigating toward a goal selected on the map. The embedded firmware, Python control interface, and ROS 2 components work together.

## Project Features

- Manual driving through a desktop interface
- Communication with the ESP32-S3 over Wi-Fi and UDP
- Receiving and visualizing data from a Vorwerk/Kobold VR100 LiDAR
- Publishing LiDAR data as ROS 2 `LaserScan` messages
- 2D mapping with Cartographer ROS and use of TF transforms
- Selecting a goal on the map
- Grid-based path planning using a custom Python A* implementation
- Path following, obstacle checks, and replanning when needed
- Safety checks intended to stop the motors if communication is lost

## Repository Structure

| Folder / section | Contents and purpose |
|---|---|
| **`ubuntu/`** | The Python interface and robot-control logic running on an Ubuntu computer. This section covers manual driving, LiDAR visualization, ROS 2 connectivity, selecting a goal on the map, and A*-based path planning. |
| **`esp32/`** | Embedded software running on the ESP32-S3, along with instructions for using the firmware. It covers Wi-Fi connectivity, UDP commands, motor-driver control, and forwarding LiDAR data. |
| **`docs/`** | Technical documentation covering system architecture, installation and first-run steps, issues encountered during development, known limitations, and the test plan/results. |
| **`tests/`** | A test plan and checklist for evaluating the hardware and software in a controlled manner. |
| **`media/`** | A section reserved for real photos of the robot, control-interface screenshots, and recordings of manual driving, autonomous movement, LiDAR data, and mapping. Add these materials as they become available. |
| **Configuration and documentation at the repository root** | Information about Python dependencies, Git ignore settings, and license selection. |

## Technologies and Software

| Technology | Purpose |
|---|---|
| ESP32-S3 and Arduino/C++ | Motor control, Wi-Fi connectivity, and LiDAR data transmission |
| TB6612FNG | Driving the DC motors |
| Wi-Fi and UDP | Command and data communication between the computer and ESP32-S3 |
| Vorwerk/Kobold VR100 LiDAR | Scanning distances around the robot |
| Ubuntu 22.04 | ROS 2 development environment |
| ROS 2 Humble | Sensor messages, map data, and TF transforms |
| Cartographer ROS | 2D mapping and pose tracking |
| Python 3 | Control interface and autonomous movement logic |
| Tkinter / ttk | Desktop control interface |
| NumPy | Numerical operations and working with map/perception data |
| Pillow (optional) | Helps render map drawings more smoothly |
| A* (custom Python implementation) | Computing a path to a goal on a grid map |
| `nav2_map_server` tools | Saving maps; this alone does not mean the Nav2 navigation stack is being used |

## Hardware and Power Setup

- ESP32-S3 development board
- Two TB6612FNG driver boards for motor control
- Vorwerk/Kobold VR100 LiDAR (not an RPLIDAR A1)
- Two LM2596 DC-DC converters, used/tested separately for motor-driver and LiDAR power
- A separate USB power bank to power the ESP32-S3

During development, approximately 6 V for the motor-driver supply and approximately 3.8 V for the LiDAR supply were discussed. These values should not be treated as verified component requirements. Check the voltage specified by the LiDAR manufacturer and measure the actual converter output. Do not connect the motor supply to the ESP32's 3V3 pin. Check the grounding and wiring of the actual circuit before applying power.

## Software Architecture

1. The Python interface on the computer controls the ESP32-S3 by sending UDP commands over Wi-Fi.
2. The ESP32-S3 controls the robot's movement through the motor drivers.
3. LiDAR data received over a serial connection is forwarded from the ESP32-S3 to the computer.
4. The Python application publishes the LiDAR data as ROS 2 `sensor_msgs/LaserScan` messages.
5. Cartographer ROS builds a 2D map from LiDAR data. The map is read as an `OccupancyGrid`, and the robot pose is tracked through TF.
6. When a goal is selected on the map, the custom Python A* implementation computes a path on the grid. Path following and obstacle checking during movement are handled by separate control logic.

The mapping configuration is LiDAR-focused: the current Cartographer configuration does not use wheel odometry or IMU input. As a result, pose and map stability may depend on sensor data and environmental conditions.

## Key Settings

- Default ESP32-S3 IP address: `192.168.1.3`
- Motor-control UDP port: `4210`
- LiDAR-data UDP port: `4211`
- LiDAR serial communication: `115200` baud
- Map resolution: approximately `0.03 m/cell`
- Goal-reached tolerance: approximately `0.15 m`

These values reflect the current defaults and configuration in the source code. Verify them for your own network and physical setup.

## Installation and Quick Start

1. Set up ROS 2 Humble on Ubuntu 22.04 and source the ROS environment in the terminal.
2. Install Cartographer ROS and the ROS packages required for saving maps.
3. Install the Python dependencies. NumPy is used; Pillow is optional for map rendering.
4. Check the ESP32-S3 IP address, UDP ports, and that the computer and robot are on the same network.
5. Verify the LiDAR data stream first; test the motors afterward at low speed.
6. Launch the Python interface in the Ubuntu environment. The ROS 2 environment must be sourced correctly for mapping features to work.

Detailed installation and safe first-run instructions are provided in the `docs/` section.

## Photos and Videos

The `media/` section is reserved for real recordings to be added as the project progresses. Potential additions include:

- Top-down and general/side-view photos of the robot
- A screenshot of the control interface
- A manual-driving video
- An autonomous-movement video
- A recording showing LiDAR data changing while the robot moves
- A mapping video

## Safety Notes

- Raise the wheels off the ground and use low speed for initial motor tests.
- Test the stop command and the behavior when communication is lost before operating the motors.
- Keep a way to physically disconnect motor power.
- Do not add Wi-Fi passwords or network details you do not want to share to the repository.
- Since LiDAR-focused mapping does not use odometry/IMU input, pose drift may occur. Computing a path to the goal does not, by itself, correct pose estimation.
