# Tomas_bot — Daniel's Robot

**Branch: `feature/bno085-imu-ekf-integration`**

Differential drive mobile robot running ROS2 Jazzy on LattePanda Alpha (Core i5) with built-in Arduino Leonardo. Features autonomous navigation using Nav2, SLAM mapping, RPLidar C1 laser scanning, and **BNO085 9-DOF IMU with Extended Kalman Filter (EKF) sensor fusion** for improved localization.

## Hardware

- **Computer:** LattePanda Alpha (Core i5) running Ubuntu 24.04 + ROS2 Jazzy
- **Microcontroller:** Built-in Arduino Leonardo (ATmega32U4)
- **Motors:** 2× 130 RPM 12V DC motors with quadrature encoders (11 PPR)
- **Motor Driver:** L298N H-Bridge
- **LiDAR:** RPLidar C1 (USB)
- **IMU:** Adafruit BNO085 9-DOF (I²C) — Game Rotation Vector + Accelerometer + Gyroscope
- **Drive:** Differential drive — 2 rear powered wheels + 1 front caster

## Robot Specs

| Parameter | Value |
|---|---|
| Chassis | 437 × 212 × 89 mm |
| Weight | 2.4 kg |
| Wheel Diameter | 69 mm |
| Wheel Width | 26 mm |
| Wheel Separation (center-to-center) | 181 mm |
| Max Linear Velocity | ~0.39 m/s (calibrated) |
| LiDAR Range | 0.15 – 8 m |
| IMU Update Rate | 20 Hz |
| EKF Output Rate | 30 Hz |

## What's New in This Branch

- **BNO085 IMU Integration** — Arduino reads BNO085 via I²C and sends quaternion + accel + gyro data over serial
- **Extended Kalman Filter** — `robot_localization` fuses wheel odometry + IMU for dramatically better localization
- **IMU in URDF** — `imu_link` visible in RViz as a blue rectangle on the chassis
- **Calibration Script** — Interactive 10-step BNO085 calibration guide
- **Diagnostic Script** — Real-time IMU health check with live dashboard
- **Encoder Pin Move** — Left encoder moved D3/D2 → D1/D0 to free I²C bus for IMU

## Project Structure

```
Tomas_bot/
├── config/
│   ├── 99-robot-devices.rules           # udev rules for RPLidar + Arduino
│   ├── ekf.yaml                         # EKF sensor fusion configuration (NEW)
│   ├── joystick_params.yaml             # PS3 gamepad configuration
│   ├── mapper_params_online_async_hardware.yaml  # SLAM Toolbox config
│   ├── my_controllers.yaml              # ros2_control reference config
│   ├── nav2_params_hardware.yaml        # Nav2 navigation config
│   ├── nav2_view.rviz                   # RViz config for navigation
│   └── slam_view.rviz                   # RViz config for SLAM mapping
├── description/
│   ├── robot_hardware.urdf.xacro        # Top-level URDF (includes IMU)
│   ├── robot_core.xacro                 # Chassis, wheels, caster
│   ├── lidar_hardware.xacro             # RPLidar C1 link/joint
│   ├── imu_hardware.xacro              # BNO085 IMU link/joint (NEW)
│   └── inertial_macros.xacro            # Inertia calculation macros
├── firmware/
│   └── motor_controller/
│       └── motor_controller.ino         # Arduino: motors + encoders + BNO085 IMU
├── launch/
│   ├── bringup_hardware.launch.py       # Full robot bringup (includes EKF)
│   ├── rsp_hardware.launch.py           # Robot state publisher
│   ├── rplidar.launch.py                # Standalone RPLidar driver
│   ├── joystick_teleop.launch.py        # PS3 gamepad teleop
│   ├── slam_hardware.launch.py          # SLAM Toolbox
│   └── navigation_hardware.launch.py    # Nav2 autonomous navigation
├── scripts/
│   ├── diff_drive_node.py               # ROS2 ↔ Arduino bridge (odom + IMU)
│   ├── joystick_teleop_node.py          # PS3 gamepad teleop node
│   ├── imu_calibration_node.py          # BNO085 calibration script (NEW)
│   └── imu_check_node.py               # IMU health check/diagnostics (NEW)
├── IMU_GUIDE.md                         # IMU setup, calibration, debugging guide (NEW)
├── INSTALLATION_GUIDE.md                # Full setup instructions
├── RUNNING_GUIDE.md                     # Operating instructions
├── CMakeLists.txt
└── package.xml
```

## Quick Start

```bash
# 1. Build
cd ~/robot_ws
colcon build --symlink-install
source install/setup.bash

# 2. Install BNO085 Arduino library (Adafruit BNO08x) via Arduino IDE Library Manager
# 3. Upload firmware to Arduino Leonardo (firmware/motor_controller/motor_controller.ino)

# 4. Calibrate IMU (one-time, after hardware setup)
ros2 run Tomas_bot imu_calibration_node.py

# 5. Verify IMU is working
ros2 run Tomas_bot imu_check_node.py

# 6. Launch robot (Terminal 1)
ros2 launch Tomas_bot bringup_hardware.launch.py

# 7. Teleop (Terminal 2)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 8. SLAM mapping (Terminal 3)
ros2 launch Tomas_bot slam_hardware.launch.py

# 9. Save map
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map

# 10. Navigate autonomously (Terminal 3 — after stopping SLAM)
ros2 launch Tomas_bot navigation_hardware.launch.py map:=$HOME/maps/my_map.yaml
```

## Verify System

```bash
# Check all nodes
ros2 node list
# Expected: /robot_state_publisher, /sllidar_node, /diff_drive_node, /ekf_filter_node

# Check key topics and data rates
ros2 topic hz /wheel/odom     # ~20 Hz (encoder odometry)
ros2 topic hz /imu/data        # ~20 Hz (BNO085 IMU)
ros2 topic hz /odom            # ~30 Hz (EKF fused output)
ros2 topic hz /scan            # ~10 Hz (RPLidar C1)

# Check TF tree
ros2 run tf2_tools view_frames
# Expected: map → odom → base_link → chassis → imu_link / laser_frame / wheels
```

## Calibration Results

Measured accuracy with PID motor control and EKF sensor fusion:

| Test | Command | Expected | Actual | Error |
|------|---------|----------|--------|-------|
| Forward | 0.15 m/s × 3s | 0.45 m | 0.47 m | +4.4% |
| Backward | 0.10 m/s × 3s | 0.30 m | 0.30 m | -0.2% |
| Rotate (0.8 rad/s) | 0.8 rad/s × 2s | 91.7° | 80° | -12° (88%) |
| Rotate (0.5 rad/s) | 0.5 rad/s × 2s | 57.3° | 38° | -19° (66%) |

> **Note:** Rotation accuracy is limited by motor stiction at low PWM. Nav2 uses ≥0.8 rad/s for in-place turns where accuracy is ~88%. The IMU+EKF fusion compensates for rotation errors during navigation.

## Documentation

- **[IMU_GUIDE.md](IMU_GUIDE.md)** — BNO085 setup, calibration, verification, and debugging
- **[INSTALLATION_GUIDE.md](INSTALLATION_GUIDE.md)** — Full Ubuntu + ROS2 + hardware setup
- **[RUNNING_GUIDE.md](RUNNING_GUIDE.md)** — Step-by-step operating instructions

## Wiring

### L298N → Arduino Leonardo

| L298N | Arduino | Function |
|-------|---------|----------|
| ENA   | D5      | Left PWM |
| IN1   | D7      | Left dir |
| IN2   | D6      | Left dir |
| IN3   | D10     | Right dir |
| IN4   | D9      | Right dir |
| ENB   | D11     | Right PWM |

### Encoders → Arduino Leonardo

| Motor | Wire   | Pin | Function | Notes |
|-------|--------|-----|----------|-------|
| Left  | Green  | **D1** | Ch A (INT3 interrupt) | **MOVED from D3** for IMU I²C |
| Left  | Yellow | **D0** | Ch B (direction) | **MOVED from D2** for IMU I²C |
| Right | Yellow | A4  | Ch A (polled) | Unchanged |
| Right | Green  | A5  | Ch B (direction) | Unchanged |

### BNO085 IMU → Arduino Leonardo (I²C)

| BNO085 | Arduino | Function |
|--------|---------|----------|
| VIN    | 5V      | Power |
| GND    | GND     | Ground |
| SDA    | D2      | I²C data |
| SCL    | D3      | I²C clock |
| INT    | D4      | Data-ready (optional) |

## Dependencies

- ROS2 Jazzy
- `navigation2`, `nav2_bringup`
- `slam_toolbox`
- `robot_localization` (EKF sensor fusion)
- `sllidar_ros2` (RPLidar C1 support)
- `robot_state_publisher`, `xacro`
- `pyserial` (Python)
- Arduino IDE + Adafruit BNO08x library (for firmware upload)

## License

MIT License — see [LICENSE.md](LICENSE.md)