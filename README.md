# Tomas_bot — Daniel's Robot

Differential drive mobile robot running ROS2 Jazzy on LattePanda Alpha (Core i5) with built-in Arduino Leonardo. Features autonomous navigation using Nav2, SLAM mapping, and RPLidar C1 laser scanning.

## Hardware

- **Computer:** LattePanda Alpha (Core i5) running Ubuntu 24.04 + ROS2 Jazzy
- **Microcontroller:** Built-in Arduino Leonardo (ATmega32U4)
- **Motors:** 2× 130 RPM 12V DC motors with quadrature encoders (11 PPR)
- **Motor Driver:** L298N H-Bridge
- **LiDAR:** RPLidar C1 (USB)
- **Drive:** Differential drive — 2 rear powered wheels + 1 front caster

## Robot Specs

| Parameter | Value |
|---|---|
| Chassis | 437 × 212 × 89 mm |
| Weight | 2.4 kg |
| Wheel Diameter | 69 mm |
| Wheel Width | 26 mm |
| Wheel Separation (center-to-center) | 181 mm |
| Max Linear Velocity | ~0.47 m/s |
| LiDAR Range | 0.15 – 8 m |

## Project Structure

```
Tomas_bot/
├── config/                              # Configuration files
│   ├── 99-robot-devices.rules           # udev rules for RPLidar + Arduino
│   ├── mapper_params_online_async_hardware.yaml  # SLAM Toolbox config
│   ├── nav2_params_hardware.yaml        # Nav2 navigation config
│   ├── my_controllers.yaml              # ros2_control reference config
│   └── nav2_view.rviz                   # RViz visualization config
├── description/                         # Robot URDF model
│   ├── robot_hardware.urdf.xacro        # Top-level URDF
│   ├── robot_core.xacro                 # Chassis, wheels, caster
│   ├── lidar_hardware.xacro             # RPLidar C1 link/joint
│   └── inertial_macros.xacro            # Inertia calculation macros
├── firmware/
│   └── motor_controller.ino             # Arduino Leonardo motor + encoder firmware
├── launch/
│   ├── bringup_hardware.launch.py       # Full robot bringup
│   ├── rsp_hardware.launch.py           # Robot state publisher
│   ├── rplidar.launch.py                # Standalone RPLidar driver
│   ├── slam_hardware.launch.py          # SLAM Toolbox
│   └── navigation_hardware.launch.py    # Nav2 autonomous navigation
├── scripts/
│   └── diff_drive_node.py               # ROS2 ↔ Arduino serial bridge
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

# 2. Upload firmware to Arduino Leonardo (via Arduino IDE)

# 3. Launch robot
ros2 launch Tomas_bot bringup_hardware.launch.py

# 4. SLAM (build a map)
ros2 launch Tomas_bot slam_hardware.launch.py

# 5. Save map
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map

# 6. Navigate autonomously
ros2 launch Tomas_bot navigation_hardware.launch.py map:=$HOME/maps/my_map.yaml
```

## Documentation

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

| Motor | Wire   | Pin | Function |
|-------|--------|-----|----------|
| Left  | Green  | D3  | Ch A (interrupt) |
| Left  | Yellow | D2  | Ch B (direction) |
| Right | Yellow | A4  | Ch A (polled) |
| Right | Green  | A5  | Ch B (direction) |

## Dependencies

- ROS2 Jazzy
- `navigation2`, `nav2_bringup`
- `slam_toolbox`
- `sllidar_ros2` (RPLidar C1 support)
- `robot_state_publisher`, `xacro`
- `pyserial` (Python)
- Arduino IDE (for firmware upload)

## License

MIT License — see [LICENSE.md](LICENSE.md)