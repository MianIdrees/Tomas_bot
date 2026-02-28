# Running Guide — Daniel's Robot (LattePanda Alpha)

Complete step-by-step guide for building, teleoperation, SLAM mapping, and autonomous navigation.

> **ROS2 Distro:** Jazzy Jalisco | **OS:** Ubuntu 24.04 | **LiDAR:** RPLidar C1 (sllidar_ros2)

---

## Prerequisites

- [x] Ubuntu 24.04 on LattePanda Alpha
- [x] ROS2 Jazzy installed (`source /opt/ros/jazzy/setup.bash`)
- [x] Arduino Leonardo firmware uploaded (`firmware/motor_controller.ino`)
- [x] udev rules installed (`config/99-robot-devices.rules`)
- [x] RPLidar C1 connected via USB → `/dev/rplidar`
- [x] Arduino Leonardo connected → `/dev/arduino`
- [x] 12V battery connected to L298N

---

## 1. Build / Compile After Changes

Run this every time you modify any file in `src/Tomas_bot/`:

```bash
# Terminal 1 — Build
cd ~/robot_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

> **Tip:** Add to `~/.bashrc` so every new terminal is ready:
> ```bash
> echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
> echo "source ~/robot_ws/install/setup.bash" >> ~/.bashrc
> ```

---

## 2. Launch Robot Bringup (ALWAYS run this first)

This starts the core robot systems: URDF/TF, LiDAR, motor controller.

```bash
# Terminal 1 — Bringup (keep running)
source ~/robot_ws/install/setup.bash
ros2 launch Tomas_bot bringup_hardware.launch.py
```

> **Tip:** RViz2 is launched by SLAM (Step 4) or Nav2 (Step 5) — not by bringup. To launch RViz2 with bringup for debugging: `ros2 launch Tomas_bot bringup_hardware.launch.py use_rviz:=true`

**What starts:**
| Node | Purpose |
|------|---------||
| `robot_state_publisher` | Publishes URDF and static TF tree |
| `sllidar_node` | Publishes `/scan` from RPLidar C1 |
| `diff_drive_node` | Arduino bridge: `/cmd_vel` → motors, encoders → `/odom` + TF `odom→base_link` |

**Verify:**
```bash
# Terminal 2
source ~/robot_ws/install/setup.bash
ros2 topic list    # Should show: /scan, /odom, /cmd_vel, /joint_states, /tf, /tf_static
ros2 node list     # Should show: /robot_state_publisher, /sllidar_node, /diff_drive_node
ros2 topic hz /scan   # Should show ~10 Hz
ros2 topic hz /odom   # Should show ~20 Hz
```

---

## 3. Teleoperation (Keyboard Driving)

```bash
# Terminal 2 — Teleop (keep running while driving)
source ~/robot_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Speed Control (teleop_twist_keyboard)

When teleop starts, the default speeds are displayed. Use these keys to adjust:

| Key | Action |
|-----|--------|
| `i` | Forward |
| `,` | Backward |
| `j` | Turn left |
| `l` | Turn right |
| `k` | **Stop** |
| `z` | **Decrease** linear speed (press multiple times to slow down) |
| `x` | **Increase** linear speed |
| `e` | **Decrease** angular (turning) speed |
| `c` | **Increase** angular (turning) speed |

> **IMPORTANT for mapping:** Press `z` several times until linear speed is **0.10–0.15 m/s** and press `e` until angular speed is **0.3–0.5 rad/s**. Slow driving = clean maps!

---

## 4. SLAM — Build a Map

With bringup running (Step 2):

```bash
# Terminal 3 — SLAM + RViz2 (keep running while mapping)
source ~/robot_ws/install/setup.bash
ros2 launch Tomas_bot slam_hardware.launch.py
```

> **Note:** RViz2 opens automatically with the SLAM launch. To disable: `ros2 launch Tomas_bot slam_hardware.launch.py use_rviz:=false`

Now drive the robot slowly using teleop (Step 3). Tips for a clean map:
- **Drive very slowly** (0.10–0.15 m/s linear speed)
- **Turn slowly** (0.3–0.5 rad/s angular speed)
- Make multiple passes through each area
- Ensure the LiDAR has clear line of sight
- Revisit areas to trigger loop closure (corrects accumulated drift)

### Save the Map

When you are satisfied with the map in RViz2:

```bash
# Terminal 5 — Save map
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

This creates:
- `~/maps/my_map.pgm` — occupancy grid image
- `~/maps/my_map.yaml` — metadata

Now stop SLAM: **Ctrl+C** in Terminal 3.

---

## 5. Autonomous Navigation (Nav2)

With bringup still running (Step 2), **stop SLAM first** (Ctrl+C), then:

```bash
# Terminal 3 — Nav2 + RViz2 (keep running)
source ~/robot_ws/install/setup.bash
ros2 launch Tomas_bot navigation_hardware.launch.py map:=$HOME/robot_ws/my_map_room.yaml
```

> **Note:** RViz2 opens automatically. To disable: `ros2 launch Tomas_bot navigation_hardware.launch.py map:=$HOME/robot_ws/my_map_room.yaml use_rviz:=false`

### Navigate in RViz2:
1. Click **"2D Pose Estimate"** → click and drag on the map where the robot currently is (sets initial localization)
2. Wait a few seconds for AMCL particle cloud to converge
3. Click **"2D Goal Pose"** → click and drag where you want the robot to go
4. The robot plans a path and navigates autonomously!

---

## 6. Speed Tuning Reference

All speed-related variables are marked with `[SPEED]` comments in the config files.

### Teleop Speed (keyboard driving)
Controlled at runtime with `z`/`x` (linear) and `e`/`c` (angular) keys in `teleop_twist_keyboard`.

### Autonomous Navigation Speed
File: `config/nav2_params_hardware.yaml`

| Parameter | Current | Description |
|-----------|---------|-------------|
| `FollowPath.desired_linear_vel` | 0.20 m/s | Main forward speed during autonomous navigation |
| `FollowPath.rotate_to_heading_angular_vel` | 0.6 rad/s | Rotation speed when turning to face goal |
| `FollowPath.min_approach_linear_velocity` | 0.02 m/s | Minimum speed when approaching goal |
| `FollowPath.regulated_linear_scaling_min_speed` | 0.05 m/s | Minimum speed during regulated scaling |
| `FollowPath.max_angular_accel` | 1.2 rad/s² | Maximum angular acceleration |
| `velocity_smoother.max_velocity` | [0.25, 0.0, 0.8] | Hard speed limits [linear, lateral, angular] |
| `velocity_smoother.min_velocity` | [-0.20, 0.0, -0.8] | Reverse speed limits |
| `velocity_smoother.max_accel` | [0.8, 0.0, 1.2] | Acceleration limits |
| `velocity_smoother.max_decel` | [-0.8, 0.0, -1.2] | Deceleration limits |
| `velocity_smoother.deadband_velocity` | [0.01, 0.0, 0.05] | Below this = send zero (filters noise) |
| `behavior_server.max_rotational_vel` | 0.5 rad/s | Max rotation for recovery behaviors |
| `behavior_server.min_rotational_vel` | 0.15 rad/s | Min rotation for recovery behaviors |
| `behavior_server.rotational_acc_lim` | 1.0 rad/s² | Rotational acceleration limit |

### Hardware Speed Limit
File: `scripts/diff_drive_node.py` and `launch/bringup_hardware.launch.py`

| Parameter | Current | Description |
|-----------|---------|-------------|
| `max_motor_speed` | 0.47 m/s | Physical max: 130 RPM × π × 0.069m (do NOT exceed this) |

---

## Quick Reference — All Commands

```bash
# ──────────────────────────────────────────────────────────
# SOURCE WORKSPACE (run in every new terminal)
# ──────────────────────────────────────────────────────────
source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash

# ──────────────────────────────────────────────────────────
# BUILD (after any code/config changes)
# ──────────────────────────────────────────────────────────
cd ~/robot_ws && colcon build --symlink-install && source install/setup.bash

# ──────────────────────────────────────────────────────────
# BRINGUP (Terminal 1 — always run first, keep running)
# No RViz by default; SLAM/Nav2 launch their own RViz
# Add use_rviz:=true to open RViz from bringup
# ──────────────────────────────────────────────────────────
ros2 launch Tomas_bot bringup_hardware.launch.py

# ──────────────────────────────────────────────────────────
# TELEOP (Terminal 2 — keyboard driving)
# Press z/x to decrease/increase linear speed
# Press e/c to decrease/increase angular speed
# For mapping use: ~0.10–0.15 m/s linear, ~0.3–0.5 rad/s angular
# ──────────────────────────────────────────────────────────
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# ──────────────────────────────────────────────────────────
# SLAM MAPPING + RVIZ2 (Terminal 3 — build a map while driving)
# RViz2 opens automatically; add use_rviz:=false to disable
# ──────────────────────────────────────────────────────────
ros2 launch Tomas_bot slam_hardware.launch.py

# ──────────────────────────────────────────────────────────
# SAVE MAP (Terminal 4 — after mapping is complete)
# ──────────────────────────────────────────────────────────
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map

# ──────────────────────────────────────────────────────────
# AUTONOMOUS NAVIGATION + RVIZ2 (Terminal 3 — after stopping SLAM)
# RViz2 opens automatically; add use_rviz:=false to disable
# ──────────────────────────────────────────────────────────
ros2 launch Tomas_bot navigation_hardware.launch.py map:=$HOME/maps/my_map.yaml

# ──────────────────────────────────────────────────────────
# DEBUG / VERIFY
# ──────────────────────────────────────────────────────────
ros2 topic list
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic echo /odom --once
ros2 node list
ros2 run tf2_tools view_frames
```

---

## Troubleshooting

### Robot doesn't move
1. Check Arduino: `ls /dev/arduino`
2. Check node: `ros2 node list | grep diff_drive`
3. Check cmd_vel: `ros2 topic echo /cmd_vel`
4. Check L298N power (12V battery connected?)

### Map has too much drift
1. **Drive slower** — reduce to 0.10 m/s linear, 0.3 rad/s angular
2. **Calibrate `ticks_per_rev`** — see Encoder Calibration below
3. **Revisit areas** — loop closure helps correct drift
4. Check `/scan` in RViz2 — laser scans should align with walls consistently

### Encoder ticks_per_rev Calibration
The default `ticks_per_rev=528` assumes 48:1 gear ratio × 11 PPR. To calibrate:
1. Open Arduino Serial Monitor (115200 baud)
2. Send `r` to reset ticks
3. Manually rotate ONE wheel exactly ONE full revolution
4. Read the tick count from the `e` messages
5. Update `ticks_per_rev` in `bringup_hardware.launch.py`
6. Rebuild: `cd ~/robot_ws && colcon build --symlink-install`

### LiDAR not publishing /scan
1. Check device: `ls /dev/rplidar`
2. Check permissions: `sudo chmod 666 /dev/rplidar`
3. Standalone test: `ros2 launch Tomas_bot rplidar.launch.py serial_port:=/dev/rplidar`

### Nav2 goal fails
1. Check map quality — re-map if needed
2. Ensure initial pose estimate is accurate (2D Pose Estimate in RViz2)
3. Check that the goal is in free space on the map
4. Check terminal output for error messages

### Wheels spin wrong direction
- Swap IN1/IN2 wires (left motor) or IN3/IN4 wires (right motor) on L298N
- OR swap motor terminal wires on L298N outputs

---

## TF Tree (expected structure)

```
map → odom → base_link → chassis → laser_frame
                       ↘ left_wheel       ↘ caster_wheel
                       ↘ right_wheel      ↘ lidar_riser
                       ↘ base_footprint   ↘ left/right_motor
                                          ↘ left/right_support
```

- `map → odom`: Published by SLAM Toolbox (during mapping) or AMCL (during navigation)
- `odom → base_link`: Published by `diff_drive_node` (from encoder odometry)
- All other transforms: Published by `robot_state_publisher` (from URDF)
