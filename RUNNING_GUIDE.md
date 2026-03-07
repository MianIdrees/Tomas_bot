# Running Guide — Daniel's Robot (LattePanda Alpha)

Complete step-by-step guide for building, teleoperation, SLAM mapping, and autonomous navigation.

> **ROS2 Distro:** Jazzy Jalisco | **OS:** Ubuntu 24.04 | **LiDAR:** RPLidar C1 (sllidar_ros2)

---

## Prerequisites

- [x] Ubuntu 24.04 on LattePanda Alpha
- [x] ROS2 Jazzy installed (`source /opt/ros/jazzy/setup.bash`)
- [x] Arduino Leonardo firmware uploaded (`firmware/motor_controller/motor_controller.ino`)
- [x] Adafruit BNO08x Arduino library installed
- [x] BNO085 IMU wired: VIN→5V, GND→GND, SDA→D2, SCL→D3
- [x] Left encoder moved: Green→D1, Yellow→D0 (freed D2/D3 for I²C)
- [x] udev rules installed (`config/99-robot-devices.rules`)
- [x] RPLidar C1 connected via USB → `/dev/rplidar`
- [x] Arduino Leonardo connected → `/dev/arduino`
- [x] 12V battery connected to L298N
- [x] (Optional) PS3-style 2.4GHz wireless gamepad + USB dongle for joystick control
- [x] IMU calibrated (`ros2 run Tomas_bot imu_calibration_node.py`) — see [IMU_GUIDE.md](IMU_GUIDE.md)

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
|------|--------|
| `robot_state_publisher` | Publishes URDF and static TF tree (includes imu_link) |
| `sllidar_node` | Publishes `/scan` from RPLidar C1 |
| `diff_drive_node` | Arduino bridge: `/cmd_vel` → motors, encoders → `/wheel/odom`, IMU → `/imu/data`, `/joint_states` |
| `ekf_filter_node` | EKF sensor fusion: `/wheel/odom` + `/imu/data` → `/odom` + TF `odom→base_link` |

**Verify:**
```bash
# Terminal 2
source ~/robot_ws/install/setup.bash
ros2 topic list    # Should show: /scan, /odom, /wheel/odom, /imu/data, /cmd_vel, /joint_states, /tf, /tf_static
ros2 node list     # Should show: /robot_state_publisher, /sllidar_node, /diff_drive_node, /ekf_filter_node
ros2 topic hz /scan        # Should show ~10 Hz
ros2 topic hz /wheel/odom  # Should show ~20 Hz (encoder odometry)
ros2 topic hz /imu/data    # Should show ~20 Hz (BNO085 IMU)
ros2 topic hz /odom        # Should show ~30 Hz (EKF fused output)
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

## 3b. Teleoperation (PS3-Style Gamepad / Joystick)

Use the generic PS3-style 2.4GHz wireless gamepad for proportional analog control.

### One-time Setup

```bash
# Install the ROS2 joy driver
sudo apt install ros-jazzy-joy

# Make the joystick script executable (if not already)
chmod +x ~/robot_ws/src/Tomas_bot/scripts/joystick_teleop_node.py

# Rebuild (after first install only)
cd ~/robot_ws && colcon build --symlink-install && source install/setup.bash
```

### Verify Controller is Detected

1. Plug the **2.4GHz USB dongle** into the LattePanda (or your PC)
2. Turn on the gamepad (LED should light up to confirm pairing)
3. Check the device:

```bash
ls /dev/input/js*          # Should show /dev/input/js0
jstest /dev/input/js0      # Move sticks/press buttons to see raw values (Ctrl+C to exit)
```

> **Tip:** If `jstest` is not installed: `sudo apt install joystick`

### Launch Joystick Teleop

With bringup running (Step 2):

```bash
# Terminal 2 — Joystick teleop (keep running while driving)
source ~/robot_ws/install/setup.bash
ros2 launch Tomas_bot joystick_teleop.launch.py
```

### Controller Layout & Controls

```
              ┌────────────────────────────────────────┐
              │           PS3-Style Gamepad             │
              │                                        │
              │    [L2]                        [R2]    │
              │    [L1]  ← ENABLE    TURBO →  [R1]    │
              │                                        │
              │     ┌─┐     [Select] [Start]    (△)    │
              │     │↑│                       (□) (○)  │
              │   ┌─┼─┼─┐                      (✕)    │
              │   │←│ │→│           ◉                  │
              │   └─┼─┼─┘     ◉   RIGHT               │
              │     │↓│       LEFT  STICK              │
              │     └─┘       STICK                    │
              │            (forward/back)  (turn L/R)  │
              └────────────────────────────────────────┘
```

| Control | Input | Action |
|---------|-------|--------|
| **Forward / Backward** | Left Stick ↑↓ | Proportional linear velocity (push more = go faster) |
| **Turn Left / Right** | Right Stick ←→ | Proportional angular velocity (push more = turn faster) |
| **Forward + Turn** | Both sticks | Arc motion — push left stick for speed, right stick to steer |
| **Enable (Deadman)** | **Hold L1** | **MUST hold to drive** — robot stops instantly when released |
| **Turbo Mode** | Hold L1 + R1 | Fast speed (~0.40 m/s linear, ~1.63 rad/s angular) |
| **Normal Mode** | Hold L1 only | Gentle speed (~0.14 m/s linear, ~1.25 rad/s angular) |
| **Emergency Stop** | Press Start | Immediately stops robot; press L1 again to resume |

### Speed Modes

| Mode | Linear Speed | Angular Speed | How to Activate |
|------|-------------|---------------|------------------|
| **Stopped** | 0 m/s | 0 rad/s | Release L1 (or press Start) |
| **Normal** | up to ~0.14 m/s | up to ~1.25 rad/s | Hold L1 + move sticks |
| **Turbo** | up to ~0.40 m/s | up to ~1.63 rad/s | Hold L1 + R1 + move sticks |

> **For SLAM mapping:** Use Normal mode (L1 only) and push the stick gently (~30%) for slow, clean maps.

### If Your Controller Mapping is Different

Generic PS3 controllers vary between manufacturers. If the sticks or buttons don't match:

1. **Identify your mapping:**
   ```bash
   # Run jstest and move each stick / press each button
   jstest /dev/input/js0
   ```
   Note which axis number changes when you push each stick, and which button number lights up.

2. **Edit the config file:** `config/joystick_params.yaml`
   ```yaml
   joystick_teleop_node:
     ros__parameters:
       linear_axis: 1        # Change to YOUR left stick Y axis number
       angular_axis: 2       # Change to YOUR right stick X axis number
       enable_button: 6      # Change to YOUR L1 button number
       turbo_button: 7       # Change to YOUR R1 button number
       estop_button: 9       # Change to YOUR Start button number
       linear_axis_inverted: false  # Set true if forward = negative on your controller
       angular_axis_inverted: false # Set true if left = positive already
   ```

3. **Rebuild and relaunch:**
   ```bash
   cd ~/robot_ws && colcon build --symlink-install && source install/setup.bash
   ros2 launch Tomas_bot joystick_teleop.launch.py
   ```

### Troubleshooting Joystick

| Problem | Solution |
|---------|----------|
| No `/dev/input/js0` | Check USB dongle is plugged in; try different USB port |
| Controller not pairing | Turn off gamepad, wait 5s, turn back on; re-plug USB dongle |
| Sticks work but robot doesn't move | **Are you holding L1?** (deadman switch required) |
| Robot moves wrong direction | Edit `config/joystick_params.yaml` — flip `linear_axis_inverted` or `angular_axis_inverted` |
| Sticks mapped to wrong function | Run `jstest` to find correct axis numbers, update config |
| Robot stutters or jerks | Increase `stick_deadzone` to 0.15 in config |
| No `/joy` topic | `ros2 topic list \| grep joy` — if missing, check joy_node is running |

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
ros2 run nav2_map_server map_saver_cli -f ~/robot_ws/my_map1
```

This creates:
- `~/robot_ws/my_map1.pgm` — occupancy grid image
- `~/robot_ws/my_map1.yaml` — metadata

Now stop SLAM: **Ctrl+C** in Terminal 3.

---

## 5. Autonomous Navigation (Nav2)

With bringup still running (Step 2), **stop SLAM first** (Ctrl+C), then:

```bash
# Terminal 3 — Nav2 + RViz2 (keep running)
source ~/robot_ws/install/setup.bash
ros2 launch Tomas_bot navigation_hardware.launch.py map:=$HOME/robot_ws/my_map1.yaml
```

> **Note:** RViz2 opens automatically. To disable: `ros2 launch Tomas_bot navigation_hardware.launch.py map:=$HOME/robot_ws/my_map1.yaml use_rviz:=false`

### Navigate in RViz2:
1. Wait ~5 seconds for AMCL to auto-initialize (initial pose is set automatically at map origin)
2. If the robot is NOT at the map origin, click **"2D Pose Estimate"** → click and drag on the map where the robot currently is
3. Click **"2D Goal Pose"** → click and drag where you want the robot to go
4. The robot plans a path and navigates autonomously!

### Headless Navigation (no RViz2):
You can send goals from the command line without RViz2:
```bash
# Send a goal to position (x=1.0, y=0.0) facing forward
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
pose:
  header:
    frame_id: map
  pose:
    position: {x: 1.0, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
" --feedback
```

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
| `FollowPath.rotate_to_heading_angular_vel` | 0.8 rad/s | Rotation speed when turning to face goal (~88% accurate) |
| `FollowPath.min_approach_linear_velocity` | 0.02 m/s | Minimum speed when approaching goal |
| `FollowPath.regulated_linear_scaling_min_speed` | 0.05 m/s | Minimum speed during regulated scaling |
| `FollowPath.max_angular_accel` | 0.8 rad/s² | Maximum angular acceleration |
| `velocity_smoother.max_velocity` | [0.20, 0.0, 0.8] | Hard speed limits [linear, lateral, angular] |
| `velocity_smoother.min_velocity` | [-0.20, 0.0, -0.8] | Reverse speed limits |
| `velocity_smoother.max_accel` | [0.6, 0.0, 1.0] | Acceleration limits |
| `velocity_smoother.max_decel` | [-0.8, 0.0, -1.2] | Deceleration limits |
| `velocity_smoother.deadband_velocity` | [0.01, 0.0, 0.05] | Below this = send zero (filters noise) |
| `behavior_server.max_rotational_vel` | 0.8 rad/s | Max rotation for recovery behaviors |
| `behavior_server.min_rotational_vel` | 0.4 rad/s | Min rotation — motors unreliable below ~0.42 rad/s |
| `behavior_server.rotational_acc_lim` | 1.0 rad/s² | Rotational acceleration limit |

### Hardware Speed Limit
File: `scripts/diff_drive_node.py` and `launch/bringup_hardware.launch.py`

| Parameter | Current | Description |
|-----------|---------|-------------|
| `max_motor_speed` | 0.391 m/s | Calibrated max: measured via raw PWM at 48 ticks/50ms × wheel geometry |

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
# JOYSTICK TELEOP (Terminal 2 — PS3 gamepad driving)
# Hold L1 to enable, L1+R1 for turbo, Start for E-stop
# Left Stick Y = forward/back, Right Stick X = turn
# ──────────────────────────────────────────────────────────
ros2 launch Tomas_bot joystick_teleop.launch.py

# ──────────────────────────────────────────────────────────
# SLAM MAPPING + RVIZ2 (Terminal 3 — build a map while driving)
# RViz2 opens automatically; add use_rviz:=false to disable
# ──────────────────────────────────────────────────────────
ros2 launch Tomas_bot slam_hardware.launch.py

# ──────────────────────────────────────────────────────────
# SAVE MAP (Terminal 4 — after mapping is complete)
# ──────────────────────────────────────────────────────────
ros2 run nav2_map_server map_saver_cli -f ~/robot_ws/my_map1

# ──────────────────────────────────────────────────────────
# AUTONOMOUS NAVIGATION + RVIZ2 (Terminal 3 — after stopping SLAM)
# RViz2 opens automatically; add use_rviz:=false to disable
# ──────────────────────────────────────────────────────────
ros2 launch Tomas_bot navigation_hardware.launch.py map:=$HOME/robot_ws/my_map1.yaml

# ──────────────────────────────────────────────────────────
# DEBUG / VERIFY
# ──────────────────────────────────────────────────────────
ros2 topic list
ros2 topic hz /scan
ros2 topic hz /wheel/odom    # Encoder-only odometry (~20 Hz)
ros2 topic hz /imu/data       # BNO085 IMU data (~20 Hz)
ros2 topic hz /odom           # EKF fused output (~30 Hz)
ros2 topic echo /odom --once
ros2 topic echo /imu/data --once
ros2 node list
ros2 run tf2_tools view_frames

# ──────────────────────────────────────────────────────────
# IMU DIAGNOSTICS (standalone — no bringup needed)
# ──────────────────────────────────────────────────────────
ros2 run Tomas_bot imu_check_node.py
ros2 run Tomas_bot imu_calibration_node.py
```

---

## 7. Motor Calibration Reference

The motor system has been calibrated with the following key parameters:

### Arduino Firmware (`firmware/motor_controller/motor_controller.ino`)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `MAX_TICKS_PER_INTERVAL` | 48 | Ticks per 50ms at full PWM (measured via raw PWM mode) |
| `MIN_PWM` | 25 | Minimum PWM to overcome motor stiction |
| `KP` | 1.0 | PID proportional gain |
| `KI` | 0.8 | PID integral gain |
| `KD` | 0.15 | PID derivative gain |
| `INTEGRAL_LIMIT` | 150 | Anti-windup clamp |
| `PID_INTERVAL_MS` | 50 | PID update interval (with dt normalization) |

### Python Node (`scripts/diff_drive_node.py`)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `max_motor_speed` | 0.391 m/s | Calibrated from 48 ticks/50ms × wheel geometry |
| `min_pwm` | 25 | Must match Arduino `MIN_PWM` |
| `wheel_separation` | 0.181 m | Center-to-center wheel distance |
| `wheel_radius` | 0.0345 m | 69mm wheels |
| `ticks_per_rev` | 528 | 11 PPR × 48:1 gear ratio |

### Hardware Limitations

- **Minimum sustainable wheel speed:** ~0.038 m/s (at PWM 25)
- **Minimum reliable angular velocity:** ~0.42 rad/s (below this, motors duty-cycle between ON/OFF)
- **Encoder resolution:** 528 ticks/rev → 0.41mm per tick

### Recalibrating Motor Speed

If you change motors, wheels, or battery voltage:

```bash
# 1. Connect to Arduino serial (115200 baud)
# 2. Send raw PWM command to bypass PID:
#    w 255 255    (both motors full speed forward)
# 3. Read tick counts over 3 seconds from 'e' messages
# 4. Calculate: max_ticks_per_50ms = total_ticks / (3000 / 50)
# 5. Calculate: max_motor_speed = (max_ticks / 528) × 2π × 0.0345 / 0.050
# 6. Update MAX_TICKS_PER_INTERVAL in firmware and max_motor_speed in launch file
# 7. Rebuild and re-upload
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
2. If robot is not at map origin, set initial pose in RViz2 (2D Pose Estimate) or restart at origin
3. Check that the goal is in free space on the map
4. Check terminal output for error messages
5. Verify `map → odom` TF: `ros2 run tf2_ros tf2_echo map odom`
6. If AMCL won't initialize, ensure LiDAR `/scan` is publishing: `ros2 topic hz /scan`

### Wheels spin wrong direction
- Swap IN1/IN2 wires (left motor) or IN3/IN4 wires (right motor) on L298N
- OR swap motor terminal wires on L298N outputs

---

## TF Tree (expected structure)

```
map → odom → base_link → chassis → laser_frame
                                  → imu_link (BNO085 IMU)
                                  → lidar_riser
                                  → caster_mount / caster_wheel
                                  → left/right_motor
                                  → left/right_support
             ↘ base_footprint
             ↘ left_wheel
             ↘ right_wheel
```

- `map → odom`: Published by SLAM Toolbox (during mapping) or AMCL (during navigation)
- `odom → base_link`: Published by **EKF** (`robot_localization`) — fused wheel encoders + BNO085 IMU
- All other transforms: Published by `robot_state_publisher` (from URDF)

---

## IMU Calibration & Diagnostics

See [IMU_GUIDE.md](IMU_GUIDE.md) for the complete BNO085 setup, calibration, and debugging guide.

### Quick IMU Check

```bash
# Run the IMU health check (standalone — no bringup needed)
ros2 run Tomas_bot imu_check_node.py

# Run the full calibration procedure (10 interactive steps)
ros2 run Tomas_bot imu_calibration_node.py
```

### Verify IMU in RViz

After bringup, the `imu_link` should appear as a small blue rectangle on the chassis in RViz:
```bash
ros2 launch Tomas_bot bringup_hardware.launch.py use_rviz:=true
```
