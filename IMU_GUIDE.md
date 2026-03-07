# BNO085 IMU Integration Guide — Tomas_bot

Complete guide for setting up, calibrating, verifying, and debugging the Adafruit BNO085 9-DOF IMU on Tomas_bot.

---

## Overview

The BNO085 IMU provides:
- **Game Rotation Vector** — fused quaternion orientation (gyro + accel, no magnetometer drift)
- **Accelerometer** — linear acceleration (m/s²) including gravity
- **Calibrated Gyroscope** — angular velocity (rad/s) with bias correction

These feed into the **Extended Kalman Filter (EKF)** via `robot_localization`, which fuses them with wheel encoder odometry for dramatically improved localization.

### Architecture

```
                    Arduino Leonardo (ATmega32U4)
                    ┌──────────────────────────────┐
 BNO085 ──I²C──→   │  Read IMU @ 20Hz             │
                    │  Read Encoders @ 20Hz        │   USB Serial (115200 baud)
 Encoders ──────→   │  PID Motor Control           │ ──────────────────────→  diff_drive_node.py
 L298N    ←─────    │  Motor PWM Output            │                          ├─→ /wheel/odom
                    └──────────────────────────────┘                          ├─→ /imu/data
                                                                              └─→ /joint_states
                                                                                     │
                    ┌──────────────────────────────┐                                 │
                    │  EKF (robot_localization)     │ ←── /wheel/odom + /imu/data ───┘
                    │  Fuses odom + IMU             │
                    │  Publishes /odom              │
                    │  Broadcasts odom → base_link  │
                    └──────────────────────────────┘
                                 │
                    ┌────────────┴────────────────────┐
                    │  Nav2 / SLAM Toolbox            │
                    │  Uses fused /odom for planning  │
                    └─────────────────────────────────┘
```

### Serial Protocol

The Arduino sends two types of messages over USB CDC at 115200 baud:

| Prefix | Format | Example |
|--------|--------|---------|
| `e` | `e <left_ticks> <right_ticks>\n` | `e 1024 -980` |
| `i` | `i <qw> <qx> <qy> <qz> <ax> <ay> <az> <gx> <gy> <gz>\n` | `i 1.0000 0.0012 -0.0003 0.0150 0.12 -0.34 9.78 0.001 -0.002 0.0003` |

Both are sent at 20Hz (every 50ms).

### Topic Flow

| Topic | Source | Consumer | Data |
|-------|--------|----------|------|
| `/wheel/odom` | diff_drive_node | EKF | Encoder-only odometry (x, y, yaw, vx, vyaw) |
| `/imu/data` | diff_drive_node | EKF | BNO085 orientation + angular velocity + linear acceleration |
| `/odom` | EKF | Nav2, SLAM | Fused odometry (wheel + IMU) |
| `/cmd_vel` | Nav2 / teleop | diff_drive_node | Velocity commands |
| `/joint_states` | diff_drive_node | robot_state_publisher | Wheel positions for URDF |

### TF Tree

```
map → odom → base_link → chassis → laser_frame
                                  → imu_link (IMU sensor)
                                  → lidar_riser
                                  → caster_mount
                                  → caster_wheel
                                  → left/right_motor
                                  → left/right_support
             ↘ base_footprint
             ↘ left_wheel
             ↘ right_wheel
```

| Transform | Published By |
|-----------|-------------|
| `map → odom` | SLAM Toolbox (mapping) or AMCL (navigation) |
| `odom → base_link` | EKF (robot_localization) — fused wheel + IMU |
| all others | robot_state_publisher (from URDF) |

---

## 1. Hardware Setup

### Pin Mapping (Arduino Leonardo)

**IMPORTANT:** The left encoder was moved from D3/D2 to D1/D0 to free the I²C bus (D2/D3) for the BNO085.

#### Motor Driver (L298N) — unchanged

| L298N | Arduino | Function |
|-------|---------|----------|
| ENA | D5 | Left motor PWM |
| IN1 | D7 | Left motor direction |
| IN2 | D6 | Left motor direction |
| IN3 | D10 | Right motor direction |
| IN4 | D9 | Right motor direction |
| ENB | D11 | Right motor PWM |

#### Encoders — LEFT ENCODER MOVED

| Motor | Wire | Arduino Pin | Function | Notes |
|-------|------|-------------|----------|-------|
| **Left** | **Green** | **D1** | Ch A (INT3 hardware interrupt) | **MOVED from D3** |
| **Left** | **Yellow** | **D0** | Ch B (direction sensing) | **MOVED from D2** |
| Right | Yellow | A4 | Ch A (polled) | Unchanged |
| Right | Green | A5 | Ch B (direction) | Unchanged |

#### BNO085 IMU (I²C)

| BNO085 | Arduino | Function |
|--------|---------|----------|
| VIN | 5V | Power (3.3V regulator on breakout) |
| GND | GND | Ground |
| SDA | D2 | I²C data (hardware SDA on Leonardo) |
| SCL | D3 | I²C clock (hardware SCL on Leonardo) |
| INT | D4 | Data-ready interrupt (optional) |

### Wiring Steps

1. **Move left encoder wires:**
   - Green wire: D3 → D1
   - Yellow wire: D2 → D0
2. **Connect BNO085:**
   - VIN → 5V (Arduino)
   - GND → GND (Arduino)
   - SDA → D2 (Arduino)
   - SCL → D3 (Arduino)
3. **Verify connections** — double-check that no two sensors share the same pin

### Physical Mounting

The IMU should be mounted:
- **Flat on the chassis** (chip-side UP, Z-axis pointing to ceiling)
- **+X arrow pointing forward** (toward the LiDAR/front of robot) — follow REP-105
- **Between the drive axle and LiDAR** — approximately at x=0.24m from chassis origin
- **Away from motors and magnets** — magnetic interference degrades IMU accuracy
- **Firmly attached** — vibrations cause noisy accelerometer readings; use double-sided tape or standoffs

---

## 2. Arduino Firmware Setup

### Install BNO085 Arduino Library

1. Open Arduino IDE
2. Go to **Tools → Manage Libraries...** (or **Sketch → Include Library → Manage Libraries...**)
3. Search for **"Adafruit BNO08x"**
4. Install **Adafruit BNO08x** (this also installs `Adafruit BusIO` and `Adafruit Unified Sensor` dependencies)
5. Also verify **Wire** library is available (built-in)

### Upload Firmware

1. Open `firmware/motor_controller/motor_controller.ino` in Arduino IDE
2. Select:
   - **Board:** Arduino Leonardo
   - **Port:** `/dev/ttyACM0` (or `/dev/arduino`)
3. Click **Upload**
4. Open Serial Monitor (115200 baud) — you should see:
   ```
   BNO085 found on I2C 0x4A
   BNO085 reports enabled (RotVec+Accel+Gyro @20Hz)
   Leonardo motor+IMU controller ready (PID + BNO085)
   ```

### Verify IMU on Serial Monitor

After uploading, you should see interleaved lines:
```
e 0 0
i 1.0000 0.0012 -0.0003 0.0150 0.12 -0.34 9.78 0.001 -0.002 0.0003
e 0 0
i 0.9999 0.0011 -0.0002 0.0148 0.11 -0.33 9.79 0.002 -0.001 0.0002
```

**If you see `! BNO085 not detected on I2C 0x4A`:**
- Check VIN→5V and GND→GND connections
- Verify SDA→D2 and SCL→D3 (not swapped)
- Try re-seating the BNO085 breakout
- Run I²C scanner sketch to detect addresses

---

## 3. Calibration

### Run the Calibration Script

```bash
# From ROS2 (with workspace sourced)
ros2 run Tomas_bot imu_calibration_node.py

# OR standalone (no ROS2 needed)
python3 ~/robot_ws/src/Tomas_bot/scripts/imu_calibration_node.py --port /dev/arduino --baud 115200
```

The script guides you through 10 steps:

| Step | Action | Duration |
|------|--------|----------|
| 1 | Gyroscope: Place flat, hold completely still | 8s |
| 2 | Accel Position 1: Normal driving position, hold still | 6s |
| 3 | Accel Position 2: Tilt nose UP ~45° | 6s |
| 4 | Accel Position 3: Tilt nose DOWN ~45° | 6s |
| 5 | Accel Position 4: Tilt left side DOWN ~45° | 6s |
| 6 | Accel Position 5: Tilt right side DOWN ~45° | 6s |
| 7 | Accel Position 6: Flip upside down, then back | 6s |
| 8 | Magnetometer: Slow figure-8 rotations (2-3 loops) | 45s |
| 9 | Magnetometer: Rotate + tilt combined | 35s |
| 10 | Verification: Flat, still, check readings | 12s |

**Tips for good calibration:**
- Do calibration **before** each mapping session if high accuracy is needed
- The BNO085 performs **dynamic calibration** continuously — this script just helps it converge faster
- Stay away from motors, metal desks, and magnetic sources during magnetometer steps
- Move slowly and smoothly during rotation steps

### Verification After Calibration

The final step checks:
- **Quaternion norm ≈ 1.0** (should be 0.98–1.02)
- **Gravity magnitude ≈ 9.81 m/s²** (should be 9.3–10.3)
- **Gyro drift < 0.01 rad/s** when stationary

---

## 4. IMU Health Check / Diagnostics

### Run the Health Check Script

```bash
# From ROS2
ros2 run Tomas_bot imu_check_node.py

# Standalone with custom port
python3 ~/robot_ws/src/Tomas_bot/scripts/imu_check_node.py --port /dev/arduino --baud 115200

# Run for specific duration (seconds)
python3 ~/robot_ws/src/Tomas_bot/scripts/imu_check_node.py --port /dev/arduino --duration 30
```

This displays a live dashboard:
```
══════════════════════════════════════════════════════════════════════
  BNO085 IMU Health Check — Live Data
  Elapsed: 12.3s  |  Samples: 245  |  Rate: 19.9 Hz
══════════════════════════════════════════════════════════════════════

  ── Orientation (Quaternion) ──
     w:  +0.9998   x:  +0.0012   y:  -0.0003   z:  +0.0150
     Norm: 1.0001 ✓

  ── Orientation (Euler degrees) ──
     Roll:    +0.14°   Pitch:   -0.03°   Yaw:    +1.72°

  ── Accelerometer (m/s²) ──
     X:   +0.120   Y:   -0.340   Z:   +9.780
     |g| = 9.792 m/s² ✓

  ── Gyroscope (rad/s) ──
     X:  +0.0010   Y:  -0.0020   Z:  +0.0003
     |ω| = 0.0023 rad/s ✓ (still)

  ── Health ──
     ✓  ALL OK — IMU readings look healthy
```

### What to check:
| Metric | Expected Value | Problem If |
|--------|---------------|------------|
| Data Rate | ~20 Hz | < 10 Hz → I²C issue |
| Quaternion Norm | ~1.000 | > 1.05 or < 0.95 → sensor error |
| Gravity Magnitude | ~9.81 m/s² | off by > 1.5 → accel not calibrated |
| Gyro at Rest | < 0.02 rad/s | > 0.05 → gyro not calibrated |
| Euler (robot flat) | Roll ≈ 0°, Pitch ≈ 0° | > ±5° → IMU not mounted flat |

---

## 5. Running the Full System

### Step-by-Step (3 Terminals)

**Terminal 1 — Robot Bringup (always first):**
```bash
source ~/robot_ws/install/setup.bash
ros2 launch Tomas_bot bringup_hardware.launch.py
```

This starts: Robot State Publisher + RPLidar C1 + diff_drive_node + EKF

**Terminal 2 — Teleop OR Joystick:**
```bash
# Keyboard teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# OR joystick teleop
ros2 launch Tomas_bot joystick_teleop.launch.py
```

**Terminal 3 — SLAM (mapping) OR Nav2 (navigation):**
```bash
# For mapping
ros2 launch Tomas_bot slam_hardware.launch.py

# For autonomous navigation (after you have a map)
ros2 launch Tomas_bot navigation_hardware.launch.py map:=$HOME/maps/my_map.yaml
```

### Verify Everything is Running

```bash
# In a new terminal:
source ~/robot_ws/install/setup.bash

# Check all nodes are running
ros2 node list
# Expected:
#   /robot_state_publisher
#   /sllidar_node
#   /diff_drive_node
#   /ekf_filter_node          ← NEW: EKF node
#   /slam_toolbox (or Nav2 nodes)

# Check topics
ros2 topic list
# Key topics to verify:
#   /wheel/odom               ← NEW: encoder-only odom (was /odom)
#   /imu/data                 ← NEW: BNO085 IMU data
#   /odom                     ← Now published by EKF (fused)
#   /scan
#   /cmd_vel
#   /joint_states
#   /tf
#   /tf_static

# Check data rates
ros2 topic hz /wheel/odom     # Should be ~20 Hz
ros2 topic hz /imu/data        # Should be ~20 Hz
ros2 topic hz /odom            # Should be ~30 Hz (EKF rate)
ros2 topic hz /scan            # Should be ~10 Hz

# Inspect IMU data
ros2 topic echo /imu/data --once
# Should show: orientation (quaternion), angular_velocity, linear_acceleration

# Inspect fused odometry
ros2 topic echo /odom --once
# Should show: position, orientation, and twist from EKF fusion

# Check TF tree
ros2 run tf2_tools view_frames
# Open frames.pdf — verify: map → odom → base_link → chassis → imu_link
```

---

## 6. Debugging Common Issues

### IMU Not Detected on Arduino

**Symptom:** Serial monitor shows `! BNO085 not detected on I2C 0x4A`

| Check | How |
|-------|-----|
| Wiring | VIN→5V, GND→GND, SDA→D2, SCL→D3 |
| I²C address | Run I²C scanner sketch — should find 0x4A |
| Power | Is VIN getting 5V? (measure with multimeter) |
| Solder joints | Cold joints on breakout board headers? |
| Library | Adafruit BNO08x installed in Arduino IDE? |

### IMU Data but No /imu/data Topic

**Symptom:** Arduino sends `i` lines, but `ros2 topic echo /imu/data` shows nothing

| Check | How |
|-------|-----|
| diff_drive_node running? | `ros2 node list \| grep diff_drive` |
| Serial connected? | Check diff_drive_node logs for "Connected to Arduino" |
| Parsing error? | Check diff_drive_node logs for serial read errors |
| IMU line format | Must be exactly 11 space-separated values starting with "i " |

### EKF Not Publishing /odom

**Symptom:** `/wheel/odom` and `/imu/data` are publishing, but `/odom` from EKF is empty or not publishing

| Check | How |
|-------|-----|
| EKF node running? | `ros2 node list \| grep ekf` |
| Correct topics? | `ros2 topic info /wheel/odom` and `/imu/data` — check publishers exist |
| TF available? | EKF needs `base_link` and `odom` frames — check URDF is published |
| Frame IDs match? | `/wheel/odom` header.frame_id = "odom", child_frame_id = "base_link" |
| IMU frame_id? | `/imu/data` header.frame_id = "imu_link" — must match URDF link name |
| EKF logs | `ros2 topic echo /rosout \| grep ekf` — look for warning/error messages |

### Robot Drifts or Rotates When Stationary

**Symptom:** Robot's position in RViz drifts even when not moving

| Cause | Fix |
|-------|-----|
| IMU not calibrated | Run calibration script |
| Gyro bias | Place robot still for 10+ seconds after boot — BNO085 auto-calibrates gyro |
| Encoder noise | Check encoder wires aren't loose; ensure clean encoder discs |
| EKF covariance too low | Increase process noise in `config/ekf.yaml` |

### Map Quality Degraded After Adding IMU

**Symptom:** SLAM map is worse than before IMU

| Cause | Fix |
|-------|-----|
| IMU orientation wrong | Ensure BNO085 +X faces robot forward, chip-side up |
| EKF trusts IMU too much | Increase IMU covariance in diff_drive_node (orientation_covariance) |
| Frame mismatch | Verify `imu_link` in URDF matches `imu_frame` parameter |
| IMU mounted loosely | Secure with double-sided tape or screws |

### Left Encoder Not Working After Pin Move

**Symptom:** Left wheel ticks stay at 0 after moving encoder to D1/D0

| Check | How |
|-------|-----|
| Wiring | Green wire on D1, Yellow wire on D0 — not swapped |
| Physical pins | D1 is the pin next to D2 on Leonardo — verify on pinout diagram |
| Interrupt | D1 = INT3 on Leonardo — `attachInterrupt(digitalPinToInterrupt(1), ...)` |
| Serial conflict | D0/D1 are TX/RX pins — but Leonardo uses USB CDC for Serial, not D0/D1 hardware UART, so this is safe |

> **Important Note:** On Arduino Leonardo (ATmega32U4), `Serial` uses the native USB port, NOT pins D0/D1. Pins D0/D1 are `Serial1` (hardware UART), which we are NOT using. So there is **no conflict** between encoder pins on D0/D1 and USB serial communication.

---

## 7. EKF Tuning Guide

The EKF configuration is in `config/ekf.yaml`. Key parameters to tune:

### When to Trust Encoders More (reduce IMU influence)

If the IMU adds noise or drift, increase IMU covariance in `scripts/diff_drive_node.py`:

```python
# In _parse_imu_line():
imu_msg.orientation_covariance[0] = 0.1    # Was 0.01 — less trust in orientation
imu_msg.angular_velocity_covariance[0] = 0.01  # Was 0.001
imu_msg.linear_acceleration_covariance[0] = 1.0  # Was 0.1
```

### When to Trust IMU More (reduce encoder influence)

If encoders slip (e.g., on smooth floors), increase odom covariance in `scripts/diff_drive_node.py`:

```python
# In publish_odometry():
odom.pose.covariance[0] = 0.2    # Was 0.05 — less trust in encoder position
odom.pose.covariance[35] = 0.5   # Was 0.1 — less trust in encoder yaw
```

### Process Noise (prediction model trust)

In `config/ekf.yaml`, the process noise diagonal controls how much the EKF trusts its own internal model between sensor updates. Larger values = more responsive to sensor changes but noisier output:

```yaml
# Increase if robot seems "sluggish" to respond
process_noise_covariance: [0.05, ...]  # x position noise

# Decrease if output is too noisy/jerky
process_noise_covariance: [0.01, ...]  # smoother but slower to respond
```

---

## 8. Pin Cross-Reference

Complete Arduino Leonardo pin allocation for Tomas_bot:

| Pin | Function | Component | Notes |
|-----|----------|-----------|-------|
| D0 | Left Encoder Ch B | Encoder | Direction sensing (MOVED from D2) |
| D1 | Left Encoder Ch A | Encoder | INT3 hardware interrupt (MOVED from D3) |
| D2 (SDA) | I²C Data | BNO085 | Hardware I²C SDA |
| D3 (SCL) | I²C Clock | BNO085 | Hardware I²C SCL |
| D4 | IMU Interrupt | BNO085 | Optional data-ready pin |
| D5 | Left Motor PWM (ENA) | L298N | PWM speed control |
| D6 | Left Motor Dir (IN2) | L298N | Direction |
| D7 | Left Motor Dir (IN1) | L298N | Direction |
| D9 | Right Motor Dir (IN4) | L298N | Direction |
| D10 | Right Motor Dir (IN3) | L298N | Direction |
| D11 | Right Motor PWM (ENB) | L298N | PWM speed control |
| A4 | Right Encoder Ch A | Encoder | Polled (no hardware interrupt) |
| A5 | Right Encoder Ch B | Encoder | Direction sensing |
| USB | Serial 115200 baud | LattePanda | CDC serial — NOT D0/D1 |

**Free pins:** D4 (used for optional IMU INT), D8, D12, D13, A0–A3

---

## 9. Quick Troubleshooting Checklist

Run this after any hardware change:

```bash
# 1. Check Arduino is detected
ls -la /dev/arduino

# 2. Check BNO085 is responding (run IMU check)
ros2 run Tomas_bot imu_check_node.py

# 3. Check all topics are publishing
ros2 topic hz /wheel/odom     # ~20 Hz from encoders
ros2 topic hz /imu/data        # ~20 Hz from BNO085
ros2 topic hz /odom            # ~30 Hz from EKF
ros2 topic hz /scan            # ~10 Hz from RPLidar

# 4. Check TF tree is complete
ros2 run tf2_tools view_frames

# 5. Check robot model in RViz
ros2 launch Tomas_bot bringup_hardware.launch.py use_rviz:=true
# Verify: imu_link visible as blue rectangle on chassis

# 6. Test motor response
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1}}' --once
# Robot should move forward briefly
```
