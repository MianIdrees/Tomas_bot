#!/usr/bin/env python3
"""
tuning_node.py — Interactive Robot Tuning Script (Branch: final-1)

Approach: Enhanced PID + Adaptive Deadband + PWM Rescaling

This script provides structured tests to tune the robot's motion parameters.
It sends cmd_vel commands directly to test rotation, linear motion, PWM dead zones,
and approach behavior — then helps you adjust parameters for optimal performance.

IMPORTANT: Run bringup_hardware.launch.py FIRST, then run this script.
           This script does NOT replace Nav2 — it helps you find the right parameter
           values to use in diff_drive_node.py and nav2_params_hardware.yaml.

Usage:
    ros2 run Tomas_bot tuning_node.py

The script has 6 chapters:
    1. PWM Dead Zone Detection — Find the minimum PWM that moves your robot
    2. Linear Motion Tuning — Tune straight-line speed and approach behavior
    3. Rotation Tuning — Tune in-place rotation (the most critical for your robot)
    4. Combined Motion — Test curves and arcs
    5. Anti-Spin Verification — Verify spin protection works
    6. Full Parameter Summary — Print all recommended values
"""

import math
import time
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class TuningNode(Node):
    def __init__(self):
        super().__init__('tuning_node')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # Current robot state from odometry
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_v = 0.0
        self.current_w = 0.0
        self.odom_received = False

        # Tuning results storage
        self.results = {}

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # Extract yaw from quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_v = msg.twist.twist.linear.x
        self.current_w = msg.twist.twist.angular.z
        self.odom_received = True

    def send_cmd(self, v, w, duration, show_live=True):
        """Send a cmd_vel command with live telemetry display.

        During execution, prints a real-time table every 0.5s showing:
          Cmd v / Cmd w   = velocity we are COMMANDING
          Odom v / Odom w = velocity the robot is ACTUALLY moving (from encoders)
          Dist            = distance traveled from start position
          ΔYaw            = heading change in degrees

        If 'Odom v' stays at 0.000 while 'Cmd v' > 0 → robot is NOT moving.
        If 'Odom v' tracks 'Cmd v' closely → motors are responding well.
        """
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)

        start = time.time()
        start_x = self.current_x
        start_y = self.current_y
        start_yaw = self.current_yaw
        last_print = -1.0
        rate = self.create_rate(20)

        if show_live:
            print(f'    ┌ LIVE ─────────────────────────────────────────────────────')
            print(f'    │ {"Time":>5s}  {"Cmd v":>6s} {"Cmd w":>6s}  {"Odom v":>6s} {"Odom w":>6s}  {"Dist":>6s}  {"ΔYaw":>7s}')

        while time.time() - start < duration:
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)

            elapsed = time.time() - start
            if show_live and elapsed - last_print >= 0.5:
                dx = self.current_x - start_x
                dy = self.current_y - start_y
                dist = math.sqrt(dx * dx + dy * dy)
                yaw_deg = math.degrees(self.current_yaw - start_yaw)
                while yaw_deg > 180: yaw_deg -= 360
                while yaw_deg < -180: yaw_deg += 360
                print(f'    │ {elapsed:5.1f}s  {v:6.3f} {w:6.3f}  {self.current_v:6.3f} {self.current_w:6.3f}  {dist:5.3f}m  {yaw_deg:+6.1f}°')
                last_print = elapsed

            rate.sleep()

        # Stop
        stop = Twist()
        for _ in range(5):
            self.cmd_pub.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.05)

        if show_live:
            print(f'    └ STOPPED ──────────────────────────────────────────────────')

    def wait_for_odom(self, timeout=5.0):
        """Wait until we receive odometry data."""
        start = time.time()
        while not self.odom_received and time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.odom_received

    def get_input(self, prompt, default=None):
        """Get user input with optional default."""
        if default is not None:
            result = input(f'{prompt} [{default}]: ').strip()
            return result if result else str(default)
        return input(f'{prompt}: ').strip()

    def print_header(self, chapter, title):
        print(f'\n{"=" * 70}')
        print(f'  CHAPTER {chapter}: {title}')
        print(f'{"=" * 70}\n')

    def print_param(self, name, value, lower, upper, description):
        """Print a parameter with its range and description."""
        print(f'  {name}: {value}')
        print(f'    Range: [{lower} .. {upper}]')
        print(f'    Effect: {description}')
        print()

    # ========================== CHAPTER 1: PWM DEAD ZONE ==========================

    def chapter_1_pwm_deadzone(self):
        self.print_header(1, 'PWM DEAD ZONE DETECTION')
        print('  This test finds the minimum PWM that actually moves your robot.')
        print('  The robot will try increasingly higher speeds until it moves.')
        print('  Watch the robot carefully — note the FIRST speed that causes motion.')
        print()
        print('  CURRENT PARAMETERS:')
        print('    min_pwm (diff_drive_node): 40')
        print('    MIN_PWM (Arduino firmware): 40')
        print()

        input('  Press ENTER to start PWM dead zone test (robot will try to move forward)...')

        # Test increasing linear velocities
        test_speeds = [0.02, 0.03, 0.04, 0.05, 0.06, 0.08, 0.10, 0.12, 0.15, 0.20]
        min_moving_speed = None

        for speed in test_speeds:
            print(f'\n  Testing v={speed:.2f} m/s for 1.5s...')
            start_x = self.current_x
            start_y = self.current_y

            self.send_cmd(speed, 0.0, 1.5)
            time.sleep(0.5)

            dx = self.current_x - start_x
            dy = self.current_y - start_y
            dist = math.sqrt(dx*dx + dy*dy)

            if dist > 0.01:  # Moved more than 1cm
                print(f'  → MOVED! (distance: {dist:.3f}m)')
                if min_moving_speed is None:
                    min_moving_speed = speed
            else:
                print(f'  → no movement (distance: {dist:.3f}m)')

            time.sleep(1.0)

        if min_moving_speed:
            # Calculate the PWM this corresponds to
            pwm_per_mps = 255.0 / 0.391
            raw_pwm = int(min_moving_speed * pwm_per_mps)
            print(f'\n  RESULT: Minimum moving speed = {min_moving_speed:.2f} m/s (raw PWM ≈ {raw_pwm})')
            print(f'  RECOMMENDATION:')
            recommended_min = max(raw_pwm + 5, 40)
            print(f'    min_pwm = {recommended_min}  (raw_pwm + 5 safety margin)')
            print(f'    rotation_min_pwm = {recommended_min + 15}  (needs more torque to start rotation)')
            self.results['min_pwm'] = recommended_min
            self.results['rotation_min_pwm'] = recommended_min + 15
        else:
            print('\n  WARNING: Robot did not move at any test speed!')
            print('  Check: Arduino connected? Motors powered? Wheels on ground?')
            self.results['min_pwm'] = 60
            self.results['rotation_min_pwm'] = 75

        print()
        self.print_param('min_pwm', self.results.get('min_pwm', 40),
                         25, 80,
                         'Minimum PWM for linear motion. Increase if robot stalls at low speeds. '
                         'Decrease if robot jerks when starting. Must match Arduino MIN_PWM.')
        self.print_param('rotation_min_pwm', self.results.get('rotation_min_pwm', 55),
                         40, 120,
                         'Minimum PWM for in-place rotation. Higher than min_pwm because starting '
                         'rotation from standstill needs more torque. Increase if robot fails to rotate.')

    # ========================== CHAPTER 2: LINEAR MOTION ==========================

    def chapter_2_linear_motion(self):
        self.print_header(2, 'LINEAR MOTION TUNING')
        print('  Tests straight-line motion at different speeds.')
        print('  Focus on: Does it move smoothly? Does it drift sideways?')
        print()

        input('  Press ENTER to test linear motion...')

        # Test forward motion at different speeds
        speeds = [0.05, 0.10, 0.15, 0.20]
        for speed in speeds:
            print(f'\n  Testing forward at v={speed:.2f} m/s for 3 seconds...')
            input(f'    Press ENTER to start...')

            start_x = self.current_x
            start_y = self.current_y
            start_yaw = self.current_yaw

            self.send_cmd(speed, 0.0, 3.0)
            time.sleep(1.0)

            dx = self.current_x - start_x
            dy = self.current_y - start_y
            dist = math.sqrt(dx*dx + dy*dy)
            yaw_drift = math.degrees(self.current_yaw - start_yaw)

            expected = speed * 3.0
            print(f'    Distance traveled: {dist:.3f}m (expected ~{expected:.3f}m)')
            print(f'    Heading drift: {yaw_drift:.1f}° (ideal: 0°)')
            print(f'    Speed accuracy: {(dist/expected)*100:.0f}%')

        print(f'\n  LINEAR TUNING PARAMETERS:')
        print()
        self.print_param('approach_min_linear_speed', 0.04,
                         0.02, 0.10,
                         'Floor for linear velocity commands. When Nav2 slows down near the goal, '
                         'this prevents the speed from dropping below motor dead zone. '
                         'Increase if robot stalls near goal. Decrease if it overshoots.')
        self.print_param('linear_deadband', 0.005,
                         0.001, 0.02,
                         'Minimum linear velocity to pass through (below this → treated as zero). '
                         'Very low to ensure even tiny forward commands reach the motors. '
                         'Increase only if motors vibrate/hum without moving at tiny speeds.')
        self.print_param('pwm_ramp_rate', 40,
                         10, 80,
                         'Max PWM change per control cycle (20Hz). Controls acceleration smoothness. '
                         'Lower = smoother but sluggish. Higher = responsive but jerky. '
                         'At 40: takes ~0.3s to reach full speed. At 20: ~0.6s. At 80: ~0.15s.')
        self.print_param('use_pwm_rescaling', True,
                         False, True,
                         'When True: maps entire PWM range above dead zone (smoother speed control). '
                         'When False: just clamps low PWM to min_pwm (simpler, on/off at low speeds). '
                         'Rescaling recommended for Nav2 since it sends many small velocity adjustments.')

    # ========================== CHAPTER 3: ROTATION ==========================

    def chapter_3_rotation(self):
        self.print_header(3, 'ROTATION TUNING (MOST CRITICAL)')
        print('  This is the key test for your robot. Tests in-place rotation at')
        print('  different angular velocities. Watch for:')
        print('    - Does the robot actually rotate?')
        print('    - Does it over-rotate (spin past target)?')
        print('    - Does it oscillate (wobble back and forth)?')
        print()

        input('  Press ENTER to start rotation tests...')

        # Test counterclockwise rotation at different angular velocities
        angular_speeds = [0.2, 0.3, 0.4, 0.5, 0.6, 0.8]
        for w in angular_speeds:
            target_deg = 90.0
            duration = math.radians(target_deg) / w

            print(f'\n  Testing CCW rotation: w={w:.1f} rad/s, target=90°, duration={duration:.1f}s')
            input(f'    Press ENTER to start...')

            start_yaw = self.current_yaw

            self.send_cmd(0.0, w, duration)
            time.sleep(1.0)

            actual_deg = math.degrees(self.current_yaw - start_yaw)
            # Normalize to [-180, 180]
            while actual_deg > 180: actual_deg -= 360
            while actual_deg < -180: actual_deg += 360

            error = actual_deg - target_deg
            print(f'    Actual rotation: {actual_deg:.1f}° (target: {target_deg:.0f}°, error: {error:+.1f}°)')

            if abs(actual_deg) < 10:
                print(f'    ⚠ PROBLEM: Robot barely rotated! Increase rotation_boost_factor or rotation_min_pwm')
            elif abs(error) > 45:
                print(f'    ⚠ PROBLEM: Large error! Decrease rotation_boost_factor if over-rotating')
            else:
                print(f'    ✓ Acceptable rotation')

        # Test clockwise
        print(f'\n  Testing CW rotation: w=-0.5 rad/s, target=-90°, duration=3.1s')
        input(f'    Press ENTER to start...')
        start_yaw = self.current_yaw
        self.send_cmd(0.0, -0.5, 3.14)
        time.sleep(1.0)
        actual_deg = math.degrees(self.current_yaw - start_yaw)
        while actual_deg > 180: actual_deg -= 360
        while actual_deg < -180: actual_deg += 360
        print(f'    Actual rotation: {actual_deg:.1f}° (target: -90°)')

        print(f'\n  ROTATION TUNING PARAMETERS:')
        print()
        self.print_param('rotation_boost_factor', 1.35,
                         1.0, 2.0,
                         'Multiplier applied to PWM during pure in-place rotation (no linear velocity). '
                         'A 2.4kg robot needs extra torque to overcome friction when spinning in place. '
                         'At 1.0: no boost. At 1.35: 35% more PWM. At 2.0: double PWM. '
                         'If robot doesn\'t rotate on sharp turns → increase (try 1.5, 1.7). '
                         'If robot over-rotates or spins → decrease (try 1.1, 1.2).')
        self.print_param('rotation_min_pwm', 55,
                         40, 120,
                         'Absolute minimum PWM for rotation commands — higher than linear min_pwm. '
                         'Starting rotation from standstill needs more torque than maintaining motion. '
                         'If robot doesn\'t start rotating → increase (try 65, 75, 85). '
                         'If robot jerks violently when starting a turn → decrease. '
                         'CONSTRAINT: Must be >= min_pwm.')
        self.print_param('angular_deadband', 0.02,
                         0.005, 0.10,
                         'Angular velocity below this threshold is treated as zero (filters jitter). '
                         'At 0.02 rad/s: very small jitter filtered, most turn commands pass through. '
                         'If robot twitches constantly → increase to 0.05. '
                         'If robot ignores turn commands → decrease to 0.01.')
        self.print_param('max_continuous_rotation_deg', 270,
                         90, 540,
                         'Anti-spin limit: max degrees of continuous same-direction rotation. '
                         'After this, angular velocity is clamped to prevent runaway 360° spins. '
                         'For normal navigation, 270° covers U-turns with margin. '
                         'If robot needs full 360° turns → set to 400+. '
                         'If robot frequently spins out → decrease to 180.')
        self.print_param('anti_spin_angular_clamp', 0.3,
                         0.0, 0.5,
                         'When anti-spin triggers, angular velocity is clamped to this value. '
                         'At 0.0: rotation stops completely when limit hit (safest). '
                         'At 0.3: allows gentle turns even after anti-spin triggers. '
                         'Increase if robot needs to complete > 270° turns without stopping.')

    # ========================== CHAPTER 4: COMBINED MOTION ==========================

    def chapter_4_combined_motion(self):
        self.print_header(4, 'COMBINED MOTION (CURVES)')
        print('  Tests motion with both linear and angular velocity (arcs/curves).')
        print('  This simulates what Nav2 commands during path following.')
        print()

        input('  Press ENTER to test arc motion...')

        # Test gentle curve
        print('\n  Test 1: Gentle left curve (v=0.10, w=0.3) for 4s')
        input('    Press ENTER...')
        start_x, start_y = self.current_x, self.current_y
        self.send_cmd(0.10, 0.3, 4.0)
        time.sleep(1.0)
        dist = math.sqrt((self.current_x - start_x)**2 + (self.current_y - start_y)**2)
        print(f'    Displacement: {dist:.3f}m')

        # Test sharp curve
        print('\n  Test 2: Sharp left curve (v=0.05, w=0.6) for 3s')
        input('    Press ENTER...')
        start_x, start_y = self.current_x, self.current_y
        self.send_cmd(0.05, 0.6, 3.0)
        time.sleep(1.0)
        dist = math.sqrt((self.current_x - start_x)**2 + (self.current_y - start_y)**2)
        print(f'    Displacement: {dist:.3f}m')

        # Test right curve
        print('\n  Test 3: Gentle right curve (v=0.10, w=-0.3) for 4s')
        input('    Press ENTER...')
        self.send_cmd(0.10, -0.3, 4.0)
        time.sleep(1.0)

        print('\n  If the robot moved smoothly on curves → current settings are good.')
        print('  If it stuttered or stopped → reduce rotation_boost_factor (only applies to PURE rotation).')

    # ========================== CHAPTER 5: ANTI-SPIN ==========================

    def chapter_5_anti_spin(self):
        self.print_header(5, 'ANTI-SPIN VERIFICATION')
        print('  This test commands continuous rotation to verify anti-spin protection.')
        print('  The robot should stop (or slow down) after ~270° of rotation.')
        print()

        input('  Press ENTER to test anti-spin (robot will rotate continuously)...')

        print('\n  Commanding w=0.5 rad/s for 15 seconds (would be ~430° without protection)...')
        print('  Watch: robot should slow/stop after ~270° (anti-spin limit)')

        start_yaw = self.current_yaw
        msg = Twist()
        msg.angular.z = 0.5
        start = time.time()
        rate = self.create_rate(20)

        print(f'    ┌ LIVE ─────────────────────────────────────────────────────')
        print(f'    │ {"Time":>5s}  {"Cmd w":>6s}  {"Odom w":>6s}  {"ΔYaw":>7s}  Status')
        last_print = -1.0

        while time.time() - start < 15.0:
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)

            elapsed = time.time() - start
            if elapsed - last_print >= 1.0:
                yaw_deg = math.degrees(self.current_yaw - start_yaw)
                while yaw_deg > 180: yaw_deg -= 360
                while yaw_deg < -180: yaw_deg += 360
                status = ''
                if abs(yaw_deg) > 200:
                    status = '← approaching anti-spin limit (270°)'
                if abs(self.current_w) < 0.05 and elapsed > 3.0:
                    status = '← MOTORS STOPPED (anti-spin triggered!)'
                print(f'    │ {elapsed:5.1f}s  {0.5:6.3f}  {self.current_w:6.3f}  {yaw_deg:+6.1f}°  {status}')
                last_print = elapsed

            rate.sleep()

        print(f'    └ STOPPED ──────────────────────────────────────────────────')

        # Stop
        stop = Twist()
        for _ in range(5):
            self.cmd_pub.publish(stop)
            time.sleep(0.05)

        print('  Test complete. Check terminal output for anti-spin trigger messages.')
        print('  If you see "Anti-spin triggered" → protection is working.')
        print('  If robot kept spinning without limit → check max_continuous_rotation_deg.')

    # ========================== CHAPTER 6: SUMMARY ==========================

    def chapter_6_summary(self):
        self.print_header(6, 'FULL PARAMETER SUMMARY')
        print('  Below are ALL tunable parameters with their recommended values,')
        print('  valid ranges, and effects. Adjust based on your test results above.')
        print()

        print('  ─── diff_drive_node.py Parameters ───')
        print()
        self.print_param('min_pwm', self.results.get('min_pwm', 40),
                         25, 80,
                         'Minimum PWM for linear motion. MUST match Arduino MIN_PWM. '
                         'Increase → more torque at low speeds (prevents stalling). '
                         'Decrease → finer low-speed control (risk of stalling).')
        self.print_param('rotation_min_pwm', self.results.get('rotation_min_pwm', 55),
                         40, 120,
                         'Minimum PWM for in-place rotation. Must be >= min_pwm. '
                         'Increase → robot can rotate from standstill on heavy loads. '
                         'Decrease → gentler rotation start (risk of not rotating).')
        self.print_param('rotation_boost_factor', 1.35,
                         1.0, 2.0,
                         'PWM multiplier for pure rotation commands. '
                         'Increase → more rotation torque. Decrease → less overcorrection.')
        self.print_param('angular_deadband', 0.02,
                         0.005, 0.10,
                         'Angular jitter filter threshold (rad/s). '
                         'Increase → less twitching, but may ignore small turns. '
                         'Decrease → more responsive to small commands.')
        self.print_param('linear_deadband', 0.005,
                         0.001, 0.02,
                         'Linear jitter filter threshold (m/s). '
                         'Keep very low to pass Nav2 approach velocities through.')
        self.print_param('pwm_ramp_rate', 40,
                         10, 80,
                         'Max PWM change per 50ms cycle. '
                         'Lower → smoother (slower response). Higher → snappier (jerkier).')
        self.print_param('approach_min_linear_speed', 0.04,
                         0.02, 0.10,
                         'Minimum linear speed floor. Prevents stalling near goal. '
                         'Increase → robot reaches goal faster. Decrease → more precise stopping.')
        self.print_param('max_continuous_rotation_deg', 270,
                         90, 540,
                         'Anti-spin limit in degrees. '
                         'Lower → safer against spins. Higher → allows larger turns.')
        self.print_param('anti_spin_angular_clamp', 0.3,
                         0.0, 0.5,
                         'Clamped angular velocity when anti-spin triggers. '
                         '0.0 = full stop. 0.3 = gentle rotation allowed.')
        self.print_param('use_pwm_rescaling', True,
                         False, True,
                         'True = smooth rescaled PWM. False = simple min-PWM clamping.')

        print('  ─── Arduino Firmware Parameters (motor_controller.ino) ───')
        print()
        self.print_param('MIN_PWM', self.results.get('min_pwm', 40),
                         25, 80,
                         'MUST match diff_drive_node.py min_pwm. Dead zone compensation in PID. '
                         'If these don\'t match, the Arduino PID will fight the ROS2 node.')
        self.print_param('KP', 1.0,
                         0.5, 3.0,
                         'Proportional gain. Increase → faster error correction. '
                         'Too high → oscillation. Too low → sluggish response.')
        self.print_param('KI', 0.8,
                         0.0, 2.0,
                         'Integral gain. Eliminates steady-state error. '
                         'Increase → better speed tracking. Too high → overshoot/windup.')
        self.print_param('KD', 0.15,
                         0.0, 1.0,
                         'Derivative gain. Dampens overshoot. '
                         'Increase → less oscillation. Too high → noisy/jerky response.')
        self.print_param('RAMP_RATE', 10.0,
                         2.0, 30.0,
                         'Target ticks/interval ramping rate. '
                         'Lower → smoother acceleration. Higher → snappier response.')
        self.print_param('INTEGRAL_LIMIT', 150.0,
                         50.0, 300.0,
                         'Max integral accumulation. Prevents windup. '
                         'Increase → better steady-state. Decrease → less overshoot risk.')

        print('  ─── Nav2 Parameters (nav2_params_hardware.yaml) ───')
        print()
        self.print_param('rotate_to_heading_angular_vel', 0.6,
                         0.3, 1.2,
                         'Angular velocity for in-place rotation commands from Nav2. '
                         'Lower → gentler turns (may stall). Higher → faster turns (may overshoot).')
        self.print_param('rotate_to_heading_min_angle', 0.5,
                         0.2, 1.0,
                         'Minimum heading error (rad) to trigger in-place rotation. '
                         'Lower → more frequent rotations. Higher → only big errors trigger rotate.')
        self.print_param('min_approach_linear_velocity', 0.04,
                         0.02, 0.10,
                         'Nav2 minimum approach speed. Works with approach_min_linear_speed. '
                         'Increase → reaches goal faster. Decrease → more precision but risks stalling.')
        self.print_param('max_angular_accel', 0.6,
                         0.3, 1.5,
                         'Nav2 angular acceleration limit. '
                         'Lower → gentler rotation start/stop. Higher → snappier.')
        self.print_param('velocity_smoother.deadband_velocity[2]', 0.02,
                         0.01, 0.10,
                         'Angular deadband in velocity smoother. '
                         'Lower → passes more small commands. Higher → filters more jitter.')

    def run(self):
        """Main interactive tuning session."""
        print()
        print('╔══════════════════════════════════════════════════════════════════════╗')
        print('║        TOMAS_BOT TUNING SCRIPT — Branch: final-1                   ║')
        print('║        Enhanced PID + Adaptive Deadband + PWM Rescaling             ║')
        print('╚══════════════════════════════════════════════════════════════════════╝')
        print()
        print('  This script helps you tune the robot for smooth autonomous navigation.')
        print('  It runs interactive tests and shows LIVE sensor data so you can see')
        print('  exactly what the robot is doing in real-time.')
        print()
        print('  HOW IT WORKS:')
        print('    1. Pick a chapter from the menu (or run all in order)')
        print('    2. Each chapter has multiple tests — press ENTER to start each one')
        print('    3. During each test, a LIVE table updates every 0.5s showing:')
        print('         Cmd v/w  = velocity we are COMMANDING the robot')
        print('         Odom v/w = velocity robot is ACTUALLY moving (from encoders)')
        print('         Dist     = how far the robot has traveled')
        print('         ΔYaw     = heading change in degrees')
        print('    4. KEY: If "Odom v" stays at 0.000 while "Cmd v" > 0 → NOT moving!')
        print('    5. After each test, results are shown automatically')
        print('    6. After all tests, a summary shows recommended parameter values')
        print('    7. You then update values in diff_drive_node.py / nav2_params.yaml')
        print()
        print('  Prerequisites:')
        print('    1. Robot hardware powered on')
        print('    2. ros2 launch Tomas_bot bringup_hardware.launch.py running')
        print('    3. Robot on the ground with space to move (~2m around it)')
        print()

        if not self.wait_for_odom(timeout=10.0):
            print('  ERROR: No odometry data received after 10 seconds!')
            print('  Make sure bringup_hardware.launch.py is running.')
            return

        print('  ✓ Odometry data received. Robot is ready.')
        print()

        chapters = [
            ('1', 'PWM Dead Zone Detection', self.chapter_1_pwm_deadzone),
            ('2', 'Linear Motion Tuning', self.chapter_2_linear_motion),
            ('3', 'Rotation Tuning', self.chapter_3_rotation),
            ('4', 'Combined Motion', self.chapter_4_combined_motion),
            ('5', 'Anti-Spin Verification', self.chapter_5_anti_spin),
            ('6', 'Full Parameter Summary', self.chapter_6_summary),
        ]

        while True:
            print('\n  Available chapters:')
            for num, name, _ in chapters:
                print(f'    {num}. {name}')
            print('    a. Run ALL chapters in order')
            print('    q. Quit')
            print()

            choice = input('  Select chapter (1-6, a, or q): ').strip().lower()

            if choice == 'q':
                print('\n  Stopping motors and exiting...')
                stop = Twist()
                for _ in range(10):
                    self.cmd_pub.publish(stop)
                    time.sleep(0.05)
                break
            elif choice == 'a':
                for _, _, func in chapters:
                    func()
                    input('\n  Press ENTER to continue to next chapter...')
            else:
                for num, _, func in chapters:
                    if choice == num:
                        func()
                        break
                else:
                    print('  Invalid choice.')


def main(args=None):
    rclpy.init(args=args)
    node = TuningNode()
    try:
        node.run()
    except KeyboardInterrupt:
        # Stop motors on Ctrl+C
        stop = Twist()
        for _ in range(10):
            node.cmd_pub.publish(stop)
            time.sleep(0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
