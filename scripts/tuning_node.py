#!/usr/bin/env python3
"""
tuning_node.py — Fully Guided Robot Tuning Script (Branch: final-1)

Approach: Enhanced PID + Adaptive Deadband + PWM Rescaling

This script walks you through every step of tuning your robot. During each test:
  - You see LIVE data showing commanded vs actual speed
  - The script tells you exactly what to look for
  - It picks the BEST value automatically from test results
  - At the end, it tells you EXACTLY which files to edit and what to change

IMPORTANT: Run bringup_hardware.launch.py FIRST in a separate terminal.

Usage:
    Terminal 1:  ros2 launch Tomas_bot bringup_hardware.launch.py
    Terminal 2:  ros2 run Tomas_bot tuning_node.py
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class TuningNode(Node):

    def __init__(self):
        super().__init__('tuning_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)

        self.x = self.y = self.yaw = 0.0
        self.odom_v = self.odom_w = 0.0
        self.odom_ok = False

        # Collected best values across all chapters
        self.best = {}

    # ------------------------------------------------------------------ helpers
    def _odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom_v = msg.twist.twist.linear.x
        self.odom_w = msg.twist.twist.angular.z
        self.odom_ok = True

    def _wait_odom(self, timeout=10.0):
        t0 = time.time()
        while not self.odom_ok and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.odom_ok

    def _send(self, v, w, duration):
        """Send cmd_vel and print live telemetry table."""
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)

        x0, y0, yaw0 = self.x, self.y, self.yaw
        t0 = time.time()
        last_print = -1.0
        rate = self.create_rate(20)

        hdr = (f'    │ {"Time":>5s}  {"Cmd v":>6s} {"Cmd w":>6s} │ '
               f'{"Odom v":>6s} {"Odom w":>6s} │ {"Dist":>6s}  {"ΔYaw":>7s} │ Status')
        print(f'    ┌─────────────────────────────────────────────────────────────────────')
        print(hdr)
        print(f'    ├─────────────────────────────────────────────────────────────────────')

        while time.time() - t0 < duration:
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)
            elapsed = time.time() - t0

            if elapsed - last_print >= 0.5:
                dist = math.hypot(self.x - x0, self.y - y0)
                dyaw = math.degrees(self.yaw - yaw0)
                while dyaw >  180: dyaw -= 360
                while dyaw < -180: dyaw += 360
                # Status indicator
                if abs(v) > 0.001 and abs(self.odom_v) < 0.005 and elapsed > 0.8:
                    st = '⚠ NOT MOVING'
                elif abs(w) > 0.001 and abs(self.odom_w) < 0.01 and elapsed > 0.8:
                    st = '⚠ NOT ROTATING'
                elif abs(v) > 0.001 and abs(self.odom_v) > 0.005:
                    st = '✓ moving'
                elif abs(w) > 0.001 and abs(self.odom_w) > 0.01:
                    st = '✓ rotating'
                else:
                    st = ''
                print(f'    │ {elapsed:5.1f}s  {v:6.3f} {w:6.3f} │ '
                      f'{self.odom_v:6.3f} {self.odom_w:6.3f} │ '
                      f'{dist:5.3f}m  {dyaw:+6.1f}° │ {st}')
                last_print = elapsed
            rate.sleep()

        # Stop
        stop = Twist()
        for _ in range(5):
            self.cmd_pub.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.05)

        # Final measurement
        dist = math.hypot(self.x - x0, self.y - y0)
        dyaw = math.degrees(self.yaw - yaw0)
        while dyaw >  180: dyaw -= 360
        while dyaw < -180: dyaw += 360
        print(f'    └── STOPPED ── Final: dist={dist:.3f}m  ΔYaw={dyaw:+.1f}°')
        return dist, dyaw

    def _hdr(self, ch, title):
        print(f'\n{"═" * 72}')
        print(f'  CHAPTER {ch}: {title}')
        print(f'{"═" * 72}')

    def _box(self, lines):
        w = max(len(l) for l in lines) + 4
        print(f'\n    ┌{"─" * w}┐')
        for l in lines:
            print(f'    │  {l:<{w-2}}│')
        print(f'    └{"─" * w}┘')

    # ====================================================================
    # CHAPTER 1 — PWM Dead Zone
    # ====================================================================
    def ch1_deadzone(self):
        self._hdr(1, 'PWM DEAD ZONE — Find minimum speed that moves the robot')
        print('''
    WHAT THIS TEST DOES:
      The robot will try to move forward at increasing speeds.
      For each speed, the LIVE table shows if the robot actually moved.
      We need to find the LOWEST speed where the robot moves reliably.

    WHAT TO LOOK FOR in the live table:
      - "⚠ NOT MOVING" in Status column = speed too low, motor dead zone
      - "✓ moving" in Status column      = motor is responding
      - "Odom v" column shows actual speed from wheel encoders
        If Odom v stays 0.000 while Cmd v > 0 → motors are stalled

    WHY THIS MATTERS:
      Your 2.4kg robot needs a minimum PWM to overcome friction.
      Below this PWM, motors hum but wheels don't turn.
      We find this threshold so the controller never sends useless commands.
''')
        input('    Press ENTER to begin (robot will try small forward movements)...\n')

        test_speeds = [0.02, 0.03, 0.04, 0.05, 0.06, 0.08, 0.10, 0.12, 0.15]
        results = []

        for spd in test_speeds:
            print(f'\n    ── Test: commanding v={spd:.2f} m/s for 1.5 seconds ──')
            dist, _ = self._send(spd, 0.0, 1.5)
            time.sleep(0.8)

            moved = dist > 0.008
            pwm_approx = int(spd / 0.391 * 255)
            results.append((spd, dist, moved, pwm_approx))

            if moved:
                print(f'    ➜ RESULT: MOVED {dist:.3f}m  (this speed ≈ PWM {pwm_approx})')
            else:
                print(f'    ➜ RESULT: DID NOT MOVE      (this speed ≈ PWM {pwm_approx})')

        # Pick best value
        moved_list = [(s, d, p) for s, d, m, p in results if m]
        not_moved = [(s, d, p) for s, d, m, p in results if not m]

        if moved_list:
            min_speed, _, raw_pwm = moved_list[0]
            best_min_pwm = max(raw_pwm + 5, 40)
            best_rot_min = best_min_pwm + 15
        else:
            best_min_pwm = 60
            best_rot_min = 75

        self.best['min_pwm'] = best_min_pwm
        self.best['rotation_min_pwm'] = best_rot_min

        self._box([
            'CHAPTER 1 RESULTS',
            '',
            'Speed  │  Distance │ PWM ≈ │ Moved?',
            '───────┼───────────┼───────┼───────',
        ] + [
            f'{s:.2f}   │  {d:.3f}m   │  {p:3d}  │ {"YES ✓" if m else "NO  ✗"}'
            for s, d, m, p in results
        ] + [
            '',
            f'First speed that moved: {moved_list[0][0]:.2f} m/s (PWM ≈ {moved_list[0][2]})' if moved_list
            else 'Robot did NOT move at any speed! Check hardware.',
            '',
            '>>> BEST VALUES TO NOTE:',
            f'    min_pwm         = {best_min_pwm}  (dead zone + 5 safety margin)',
            f'    rotation_min_pwm = {best_rot_min}  (min_pwm + 15 for rotation torque)',
            '',
            'WHAT THESE MEAN:',
            f'  min_pwm: any motor command below PWM {best_min_pwm} is wasted energy.',
            f'  rotation_min_pwm: rotation from standstill needs even more ({best_rot_min}).',
        ])

    # ====================================================================
    # CHAPTER 2 — Linear Motion
    # ====================================================================
    def ch2_linear(self):
        self._hdr(2, 'LINEAR MOTION — Tune straight-line speed & goal approach')
        print('''
    WHAT THIS TEST DOES:
      Sends forward commands at different speeds for 3 seconds each.
      Measures actual distance vs expected distance.

    WHAT TO LOOK FOR in the live table:
      - "Odom v" should be close to "Cmd v" = good speed tracking
      - "Dist" should grow steadily = smooth motion
      - "ΔYaw" should stay near 0° = going straight

    AFTER EACH TEST:
      - "Speed accuracy" near 100% = motor responds well to this speed
      - Low accuracy at low speeds = robot stalls → need higher floor speed

    WHY THIS MATTERS:
      Nav2 slows down near the goal. If motors stall at low speeds,
      the robot stops before reaching the goal. We find the minimum
      speed that still gives good accuracy.
''')
        input('    Press ENTER to begin linear tests...\n')

        speeds = [0.04, 0.06, 0.10, 0.15, 0.20]
        results = []

        for spd in speeds:
            expected_dist = spd * 3.0
            print(f'\n    ── Test: v={spd:.2f} m/s for 3s (expected ≈ {expected_dist:.3f}m) ──')
            input(f'    Press ENTER to run this test...')
            dist, dyaw = self._send(spd, 0.0, 3.0)
            time.sleep(1.0)

            accuracy = (dist / expected_dist * 100) if expected_dist > 0 else 0
            error_m = expected_dist - dist
            results.append((spd, dist, expected_dist, accuracy, error_m, dyaw))

            print(f'\n    ➜ RESULT for v={spd:.2f}:')
            print(f'       Expected distance : {expected_dist:.3f}m')
            print(f'       Actual distance   : {dist:.3f}m')
            print(f'       Error             : {error_m:+.3f}m')
            print(f'       Speed accuracy    : {accuracy:.1f}%')
            print(f'       Heading drift     : {dyaw:+.1f}° (ideal: 0°)')
            if accuracy > 80:
                print(f'       ✓ GOOD — this speed works well')
            elif accuracy > 50:
                print(f'       ⚠ FAIR — some speed loss, usable')
            else:
                print(f'       ✗ POOR — robot struggles at this speed')

        # Find the minimum speed with >70% accuracy → approach_min_linear_speed
        good_speeds = [s for s, d, e, a, em, y in results if a > 70]
        best_approach = good_speeds[0] if good_speeds else 0.06
        self.best['approach_min_linear_speed'] = best_approach

        self._box([
            'CHAPTER 2 RESULTS',
            '',
            'Speed  │ Expected │ Actual  │ Error    │ Accuracy │ Drift  │ Verdict',
            '───────┼──────────┼─────────┼──────────┼──────────┼────────┼────────',
        ] + [
            f'{s:.2f}   │ {e:.3f}m  │ {d:.3f}m │ {em:+.3f}m  │ {a:5.1f}%  │ {y:+5.1f}° │ '
            + ('✓ GOOD' if a > 80 else '⚠ FAIR' if a > 50 else '✗ POOR')
            for s, d, e, a, em, y in results
        ] + [
            '',
            '>>> BEST VALUE TO NOTE:',
            f'    approach_min_linear_speed = {best_approach}',
            '',
            'WHAT THIS MEANS:',
            f'  The lowest speed where accuracy > 70%. Below {best_approach} m/s,',
            '  the robot loses too much speed to friction. Nav2 will not',
            '  be allowed to command speeds below this near the goal.',
        ])

    # ====================================================================
    # CHAPTER 3 — Rotation (MOST CRITICAL)
    # ====================================================================
    def ch3_rotation(self):
        self._hdr(3, 'ROTATION — Tune in-place turns (MOST CRITICAL TEST)')
        print('''
    WHAT THIS TEST DOES:
      Commands the robot to rotate exactly 90° at different angular speeds.
      Measures how much it ACTUALLY rotated.

    WHAT TO LOOK FOR in the live table:
      - "Odom w" should be non-zero = robot is actually rotating
      - "ΔYaw" should climb toward +90° = heading is changing
      - "⚠ NOT ROTATING" = motor can't overcome friction → need more power

    AFTER EACH TEST:
      - Error close to 0° = perfect rotation
      - Large negative error (e.g. -60°) = UNDER-rotated → need more boost
      - Large positive error (e.g. +50°) = OVER-rotated → need less boost

    WHY THIS MATTERS:
      This is the #1 problem. Nav2 says "turn 90°" but the robot barely
      moves because 2.4kg is too heavy for the small motors. The
      rotation_boost_factor multiplies PWM during pure rotation to fix this.
''')
        input('    Press ENTER to begin rotation tests...\n')

        angular_speeds = [0.2, 0.3, 0.4, 0.5, 0.6, 0.8]
        results = []
        target = 90.0

        for w in angular_speeds:
            duration = math.radians(target) / w
            print(f'\n    ── Test: rotate CCW at w={w:.1f} rad/s for {duration:.1f}s (target: 90°) ──')
            input(f'    Press ENTER to run this test...')
            _, dyaw = self._send(0.0, w, duration)
            time.sleep(1.0)

            error = dyaw - target
            error_pct = (error / target) * 100
            results.append((w, dyaw, error, error_pct))

            print(f'\n    ➜ RESULT for w={w:.1f} rad/s:')
            print(f'       Target rotation   : +90.0°')
            print(f'       Actual rotation   : {dyaw:+.1f}°')
            print(f'       Error             : {error:+.1f}° ({error_pct:+.1f}%)')
            if abs(dyaw) < 15:
                print(f'       ✗ FAILED — robot barely rotated!')
                print(f'         → rotation_boost_factor is too low, or rotation_min_pwm too low')
            elif abs(error) < 20:
                print(f'       ✓ GOOD — close to 90° target')
            elif error > 20:
                print(f'       ⚠ OVER-ROTATED by {error:.0f}°')
                print(f'         → decrease rotation_boost_factor')
            else:
                print(f'       ⚠ UNDER-ROTATED by {abs(error):.0f}°')
                print(f'         → increase rotation_boost_factor or rotation_min_pwm')

        # Test clockwise too
        print(f'\n    ── Test: rotate CW (clockwise) at w=-0.5 rad/s, target: -90° ──')
        input('    Press ENTER...')
        _, cw_dyaw = self._send(0.0, -0.5, math.radians(90) / 0.5)
        cw_error = cw_dyaw - (-90.0)
        print(f'\n    ➜ CW RESULT: actual={cw_dyaw:+.1f}° target=-90° error={cw_error:+.1f}°')

        # Automatically recommend boost factor
        best_w_entry = min(results, key=lambda r: abs(r[2]))
        best_speed, best_actual = best_w_entry[0], best_w_entry[1]
        avg_error_pct = sum(r[3] for r in results) / len(results)
        current_boost = 1.35

        if avg_error_pct < -30:
            rec_boost = min(current_boost + 0.2, 2.0)
            boost_note = 'Robot under-rotates on average → INCREASE boost'
        elif avg_error_pct > 30:
            rec_boost = max(current_boost - 0.15, 1.0)
            boost_note = 'Robot over-rotates on average → DECREASE boost'
        else:
            rec_boost = current_boost
            boost_note = 'Current boost (1.35) is working well — keep it'

        self.best['rotation_boost_factor'] = rec_boost

        self._box([
            'CHAPTER 3 RESULTS',
            '',
            'ω (rad/s) │ Target │ Actual  │ Error   │ Error %  │ Verdict',
            '──────────┼────────┼─────────┼─────────┼──────────┼────────',
        ] + [
            f'  {w:.1f}     │ +90.0° │ {a:+6.1f}° │ {e:+6.1f}° │ {ep:+6.1f}% │ '
            + ('✓ GOOD' if abs(e) < 20 else '✗ UNDER' if e < -20 else '⚠ OVER')
            for w, a, e, ep in results
        ] + [
            '',
            f'Clockwise test: actual={cw_dyaw:+.1f}° error={cw_error:+.1f}°',
            '',
            f'Best angular speed: {best_speed:.1f} rad/s (closest to 90°: {best_actual:+.1f}°)',
            f'Average error across all speeds: {avg_error_pct:+.1f}%',
            '',
            f'{boost_note}',
            '',
            '>>> BEST VALUE TO NOTE:',
            f'    rotation_boost_factor = {rec_boost:.2f}',
            '',
            'HOW TO INTERPRET:',
            '  The boost factor multiplies PWM during pure in-place rotation.',
            f'  At {rec_boost:.2f}: rotation gets {rec_boost*100-100:.0f}% more PWM than normal.',
            '  If still under-rotating after change → also increase rotation_min_pwm.',
        ])

    # ====================================================================
    # CHAPTER 4 — Combined Motion (Arcs)
    # ====================================================================
    def ch4_arcs(self):
        self._hdr(4, 'ARC MOTION — Test curves like Nav2 path following')
        print('''
    WHAT THIS TEST DOES:
      Sends combined forward + turn commands (curves/arcs).
      This simulates what Nav2 sends during normal path following.

    WHAT TO LOOK FOR in the live table:
      - Both "Odom v" and "Odom w" should be non-zero = moving + turning
      - "Dist" should increase = robot is actually moving forward
      - "ΔYaw" should change = robot is also turning

    WHY THIS MATTERS:
      Nav2 rarely commands pure rotation — mostly it sends arcs.
      The rotation_boost_factor only applies to PURE rotation (w only).
      Arcs use normal PWM, so if arcs fail → check min_pwm, not boost.
''')
        input('    Press ENTER to begin arc tests...\n')

        arcs = [
            (0.10,  0.3, 4.0, 'Gentle LEFT curve'),
            (0.05,  0.6, 3.0, 'Sharp LEFT curve (like a tight corner)'),
            (0.10, -0.3, 4.0, 'Gentle RIGHT curve'),
        ]

        for v, w, dur, desc in arcs:
            print(f'\n    ── Test: {desc} (v={v}, w={w}) for {dur}s ──')
            input('    Press ENTER...')
            dist, dyaw = self._send(v, w, dur)
            time.sleep(1.0)
            print(f'    ➜ Displacement: {dist:.3f}m, heading change: {dyaw:+.1f}°')
            if dist < 0.02:
                print(f'       ✗ Robot barely moved! → increase min_pwm')
            else:
                print(f'       ✓ Arc motion working')

        self._box([
            'CHAPTER 4 NOTE:',
            '',
            'If arcs worked smoothly → PWM rescaling is doing its job.',
            'If robot stalled on sharp curves → the issue is min_pwm (too low),',
            'NOT rotation_boost_factor (which only helps pure rotation).',
        ])

    # ====================================================================
    # CHAPTER 5 — Anti-Spin Verification
    # ====================================================================
    def ch5_antispin(self):
        self._hdr(5, 'ANTI-SPIN — Verify 360° spin protection')
        print('''
    WHAT THIS TEST DOES:
      Commands continuous rotation at 0.5 rad/s for 15 seconds.
      Without protection, that would be ~430° of rotation.
      The anti-spin limit (270°) should stop the robot before 360°.

    WHAT TO LOOK FOR:
      - Robot starts rotating normally (✓ rotating)
      - Around 200-270°: robot slows or stops
      - "◀ ANTI-SPIN TRIGGERED" appears = protection is working
      - Check the diff_drive_node terminal too for log messages

    WHY THIS MATTERS:
      Without anti-spin, a Nav2 heading error can make the robot
      spin endlessly (360°+ overshoot). This safety limit prevents that.
''')
        input('    Press ENTER to begin anti-spin test...\n')

        msg = Twist()
        msg.angular.z = 0.5
        t0 = time.time()
        yaw0 = self.yaw
        rate = self.create_rate(20)
        last_print = -1.0

        print(f'    ┌─────────────────────────────────────────────────────────────')
        print(f'    │ {"Time":>5s}  {"Cmd w":>6s} │ {"Odom w":>6s} │ {"ΔYaw":>7s} │ Status')
        print(f'    ├─────────────────────────────────────────────────────────────')

        while time.time() - t0 < 15.0:
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)
            elapsed = time.time() - t0
            if elapsed - last_print >= 1.0:
                dyaw = math.degrees(self.yaw - yaw0)
                while dyaw > 180: dyaw -= 360
                while dyaw < -180: dyaw += 360
                if abs(self.odom_w) < 0.05 and elapsed > 3.0:
                    st = '◀ ANTI-SPIN TRIGGERED! Motors stopped.'
                elif abs(dyaw) > 200:
                    st = '⚠ approaching 270° limit...'
                else:
                    st = '✓ rotating'
                print(f'    │ {elapsed:5.1f}s  {0.5:6.3f} │ {self.odom_w:6.3f} │ {dyaw:+6.1f}° │ {st}')
                last_print = elapsed
            rate.sleep()

        stop = Twist()
        for _ in range(5):
            self.cmd_pub.publish(stop)
            time.sleep(0.05)
        print(f'    └─────────────────────────────────────────────────────────────')

        self._box([
            'CHAPTER 5 RESULTS',
            '',
            'If you saw "ANTI-SPIN TRIGGERED" → protection works! ✓',
            'If robot kept spinning non-stop  → check diff_drive_node logs',
            '',
            'Current settings:',
            '  max_continuous_rotation_deg = 270  (how far before stopping)',
            '  anti_spin_angular_clamp = 0.3      (speed limit after trigger)',
            '',
            'When to adjust:',
            '  Robot needs U-turns (180°) → 270 is fine (has margin)',
            '  Robot spins past 270° normally → decrease to 180',
            '  Robot can\'t complete U-turns → increase to 360',
        ])

    # ====================================================================
    # CHAPTER 6 — Final Summary + WHERE TO EDIT
    # ====================================================================
    def ch6_summary(self):
        self._hdr(6, 'FINAL SUMMARY — Your best values & exactly where to edit')

        b = self.best
        min_pwm = b.get('min_pwm', 40)
        rot_min = b.get('rotation_min_pwm', 55)
        boost = b.get('rotation_boost_factor', 1.35)
        approach = b.get('approach_min_linear_speed', 0.04)

        print(f'''
    ╔══════════════════════════════════════════════════════════════════╗
    ║              YOUR TUNED VALUES (from tests above)              ║
    ╠══════════════════════════════════════════════════════════════════╣
    ║                                                                ║
    ║  min_pwm                   = {min_pwm:<5}                          ║
    ║  rotation_min_pwm          = {rot_min:<5}                          ║
    ║  rotation_boost_factor     = {boost:<5.2f}                          ║
    ║  approach_min_linear_speed = {approach:<5}                          ║
    ║  angular_deadband          = 0.02  (keep unless twitching)     ║
    ║  linear_deadband           = 0.005 (keep unless humming)       ║
    ║  pwm_ramp_rate             = 40    (keep unless jerky)         ║
    ║  max_continuous_rotation   = 270   (keep unless spins out)     ║
    ║  use_pwm_rescaling         = True  (keep for smooth control)   ║
    ║                                                                ║
    ╚══════════════════════════════════════════════════════════════════╝

    ═══════════════════════════════════════════════════════════════════
    STEP-BY-STEP: WHERE TO PUT THESE VALUES
    ═══════════════════════════════════════════════════════════════════

    ── FILE 1: scripts/diff_drive_node.py ──────────────────────────

    Open the file and find these lines (near the top, around line 63-111).
    Change ONLY the number after the comma:

      Line ~63:  self.declare_parameter('min_pwm', 40)
                                                    ^^
                 Change 40 → {min_pwm}

      Line ~74:  self.declare_parameter('rotation_boost_factor', 1.35)
                                                                 ^^^^
                 Change 1.35 → {boost:.2f}

      Line ~81:  self.declare_parameter('rotation_min_pwm', 55)
                                                            ^^
                 Change 55 → {rot_min}

      Line ~104: self.declare_parameter('approach_min_linear_speed', 0.04)
                                                                     ^^^^
                 Change 0.04 → {approach}

    ── FILE 2: firmware/motor_controller/motor_controller.ino ──────

    Open the file and find this line (around line 215):

      #define MIN_PWM  40
                       ^^
      Change 40 → {min_pwm}

      ⚠ CRITICAL: This number MUST be exactly the same as min_pwm above!
      If they don't match, the Arduino PID will fight the ROS2 controller.

      After changing → re-upload the sketch to Arduino via Arduino IDE.

    ── FILE 3: config/nav2_params_hardware.yaml (OPTIONAL) ─────────

    Only change if rotation tests showed big problems:

      rotate_to_heading_angular_vel: 0.6
        → If over-rotating in Nav2: decrease to 0.4
        → If not rotating in Nav2: increase to 0.8

      min_approach_linear_velocity: 0.04
        → Change to match your approach_min_linear_speed: {approach}

    ═══════════════════════════════════════════════════════════════════
    AFTER EDITING — How to apply the changes:
    ═══════════════════════════════════════════════════════════════════

    Step 1: If you changed motor_controller.ino:
            → Open Arduino IDE → Upload sketch to Arduino Leonardo

    Step 2: Rebuild the ROS2 package:
            cd ~/robot_ws && colcon build --packages-select Tomas_bot

    Step 3: Source the workspace:
            source ~/robot_ws/install/setup.bash

    Step 4: Restart the robot:
            ros2 launch Tomas_bot bringup_hardware.launch.py

    Step 5: Test with Nav2:
            ros2 launch Tomas_bot navigation_hardware.launch.py map:=<your_map>
            Set a goal in RViz2 and watch if turns and approach improve.

    Step 6: If still not perfect → run this tuning script again!

    ═══════════════════════════════════════════════════════════════════
    FULL PARAMETER REFERENCE (all values, ranges, and effects):
    ═══════════════════════════════════════════════════════════════════
''')
        params = [
            ('── scripts/diff_drive_node.py ──', [
                ('min_pwm', min_pwm, 25, 80,
                 'Motor dead zone threshold. Below this PWM, wheels don\'t turn.\n'
                 '          MUST match Arduino MIN_PWM. Increase if robot stalls.'),
                ('rotation_min_pwm', rot_min, 40, 120,
                 'Minimum PWM for in-place rotation. Higher than min_pwm because\n'
                 '          starting rotation from standstill needs extra torque.'),
                ('rotation_boost_factor', boost, 1.0, 2.0,
                 'Multiplier on PWM during pure rotation (no forward speed).\n'
                 '          1.35 = 35% more power for turns. Increase if under-rotating.'),
                ('approach_min_linear_speed', approach, 0.02, 0.10,
                 'Speed floor near goal. Prevents motor stall during final approach.\n'
                 '          Increase if robot stalls near goal. Decrease if it overshoots.'),
                ('angular_deadband', 0.02, 0.005, 0.10,
                 'Angular commands below this → treated as zero. Filters jitter.\n'
                 '          Increase if robot twitches constantly. Decrease if ignoring turns.'),
                ('linear_deadband', 0.005, 0.001, 0.02,
                 'Linear commands below this → treated as zero. Keep very low.'),
                ('pwm_ramp_rate', 40, 10, 80,
                 'Max PWM change per cycle (20Hz). Lower = smoother, higher = snappier.\n'
                 '          At 40: reaches full speed in ~0.3s. At 20: ~0.6s.'),
                ('max_continuous_rotation_deg', 270, 90, 540,
                 'Anti-spin: max degrees of same-direction rotation before clamping.\n'
                 '          Covers 270° which handles U-turns (180°) with margin.'),
                ('anti_spin_angular_clamp', 0.3, 0.0, 0.5,
                 'Clamped angular speed when anti-spin triggers.\n'
                 '          0.0 = full stop. 0.3 = allows gentle continued rotation.'),
                ('use_pwm_rescaling', True, False, True,
                 'Remap PWM range above dead zone for smoother speed control.\n'
                 '          True = smooth. False = simple min-PWM clamping.'),
            ]),
            ('── firmware/motor_controller.ino ──', [
                ('MIN_PWM', min_pwm, 25, 80,
                 'MUST match diff_drive_node.py min_pwm exactly!'),
                ('KP', 1.0, 0.5, 3.0,
                 'PID proportional gain. Higher = faster correction, risk of oscillation.'),
                ('KI', 0.8, 0.0, 2.0,
                 'PID integral gain. Higher = better steady-state tracking.'),
                ('KD', 0.15, 0.0, 1.0,
                 'PID derivative gain. Higher = less oscillation, risk of noise.'),
                ('RAMP_RATE', 10.0, 2.0, 30.0,
                 'Arduino-side target ramping speed.'),
            ]),
            ('── config/nav2_params_hardware.yaml ──', [
                ('rotate_to_heading_angular_vel', 0.6, 0.3, 1.2,
                 'How fast Nav2 commands rotation. Lower = gentler, higher = faster.'),
                ('min_approach_linear_velocity', approach, 0.02, 0.10,
                 'Nav2 approach speed. Should match approach_min_linear_speed.'),
                ('max_angular_accel', 0.6, 0.3, 1.5,
                 'Angular acceleration limit from Nav2.'),
            ]),
        ]

        for section, plist in params:
            print(f'    {section}\n')
            for name, val, lo, hi, desc in plist:
                print(f'      {name}: {val}')
                print(f'        Range  : [{lo} .. {hi}]')
                print(f'        Effect : {desc}')
                print()

    # ====================================================================
    # Main menu
    # ====================================================================
    def run(self):
        print()
        print('╔══════════════════════════════════════════════════════════════════════╗')
        print('║        TOMAS_BOT TUNING SCRIPT — Branch: final-1                   ║')
        print('║        Enhanced PID + Adaptive Deadband + PWM Rescaling             ║')
        print('╚══════════════════════════════════════════════════════════════════════╝')
        print('''
    ═══════════════════════════════════════════════════════════════════
    HOW THIS SCRIPT WORKS (read this first!)
    ═══════════════════════════════════════════════════════════════════

    SETUP:
      Terminal 1: ros2 launch Tomas_bot bringup_hardware.launch.py
      Terminal 2: ros2 run Tomas_bot tuning_node.py  (this script)

    WHAT HAPPENS:
      1. You pick a chapter from the menu (or "a" for all)
      2. Each chapter has multiple tests — press ENTER to start each one
      3. The robot moves automatically while you watch the LIVE TABLE:

         │ Time  Cmd v  Cmd w │ Odom v Odom w │  Dist    ΔYaw │ Status
         │  0.5s 0.100  0.000 │ 0.092  0.003  │ 0.045m  +0.2° │ ✓ moving
         │  1.0s 0.100  0.000 │ 0.000  0.000  │ 0.045m  +0.2° │ ⚠ NOT MOVING

         Cmd v/w  = what we are TELLING the robot to do
         Odom v/w = what the robot is ACTUALLY doing (encoder feedback)
         Dist     = total distance traveled from start of this test
         ΔYaw     = heading change in degrees from start of this test
         Status   = instant verdict: ✓ working or ⚠ problem detected

      4. After each test → a RESULT box shows expected vs actual + error %
      5. After all tests → script picks the BEST VALUE automatically
      6. Chapter 6 → shows EXACTLY which files to open and what to change

    YOU DON'T NEED TO CALCULATE ANYTHING — the script does it for you.
    Just watch the live table, press ENTER between tests, and at the end
    you get a complete list of values and where to put them.
''')
        if not self._wait_odom():
            print('    ERROR: No odometry! Is bringup_hardware.launch.py running?')
            return
        print('    ✓ Odometry received. Robot is ready.\n')

        chapters = [
            ('1', 'PWM Dead Zone Detection', self.ch1_deadzone),
            ('2', 'Linear Motion Tuning', self.ch2_linear),
            ('3', 'Rotation Tuning (CRITICAL)', self.ch3_rotation),
            ('4', 'Arc Motion — Curves', self.ch4_arcs),
            ('5', 'Anti-Spin Protection Test', self.ch5_antispin),
            ('6', 'Final Summary + Where to Edit', self.ch6_summary),
        ]

        while True:
            print('\n    ─── MENU ───')
            for n, name, _ in chapters:
                print(f'      {n}. {name}')
            print('      a. Run ALL chapters in order (recommended first time)')
            print('      q. Quit & stop motors\n')

            choice = input('    Select (1-6, a, q): ').strip().lower()
            if choice == 'q':
                stop = Twist()
                for _ in range(10):
                    self.cmd_pub.publish(stop)
                    time.sleep(0.05)
                print('\n    Motors stopped. Goodbye!\n')
                break
            elif choice == 'a':
                for _, _, fn in chapters:
                    fn()
                    if fn != self.ch6_summary:
                        input('\n    Press ENTER for next chapter...')
            else:
                match = [fn for n, _, fn in chapters if n == choice]
                if match:
                    match[0]()
                else:
                    print('    Invalid choice.')


def main(args=None):
    rclpy.init(args=args)
    node = TuningNode()
    try:
        node.run()
    except KeyboardInterrupt:
        stop = Twist()
        for _ in range(10):
            node.cmd_pub.publish(stop)
            time.sleep(0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
