#!/usr/bin/env python3
"""
tuning_node.py — Fully Guided Robot Tuning Script (Branch: final-2)

Approach: Motion Profile State Machine (IDLE / ROTATING / LINEAR / ARC)

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

        x0, y0 = self.x, self.y
        last_yaw = self.yaw
        cumulative_yaw = 0.0  # radians, unwrapped — handles >180° rotation
        t0 = time.time()
        last_print = -1.0

        hdr = (f'    │ {"Time":>5s}  {"Cmd v":>6s} {"Cmd w":>6s} │ '
               f'{"Odom v":>6s} {"Odom w":>6s} │ {"Dist":>6s}  {"ΔYaw":>7s} │ Status')
        print(f'    ┌─────────────────────────────────────────────────────────────────────')
        print(hdr)
        print(f'    ├─────────────────────────────────────────────────────────────────────')

        while time.time() - t0 < duration:
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)

            # Accumulate yaw change (handles wrapping past ±180°)
            delta = self.yaw - last_yaw
            if delta > math.pi: delta -= 2 * math.pi
            if delta < -math.pi: delta += 2 * math.pi
            cumulative_yaw += delta
            last_yaw = self.yaw

            elapsed = time.time() - t0

            if elapsed - last_print >= 0.5:
                dist = math.hypot(self.x - x0, self.y - y0)
                dyaw = math.degrees(cumulative_yaw)
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
            time.sleep(0.05)

        stop = Twist()
        for _ in range(5):
            self.cmd_pub.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.05)
            delta = self.yaw - last_yaw
            if delta > math.pi: delta -= 2 * math.pi
            if delta < -math.pi: delta += 2 * math.pi
            cumulative_yaw += delta
            last_yaw = self.yaw

        dist = math.hypot(self.x - x0, self.y - y0)
        dyaw = math.degrees(cumulative_yaw)
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
      The robot tries to move forward at increasing speeds.
      For each speed, the LIVE table shows if the robot actually moved.
      We need to find the LOWEST speed where the robot moves reliably.

    WHAT TO LOOK FOR in the live table:
      - "⚠ NOT MOVING" in Status column = speed too low, motor dead zone
      - "✓ moving" in Status column      = motor is responding!
      - "Odom v" column: 0.000 = not moving, > 0 = wheels are turning

    WHY THIS MATTERS:
      Your 2.4kg robot needs a minimum PWM to overcome friction.
      Below this, motors hum but wheels don't turn.
      The state machine uses this value for duty cycling and minimum commands.
''')
        input('    Press ENTER to begin (robot tries small forward movements)...\n')

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

        moved_list = [(s, d, p) for s, d, m, p in results if m]
        if moved_list:
            _, _, raw_pwm = moved_list[0]
            best_min_pwm = max(raw_pwm + 5, 45)
        else:
            best_min_pwm = 60

        self.best['min_pwm'] = best_min_pwm

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
            '>>> BEST VALUE TO NOTE:',
            f'    min_pwm = {best_min_pwm}  (dead zone + 5 safety margin)',
            '',
            'WHAT THIS MEANS:',
            f'  Motor commands below PWM {best_min_pwm} are wasted — wheels won\'t turn.',
            f'  The state machine will NEVER send PWM between 1 and {best_min_pwm-1}.',
            f'  For speeds that need lower PWM, it uses duty cycling instead',
            f'  (alternating {best_min_pwm} and 0 to get an average below {best_min_pwm}).',
        ])

    # ====================================================================
    # CHAPTER 2 — Rotation (MOST CRITICAL)
    # ====================================================================
    def ch2_rotation(self):
        self._hdr(2, 'ROTATION — Tune in-place rotation (MOST CRITICAL TEST)')
        print('''
    WHAT THIS TEST DOES:
      Commands the robot to rotate exactly 90° at different angular speeds.
      Measures how much it ACTUALLY rotated vs the 90° target.

    HOW THE STATE MACHINE HANDLES ROTATION:
      Unlike final-1 (which multiplies PWM), final-2 uses a FIXED PWM value
      (rotation_pwm = 65) for all rotations, with a soft-start ramp.
      The PWM is scaled by angular velocity (floor = min_pwm/rotation_pwm).
      Soft-start ramps from min_pwm to rotation_pwm (never below dead zone).

    WHAT TO LOOK FOR in the live table:
      - "Odom w" should be non-zero = robot is actually rotating!
      - "ΔYaw" should climb toward +90° = heading is changing
      - "⚠ NOT ROTATING" = rotation_pwm too low → need to increase it
      - Watch the first 0.5s: robot should start smoothly (soft-start)

    AFTER EACH TEST:
      - Error close to 0° = perfect
      - Error << -20° = under-rotated → INCREASE rotation_pwm
      - Error >> +20° = over-rotated → DECREASE rotation_pwm
''')
        input('    Press ENTER to begin rotation tests...\n')

        angular_speeds = [0.3, 0.4, 0.5, 0.6, 0.8]
        results = []
        target = 90.0

        for w in angular_speeds:
            duration = math.radians(target) / w
            print(f'\n    ── Test: rotate CCW at w={w:.1f} rad/s for {duration:.1f}s (target: 90°) ──')
            input(f'    Press ENTER to run this test...')
            _, dyaw = self._send(0.0, w, duration)
            time.sleep(2.0)

            error = dyaw - target
            error_pct = (error / target) * 100
            results.append((w, dyaw, error, error_pct))

            print(f'\n    ➜ RESULT for w={w:.1f} rad/s:')
            print(f'       Target rotation   : +90.0°')
            print(f'       Actual rotation   : {dyaw:+.1f}°')
            print(f'       Error             : {error:+.1f}° ({error_pct:+.1f}%)')
            if abs(dyaw) < 15:
                print(f'       ✗ FAILED — robot barely rotated!')
                print(f'         → rotation_pwm is too low. Try 80, 90, or 100.')
            elif abs(error) < 20:
                print(f'       ✓ GOOD — close to 90° target')
            elif error > 20:
                print(f'       ⚠ OVER-ROTATED by {error:.0f}°')
                print(f'         → decrease rotation_pwm (try 60, 55)')
                print(f'         → or increase rotation_soft_start_steps (try 8, 10)')
            else:
                print(f'       ⚠ UNDER-ROTATED by {abs(error):.0f}°')
                print(f'         → increase rotation_pwm (try 80, 90, 100)')

        # Test CW
        print(f'\n    ── Test: rotate CW (clockwise) at w=-0.5 rad/s, target: -90° ──')
        input('    Press ENTER...')
        _, cw_dyaw = self._send(0.0, -0.5, math.radians(90) / 0.5)
        cw_error = cw_dyaw - (-90.0)
        print(f'\n    ➜ CW RESULT: actual={cw_dyaw:+.1f}° target=-90° error={cw_error:+.1f}°')

        # Auto-recommend rotation_pwm
        avg_error_pct = sum(r[3] for r in results) / len(results)
        current_pwm = 65

        if avg_error_pct < -25:
            rec_pwm = min(current_pwm + 10, 130)
            pwm_note = 'Robot under-rotates on average → INCREASE rotation_pwm'
        elif avg_error_pct > 25:
            rec_pwm = max(current_pwm - 5, 57)
            pwm_note = 'Robot over-rotates on average → DECREASE rotation_pwm'
        else:
            rec_pwm = current_pwm
            pwm_note = f'Current rotation_pwm ({current_pwm}) is working well — keep it'

        # Check if soft-start helps
        first_result = results[0]  # lowest speed
        if abs(first_result[1]) < 15 and abs(results[-1][1]) > 60:
            soft_note = 'Low-speed rotation fails but high-speed works → increase rotation_soft_start_steps to 8'
            rec_soft = 8
        elif any(abs(r[2]) > 30 for r in results):
            soft_note = 'Some tests over-rotated → try rotation_soft_start_steps = 8'
            rec_soft = 8
        else:
            soft_note = 'Soft-start at 5 steps is working well'
            rec_soft = 5

        self.best['rotation_pwm'] = rec_pwm
        self.best['rotation_soft_start_steps'] = rec_soft

        best_entry = min(results, key=lambda r: abs(r[2]))

        self._box([
            'CHAPTER 2 RESULTS',
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
            f'Best speed: {best_entry[0]:.1f} rad/s (actual: {best_entry[1]:+.1f}°, closest to 90°)',
            f'Average error: {avg_error_pct:+.1f}%',
            '',
            f'{pwm_note}',
            f'{soft_note}',
            '',
            '>>> BEST VALUES TO NOTE:',
            f'    rotation_pwm             = {rec_pwm}',
            f'    rotation_soft_start_steps = {rec_soft}',
            '',
            'HOW THE STATE MACHINE ROTATION WORKS:',
            f'  The robot sends FIXED PWM={rec_pwm} to both motors (opposite directions).',
            f'  Soft-start ramps from 0 to {rec_pwm} over {rec_soft} steps ({rec_soft*50}ms).',
            '  This is different from final-1 which multiplies velocity-proportional PWM.',
        ])

    # ====================================================================
    # CHAPTER 3 — Linear Motion
    # ====================================================================
    def ch3_linear(self):
        self._hdr(3, 'LINEAR MOTION — Tune straight-line speed & goal approach')
        print('''
    WHAT THIS TEST DOES:
      Sends forward commands at different speeds for 3 seconds each.
      Measures actual distance vs expected distance.

    WHAT TO LOOK FOR in the live table:
      - "Odom v" should be close to "Cmd v" = good speed tracking
      - "Dist" should grow steadily = smooth motion
      - "ΔYaw" should stay near 0° = going straight

    AFTER EACH TEST:
      - "Speed accuracy" near 100% = motor responds well
      - Low accuracy at low speeds = need higher linear_min_speed

    WHY THIS MATTERS:
      Nav2 slows down near the goal. The LINEAR state has a speed floor
      (linear_min_speed) that prevents commands below motor dead zone.
      We need to find what that floor should be.
''')
        input('    Press ENTER to begin linear tests...\n')

        speeds = [0.04, 0.06, 0.10, 0.15, 0.20]
        results = []

        for spd in speeds:
            expected = spd * 3.0
            print(f'\n    ── Test: v={spd:.2f} m/s for 3s (expected ≈ {expected:.3f}m) ──')
            input(f'    Press ENTER to run this test...')
            dist, dyaw = self._send(spd, 0.0, 3.0)
            time.sleep(1.0)

            accuracy = (dist / expected * 100) if expected > 0 else 0
            error_m = expected - dist
            results.append((spd, dist, expected, accuracy, error_m, dyaw))

            print(f'\n    ➜ RESULT for v={spd:.2f}:')
            print(f'       Expected distance : {expected:.3f}m')
            print(f'       Actual distance   : {dist:.3f}m')
            print(f'       Error             : {error_m:+.3f}m')
            print(f'       Speed accuracy    : {accuracy:.1f}%')
            print(f'       Heading drift     : {dyaw:+.1f}° (ideal: 0°)')
            if accuracy > 80:
                print(f'       ✓ GOOD — LINEAR state handles this speed well')
            elif accuracy > 50:
                print(f'       ⚠ FAIR — some speed loss')
            else:
                print(f'       ✗ POOR — robot struggles (duty cycling may help here)')

        good_speeds = [s for s, d, e, a, em, y in results if a > 70]
        best_min_speed = good_speeds[0] if good_speeds else 0.06
        self.best['linear_min_speed'] = best_min_speed

        self._box([
            'CHAPTER 3 RESULTS',
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
            f'    linear_min_speed = {best_min_speed}',
            '',
            'WHAT THIS MEANS:',
            f'  The lowest speed where accuracy > 70%.',
            f'  The LINEAR state will never let speed drop below {best_min_speed} m/s.',
            f'  When Nav2 commands a lower speed near the goal,',
            f'  the state machine enforces this floor to prevent stalling.',
        ])

    # ====================================================================
    # CHAPTER 4 — Arc Motion
    # ====================================================================
    def ch4_arcs(self):
        self._hdr(4, 'ARC MOTION — Test curves (Nav2 path following)')
        print('''
    WHAT THIS TEST DOES:
      Sends combined forward + turn commands (curves/arcs).
      In the state machine, this activates the ARC state.

    HOW ARC STATE WORKS:
      The ARC state applies arc_angular_scale (0.90) to the angular
      component. This controls steering strength on curves.
      If arc_angular_scale = 0.90, angular power is reduced by 10%.

    WHAT TO LOOK FOR:
      - Robot moves smoothly along a curve (not jerky)
      - Both "Odom v" and "Odom w" are non-zero = moving + turning
      - "Dist" increases = forward motion is happening
      - "ΔYaw" changes = steering is working
      - If robot under-steers (wide turns) → increase arc_angular_scale
      - If robot over-steers (tight spiral) → decrease arc_angular_scale
''')
        input('    Press ENTER to begin arc tests...\n')

        arcs = [
            (0.10,  0.3, 4.0, 'Gentle LEFT curve'),
            (0.05,  0.6, 3.0, 'Sharp LEFT curve (tight corner)'),
            (0.10, -0.3, 4.0, 'Gentle RIGHT curve'),
        ]
        arc_results = []

        for v, w, dur, desc in arcs:
            expected_yaw = math.degrees(w * dur)
            print(f'\n    ── Test: {desc} (v={v}, w={w}) for {dur}s ──')
            print(f'       Expected heading change ≈ {expected_yaw:+.0f}°')
            input('    Press ENTER...')
            dist, dyaw = self._send(v, w, dur)
            time.sleep(1.0)
            yaw_error = dyaw - expected_yaw
            arc_results.append((desc, v, w, dist, dyaw, expected_yaw, yaw_error))

            print(f'    ➜ Displacement: {dist:.3f}m')
            print(f'       Expected ΔYaw: {expected_yaw:+.1f}°, Actual ΔYaw: {dyaw:+.1f}°, Error: {yaw_error:+.1f}°')
            if dist < 0.02:
                print(f'       ✗ Robot barely moved! → increase min_pwm')
            elif abs(yaw_error) > 30:
                if yaw_error > 0:
                    print(f'       ⚠ OVER-STEERING → decrease arc_angular_scale (try 0.75, 0.7)')
                else:
                    print(f'       ⚠ UNDER-STEERING → increase arc_angular_scale (try 0.95, 1.0)')
            else:
                print(f'       ✓ Arc motion working well')

        avg_yaw_error = sum(abs(r[6]) for r in arc_results) / len(arc_results)
        avg_signed = sum(r[6] for r in arc_results) / len(arc_results)

        current_scale = 0.90
        if avg_signed > 15:
            rec_scale = max(current_scale - 0.10, 0.5)
            scale_note = 'Over-steering on average → DECREASE arc_angular_scale'
        elif avg_signed < -15:
            rec_scale = min(current_scale + 0.10, 1.2)
            scale_note = 'Under-steering on average → INCREASE arc_angular_scale'
        else:
            rec_scale = current_scale
            scale_note = f'Current arc_angular_scale ({current_scale}) works well'

        self.best['arc_angular_scale'] = rec_scale

        self._box([
            'CHAPTER 4 RESULTS',
            '',
        ] + [
            f'{d}: dist={di:.3f}m  ΔYaw={dy:+.1f}° (expected {ey:+.0f}°) err={ye:+.1f}°'
            for d, v, w, di, dy, ey, ye in arc_results
        ] + [
            '',
            f'Average heading error: {avg_signed:+.1f}°',
            f'{scale_note}',
            '',
            '>>> BEST VALUE TO NOTE:',
            f'    arc_angular_scale = {rec_scale:.2f}',
            '',
            'HOW ARC_ANGULAR_SCALE WORKS:',
            f'  During arc motion, angular PWM is multiplied by {rec_scale:.2f}.',
            f'  At 1.0: full angular power (may over-steer).',
            f'  At 0.85: 85% angular power (reduces over-steering).',
            f'  At 0.7: 70% angular power (much gentler turns).',
        ])

    # ====================================================================
    # CHAPTER 5 — Duty Cycling
    # ====================================================================
    def ch5_duty(self):
        self._hdr(5, 'DUTY CYCLING — Test very low speed control')
        print('''
    WHAT THIS TEST DOES:
      Sends very low speed commands that are BELOW the motor dead zone.
      The state machine handles these with DUTY CYCLING:
        - Alternates between min_pwm and 0
        - The ON/OFF ratio creates an average speed below dead zone
        - Example: 50% duty = min_pwm for 2 cycles, 0 for 2 cycles

    WHAT TO LOOK FOR:
      - Robot should inch forward even at very low speeds
      - Motion may be pulsing/jerky (that's normal for duty cycling)
      - "✓ moving" means duty cycling is working!
      - "⚠ NOT MOVING" means duty_cycle_period may need adjustment

    WHY THIS MATTERS:
      Without duty cycling, any speed below dead zone = robot stalls.
      With duty cycling, the robot can creep toward the goal at any speed.
''')
        input('    Press ENTER to test very low speeds...\n')

        test_speeds = [0.03, 0.04, 0.05]
        results = []

        for spd in test_speeds:
            print(f'\n    ── Test: v={spd:.2f} m/s for 4 seconds (should use duty cycling) ──')
            input('    Press ENTER...')
            dist, _ = self._send(spd, 0.0, 4.0)
            time.sleep(1.0)

            expected = spd * 4.0
            moved = dist > 0.005
            results.append((spd, dist, expected, moved))

            print(f'    ➜ Distance: {dist:.3f}m (expected ≈ {expected:.3f}m)')
            if moved:
                print(f'       ✓ Duty cycling working — robot moved at sub-minimum speed!')
            else:
                print(f'       ✗ No movement — try increasing duty_cycle_period (8, 10)')

        moved_count = sum(1 for _, _, _, m in results if m)

        if moved_count >= 2:
            rec_period = 6
            period_note = 'Duty cycling works well at period=6'
        elif moved_count == 1:
            rec_period = 8
            period_note = 'Only 1 test passed → increase duty_cycle_period to 8'
        else:
            rec_period = 10
            period_note = 'Duty cycling failed → increase duty_cycle_period to 10'

        self.best['duty_cycle_period'] = rec_period

        self._box([
            'CHAPTER 5 RESULTS',
            '',
        ] + [
            f'v={s:.2f}: dist={d:.3f}m (expected {e:.3f}m) → {"✓ MOVED" if m else "✗ STALLED"}'
            for s, d, e, m in results
        ] + [
            '',
            f'{moved_count}/{len(results)} low-speed tests showed movement',
            '',
            'Current duty cycling settings:',
            f'  duty_cycle_enabled = True',
            f'  duty_cycle_period  = 6  (6 cycles × 50ms = 300ms per period)',
            '',
            f'{period_note}',
            '',
            '>>> BEST VALUE TO NOTE:',
            f'    duty_cycle_period = {rec_period}',
            '',
            'If all tests showed movement → settings are fine',
            'If some failed → increase duty_cycle_period (more ON time per burst)',
        ])

    # ====================================================================
    # CHAPTER 6 — Rotation Watchdog
    # ====================================================================
    def ch6_watchdog(self):
        self._hdr(6, 'ROTATION WATCHDOG — Verify spin protection')
        print('''
    WHAT THIS TEST DOES:
      Commands continuous rotation at 0.5 rad/s for 10 seconds.
      The state machine's watchdog (rotation_max_duration = 6.0s)
      should stop the motors before the full 10 seconds.

    WHAT TO LOOK FOR:
      - Robot starts rotating normally (✓ rotating)
      - Around 8 seconds: robot stops (◀ WATCHDOG)
      - Check diff_drive_node terminal for "Rotation watchdog" message

    WHY THIS MATTERS:
      Without the watchdog, a Nav2 heading error could make the robot
      spin continuously. The watchdog is a safety limit.
''')
        input('    Press ENTER to begin watchdog test...\n')

        msg = Twist()
        msg.angular.z = 0.5
        t0 = time.time()
        last_yaw = self.yaw
        cumulative_yaw = 0.0
        last_print = -1.0

        print(f'    ┌─────────────────────────────────────────────────────────────')
        print(f'    │ {"Time":>5s}  {"Cmd w":>6s} │ {"Odom w":>6s} │ {"ΔYaw":>7s} │ Status')
        print(f'    ├─────────────────────────────────────────────────────────────')

        while time.time() - t0 < 10.0:
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)
            delta = self.yaw - last_yaw
            if delta > math.pi: delta -= 2 * math.pi
            if delta < -math.pi: delta += 2 * math.pi
            cumulative_yaw += delta
            last_yaw = self.yaw
            elapsed = time.time() - t0
            if elapsed - last_print >= 1.0:
                dyaw = math.degrees(cumulative_yaw)
                if abs(self.odom_w) < 0.05 and elapsed > 3.0:
                    st = '◀ WATCHDOG TRIGGERED! Motors stopped.'
                elif elapsed > 4.5:
                    st = '⚠ approaching 6s watchdog limit...'
                else:
                    st = '✓ rotating'
                print(f'    │ {elapsed:5.1f}s  {0.5:6.3f} │ {self.odom_w:6.3f} │ {dyaw:+6.1f}° │ {st}')
                last_print = elapsed
            time.sleep(0.05)

        stop = Twist()
        for _ in range(5):
            self.cmd_pub.publish(stop)
            time.sleep(0.05)
        print(f'    └─────────────────────────────────────────────────────────────')

        self._box([
            'CHAPTER 6 RESULTS',
            '',
            'If you saw "WATCHDOG TRIGGERED" → protection works! ✓',
            'If robot kept spinning all 10 seconds → check diff_drive_node logs',
            '',
            'Current setting: rotation_max_duration = 6.0 seconds',
            '',
            'When to adjust:',
            '  Robot needs slow U-turns → keep 6.0 or increase to 8.0',
            '  Robot spins too much → decrease to 4.0 or 5.0',
            '  At 0.5 rad/s: 6s allows ≈ 170° rotation (covers most turns)',
        ])

    # ====================================================================
    # CHAPTER 7 — Final Summary + WHERE TO EDIT
    # ====================================================================
    def ch7_summary(self):
        self._hdr(7, 'FINAL SUMMARY — Your best values & exactly where to edit')

        b = self.best
        min_pwm = b.get('min_pwm', 57)
        rot_pwm = b.get('rotation_pwm', 65)
        rot_soft = b.get('rotation_soft_start_steps', 5)
        lin_min = b.get('linear_min_speed', 0.10)
        arc_scale = b.get('arc_angular_scale', 0.90)

        print(f'''
    ╔══════════════════════════════════════════════════════════════════╗
    ║              YOUR TUNED VALUES (from tests above)              ║
    ╠══════════════════════════════════════════════════════════════════╣
    ║                                                                ║
    ║  min_pwm                    = {min_pwm:<5}                         ║
    ║  rotation_pwm               = {rot_pwm:<5}                         ║
    ║  rotation_soft_start_steps  = {rot_soft:<5}                         ║
    ║  linear_min_speed           = {lin_min:<5}                         ║
    ║  arc_angular_scale          = {arc_scale:<5.2f}                         ║
    ║  rotation_max_duration      = 6.0   (keep unless spins out)    ║
    ║  angular_deadband           = 0.03  (keep unless twitching)    ║
    ║  linear_deadband            = 0.005 (keep unless humming)      ║
    ║  linear_ramp_rate           = 35    (keep unless jerky)        ║
    ║  arc_ramp_rate              = 30    (keep unless wobbling)     ║
    ║  duty_cycle_enabled         = True  (keep for low-speed ctrl)  ║
    ║  duty_cycle_period          = 6     (keep unless jerky pulses) ║
    ║  motor_trim                 = 0.03  (compensates motor drift)  ║
    ║                                                                ║
    ╚══════════════════════════════════════════════════════════════════╝

    ═══════════════════════════════════════════════════════════════════
    STEP-BY-STEP: WHERE TO PUT THESE VALUES
    ═══════════════════════════════════════════════════════════════════

    ── FILE 1: scripts/diff_drive_node.py ──────────────────────────

    Open the file and find these lines (around line 72-155).
    Change ONLY the number after the comma:

      Line ~72:  self.declare_parameter('min_pwm', 45)
                                                    ^^
                 Change 45 → {min_pwm}

      Line ~92:  self.declare_parameter('rotation_pwm', 65)
                                                        ^^
                 Change 65 → {rot_pwm}

      Line ~101: self.declare_parameter('rotation_soft_start_steps', 12)
                                                                     ^^
                 Change 12 → {rot_soft}

      Line ~125: self.declare_parameter('linear_min_speed', 0.10)
                                                            ^^^^
                 Change 0.10 → {lin_min}

      Line ~140: self.declare_parameter('arc_angular_scale', 1.0)
                                                             ^^^
                 Change 1.0 → {arc_scale:.2f}

    ── FILE 2: firmware/motor_controller/motor_controller.ino ──────

    Open the file and find this line (around line 215):

      #define MIN_PWM  45
                       ^^
      Change 45 → {min_pwm}

      ⚠ CRITICAL: This MUST be exactly the same as min_pwm above!
      If they don't match, Arduino PID fights the ROS2 controller.

      After changing → re-upload sketch to Arduino via Arduino IDE.

    ── FILE 3: config/nav2_params_hardware.yaml (OPTIONAL) ─────────

    Only change if rotation tests showed big problems:

      rotate_to_heading_angular_vel: 0.5
        → If over-rotating in Nav2: decrease to 0.3
        → If not rotating in Nav2: increase to 0.7

      min_approach_linear_velocity: 0.05
        → Change to match your linear_min_speed: {lin_min}

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
                ('min_pwm', min_pwm, 30, 80,
                 'Motor dead zone threshold. Below this PWM, wheels don\'t turn.\n'
                 '          MUST match Arduino MIN_PWM. Duty cycling uses this as ON level.'),
                ('angular_deadband', 0.03, 0.01, 0.10,
                 'Angular velocity below this → state = IDLE (no rotation).\n'
                 '          Increase if robot twitches. Decrease if ignoring small turns.'),
                ('linear_deadband', 0.005, 0.001, 0.02,
                 'Linear velocity below this → state = IDLE (no movement).\n'
                 '          Keep very low so Nav2 approach commands pass through.'),
                ('rotation_pwm', rot_pwm, 57, 130,
                 'Target PWM for ROTATING state. Soft-start ramps from min_pwm to this.\n'
                 '          Robot responds immediately (never below dead zone).\n'
                 '          Increase if robot won\'t rotate. Decrease if over-rotating.'),
                ('rotation_soft_start_steps', rot_soft, 1, 15,
                 'Number of 50ms cycles to ramp from min_pwm to rotation_pwm.\n'
                 '          Robot moves from cycle 1 (always above dead zone).\n'
                 '          At 5 steps: 250ms ramp. At 8: 400ms ramp.'),
                ('rotation_max_duration', 6.0, 2.0, 15.0,
                 'Watchdog: max seconds of continuous rotation before stopping.\n'
                 '          At 0.5 rad/s and 6s: allows ≈170° rotation.'),
                ('linear_ramp_rate', 35, 10, 80,
                 'Max PWM change per cycle in LINEAR state.\n'
                 '          Lower = smoother acceleration. Higher = snappier.'),
                ('linear_min_speed', lin_min, 0.02, 0.10,
                 'Speed floor in LINEAR state. Prevents stalling near goal.\n'
                 '          Increase if stalling near goal. Decrease if overshooting.'),
                ('arc_ramp_rate', 30, 10, 60,
                 'Max PWM change per cycle in ARC state.\n'
                 '          Slower than linear ramp because steering is more sensitive.'),
                ('arc_angular_scale', arc_scale, 0.5, 1.2,
                 'Scale factor for angular component during ARC state.\n'
                 '          At 1.0: full angular power.\n'
                 '          Decrease for gentler curves. Increase for sharper curves.'),
                ('duty_cycle_enabled', True, False, True,
                 'Enable duty cycling for sub-minimum speeds.\n'
                 '          True: alternates min_pwm/0 for smooth low-speed control.\n'
                 '          False: clamps to min_pwm (jumpy at low speeds).'),
                ('duty_cycle_period', 6, 2, 10,
                 'Cycles per duty period. At 20Hz: period × 50ms.\n'
                 '          Higher = smoother average. Lower = faster response.'),
                ('motor_trim', 0.03, -0.10, 0.10,
                 'Motor asymmetry compensation. Positive = reduce left, boost right.\n'
                 '          Compensates drift during straight-line driving.\n'
                 '          Increase if robot drifts right. Decrease (negative) if drifts left.'),
            ]),
            ('── firmware/motor_controller.ino ──', [
                ('MIN_PWM', min_pwm, 30, 80,
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
                ('rotate_to_heading_angular_vel', 0.5, 0.2, 1.0,
                 'How fast Nav2 commands rotation. Lower = gentler, higher = faster.\n'
                 '          State machine scales rotation_pwm by this ÷ 0.8.'),
                ('min_approach_linear_velocity', 0.05, 0.02, 0.10,
                 'Nav2 approach speed. Should match linear_min_speed.'),
                ('max_angular_accel', 0.5, 0.3, 1.5,
                 'Angular acceleration limit. Low because state machine has soft-start.'),
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
        print('║        TOMAS_BOT TUNING SCRIPT — Branch: final-2                   ║')
        print('║        Motion Profile State Machine                                 ║')
        print('╚══════════════════════════════════════════════════════════════════════╝')
        print('''
    ═══════════════════════════════════════════════════════════════════
    COMPLETE GUIDE — BUILD, RUN, TUNE, APPLY
    ═══════════════════════════════════════════════════════════════════

    ┌──────────────────────────────────────────────────────────────────┐
    │  STEP 1: BUILD THE PROJECT                                      │
    │                                                                  │
    │  If this is your first time, or you just switched branches:      │
    │                                                                  │
    │    cd ~/robot_ws                                                 │
    │    colcon build --packages-select Tomas_bot                      │
    │    source ~/robot_ws/install/setup.bash                          │
    │                                                                  │
    │  If you changed motor_controller.ino (Arduino firmware):         │
    │    1. Open Arduino IDE                                           │
    │    2. Open firmware/motor_controller/motor_controller.ino        │
    │    3. Select Board → Arduino Leonardo                            │
    │    4. Select the correct USB port                                │
    │    5. Click Upload                                               │
    │    ⚠ You MUST upload the Arduino sketch if you switch branches!  │
    │      final-2 uses MIN_PWM=57 (tuned from dead zone tests)        │
    └──────────────────────────────────────────────────────────────────┘

    ┌──────────────────────────────────────────────────────────────────┐
    │  STEP 2: RUN THE TUNING SCRIPT                                  │
    │                                                                  │
    │  Open TWO terminals:                                             │
    │                                                                  │
    │  Terminal 1 — Start the robot:                                   │
    │    ros2 launch Tomas_bot bringup_hardware.launch.py              │
    │    (wait until you see "diff_drive_node started" in the logs)    │
    │                                                                  │
    │  Terminal 2 — Run tuning:                                        │
    │    ros2 run Tomas_bot tuning_node.py                             │
    │                                                                  │
    │  Place robot on the FLOOR with space around it (1m+ clearance).  │
    │  Select "a" to run ALL chapters, or pick individual chapters.    │
    │  Press ENTER before each test — the robot will move on its own.  │
    └──────────────────────────────────────────────────────────────────┘

    ┌──────────────────────────────────────────────────────────────────┐
    │  STEP 3: WHAT THE SCRIPT TESTS & WHAT PARAMETERS YOU GET        │
    │                                                                  │
    │  Ch 1: Dead Zone    → finds min_pwm                              │
    │  Ch 2: Rotation     → finds rotation_pwm,                        │
    │                        rotation_soft_start_steps                  │
    │  Ch 3: Linear       → finds linear_min_speed                     │
    │  Ch 4: Arcs/Curves  → finds arc_angular_scale                    │
    │  Ch 5: Duty Cycling → validates duty_cycle_period                 │
    │  Ch 6: Watchdog     → validates rotation_max_duration             │
    │  Ch 7: SUMMARY      → shows ALL best values + exact edit spots   │
    │                                                                  │
    │  The script picks the best values automatically.                 │
    │  You do NOT need to calculate anything yourself.                  │
    └──────────────────────────────────────────────────────────────────┘

    ┌──────────────────────────────────────────────────────────────────┐
    │  STEP 4: AFTER TUNING — WHERE TO EDIT (3 files, 2 mandatory)    │
    │                                                                  │
    │  FILE 1 (MANDATORY): scripts/diff_drive_node.py                  │
    │    Lines to edit (change the number after the comma):            │
    │      Line ~72:  min_pwm .................. default: 57           │
    │      Line ~80:  angular_deadband ......... default: 0.03         │
    │      Line ~86:  linear_deadband .......... default: 0.005        │
    │      Line ~92:  rotation_pwm ............. default: 65           │
    │      Line ~101: rotation_soft_start_steps  default: 5            │
    │      Line ~109: rotation_max_duration .... default: 6.0          │
    │      Line ~117: linear_ramp_rate ......... default: 35           │
    │      Line ~125: linear_min_speed ......... default: 0.10         │
    │      Line ~133: arc_ramp_rate ............ default: 30           │
    │      Line ~140: arc_angular_scale ........ default: 0.90         │
    │      Line ~149: duty_cycle_enabled ....... default: True         │
    │      Line ~155: duty_cycle_period ........ default: 6            │
    │                                                                  │
    │  FILE 2 (MANDATORY): firmware/motor_controller/motor_controller  │
    │    Line ~215: #define MIN_PWM  57                                 │
    │    ⚠ MUST match min_pwm from File 1!                             │
    │    ⚠ After changing → re-upload to Arduino!                      │
    │                                                                  │
    │  FILE 3 (OPTIONAL): config/nav2_params_hardware.yaml             │
    │    rotate_to_heading_angular_vel: 0.5                            │
    │    min_approach_linear_velocity:  0.05                            │
    │    (only change if Nav2 navigation has problems)                 │
    └──────────────────────────────────────────────────────────────────┘

    ┌──────────────────────────────────────────────────────────────────┐
    │  STEP 5: APPLY CHANGES                                          │
    │                                                                  │
    │  After editing the files:                                        │
    │                                                                  │
    │  1. If you changed motor_controller.ino:                         │
    │     → Open Arduino IDE → Upload to Arduino Leonardo              │
    │                                                                  │
    │  2. Rebuild:                                                     │
    │     cd ~/robot_ws && colcon build --packages-select Tomas_bot    │
    │                                                                  │
    │  3. Source:                                                       │
    │     source ~/robot_ws/install/setup.bash                         │
    │                                                                  │
    │  4. Test:                                                         │
    │     ros2 launch Tomas_bot bringup_hardware.launch.py             │
    │     ros2 launch Tomas_bot navigation_hardware.launch.py          │
    │       map:=<your_map_yaml>                                       │
    │     Set a 2D Goal in RViz2 and see if behavior improved.         │
    │                                                                  │
    │  5. Still not perfect? Run this tuning script again!              │
    └──────────────────────────────────────────────────────────────────┘

    ═══════════════════════════════════════════════════════════════════
    HOW THIS SCRIPT WORKS
    ═══════════════════════════════════════════════════════════════════

    SETUP:
      Terminal 1: ros2 launch Tomas_bot bringup_hardware.launch.py
      Terminal 2: ros2 run Tomas_bot tuning_node.py  (this script)

    WHAT HAPPENS:
      1. You pick a chapter from the menu (or "a" for all)
      2. Each chapter has tests — press ENTER to start each one
      3. The robot moves automatically while you watch the LIVE TABLE:

         │ Time  Cmd v  Cmd w │ Odom v Odom w │  Dist    ΔYaw │ Status
         │  0.5s 0.100  0.000 │ 0.092  0.003  │ 0.045m  +0.2° │ ✓ moving
         │  1.0s 0.100  0.000 │ 0.000  0.000  │ 0.045m  +0.2° │ ⚠ NOT MOVING

         HOW TO READ THE TABLE:
           Cmd v/w  = what we TELL the robot to do (the command)
           Odom v/w = what the robot ACTUALLY does (encoder feedback)
           Dist     = total distance traveled from start of this test
           ΔYaw     = heading change in degrees from start of this test
           Status   = instant verdict: ✓ working or ⚠ problem

         KEY RULE: If "Odom v" = 0.000 while "Cmd v" > 0 → robot is STUCK!

      4. After each test → RESULT box: expected vs actual + error + verdict
      5. After all tests → script picks BEST VALUES automatically
      6. Chapter 7 → tells you EXACTLY which files to open, which lines
         to change, and what values to type

    YOU DON'T NEED TO CALCULATE ANYTHING.
    Just watch, press ENTER, and follow the instructions at the end.
''')
        if not self._wait_odom():
            print('    ERROR: No odometry! Is bringup_hardware.launch.py running?')
            return
        print('    ✓ Odometry received. Robot is ready.\n')

        chapters = [
            ('1', 'PWM Dead Zone Detection', self.ch1_deadzone),
            ('2', 'Rotation Tuning (CRITICAL)', self.ch2_rotation),
            ('3', 'Linear Motion Tuning', self.ch3_linear),
            ('4', 'Arc Motion — Curves', self.ch4_arcs),
            ('5', 'Duty Cycling — Low Speed', self.ch5_duty),
            ('6', 'Rotation Watchdog Test', self.ch6_watchdog),
            ('7', 'Final Summary + Where to Edit', self.ch7_summary),
        ]

        while True:
            print('\n    ─── MENU ───')
            for n, name, _ in chapters:
                print(f'      {n}. {name}')
            print('      a. Run ALL chapters in order (recommended first time)')
            print('      q. Quit & stop motors\n')

            choice = input('    Select (1-7, a, q): ').strip().lower()
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
                    if fn != self.ch7_summary:
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
