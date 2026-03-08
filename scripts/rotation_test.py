#!/usr/bin/env python3
"""
rotation_test.py — Rotation Behavior Diagnostic

Measures the robot's actual rotation response to understand:
  - How much the robot actually rotates for a given command duration
  - Overshoot: how far the robot coasts after cmd_vel stops
  - Settling time: how long until angular velocity reaches ~0
  - Asymmetry: does CW behave differently from CCW
  - Different angular velocities: 0.2, 0.3, 0.5, 0.8 rad/s

Run bringup_hardware.launch.py FIRST in a separate terminal.

Usage:
    Terminal 1:  ros2 launch Tomas_bot bringup_hardware.launch.py
    Terminal 2:  ros2 run Tomas_bot rotation_test.py
"""

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class RotationTest(Node):

    def __init__(self):
        super().__init__('rotation_test')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)

        self.yaw = 0.0
        self.odom_w = 0.0
        self.odom_ok = False

    def _odom_cb(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom_w = msg.twist.twist.angular.z
        self.odom_ok = True

    def _wait_odom(self, timeout=10.0):
        t0 = time.time()
        while not self.odom_ok and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.odom_ok

    def _stop(self, settle_time=0.5):
        """Send stop and wait for robot to physically settle."""
        msg = Twist()
        for _ in range(10):
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)
        time.sleep(settle_time)
        # Drain odom updates
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.02)

    def _track_yaw(self, last_yaw):
        """Return (delta, new_last_yaw) with wrapping handled."""
        delta = self.yaw - last_yaw
        if delta > math.pi:
            delta -= 2 * math.pi
        if delta < -math.pi:
            delta += 2 * math.pi
        return delta, self.yaw

    def run_single_test(self, cmd_w, duration, label):
        """
        Command rotation at cmd_w for `duration` seconds, then stop.
        Track yaw during command AND after stop to measure overshoot.
        Returns dict with all measurements.
        """
        self._stop(1.0)  # Ensure settled before test

        # Drain odom
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.02)

        last_yaw = self.yaw
        cumulative_during = 0.0
        peak_odom_w = 0.0
        start_response_time = None

        msg = Twist()
        msg.angular.z = float(cmd_w)
        t0 = time.time()

        # === PHASE 1: Command rotation ===
        samples_during = []
        while time.time() - t0 < duration:
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)
            delta, last_yaw = self._track_yaw(last_yaw)
            cumulative_during += delta

            if abs(self.odom_w) > abs(peak_odom_w):
                peak_odom_w = self.odom_w

            # Detect when robot actually starts moving
            if start_response_time is None and abs(self.odom_w) > 0.05:
                start_response_time = time.time() - t0

            elapsed = time.time() - t0
            samples_during.append({
                't': elapsed,
                'yaw_deg': math.degrees(cumulative_during),
                'odom_w': self.odom_w
            })
            time.sleep(0.04)  # ~25Hz sampling

        yaw_at_stop_cmd = cumulative_during  # Yaw when we send stop

        # === PHASE 2: Stop and measure coast/overshoot ===
        stop_msg = Twist()
        t_stop = time.time()
        cumulative_after = 0.0
        settle_time = None

        samples_coast = []
        while time.time() - t_stop < 3.0:  # Monitor for 3s after stop
            self.cmd_pub.publish(stop_msg)
            rclpy.spin_once(self, timeout_sec=0.01)
            delta, last_yaw = self._track_yaw(last_yaw)
            cumulative_after += delta

            elapsed_after = time.time() - t_stop
            samples_coast.append({
                't': elapsed_after,
                'yaw_deg': math.degrees(cumulative_after),
                'odom_w': self.odom_w
            })

            # Detect settling (angular velocity near zero for 0.3s)
            if settle_time is None and elapsed_after > 0.1 and abs(self.odom_w) < 0.03:
                settle_time = elapsed_after

            time.sleep(0.04)

        total_yaw = cumulative_during + cumulative_after
        overshoot = cumulative_after

        result = {
            'label': label,
            'cmd_w': cmd_w,
            'duration': duration,
            'yaw_at_stop_deg': math.degrees(yaw_at_stop_cmd),
            'overshoot_deg': math.degrees(overshoot),
            'total_yaw_deg': math.degrees(total_yaw),
            'peak_odom_w': peak_odom_w,
            'response_time': start_response_time if start_response_time else -1,
            'settle_time': settle_time if settle_time else -1,
            'samples_during': samples_during,
            'samples_coast': samples_coast,
        }
        return result

    def print_result(self, r):
        """Print a single test result."""
        direction = "CCW" if r['cmd_w'] > 0 else "CW"
        print(f"    │ {r['label']:<22s} │ cmd_w={r['cmd_w']:+.2f} ({direction:>3s}) │ "
              f"dur={r['duration']:.1f}s │")
        print(f"    │   Yaw at stop cmd:    {r['yaw_at_stop_deg']:+7.1f}°                       │")
        print(f"    │   Coast overshoot:     {r['overshoot_deg']:+7.1f}°                       │")
        print(f"    │   TOTAL yaw:           {r['total_yaw_deg']:+7.1f}°                       │")
        print(f"    │   Peak odom ω:         {r['peak_odom_w']:+6.3f} rad/s                   │")
        print(f"    │   Response time:        {r['response_time']:5.3f}s                        │")
        settle_str = f"{r['settle_time']:5.3f}s" if r['settle_time'] > 0 else ">3.0s"
        print(f"    │   Settle time (after):  {settle_str}                        │")
        print(f"    ├──────────────────────────────────────────────────────────────┤")

    def print_timeline(self, r, phase='during', max_lines=12):
        """Print sampled timeline for a phase."""
        samples = r[f'samples_{phase}']
        if not samples:
            return
        step = max(1, len(samples) // max_lines)
        phase_label = "COMMANDING" if phase == 'during' else "COAST (after stop)"
        print(f"    │   [{phase_label}]")
        print(f"    │   {'Time':>6s}  {'Yaw°':>8s}  {'odom_w':>8s}")
        for i in range(0, len(samples), step):
            s = samples[i]
            print(f"    │   {s['t']:6.2f}s  {s['yaw_deg']:+8.1f}°  {s['odom_w']:+8.3f}")
        # Always show last sample
        s = samples[-1]
        print(f"    │   {s['t']:6.2f}s  {s['yaw_deg']:+8.1f}°  {s['odom_w']:+8.3f}  (final)")

    def run_all(self):
        """Run the complete rotation diagnostic suite."""
        print()
        print("    ╔══════════════════════════════════════════════════════════════╗")
        print("    ║          ROTATION BEHAVIOR DIAGNOSTIC                       ║")
        print("    ╠══════════════════════════════════════════════════════════════╣")
        print("    ║ This script commands rotation at various angles & speeds,   ║")
        print("    ║ then measures actual yaw, overshoot, coast, and settling.   ║")
        print("    ║                                                             ║")
        print("    ║ MAKE SURE:                                                  ║")
        print("    ║   1. bringup_hardware.launch.py is running                  ║")
        print("    ║   2. Robot is on the ground with space to rotate            ║")
        print("    ║   3. /odom topic is publishing (EKF running)                ║")
        print("    ╚══════════════════════════════════════════════════════════════╝")
        print()

        if not self._wait_odom():
            print("    ❌ No odometry data! Is bringup_hardware running?")
            return

        print("    ✓ Odometry connected.")
        print()
        input("    Press ENTER to start rotation diagnostics...\n")

        results = []

        # ────────────────────────────────────────────────────────────
        # TEST GROUP 1: Fixed duration (1.0s), varying angular velocity
        # Purpose: See how speed affects actual rotation & overshoot
        # ────────────────────────────────────────────────────────────
        print("    ══════════════════════════════════════════════════════════════")
        print("    GROUP 1: Fixed 1.0s duration, varying angular velocity")
        print("    Purpose: How does cmd_w affect actual rotation & overshoot?")
        print("    ══════════════════════════════════════════════════════════════")
        print()

        for w in [0.2, 0.3, 0.5, 0.8]:
            input(f"    Press ENTER for test: w={w} rad/s, 1.0s (CCW)...")
            r = self.run_single_test(w, 1.0, f"w={w} 1.0s CCW")
            self.print_result(r)
            self.print_timeline(r, 'during')
            self.print_timeline(r, 'coast')
            results.append(r)
            print()

        # ────────────────────────────────────────────────────────────
        # TEST GROUP 2: Fixed angular velocity (0.5), varying duration
        # Purpose: See if longer commands → proportional rotation
        # ────────────────────────────────────────────────────────────
        print("    ══════════════════════════════════════════════════════════════")
        print("    GROUP 2: Fixed w=0.5 rad/s, varying duration")
        print("    Purpose: Is rotation proportional to command duration?")
        print("    ══════════════════════════════════════════════════════════════")
        print()

        for dur in [0.5, 1.0, 2.0, 3.0]:
            input(f"    Press ENTER for test: w=0.5 rad/s, {dur}s (CCW)...")
            r = self.run_single_test(0.5, dur, f"w=0.5 {dur}s CCW")
            self.print_result(r)
            self.print_timeline(r, 'during')
            self.print_timeline(r, 'coast')
            results.append(r)
            print()

        # ────────────────────────────────────────────────────────────
        # TEST GROUP 3: CW vs CCW symmetry at w=0.5
        # Purpose: Check if rotation is symmetric in both directions
        # ────────────────────────────────────────────────────────────
        print("    ══════════════════════════════════════════════════════════════")
        print("    GROUP 3: CW vs CCW symmetry (w=±0.5, 1.5s)")
        print("    Purpose: Is rotation symmetric in both directions?")
        print("    ══════════════════════════════════════════════════════════════")
        print()

        input("    Press ENTER for test: w=+0.5 rad/s, 1.5s (CCW)...")
        r = self.run_single_test(0.5, 1.5, "w=+0.5 1.5s CCW")
        self.print_result(r)
        self.print_timeline(r, 'during')
        self.print_timeline(r, 'coast')
        results.append(r)
        print()

        input("    Press ENTER for test: w=-0.5 rad/s, 1.5s (CW)...")
        r = self.run_single_test(-0.5, 1.5, "w=-0.5 1.5s CW")
        self.print_result(r)
        self.print_timeline(r, 'during')
        self.print_timeline(r, 'coast')
        results.append(r)
        print()

        # ────────────────────────────────────────────────────────────
        # TEST GROUP 4: Target angle tests (Nav2-like behavior)
        # Purpose: Simulate what Nav2 does — rotate to a heading
        # Commands rotation, monitors yaw, stops when target reached
        # ────────────────────────────────────────────────────────────
        print("    ══════════════════════════════════════════════════════════════")
        print("    GROUP 4: Target angle tests (Nav2-like rotate-to-heading)")
        print("    Purpose: Command w=0.3 until target angle, measure overshoot")
        print("    ══════════════════════════════════════════════════════════════")
        print()

        for target_deg in [45, 90, 135, 180]:
            input(f"    Press ENTER for test: rotate to {target_deg}° (CCW, w=0.3)...")
            r = self.run_target_angle_test(0.3, target_deg, f"target {target_deg}° CCW")
            self.print_target_result(r)
            self.print_timeline(r, 'during')
            self.print_timeline(r, 'coast')
            results.append(r)
            print()

        # ────────────────────────────────────────────────────────────
        # TEST GROUP 5: Quick stop-start bursts (Nav2 oscillation sim)
        # Purpose: Simulate Nav2 sending short rotation pulses
        # ────────────────────────────────────────────────────────────
        print("    ══════════════════════════════════════════════════════════════")
        print("    GROUP 5: Short burst tests (Nav2 pulse simulation)")
        print("    Purpose: Short 0.3s pulses — how much does robot actually turn?")
        print("    ══════════════════════════════════════════════════════════════")
        print()

        for w in [0.3, 0.5]:
            input(f"    Press ENTER for test: 3x short bursts at w={w}...")
            burst_results = []
            for i in range(3):
                r = self.run_single_test(w, 0.3, f"burst{i+1} w={w}")
                burst_results.append(r)
                print(f"    │ Burst {i+1}: total={r['total_yaw_deg']:+.1f}° "
                      f"(cmd={r['yaw_at_stop_deg']:+.1f}° + coast={r['overshoot_deg']:+.1f}°)")
            avg_total = sum(b['total_yaw_deg'] for b in burst_results) / 3
            avg_overshoot = sum(b['overshoot_deg'] for b in burst_results) / 3
            print(f"    │ Average: total={avg_total:+.1f}°, overshoot={avg_overshoot:+.1f}°")
            print(f"    ├──────────────────────────────────────────────────────────────┤")
            results.extend(burst_results)
            print()

        # ────────────────────────────────────────────────────────────
        # SUMMARY
        # ────────────────────────────────────────────────────────────
        self.print_summary(results)

    def run_target_angle_test(self, cmd_w, target_deg, label):
        """
        Rotate until yaw reaches target_deg, then stop.
        Measures overshoot after stop command.
        """
        self._stop(1.0)
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.02)

        target_rad = math.radians(target_deg)
        last_yaw = self.yaw
        cumulative = 0.0
        peak_odom_w = 0.0
        start_response_time = None

        msg = Twist()
        msg.angular.z = float(cmd_w)
        t0 = time.time()

        samples_during = []
        # Command until target reached or 10s timeout
        while time.time() - t0 < 10.0:
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)
            delta, last_yaw = self._track_yaw(last_yaw)
            cumulative += delta

            if abs(self.odom_w) > abs(peak_odom_w):
                peak_odom_w = self.odom_w

            if start_response_time is None and abs(self.odom_w) > 0.05:
                start_response_time = time.time() - t0

            elapsed = time.time() - t0
            samples_during.append({
                't': elapsed,
                'yaw_deg': math.degrees(cumulative),
                'odom_w': self.odom_w
            })

            # Stop when target reached
            if abs(cumulative) >= target_rad:
                break

            time.sleep(0.04)

        cmd_duration = time.time() - t0
        yaw_at_stop = cumulative

        # Coast phase
        stop_msg = Twist()
        t_stop = time.time()
        cumulative_after = 0.0
        settle_time = None
        samples_coast = []

        while time.time() - t_stop < 3.0:
            self.cmd_pub.publish(stop_msg)
            rclpy.spin_once(self, timeout_sec=0.01)
            delta, last_yaw = self._track_yaw(last_yaw)
            cumulative_after += delta

            elapsed_after = time.time() - t_stop
            samples_coast.append({
                't': elapsed_after,
                'yaw_deg': math.degrees(cumulative_after),
                'odom_w': self.odom_w
            })

            if settle_time is None and elapsed_after > 0.1 and abs(self.odom_w) < 0.03:
                settle_time = elapsed_after

            time.sleep(0.04)

        return {
            'label': label,
            'cmd_w': cmd_w,
            'target_deg': target_deg,
            'duration': cmd_duration,
            'yaw_at_stop_deg': math.degrees(yaw_at_stop),
            'overshoot_deg': math.degrees(cumulative_after),
            'total_yaw_deg': math.degrees(yaw_at_stop + cumulative_after),
            'error_deg': math.degrees(yaw_at_stop + cumulative_after) - target_deg,
            'peak_odom_w': peak_odom_w,
            'response_time': start_response_time if start_response_time else -1,
            'settle_time': settle_time if settle_time else -1,
            'samples_during': samples_during,
            'samples_coast': samples_coast,
        }

    def print_target_result(self, r):
        """Print target-angle test result."""
        print(f"    │ {r['label']:<22s} │ cmd_w={r['cmd_w']:+.2f}   │ "
              f"target={r['target_deg']}°       │")
        print(f"    │   Command duration:    {r['duration']:5.2f}s                          │")
        print(f"    │   Yaw at stop cmd:    {r['yaw_at_stop_deg']:+7.1f}°                       │")
        print(f"    │   Coast overshoot:     {r['overshoot_deg']:+7.1f}°                       │")
        print(f"    │   TOTAL yaw:           {r['total_yaw_deg']:+7.1f}°                       │")
        print(f"    │   ERROR from target:   {r['error_deg']:+7.1f}°                       │")
        print(f"    │   Peak odom ω:         {r['peak_odom_w']:+6.3f} rad/s                   │")
        settle_str = f"{r['settle_time']:5.3f}s" if r['settle_time'] > 0 else ">3.0s"
        print(f"    │   Settle time (after):  {settle_str}                        │")
        print(f"    ├──────────────────────────────────────────────────────────────┤")

    def print_summary(self, results):
        """Print final summary of all tests."""
        print()
        print("    ╔══════════════════════════════════════════════════════════════╗")
        print("    ║                   ROTATION DIAGNOSTIC SUMMARY               ║")
        print("    ╚══════════════════════════════════════════════════════════════╝")
        print()
        print(f"    {'Test':<24s} {'cmd_w':>6s} {'dur':>5s} {'@stop':>7s} "
              f"{'coast':>7s} {'total':>7s} {'peak_w':>7s} {'settle':>6s}")
        print(f"    {'─'*24} {'─'*6} {'─'*5} {'─'*7} {'─'*7} {'─'*7} {'─'*7} {'─'*6}")

        overshoot_values = []
        for r in results:
            settle_str = f"{r['settle_time']:.2f}" if r['settle_time'] > 0 else ">3.0"
            print(f"    {r['label']:<24s} {r['cmd_w']:+5.2f} {r['duration']:5.1f} "
                  f"{r['yaw_at_stop_deg']:+6.1f}° {r['overshoot_deg']:+6.1f}° "
                  f"{r['total_yaw_deg']:+6.1f}° {r['peak_odom_w']:+6.3f} {settle_str:>6s}")
            overshoot_values.append(abs(r['overshoot_deg']))

        print()
        if overshoot_values:
            avg_overshoot = sum(overshoot_values) / len(overshoot_values)
            max_overshoot = max(overshoot_values)
            print(f"    Average coast overshoot: {avg_overshoot:.1f}°")
            print(f"    Maximum coast overshoot: {max_overshoot:.1f}°")

        print()
        print("    ═══════════════════════════════════════════════════════════════")
        print("    KEY OBSERVATIONS TO REPORT:")
        print("    ═══════════════════════════════════════════════════════════════")
        print("    1. Is coast overshoot consistently large (>20°)?")
        print("       → Needs active braking or momentum compensation")
        print("    2. Is overshoot near-zero but robot still spins 360°?")
        print("       → Problem is in Nav2 controller, not motor layer")
        print("    3. Is there a response delay before rotation starts?")
        print("       → Soft-start or dead-zone issue")
        print("    4. Is CW ≠ CCW (asymmetric rotation)?")
        print("       → Motor trim or mechanical issue")
        print("    5. Do short bursts produce unpredictable rotation?")
        print("       → Motor nonlinearity at low PWM")
        print()
        print("    Copy ALL output above and share it for tuning decisions.")
        print()


def main():
    rclpy.init()
    node = RotationTest()
    try:
        node.run_all()
    except KeyboardInterrupt:
        # Stop motors on Ctrl+C
        msg = Twist()
        for _ in range(10):
            node.cmd_pub.publish(msg)
            time.sleep(0.02)
        print("\n    Interrupted — motors stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
