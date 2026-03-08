#!/usr/bin/env python3
"""
tuning_node.py — Interactive Robot Tuning Script (Branch: final-2)

Approach: Motion Profile State Machine

This script provides structured tests to tune the robot's state-machine-based
motion controller. It tests each motion state (ROTATING, LINEAR, ARC) separately
and helps you find optimal parameters.

The state machine approach is fundamentally different from branch final-1:
  - Instead of boosting/rescaling PWM uniformly, each motion type has its own
    PWM generation strategy and parameters.
  - Rotation uses a fixed PWM with soft-start ramp (not velocity-proportional).
  - Linear motion uses velocity-proportional PWM with a speed floor.
  - Arc motion scales the angular component to prevent over-steering.
  - Low speeds use duty-cycling (alternating min_pwm/0) instead of rescaling.

IMPORTANT: Run bringup_hardware.launch.py FIRST, then run this script.

Usage:
    ros2 run Tomas_bot tuning_node.py

The script has 7 chapters:
    1. PWM Dead Zone Detection — Find minimum PWM that moves your robot
    2. Rotation State Tuning — Tune in-place rotation (soft-start, fixed PWM)
    3. Linear State Tuning — Tune straight-line speed and approach behavior
    4. Arc State Tuning — Tune curve following with angular scaling
    5. Duty Cycling Test — Verify low-speed duty cycling works
    6. Rotation Watchdog Test — Verify spin protection works
    7. Full Parameter Summary — Print all recommended values
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

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_v = 0.0
        self.current_w = 0.0
        self.odom_received = False

        self.results = {}

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_v = msg.twist.twist.linear.x
        self.current_w = msg.twist.twist.angular.z
        self.odom_received = True

    def send_cmd(self, v, w, duration):
        """Send a cmd_vel command for a specified duration (seconds)."""
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)

        start = time.time()
        rate = self.create_rate(20)
        while time.time() - start < duration:
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)
            rate.sleep()

        stop = Twist()
        for _ in range(5):
            self.cmd_pub.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.05)

    def wait_for_odom(self, timeout=5.0):
        start = time.time()
        while not self.odom_received and time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.odom_received

    def print_header(self, chapter, title):
        print(f'\n{"=" * 70}')
        print(f'  CHAPTER {chapter}: {title}')
        print(f'{"=" * 70}\n')

    def print_param(self, name, value, lower, upper, description):
        print(f'  {name}: {value}')
        print(f'    Range: [{lower} .. {upper}]')
        print(f'    Effect: {description}')
        print()

    # ========================== CHAPTER 1: PWM DEAD ZONE ==========================

    def chapter_1_pwm_deadzone(self):
        self.print_header(1, 'PWM DEAD ZONE DETECTION')
        print('  This test finds the minimum PWM that actually moves your robot.')
        print('  The state machine sends commands above this threshold for all motion.')
        print()
        print('  CURRENT PARAMETERS:')
        print('    min_pwm (diff_drive_node): 45')
        print('    MIN_PWM (Arduino firmware): 45')
        print()

        input('  Press ENTER to start PWM dead zone test...')

        test_speeds = [0.02, 0.03, 0.04, 0.05, 0.06, 0.08, 0.10, 0.12, 0.15, 0.20]
        min_moving_speed = None

        for speed in test_speeds:
            print(f'  Testing v={speed:.2f} m/s ... ', end='', flush=True)
            start_x = self.current_x
            start_y = self.current_y

            self.send_cmd(speed, 0.0, 1.5)
            time.sleep(0.5)

            dx = self.current_x - start_x
            dy = self.current_y - start_y
            dist = math.sqrt(dx*dx + dy*dy)

            if dist > 0.01:
                print(f'MOVED! (distance: {dist:.3f}m)')
                if min_moving_speed is None:
                    min_moving_speed = speed
            else:
                print(f'no movement (distance: {dist:.3f}m)')

            time.sleep(1.0)

        if min_moving_speed:
            pwm_per_mps = 255.0 / 0.391
            raw_pwm = int(min_moving_speed * pwm_per_mps)
            print(f'\n  RESULT: Minimum moving speed = {min_moving_speed:.2f} m/s (raw PWM ≈ {raw_pwm})')
            recommended_min = max(raw_pwm + 5, 45)
            print(f'  RECOMMENDATION: min_pwm = {recommended_min}')
            self.results['min_pwm'] = recommended_min
        else:
            print('\n  WARNING: Robot did not move! Check hardware.')
            self.results['min_pwm'] = 60

        self.print_param('min_pwm', self.results.get('min_pwm', 45),
                         30, 80,
                         'Minimum PWM for all motor commands. The duty-cycle system in the state '
                         'machine will alternate between this PWM and 0 for speeds below this threshold. '
                         'Must match Arduino MIN_PWM exactly.')

    # ========================== CHAPTER 2: ROTATION ==========================

    def chapter_2_rotation(self):
        self.print_header(2, 'ROTATION STATE TUNING (MOST CRITICAL)')
        print('  The state machine uses a FIXED PWM for rotation (not velocity-proportional).')
        print('  This test finds the right rotation_pwm value.')
        print()
        print('  Unlike branch final-1 (PWM rescaling), this approach:')
        print('    - Uses a constant rotation_pwm with soft-start ramping')
        print('    - Scales 0.6x-1.0x based on Nav2 angular velocity magnitude')
        print('    - Has a watchdog that stops rotation after max_duration')
        print()

        input('  Press ENTER to start rotation tests...')

        # Test different angular velocities to see how the state machine handles them
        angular_speeds = [0.2, 0.3, 0.4, 0.5, 0.6]
        for w in angular_speeds:
            target_deg = 90.0
            duration = math.radians(target_deg) / w

            print(f'\n  Testing CCW rotation: w={w:.1f} rad/s, target=90°, duration={duration:.1f}s')
            input(f'    Press ENTER to start...')

            start_yaw = self.current_yaw
            self.send_cmd(0.0, w, duration)
            time.sleep(1.0)

            actual_deg = math.degrees(self.current_yaw - start_yaw)
            while actual_deg > 180: actual_deg -= 360
            while actual_deg < -180: actual_deg += 360

            error = actual_deg - target_deg
            print(f'    Actual rotation: {actual_deg:.1f}° (target: {target_deg:.0f}°, error: {error:+.1f}°)')

            if abs(actual_deg) < 10:
                print(f'    ⚠ PROBLEM: Robot barely rotated! Increase rotation_pwm.')
            elif abs(error) > 45:
                print(f'    ⚠ PROBLEM: Large error! Adjust rotation_pwm or soft_start_steps.')
            else:
                print(f'    ✓ Acceptable rotation')

        # Test soft-start effect
        print(f'\n  Testing CW rotation: w=-0.5 rad/s, target=-90°')
        print(f'    Watch for smooth start (soft-start) vs jerky start')
        input(f'    Press ENTER to start...')
        start_yaw = self.current_yaw
        self.send_cmd(0.0, -0.5, 3.14)
        time.sleep(1.0)
        actual_deg = math.degrees(self.current_yaw - start_yaw)
        while actual_deg > 180: actual_deg -= 360
        while actual_deg < -180: actual_deg += 360
        print(f'    Actual rotation: {actual_deg:.1f}° (target: -90°)')

        print(f'\n  ROTATION STATE PARAMETERS:')
        print()
        self.print_param('rotation_pwm', 70,
                         45, 130,
                         'Fixed base PWM for in-place rotation. Unlike velocity-proportional PWM, '
                         'this guarantees the motors always get enough power to rotate. '
                         'The actual PWM is scaled 0.6x-1.0x based on angular velocity magnitude. '
                         'If robot doesn\'t rotate → increase (try 80, 90, 100). '
                         'If robot over-rotates → decrease (try 60, 55). '
                         'IMPORTANT: This is the PWM after soft-start completes.')
        self.print_param('rotation_soft_start_steps', 5,
                         1, 15,
                         'Number of 50ms cycles to ramp from 0 to rotation_pwm. '
                         'At 5: takes 250ms. At 10: takes 500ms. At 1: instant (no soft-start). '
                         'If robot jerks/overshoots when starting turns → increase (8-10). '
                         'If robot feels sluggish to start rotating → decrease (2-3). '
                         'Critical for preventing 360° overshoot on heavy robots.')
        self.print_param('rotation_max_duration', 8.0,
                         2.0, 15.0,
                         'Watchdog: max seconds of continuous rotation. After this, motors stop. '
                         'At 0.5 rad/s and 8s → max ~230° rotation. '
                         'If robot needs U-turns → keep ≥ 6.0. '
                         'If robot frequently spins out → decrease to 4.0.')

    # ========================== CHAPTER 3: LINEAR ==========================

    def chapter_3_linear(self):
        self.print_header(3, 'LINEAR STATE TUNING')
        print('  Tests straight-line motion with the LINEAR state machine profile.')
        print('  Focus on: smooth start, consistent speed, no stalling near goal.')
        print()

        input('  Press ENTER to test linear motion...')

        speeds = [0.04, 0.06, 0.10, 0.15, 0.20]
        for speed in speeds:
            print(f'\n  Testing forward at v={speed:.2f} m/s for 3 seconds...')
            input(f'    Press ENTER to start...')

            start_x, start_y = self.current_x, self.current_y
            self.send_cmd(speed, 0.0, 3.0)
            time.sleep(1.0)

            dist = math.sqrt((self.current_x - start_x)**2 + (self.current_y - start_y)**2)
            expected = speed * 3.0
            yaw_drift = math.degrees(self.current_yaw)

            print(f'    Distance: {dist:.3f}m (expected ~{expected:.3f}m)')
            accuracy = (dist/expected)*100 if expected > 0 else 0
            print(f'    Speed accuracy: {accuracy:.0f}%')

            if dist < 0.01:
                print(f'    ⚠ No movement! Increase min_pwm or linear_min_speed.')
            elif accuracy < 50:
                print(f'    ⚠ Low accuracy. Check duty cycling settings.')

        print(f'\n  LINEAR STATE PARAMETERS:')
        print()
        self.print_param('linear_ramp_rate', 35,
                         10, 80,
                         'Max PWM change per 50ms cycle for LINEAR state. '
                         'At 35: reaches full speed in ~0.36s. At 20: ~0.64s. At 60: ~0.21s. '
                         'If robot jerks when starting forward → decrease. '
                         'If robot is too slow to start → increase.')
        self.print_param('linear_min_speed', 0.04,
                         0.02, 0.10,
                         'Speed floor while in LINEAR state. Prevents Nav2 approach scaling from '
                         'commanding speeds below motor dead zone. '
                         'If robot stalls near goal → increase (try 0.06, 0.08). '
                         'If robot overshoots goal → decrease (try 0.03).')
        self.print_param('linear_deadband', 0.005,
                         0.001, 0.02,
                         'Linear velocity threshold below which → zero. '
                         'Keep very low to pass Nav2 approach commands through.')

    # ========================== CHAPTER 4: ARC ==========================

    def chapter_4_arc(self):
        self.print_header(4, 'ARC STATE TUNING (CURVE FOLLOWING)')
        print('  Tests combined linear + angular motion (what Nav2 uses for path following).')
        print('  The ARC state scales angular contribution to prevent over-steering.')
        print()

        input('  Press ENTER to test arc motion...')

        # Gentle curve
        print('\n  Test 1: Gentle left curve (v=0.10, w=0.3) for 4s')
        input('    Press ENTER...')
        start_x, start_y = self.current_x, self.current_y
        start_yaw = self.current_yaw
        self.send_cmd(0.10, 0.3, 4.0)
        time.sleep(1.0)
        dist = math.sqrt((self.current_x - start_x)**2 + (self.current_y - start_y)**2)
        heading_change = math.degrees(self.current_yaw - start_yaw)
        print(f'    Displacement: {dist:.3f}m, heading change: {heading_change:.1f}°')

        # Sharp curve
        print('\n  Test 2: Sharp left curve (v=0.05, w=0.5) for 3s')
        input('    Press ENTER...')
        start_x, start_y = self.current_x, self.current_y
        start_yaw = self.current_yaw
        self.send_cmd(0.05, 0.5, 3.0)
        time.sleep(1.0)
        dist = math.sqrt((self.current_x - start_x)**2 + (self.current_y - start_y)**2)
        heading_change = math.degrees(self.current_yaw - start_yaw)
        print(f'    Displacement: {dist:.3f}m, heading change: {heading_change:.1f}°')

        # Right curve
        print('\n  Test 3: Gentle right curve (v=0.10, w=-0.3) for 4s')
        input('    Press ENTER...')
        self.send_cmd(0.10, -0.3, 4.0)
        time.sleep(1.0)

        print(f'\n  ARC STATE PARAMETERS:')
        print()
        self.print_param('arc_ramp_rate', 30,
                         10, 60,
                         'Max PWM change per cycle during arc/curve following. '
                         'Slower than linear ramp because differential PWM causes heading changes. '
                         'If robot wobbles on curves → decrease (try 20). '
                         'If curves feel sluggish → increase (try 45).')
        self.print_param('arc_angular_scale', 0.85,
                         0.5, 1.2,
                         'Scale factor for angular component during arc motion. '
                         'Reduces angular influence when combined with linear to prevent over-steering. '
                         'At 0.85: angular contribution is 85% of calculated value. '
                         'If robot over-steers on curves → decrease (try 0.7). '
                         'If robot under-steers (wide turns) → increase (try 1.0, 1.1).')

    # ========================== CHAPTER 5: DUTY CYCLING ==========================

    def chapter_5_duty_cycling(self):
        self.print_header(5, 'DUTY CYCLING TEST')
        print('  Tests sub-minimum speed duty cycling.')
        print('  The state machine alternates between min_pwm and 0 to achieve')
        print('  effective speeds below the motor dead zone (e.g., for goal approach).')
        print()

        input('  Press ENTER to test very low speeds (duty cycling)...')

        # Test very low speed that should trigger duty cycling
        test_speeds = [0.02, 0.03, 0.04, 0.05]
        for speed in test_speeds:
            print(f'\n  Testing v={speed:.2f} m/s for 4 seconds (should use duty cycling)...')
            input(f'    Press ENTER to start...')

            start_x, start_y = self.current_x, self.current_y
            self.send_cmd(speed, 0.0, 4.0)
            time.sleep(1.0)

            dist = math.sqrt((self.current_x - start_x)**2 + (self.current_y - start_y)**2)
            expected = speed * 4.0
            print(f'    Distance: {dist:.3f}m (expected ~{expected:.3f}m)')

            if dist > 0.005:
                print(f'    ✓ Robot moved! Duty cycling working.')
            else:
                print(f'    ⚠ No movement. Duty cycling may not produce enough average PWM.')

        print(f'\n  DUTY CYCLING PARAMETERS:')
        print()
        self.print_param('duty_cycle_enabled', True,
                         False, True,
                         'Enable/disable duty cycling for sub-min_pwm speeds. '
                         'When True: alternates between min_pwm and 0 for smooth low-speed control. '
                         'When False: clamps to min_pwm (jumpy at low speeds). '
                         'Recommended: True for Nav2 (sends many small velocity adjustments).')
        self.print_param('duty_cycle_period', 4,
                         2, 10,
                         'Number of control cycles per duty period. '
                         'At 4 and 20Hz: period = 200ms. If target is 50% of min → 2 ON, 2 OFF. '
                         'Higher → smoother average (less pulsing). Lower → faster response. '
                         'If low-speed motion is too jerky → increase to 6-8. '
                         'If robot doesn\'t respond fast enough → decrease to 2-3.')

    # ========================== CHAPTER 6: ROTATION WATCHDOG ==========================

    def chapter_6_watchdog(self):
        self.print_header(6, 'ROTATION WATCHDOG TEST')
        print('  This test commands continuous rotation to verify the watchdog stops it.')
        print('  The state machine should stop motors after rotation_max_duration seconds.')
        print()

        input('  Press ENTER to test watchdog (robot will rotate for up to 10 seconds)...')

        print('\n  Commanding w=0.5 rad/s for 10 seconds...')
        print('  Watch: robot should stop after ~8s (rotation_max_duration)')

        start = time.time()
        start_yaw = self.current_yaw
        msg = Twist()
        msg.angular.z = 0.5
        rate = self.create_rate(20)

        while time.time() - start < 10.0:
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)
            rate.sleep()

        stop = Twist()
        for _ in range(5):
            self.cmd_pub.publish(stop)
            time.sleep(0.05)

        total_time = time.time() - start
        print(f'  Test completed after {total_time:.1f}s')
        print('  Check terminal for watchdog messages.')
        print('  If you see "Rotation watchdog" → protection is working.')
        print('  If robot kept spinning → increase rotation_max_duration or check state machine.')

    # ========================== CHAPTER 7: SUMMARY ==========================

    def chapter_7_summary(self):
        self.print_header(7, 'FULL PARAMETER SUMMARY')
        print('  ALL tunable parameters for the state machine approach.')
        print()

        print('  ─── MOTION STATE THRESHOLDS ───')
        print()
        self.print_param('angular_deadband', 0.03,
                         0.01, 0.10,
                         'Angular velocity → zero threshold (rad/s). '
                         'Determines transition between IDLE/ROTATING states. '
                         'Lower → more responsive. Higher → filters more jitter.')
        self.print_param('linear_deadband', 0.005,
                         0.001, 0.02,
                         'Linear velocity → zero threshold (m/s). '
                         'Determines transition between IDLE/LINEAR states.')

        print('  ─── ROTATION STATE ───')
        print()
        self.print_param('rotation_pwm', 70,
                         45, 130,
                         'Fixed base PWM for in-place rotation. Scaled 0.6-1.0x by angular velocity. '
                         'If no rotation → increase. If over-rotation → decrease.')
        self.print_param('rotation_soft_start_steps', 5,
                         1, 15,
                         'Cycles to ramp to rotation_pwm. At 20Hz: steps × 50ms = ramp time. '
                         'More steps → gentler start, less overshoot.')
        self.print_param('rotation_max_duration', 8.0,
                         2.0, 15.0,
                         'Watchdog: max continuous rotation seconds. Safety against runaway spins.')

        print('  ─── LINEAR STATE ───')
        print()
        self.print_param('linear_ramp_rate', 35,
                         10, 80,
                         'PWM change per cycle for linear motion. Lower → smoother. Higher → snappier.')
        self.print_param('linear_min_speed', 0.04,
                         0.02, 0.10,
                         'Speed floor in LINEAR state. Prevents stalling near goal.')

        print('  ─── ARC STATE ───')
        print()
        self.print_param('arc_ramp_rate', 30,
                         10, 60,
                         'PWM change per cycle for curves. Slower than linear for stability.')
        self.print_param('arc_angular_scale', 0.85,
                         0.5, 1.2,
                         'Angular component scaling during arcs. Lower → less steering. Higher → sharper turns.')

        print('  ─── DUTY CYCLING (LOW SPEED) ───')
        print()
        self.print_param('duty_cycle_enabled', True,
                         False, True,
                         'Alternate min_pwm/0 for sub-threshold speeds. Smoother than clamping.')
        self.print_param('duty_cycle_period', 4,
                         2, 10,
                         'Cycles per duty period. Higher → smoother average. Lower → faster response.')

        print('  ─── MOTOR (shared with Arduino) ───')
        print()
        self.print_param('min_pwm', self.results.get('min_pwm', 45),
                         30, 80,
                         'Motor dead zone threshold. MUST match Arduino MIN_PWM. '
                         'Duty cycling generates PWM ≥ this value (or 0).')

        print('  ─── Arduino Firmware (motor_controller.ino) ───')
        print()
        self.print_param('MIN_PWM', self.results.get('min_pwm', 45),
                         30, 80,
                         'MUST match diff_drive_node.py min_pwm. Dead zone in PID.')
        self.print_param('KP', 1.0,
                         0.5, 3.0,
                         'Proportional gain. Higher → faster correction. Too high → oscillation.')
        self.print_param('KI', 0.8,
                         0.0, 2.0,
                         'Integral gain. Higher → better tracking. Too high → overshoot.')
        self.print_param('KD', 0.15,
                         0.0, 1.0,
                         'Derivative gain. Higher → less oscillation. Too high → noise.')
        self.print_param('RAMP_RATE', 10.0,
                         2.0, 30.0,
                         'Arduino-side target ramping. Lower → smoother. Higher → snappier.')
        self.print_param('INTEGRAL_LIMIT', 150.0,
                         50.0, 300.0,
                         'Max integral accumulation (anti-windup).')

        print('  ─── Nav2 Parameters (nav2_params_hardware.yaml) ───')
        print()
        self.print_param('rotate_to_heading_angular_vel', 0.5,
                         0.2, 1.0,
                         'Nav2 angular velocity for in-place rotation. '
                         'State machine scales rotation_pwm by this ÷ 0.8. '
                         'Lower → gentler. Higher → faster turns.')
        self.print_param('rotate_to_heading_min_angle', 0.4,
                         0.2, 1.0,
                         'Min heading error to trigger rotation. Lower → more frequent, softer rotations.')
        self.print_param('min_approach_linear_velocity', 0.05,
                         0.02, 0.10,
                         'Nav2 minimum approach speed. State machine duty-cycles below this.')
        self.print_param('max_angular_accel', 0.5,
                         0.3, 1.5,
                         'Nav2 angular acceleration. Low because state machine provides soft-start.')

    def run(self):
        """Main interactive tuning session."""
        print()
        print('╔══════════════════════════════════════════════════════════════════════╗')
        print('║        TOMAS_BOT TUNING SCRIPT — Branch: final-2                   ║')
        print('║        Motion Profile State Machine                                 ║')
        print('╚══════════════════════════════════════════════════════════════════════╝')
        print()
        print('  This script helps you tune the state-machine-based motion controller.')
        print('  Each motion type (ROTATING, LINEAR, ARC) has separate parameters.')
        print()
        print('  Prerequisites:')
        print('    1. Robot hardware powered on')
        print('    2. ros2 launch Tomas_bot bringup_hardware.launch.py running')
        print('    3. Robot on the ground with space to move')
        print()

        if not self.wait_for_odom(timeout=10.0):
            print('  ERROR: No odometry data received!')
            print('  Make sure bringup_hardware.launch.py is running.')
            return

        print('  ✓ Odometry data received. Robot is ready.')
        print()

        chapters = [
            ('1', 'PWM Dead Zone Detection', self.chapter_1_pwm_deadzone),
            ('2', 'Rotation State Tuning', self.chapter_2_rotation),
            ('3', 'Linear State Tuning', self.chapter_3_linear),
            ('4', 'Arc State Tuning', self.chapter_4_arc),
            ('5', 'Duty Cycling Test', self.chapter_5_duty_cycling),
            ('6', 'Rotation Watchdog Test', self.chapter_6_watchdog),
            ('7', 'Full Parameter Summary', self.chapter_7_summary),
        ]

        while True:
            print('\n  Available chapters:')
            for num, name, _ in chapters:
                print(f'    {num}. {name}')
            print('    a. Run ALL chapters in order')
            print('    q. Quit')
            print()

            choice = input('  Select chapter (1-7, a, or q): ').strip().lower()

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
        stop = Twist()
        for _ in range(10):
            node.cmd_pub.publish(stop)
            time.sleep(0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
