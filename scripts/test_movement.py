#!/usr/bin/env python3
"""Quick test: send forward motion and log wheel odom + IMU during movement."""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import math
import time

class TestMovement(Node):
    def __init__(self):
        super().__init__('test_movement')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/wheel/odom', self.odom_cb, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)
        self.fused_sub = self.create_subscription(Odometry, '/odom', self.fused_cb, 10)

        self.start_time = time.time()
        self.sample_count = 0
        self.phase = 'wait'  # wait -> forward -> stop -> rotate -> stop -> done
        self.phase_start = time.time()

        # Header
        print(f"{'Time':>6} {'Phase':>8} | {'Enc_X':>8} {'Enc_Y':>8} {'Enc_Yaw':>9} "
              f"{'Enc_vx':>8} {'Enc_vz':>8} | {'IMU_gz':>8} {'IMU_qz':>8} {'IMU_qw':>8} "
              f"{'IMU_az':>8} | {'EKF_X':>8} {'EKF_Y':>8} {'EKF_Yaw':>9}")
        print("-" * 140)

        self.last_odom = None
        self.last_imu = None
        self.last_fused = None

        # Capture phase boundaries for summary
        self.phase_snapshots = {}  # phase -> {'start': {...}, 'end': {...}}
        self.prev_phase = 'wait'

        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_cb(self, msg):
        self.last_odom = msg

    def imu_cb(self, msg):
        self.last_imu = msg

    def fused_cb(self, msg):
        self.last_fused = msg

    def quat_to_yaw(self, q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def _snapshot(self):
        """Capture current sensor state as a dict."""
        if not self.last_odom or not self.last_imu:
            return None
        o = self.last_odom
        i = self.last_imu
        enc_yaw = self.quat_to_yaw(o.pose.pose.orientation)
        imu_yaw = self.quat_to_yaw(i.orientation)
        f_x = f_y = f_yaw = 0.0
        if self.last_fused:
            f = self.last_fused
            f_x = f.pose.pose.position.x
            f_y = f.pose.pose.position.y
            f_yaw = self.quat_to_yaw(f.pose.pose.orientation)
        return {
            'enc_x': o.pose.pose.position.x,
            'enc_y': o.pose.pose.position.y,
            'enc_yaw': enc_yaw,
            'enc_vx': o.twist.twist.linear.x,
            'enc_vz': o.twist.twist.angular.z,
            'imu_gz': i.angular_velocity.z,
            'imu_yaw': imu_yaw,
            'imu_az': i.linear_acceleration.z,
            'ekf_x': f_x, 'ekf_y': f_y, 'ekf_yaw': f_yaw,
        }

    def control_loop(self):
        t = time.time() - self.start_time
        elapsed_phase = time.time() - self.phase_start

        cmd = Twist()

        # State machine
        if self.phase == 'wait':
            if elapsed_phase > 1.0:
                self.phase = 'forward'
                self.phase_start = time.time()
                print(f"\n>>> PHASE: Forward 0.15 m/s for 2 seconds")
        elif self.phase == 'forward':
            cmd.linear.x = 0.15
            if elapsed_phase > 2.0:
                self.phase = 'pause1'
                self.phase_start = time.time()
                print(f"\n>>> PHASE: Pause 1.5 seconds")
        elif self.phase == 'pause1':
            if elapsed_phase > 1.5:
                self.phase = 'rotate'
                self.phase_start = time.time()
                print(f"\n>>> PHASE: Rotate 0.3 rad/s for 2 seconds")
        elif self.phase == 'rotate':
            cmd.angular.z = 0.3
            if elapsed_phase > 2.0:
                self.phase = 'pause2'
                self.phase_start = time.time()
                print(f"\n>>> PHASE: Pause 1.5 seconds")
        elif self.phase == 'pause2':
            if elapsed_phase > 1.5:
                self.phase = 'backward'
                self.phase_start = time.time()
                print(f"\n>>> PHASE: Backward -0.1 m/s for 1.5 seconds")
        elif self.phase == 'backward':
            cmd.linear.x = -0.1
            if elapsed_phase > 1.5:
                self.phase = 'done'
                self.phase_start = time.time()
                print(f"\n>>> PHASE: Done — stopping")
        elif self.phase == 'done':
            if elapsed_phase > 1.0:
                raise SystemExit(0)

        self.cmd_pub.publish(cmd)

        # Track phase transitions for summary
        snap = self._snapshot()
        if snap:
            if self.phase != self.prev_phase:
                # End of previous phase
                if self.prev_phase not in ('wait', 'done'):
                    if self.prev_phase not in self.phase_snapshots:
                        self.phase_snapshots[self.prev_phase] = {}
                    self.phase_snapshots[self.prev_phase]['end'] = snap
                # Start of new phase
                if self.phase not in ('wait', 'done'):
                    if self.phase not in self.phase_snapshots:
                        self.phase_snapshots[self.phase] = {}
                    self.phase_snapshots[self.phase]['start'] = snap
                self.prev_phase = self.phase

        # Log data every 200ms
        self.sample_count += 1
        if self.sample_count % 2 == 0 and self.last_odom and self.last_imu:
            o = self.last_odom
            i = self.last_imu
            enc_yaw = self.quat_to_yaw(o.pose.pose.orientation)

            f_x = f_y = f_yaw = 0.0
            if self.last_fused:
                f = self.last_fused
                f_x = f.pose.pose.position.x
                f_y = f.pose.pose.position.y
                f_yaw = self.quat_to_yaw(f.pose.pose.orientation)

            print(f"{t:6.1f} {self.phase:>8} | "
                  f"{o.pose.pose.position.x:8.4f} {o.pose.pose.position.y:8.4f} {math.degrees(enc_yaw):9.2f}° "
                  f"{o.twist.twist.linear.x:8.4f} {o.twist.twist.angular.z:8.4f} | "
                  f"{i.angular_velocity.z:8.4f} {i.orientation.z:8.4f} {i.orientation.w:8.4f} "
                  f"{i.linear_acceleration.z:8.4f} | "
                  f"{f_x:8.4f} {f_y:8.4f} {math.degrees(f_yaw):9.2f}°")


def main():
    rclpy.init()
    node = TestMovement()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        # Send stop
        cmd = Twist()
        node.cmd_pub.publish(cmd)

        # Capture final snapshot as 'end' for the last active phase
        snap = node._snapshot()
        if snap:
            for phase in ('backward',):
                if phase in node.phase_snapshots and 'end' not in node.phase_snapshots[phase]:
                    node.phase_snapshots[phase]['end'] = snap

        # Print summary
        print("\n" + "=" * 80)
        print("MOVEMENT TEST SUMMARY")
        print("=" * 80)
        for phase_name, expected in [
            ('forward',  {'cmd': '0.15 m/s linear.x for 2.0s', 'expected_x': 0.30, 'expected_yaw': 0.0}),
            ('rotate',   {'cmd': '0.3 rad/s angular.z for 2.0s', 'expected_x': 0.0, 'expected_yaw': 0.6}),
            ('backward', {'cmd': '-0.1 m/s linear.x for 1.5s', 'expected_x': -0.15, 'expected_yaw': 0.0}),
        ]:
            data = node.phase_snapshots.get(phase_name)
            if not data or 'start' not in data or 'end' not in data:
                print(f"\n  {phase_name.upper()}: No data captured")
                continue
            s, e = data['start'], data['end']
            dx = e['enc_x'] - s['enc_x']
            dy = e['enc_y'] - s['enc_y']
            dist = math.sqrt(dx*dx + dy*dy)
            dyaw_enc = math.degrees(e['enc_yaw'] - s['enc_yaw'])
            # Normalize to [-180, 180]
            dyaw_enc = (dyaw_enc + 180) % 360 - 180
            dyaw_ekf = math.degrees(e['ekf_yaw'] - s['ekf_yaw'])
            dyaw_ekf = (dyaw_ekf + 180) % 360 - 180

            print(f"\n  {phase_name.upper()} ({expected['cmd']})")
            print(f"    Encoder:  dX={dx:+.4f}m  dY={dy:+.4f}m  dist={dist:.4f}m  dYaw={dyaw_enc:+.2f}°")
            print(f"    EKF:      dX={e['ekf_x']-s['ekf_x']:+.4f}m  dY={e['ekf_y']-s['ekf_y']:+.4f}m  dYaw={dyaw_ekf:+.2f}°")

            if phase_name == 'forward':
                exp_dist = expected['expected_x']
                err_dist = abs(dist - exp_dist) / exp_dist * 100
                print(f"    Expected: ~{exp_dist:.2f}m forward, ~0° yaw")
                print(f"    Error:    dist={err_dist:.1f}%  yaw={abs(dyaw_enc):.2f}° (encoder)  yaw={abs(dyaw_ekf):.2f}° (EKF)")
            elif phase_name == 'rotate':
                exp_yaw = math.degrees(expected['expected_yaw'])
                err_yaw = abs(dyaw_enc) - exp_yaw
                print(f"    Expected: ~0m displacement, ~{exp_yaw:.1f}° yaw")
                print(f"    Error:    displacement={dist:.4f}m  yaw_overshoot={err_yaw:+.2f}° (enc)  EKF_yaw_overshoot={abs(dyaw_ekf)-exp_yaw:+.2f}° (EKF)")
            elif phase_name == 'backward':
                exp_dist = abs(expected['expected_x'])
                err_dist = abs(dist - exp_dist) / exp_dist * 100
                print(f"    Expected: ~{exp_dist:.2f}m backward, ~0° yaw")
                print(f"    Error:    dist={err_dist:.1f}%  yaw={abs(dyaw_enc):.2f}° (encoder)  yaw={abs(dyaw_ekf):.2f}° (EKF)")

        print("\n" + "=" * 80)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
