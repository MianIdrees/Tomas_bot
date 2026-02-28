#!/usr/bin/env python3
"""
diff_drive_node.py - ROS2 Serial Bridge for Arduino Leonardo Motor Controller

This node bridges ROS2 and the Arduino Leonardo motor controller on LattePanda Alpha:
  - Subscribes to /cmd_vel (Twist) -> sends PWM commands to Arduino Leonardo
  - Reads encoder ticks from Arduino -> publishes /odom (Odometry) and TF odom->base_link
  - Publishes /joint_states for wheel joint visualization

Hardware: Daniel's Robot - LattePanda Alpha + built-in Arduino Leonardo
  Motors: 130 RPM 12V DC with quadrature encoders (11 PPR)
  Wheels: 69mm diameter (0.0345m radius)
  Wheel separation (center-to-center): 0.181m
  Gear ratio: ~48:1 -> ticks_per_rev ~= 528

Differential Drive Kinematics:
  Forward kinematics (encoders -> odometry):
        v_left  = (delta_left_ticks  / ticks_per_rev) * 2*pi * wheel_radius / dt
        v_right = (delta_right_ticks / ticks_per_rev) * 2*pi * wheel_radius / dt
        v = (v_right + v_left) / 2
        omega = (v_right - v_left) / wheel_separation

    Inverse kinematics (cmd_vel -> motor PWM):
        v_left  = v - omega * wheel_separation / 2
        v_right = v + omega * wheel_separation / 2
    PWM proportional to velocity (with configurable scaling)

Serial Protocol (USB CDC, 115200 baud):
    TX to Arduino:  m <left_pwm> <right_pwm>\n
    RX from Arduino: e <left_ticks> <right_ticks>\n
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

import serial


def quaternion_from_yaw(yaw):
    """Create a Quaternion message from a yaw angle."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class DiffDriveNode(Node):
    def __init__(self):
        super().__init__('diff_drive_node')

        # ========================== PARAMETERS ==========================
        self.declare_parameter('serial_port', '/dev/arduino')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_separation', 0.181)  # Center-to-center: 181mm
        self.declare_parameter('wheel_radius', 0.0345)     # 69mm wheels
        self.declare_parameter('ticks_per_rev', 528.0)     # 11 PPR x 48:1 gear (CALIBRATE)
        self.declare_parameter('max_motor_speed', 0.4)    # [SPEED] Hardware max: 130 RPM × π × 0.069m ≈ 0.47 m/s
        self.declare_parameter('min_pwm', 17)                # Must match Arduino firmware MIN_PWM — motor dead zone threshold
        self.declare_parameter('angular_deadband', 0.05)     # rad/s — filter only jitter, allow real turn commands through
        self.declare_parameter('pwm_ramp_rate', 6)          # Max PWM change per cmd_vel cycle (smooth accel/decel)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_rate', 20.0)       # Hz

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.wheel_sep = self.get_parameter('wheel_separation').value
        self.wheel_rad = self.get_parameter('wheel_radius').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        self.max_motor_speed = self.get_parameter('max_motor_speed').value
        self.min_pwm = self.get_parameter('min_pwm').value
        self.angular_deadband = self.get_parameter('angular_deadband').value
        self.pwm_ramp_rate = self.get_parameter('pwm_ramp_rate').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # Derived constants
        self.meters_per_tick = (2.0 * math.pi * self.wheel_rad) / self.ticks_per_rev
        # PWM scaling: PWM_value = velocity / max_motor_speed * 255
        self.pwm_per_mps = 255.0 / self.max_motor_speed

        # ========================== ODOMETRY STATE ==========================
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v_linear = 0.0
        self.v_angular = 0.0

        self.last_left_ticks = None
        self.last_right_ticks = None
        self.last_odom_time = None

        # Wheel positions for joint_states (radians)
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0

        # ========================== SERIAL CONNECTION ==========================
        self.ser = None
        self.serial_lock = threading.Lock()
        self.connect_serial()

        # ========================== ROS2 INTERFACES ==========================
        # Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )

        # Publishers
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer for reading serial and publishing odometry
        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self.update_callback)

        # Watchdog: stop motors if no cmd_vel for 0.5s
        self.last_cmd_time = self.get_clock().now()
        self.cmd_timeout = 0.5  # seconds

        # Store last motor command for continuous resending
        self.last_left_pwm = 0
        self.last_right_pwm = 0

        self.get_logger().info(
            f'DiffDriveNode started: port={self.serial_port}, '
            f'wheel_sep={self.wheel_sep}, wheel_rad={self.wheel_rad}, '
            f'ticks_per_rev={self.ticks_per_rev}'
        )

    def connect_serial(self):
        """Attempt to open the serial port to the Arduino."""
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=0.05
            )
            # Wait for Arduino to reset after serial connection
            time.sleep(2.0)
            # Flush any startup messages
            self.ser.reset_input_buffer()
            self.get_logger().info(f'Connected to Arduino on {self.serial_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {self.serial_port}: {e}')
            self.ser = None

    def cmd_vel_callback(self, msg: Twist):
        """Convert cmd_vel to motor PWM commands with 3-layer intelligent control.

        The layers work together to produce smooth, reliable motion:

        Layer 1 — Deadband: Filters tiny jitter commands (< 0.03 rad/s angular,
          < 0.01 m/s linear) that cause goal oscillation. The velocity_smoother
          also filters angular < 0.05 rad/s, so this mainly catches teleop noise.

        Layer 2 — Rescale: Maps raw PWM [1..255] → [min_pwm..255] so that ANY
          non-zero command produces enough voltage to overcome motor stiction.
          Without this, turn commands (which only need ~49 PWM per wheel) pass
          through the velocity_smoother ramp as 5→11→17→... and those low values
          produce Arduino PID targets of 1-3 ticks — too low for reliable control.

        Layer 3 — Ramp: Limits how fast the rescaled PWM changes per cycle.
          This prevents jerky starts. Stopping is IMMEDIATE (no ramp delay).
          Because ramping operates on rescaled values, even the first ramp step
          (e.g., 30 PWM) is above the motor's working threshold.
        """
        self.last_cmd_time = self.get_clock().now()

        v = msg.linear.x   # m/s
        w = msg.angular.z   # rad/s

        # --- Layer 1: Deadband ---
        if abs(w) < self.angular_deadband:
            w = 0.0
        if abs(v) < 0.01:
            v = 0.0

        # Inverse kinematics: compute wheel velocities
        v_left = v - (w * self.wheel_sep / 2.0)
        v_right = v + (w * self.wheel_sep / 2.0)

        # Convert to raw PWM (-255 to 255)
        left_pwm = int(v_left * self.pwm_per_mps)
        right_pwm = int(v_right * self.pwm_per_mps)

        # Clamp to valid range
        left_pwm = max(-255, min(255, left_pwm))
        right_pwm = max(-255, min(255, right_pwm))

        # --- Layer 2: Rescale into motor's working range ---
        left_pwm = self._rescale_pwm(left_pwm)
        right_pwm = self._rescale_pwm(right_pwm)

        # --- Layer 3: Smooth ramp (operates on rescaled values) ---
        left_pwm = self._ramp_pwm(left_pwm, self.last_left_pwm)
        right_pwm = self._ramp_pwm(right_pwm, self.last_right_pwm)

        # Store and send
        self.last_left_pwm = left_pwm
        self.last_right_pwm = right_pwm
        self.send_motor_command(left_pwm, right_pwm)

    def _rescale_pwm(self, pwm):
        """Map [1..255] → [min_pwm..255] so any command moves the motor.

        This eliminates the dead zone where PWM < 35 produces no motion.
        The mapping is linear and preserves proportionality:
          raw   0 →   0  (stopped)
          raw   1 →  35  (minimum working PWM)
          raw  49 →  77  (typical turn speed)
          raw 128 → 145  (medium speed)
          raw 255 → 255  (full speed)
        """
        if pwm == 0:
            return 0
        sign = 1 if pwm > 0 else -1
        abs_pwm = abs(pwm)
        scaled = self.min_pwm + (abs_pwm - 1) * (255 - self.min_pwm) / 254.0
        return sign * int(round(scaled))

    def _ramp_pwm(self, target, current):
        """Limit PWM change per cycle for smooth acceleration.

        Operates on rescaled values so even the first ramp step is above
        the motor's working threshold. At 10Hz smoother rate with ramp=30:
          0→77 PWM (turn) ramps in ~0.3s: 0→30→60→77
          0→77 only takes 3 cycles because rescale ensures values above 35.
        Stopping is IMMEDIATE — no delay when Nav2 says stop.
        """
        if target == 0:
            return 0
        # Direction reversal: stop first, then ramp in new direction
        if (target > 0 and current < 0) or (target < 0 and current > 0):
            return 0
        diff = target - current
        if abs(diff) <= self.pwm_ramp_rate:
            return target
        return current + self.pwm_ramp_rate * (1 if diff > 0 else -1)

    def send_motor_command(self, left_pwm, right_pwm):
        """Send motor command to Arduino via serial."""
        if self.ser is None or not self.ser.is_open:
            return
        cmd = f'm {left_pwm} {right_pwm}\n'
        try:
            with self.serial_lock:
                self.ser.write(cmd.encode('ascii'))
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial write error: {e}')

    def read_encoder_data(self):
        """Read and parse encoder data from Arduino. Returns (left_ticks, right_ticks) or None."""
        if self.ser is None or not self.ser.is_open:
            return None

        try:
            with self.serial_lock:
                # Read all available lines, use the latest encoder message
                latest = None
                while self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('ascii', errors='ignore').strip()
                    if line.startswith('e '):
                        latest = line

                if latest is None:
                    return None

                parts = latest.split()
                if len(parts) == 3:
                    left_ticks = int(parts[1])
                    right_ticks = int(parts[2])
                    # Negate both encoders: hardware wiring gives inverted sign
                    # (forward motion produces negative ticks from Arduino)
                    return (-left_ticks, -right_ticks)
        except (serial.SerialException, ValueError, UnicodeDecodeError) as e:
            self.get_logger().warn(f'Serial read error: {e}')

        return None

    def update_callback(self):
        """Main update loop: read encoders, compute odometry, publish."""
        now = self.get_clock().now()

        # Watchdog: stop motors if cmd_vel timeout, otherwise resend last command
        # Continuous resending prevents Arduino firmware timeout and keeps PID stable
        dt_cmd = (now - self.last_cmd_time).nanoseconds / 1e9
        if dt_cmd > self.cmd_timeout:
            self.last_left_pwm = 0
            self.last_right_pwm = 0
            self.send_motor_command(0, 0)
        else:
            # Resend last command at 20Hz to keep Arduino PID continuously active
            self.send_motor_command(self.last_left_pwm, self.last_right_pwm)

        # Read encoders
        enc_data = self.read_encoder_data()
        if enc_data is None:
            return

        left_ticks, right_ticks = enc_data

        # Initialize on first reading
        if self.last_left_ticks is None:
            self.last_left_ticks = left_ticks
            self.last_right_ticks = right_ticks
            self.last_odom_time = now
            return

        # Time delta
        dt = (now - self.last_odom_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        # Compute deltas
        delta_left = left_ticks - self.last_left_ticks
        delta_right = right_ticks - self.last_right_ticks

        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks
        self.last_odom_time = now

        # Distance traveled by each wheel
        dist_left = delta_left * self.meters_per_tick
        dist_right = delta_right * self.meters_per_tick

        # Update wheel positions (radians) for joint_states
        self.left_wheel_pos += delta_left * (2.0 * math.pi / self.ticks_per_rev)
        self.right_wheel_pos += delta_right * (2.0 * math.pi / self.ticks_per_rev)

        # Differential drive forward kinematics
        dist_center = (dist_right + dist_left) / 2.0
        delta_theta = (dist_right - dist_left) / self.wheel_sep

        # Update pose
        if abs(delta_theta) < 1e-6:
            # Straight line
            self.x += dist_center * math.cos(self.theta)
            self.y += dist_center * math.sin(self.theta)
        else:
            # Arc
            radius = dist_center / delta_theta
            self.x += radius * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
            self.y -= radius * (math.cos(self.theta + delta_theta) - math.cos(self.theta))

        self.theta += delta_theta
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Velocities
        self.v_linear = dist_center / dt
        self.v_angular = delta_theta / dt

        # Publish odometry
        self.publish_odometry(now)

        # Publish joint states
        self.publish_joint_states(now)

        # Publish TF
        if self.publish_tf:
            self.publish_odom_tf(now)

    def publish_odometry(self, stamp):
        """Publish /odom message."""
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        # Pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion_from_yaw(self.theta)

        # Pose covariance (diagonal) — higher values tell SLAM/AMCL that odometry
        # is less reliable, so scan matching is trusted more (reduces map drift)
        # [TUNING] Increase these if map still drifts; decrease if map is jittery
        odom.pose.covariance[0] = 0.05   # x position variance (meters²)
        odom.pose.covariance[7] = 0.05   # y position variance (meters²)
        odom.pose.covariance[35] = 0.1   # yaw orientation variance (radians²)

        # Twist
        odom.twist.twist.linear.x = self.v_linear
        odom.twist.twist.angular.z = self.v_angular

        # Twist covariance — how noisy the velocity estimates are
        # [TUNING] Increase if velocity estimates are inaccurate
        odom.twist.covariance[0] = 0.05  # linear velocity variance
        odom.twist.covariance[35] = 0.1  # angular velocity variance

        self.odom_pub.publish(odom)

    def publish_joint_states(self, stamp):
        """Publish /joint_states for wheel joints (needed for URDF visualization)."""
        js = JointState()
        js.header.stamp = stamp.to_msg()
        js.name = ['left_wheel_joint', 'right_wheel_joint']
        js.position = [self.left_wheel_pos, self.right_wheel_pos]
        js.velocity = []
        js.effort = []
        self.joint_pub.publish(js)

    def publish_odom_tf(self, stamp):
        """Broadcast odom -> base_link transform."""
        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = quaternion_from_yaw(self.theta)

        self.tf_broadcaster.sendTransform(t)

    def destroy_node(self):
        """Clean shutdown: stop motors and close serial."""
        self.get_logger().info('Shutting down - stopping motors')
        self.send_motor_command(0, 0)
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
