#!/usr/bin/env python3
"""
diff_drive_node.py - ROS2 Serial Bridge for Arduino Leonardo Motor + IMU Controller

BRANCH: final-1 — Enhanced PID + Adaptive Deadband + PWM Rescaling

Approach: Solves sharp-turn stalling, excessive spinning, and slow goal approach by:
  1. PWM rescaling: Maps velocity→PWM above the motor dead zone so every command moves
  2. Rotation boost: Extra PWM kick for in-place turns (pure angular, no linear)
  3. Anti-spin protection: Detects accumulated rotation and clamps angular velocity
  4. Goal approach floor: Prevents the robot from becoming too slow near the goal

This node bridges ROS2 and the Arduino Leonardo controller on LattePanda Alpha:
  - Subscribes to /cmd_vel (Twist) -> sends PWM commands to Arduino Leonardo
  - Reads encoder ticks from Arduino -> publishes /wheel/odom (Odometry)
  - Reads BNO085 IMU data from Arduino -> publishes /imu/data (Imu)
  - Publishes /joint_states for wheel joint visualization
  - TF odom->base_link is handled by robot_localization EKF (not this node)

Serial Protocol (USB CDC, 115200 baud):
    TX to Arduino:  m <left_pwm> <right_pwm>\n
    RX from Arduino: e <left_ticks> <right_ticks>\n
                     i <qw> <qx> <qy> <qz> <ax> <ay> <az> <gx> <gy> <gz>\n
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
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
        self.declare_parameter('max_motor_speed', 0.391)   # [SPEED] Measured: ~48 ticks/50ms × 20Hz × 0.000411 m/tick (raw PWM 255)
        self.declare_parameter('min_pwm', 40)               # Raised from 25: heavy 2.4kg robot needs more PWM to overcome stiction
        self.declare_parameter('angular_deadband', 0.02)    # rad/s — lowered to let small turn commands through (was 0.05)
        self.declare_parameter('linear_deadband', 0.005)    # m/s — very small to catch near-zero linear commands
        self.declare_parameter('pwm_ramp_rate', 40)         # Max PWM change per cmd_vel cycle — slightly slower ramp for stability
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', False)         # EKF publishes odom->base_link TF
        self.declare_parameter('publish_rate', 20.0)        # Hz
        self.declare_parameter('imu_frame', 'imu_link')

        # --- NEW: Rotation boost parameters ---
        self.declare_parameter('rotation_boost_factor', 1.0)
        # [TUNING] Multiplier for PWM during pure in-place rotation (no linear velocity).
        # Applied to raw IK PWM before the min-PWM floor.
        # Range: 0.3 (strong damping) to 2.0 (double PWM). Default: 1.0 (no change).
        # Since rotation skips PWM rescaling, this directly scales the raw command.
        # If robot over-rotates → decrease toward 0.5-0.7
        # If robot under-rotates → increase toward 1.3-1.5

        self.declare_parameter('rotation_min_pwm', 40)
        # [TUNING] Minimum PWM floor for in-place rotation commands.
        # Pure rotation skips PWM rescaling (which over-amplifies rotation), so
        # this is just a simple floor on the raw IK PWM.
        # MUST be >= Arduino MIN_PWM (40) to avoid duty-cycling jerk.
        # Range: 40 to 60.
        # At 30: below Arduino MIN_PWM → duty-cycling between 0 and 40 = jerky rotation.
        # At 40: matches Arduino MIN_PWM → smooth continuous rotation.
        # At 50+: risk of over-rotation.
        # If Nav2 turns too slow → increase (try 45)
        # If robot over-shoots turns → decrease to 40 (never below)

        # --- NEW: Anti-spin protection ---
        self.declare_parameter('max_continuous_rotation_deg', 270.0)
        # [TUNING] Maximum degrees of continuous rotation before clamping angular velocity.
        # Prevents 360° runaway spins. Accumulates rotation from angular velocity feedback.
        # Range: 90.0 to 540.0.
        # If robot needs to do U-turns (180°), keep this >= 200.
        # If robot spins out of control frequently → decrease to 180.
        # If robot can't complete needed turns → increase to 360.

        self.declare_parameter('anti_spin_angular_clamp', 0.3)
        # [TUNING] When anti-spin triggers, clamp angular velocity to this value (rad/s).
        # Range: 0.0 to 0.5. Set to 0.0 to fully stop rotation when limit hit.
        # Higher values allow gentle turns even after anti-spin triggers.

        # --- NEW: Goal approach floor ---
        self.declare_parameter('approach_min_linear_speed', 0.04)
        # [TUNING] Minimum linear velocity (m/s) when the robot is moving forward.
        # Prevents Nav2's approach scaling from commanding speeds below motor dead zone.
        # Range: 0.02 to 0.10. Must produce PWM >= min_pwm after rescaling.
        # If robot stalls near goal → increase (try 0.06, 0.08)
        # If robot overshoots goal → decrease toward 0.03

        self.declare_parameter('use_pwm_rescaling', True)
        # [TUNING] Enable/disable PWM rescaling.
        # When True: maps [0..255] → [min_pwm..255], so every non-zero command moves the motor.
        # When False: uses raw PWM with min_pwm clamping (original behavior).
        # Rescaling is better for smooth speed control; clamping is simpler to debug.

        # Load all parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.wheel_sep = self.get_parameter('wheel_separation').value
        self.wheel_rad = self.get_parameter('wheel_radius').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        self.max_motor_speed = self.get_parameter('max_motor_speed').value
        self.min_pwm = self.get_parameter('min_pwm').value
        self.angular_deadband = self.get_parameter('angular_deadband').value
        self.linear_deadband = self.get_parameter('linear_deadband').value
        self.pwm_ramp_rate = self.get_parameter('pwm_ramp_rate').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.imu_frame = self.get_parameter('imu_frame').value

        self.rotation_boost_factor = self.get_parameter('rotation_boost_factor').value
        self.rotation_min_pwm = self.get_parameter('rotation_min_pwm').value
        self.max_continuous_rotation_rad = math.radians(self.get_parameter('max_continuous_rotation_deg').value)
        self.anti_spin_angular_clamp = self.get_parameter('anti_spin_angular_clamp').value
        self.approach_min_linear_speed = self.get_parameter('approach_min_linear_speed').value
        self.use_pwm_rescaling = self.get_parameter('use_pwm_rescaling').value

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

        # ========================== ANTI-SPIN STATE ==========================
        self.accumulated_rotation = 0.0      # radians accumulated in same direction
        self.last_angular_sign = 0           # +1, -1, or 0
        self.anti_spin_active = False
        self.last_odom_angular = 0.0         # actual odom angular velocity for anti-spin

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
        self.odom_pub = self.create_publisher(Odometry, '/wheel/odom', qos)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', qos)

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
        self.pwm_log_counter = 0  # Periodic PWM diagnostic logging

        self.get_logger().info(
            f'DiffDriveNode started: port={self.serial_port}, '
            f'wheel_sep={self.wheel_sep}, wheel_rad={self.wheel_rad}, '
            f'ticks_per_rev={self.ticks_per_rev}'
        )
        self.get_logger().info(
            f'Motor params: max_speed={self.max_motor_speed} m/s, '
            f'min_pwm={self.min_pwm}, rotation_min_pwm={self.rotation_min_pwm}, '
            f'ramp_rate={self.pwm_ramp_rate}, pwm_per_mps={self.pwm_per_mps:.1f}'
        )
        self.get_logger().info(
            f'Rotation: boost={self.rotation_boost_factor}, '
            f'anti_spin_limit={math.degrees(self.max_continuous_rotation_rad):.0f}deg, '
            f'approach_min={self.approach_min_linear_speed} m/s, '
            f'rescaling={"ON" if self.use_pwm_rescaling else "OFF"}'
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
        """Convert cmd_vel to motor PWM commands.

        Pipeline (Branch final-1 — Enhanced PID + Adaptive Deadband):
        1. Deadband: Filter jitter with separate angular/linear thresholds
        2. Anti-spin: Track accumulated rotation, clamp if excessive
        3. Approach floor: Enforce minimum linear speed when moving
        4. Inverse kinematics: cmd_vel → per-wheel velocities → raw PWM
        5. Rotation boost: Extra PWM for pure in-place turns
        6. PWM rescaling: Map to usable range above motor dead zone
        7. Ramp: Limit PWM change per cycle for smooth acceleration
        8. Arduino PID handles final speed tracking
        """
        self.last_cmd_time = self.get_clock().now()

        v = msg.linear.x   # m/s
        w = msg.angular.z   # rad/s

        # --- Layer 1: Deadband ---
        if abs(w) < self.angular_deadband:
            w = 0.0
        if abs(v) < self.linear_deadband:
            v = 0.0

        # --- Layer 2: Anti-spin protection ---
        # Track accumulated rotation using ACTUAL odom angular velocity (not commanded).
        # This is critical because the boosted PWM can make the robot spin 3-7x faster
        # than the commanded angular velocity. Using commanded w would grossly
        # underestimate actual rotation and let the robot spin out of control.
        actual_w = self.last_odom_angular  # from latest odometry callback
        if w != 0.0:
            current_sign = 1 if w > 0 else -1
            if current_sign == self.last_angular_sign:
                # Same direction: accumulate using ACTUAL angular velocity
                self.accumulated_rotation += abs(actual_w) * (1.0 / self.publish_rate)
            else:
                # Direction changed: reset accumulator
                self.accumulated_rotation = 0.0
            self.last_angular_sign = current_sign

            # Clamp if accumulated too much rotation
            if self.accumulated_rotation > self.max_continuous_rotation_rad:
                if not self.anti_spin_active:
                    self.get_logger().warn(
                        f'Anti-spin triggered: {math.degrees(self.accumulated_rotation):.0f}° accumulated '
                        f'(actual ω={actual_w:.2f} rad/s), setting w=0'
                    )
                    self.anti_spin_active = True
                # Full stop — clamping to 0.3 still causes overshoot given the massive actual speeds
                w = 0.0
        else:
            # No angular command: reset accumulator
            self.accumulated_rotation = 0.0
            self.last_angular_sign = 0
            if self.anti_spin_active:
                self.get_logger().info('Anti-spin reset')
                self.anti_spin_active = False

        # --- Layer 3: Approach speed floor ---
        # If Nav2 commands a very slow linear speed, enforce a minimum so the
        # robot doesn't stall near the goal. Only when actually trying to move forward.
        if v != 0.0 and abs(v) < self.approach_min_linear_speed:
            v = math.copysign(self.approach_min_linear_speed, v)

        # Detect pure rotation: significant angular, negligible linear
        is_pure_rotation = (abs(w) > self.angular_deadband and abs(v) < self.linear_deadband)

        # Inverse kinematics: compute wheel velocities
        v_left = v - (w * self.wheel_sep / 2.0)
        v_right = v + (w * self.wheel_sep / 2.0)

        # Convert to raw PWM (-255 to 255)
        left_pwm = int(v_left * self.pwm_per_mps)
        right_pwm = int(v_right * self.pwm_per_mps)

        # --- Layer 4: Rotation boost for pure in-place turns ---
        if is_pure_rotation:
            left_pwm = int(left_pwm * self.rotation_boost_factor)
            right_pwm = int(right_pwm * self.rotation_boost_factor)

        # Clamp to valid range
        left_pwm = max(-255, min(255, left_pwm))
        right_pwm = max(-255, min(255, right_pwm))

        # --- Layer 5: PWM rescaling or min-PWM clamping ---
        # Pure rotation SKIPS rescaling. The rescaling maps [1,255] → [min_pwm,255],
        # which over-amplifies small rotation commands (e.g. raw PWM 11 → 49).
        # Rotation needs much less torque than linear motion, and the Arduino PID
        # naturally duty-cycles below its MIN_PWM for accurate low-speed rotation.
        if is_pure_rotation:
            left_pwm = self._apply_min_pwm(left_pwm, self.rotation_min_pwm)
            right_pwm = self._apply_min_pwm(right_pwm, self.rotation_min_pwm)
        elif self.use_pwm_rescaling:
            left_pwm = self._rescale_pwm(left_pwm, self.min_pwm)
            right_pwm = self._rescale_pwm(right_pwm, self.min_pwm)
        else:
            left_pwm = self._apply_min_pwm(left_pwm, self.min_pwm)
            right_pwm = self._apply_min_pwm(right_pwm, self.min_pwm)

        # --- Layer 6: Smooth ramp ---
        left_pwm = self._ramp_pwm(left_pwm, self.last_left_pwm)
        right_pwm = self._ramp_pwm(right_pwm, self.last_right_pwm)

        # Store and send
        self.last_left_pwm = left_pwm
        self.last_right_pwm = right_pwm
        self.send_motor_command(left_pwm, right_pwm)

        # Periodic PWM diagnostic logging (every ~2s at 20Hz)
        self.pwm_log_counter += 1
        if self.pwm_log_counter >= 40:
            self.pwm_log_counter = 0
            if left_pwm != 0 or right_pwm != 0:
                self.get_logger().info(
                    f'[PWM] cmd_vel: v={v:.3f} w={w:.3f} → '
                    f'PWM L={left_pwm} R={right_pwm}'
                )

    def _apply_min_pwm(self, pwm, effective_min=None):
        """Clamp non-zero PWM to at least ±effective_min.

        This ensures the Arduino PID always gets a target above the motor's
        dead-zone. effective_min defaults to self.min_pwm if not specified.
        """
        if pwm == 0:
            return 0
        if effective_min is None:
            effective_min = self.min_pwm
        sign = 1 if pwm > 0 else -1
        return sign * max(abs(pwm), effective_min)

    def _rescale_pwm(self, pwm, effective_min=None):
        """Map [1..255] → [effective_min..255] so any command moves the motor.

        This eliminates the dead zone where PWM < min produces no motion.
        The mapping is linear and preserves proportionality.
        effective_min defaults to self.min_pwm if not specified.
        """
        if pwm == 0:
            return 0
        if effective_min is None:
            effective_min = self.min_pwm
        sign = 1 if pwm > 0 else -1
        abs_pwm = abs(pwm)
        scaled = effective_min + (abs_pwm - 1) * (255 - effective_min) / 254.0
        return sign * int(round(scaled))

    def _ramp_pwm(self, target, current):
        """Limit PWM change per cycle for smooth acceleration.

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

    def read_serial_data(self):
        """Read and parse encoder + IMU data from Arduino.
        Returns (left_ticks, right_ticks) or None for encoder data.
        Also publishes IMU data when 'i' lines are received.
        """
        if self.ser is None or not self.ser.is_open:
            return None

        try:
            with self.serial_lock:
                # Read all available lines, use the latest encoder message
                latest_enc = None
                while self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('ascii', errors='ignore').strip()
                    if line.startswith('e '):
                        latest_enc = line
                    elif line.startswith('i '):
                        self._parse_imu_line(line)

                if latest_enc is None:
                    return None

                parts = latest_enc.split()
                if len(parts) == 3:
                    left_ticks = int(parts[1])
                    right_ticks = int(parts[2])
                    return (left_ticks, right_ticks)
        except (serial.SerialException, ValueError, UnicodeDecodeError) as e:
            self.get_logger().warn(f'Serial read error: {e}')

        return None

    def _parse_imu_line(self, line):
        """Parse IMU serial line and publish sensor_msgs/Imu message.
        Format: i <qw> <qx> <qy> <qz> <ax> <ay> <az> <gx> <gy> <gz>
        """
        try:
            parts = line.split()
            if len(parts) != 11:
                return
            qw, qx, qy, qz = float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4])
            ax, ay, az = float(parts[5]), float(parts[6]), float(parts[7])
            gx, gy, gz = float(parts[8]), float(parts[9]), float(parts[10])

            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.imu_frame

            # Orientation from BNO085 Game Rotation Vector
            imu_msg.orientation.w = qw
            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz
            # Orientation covariance — set high since EKF does NOT fuse orientation
            # (kept for completeness; orientation fields are still populated for diagnostics)
            imu_msg.orientation_covariance[0] = 0.1
            imu_msg.orientation_covariance[4] = 0.1
            imu_msg.orientation_covariance[8] = 0.1

            # Angular velocity from calibrated gyroscope (rad/s)
            # Only vyaw (z) is fused by EKF — used as secondary to encoders
            imu_msg.angular_velocity.x = gx
            imu_msg.angular_velocity.y = gy
            imu_msg.angular_velocity.z = gz
            imu_msg.angular_velocity_covariance[0] = 0.01
            imu_msg.angular_velocity_covariance[4] = 0.01
            imu_msg.angular_velocity_covariance[8] = 0.01

            # Linear acceleration (m/s^2) — NOT fused by EKF (too noisy)
            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az
            imu_msg.linear_acceleration_covariance[0] = 1.0
            imu_msg.linear_acceleration_covariance[4] = 1.0
            imu_msg.linear_acceleration_covariance[8] = 1.0

            self.imu_pub.publish(imu_msg)
        except (ValueError, IndexError):
            pass

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

        # Read encoders and IMU
        enc_data = self.read_serial_data()
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

        # Update actual angular velocity for anti-spin tracking
        self.last_odom_angular = self.v_angular

        # Publish odometry
        self.publish_odometry(now)

        # Publish joint states
        self.publish_joint_states(now)

        # Publish TF
        if self.publish_tf:
            self.publish_odom_tf(now)

    def publish_odometry(self, stamp):
        """Publish /wheel/odom message (encoder-only odometry for EKF fusion)."""
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
