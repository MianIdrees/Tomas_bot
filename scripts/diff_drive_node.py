#!/usr/bin/env python3
"""
diff_drive_node.py - ROS2 Serial Bridge for Arduino Leonardo Motor + IMU Controller

BRANCH: final-2 — Motion Profile State Machine

Approach: Solves sharp-turn stalling, excessive spinning, and slow goal approach by:
  1. State machine: IDLE / ROTATING / LINEAR / ARC with per-state PWM profiles
  2. Soft-start rotation: Ramps angular PWM gradually to avoid jerks and overshooting
  3. Rotation watchdog: Time-limited rotation with automatic cutoff
  4. Duty-cycling for low speed: Alternates between MIN_PWM and 0 to achieve sub-min speeds
  5. Feedforward compensation: Adds offset PWM based on commanded velocity direction

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
from enum import Enum

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


class MotionState(Enum):
    IDLE = 0
    ROTATING = 1     # Pure in-place rotation (no linear velocity)
    LINEAR = 2       # Pure forward/backward (no angular velocity)
    ARC = 3          # Combined linear + angular (curve following)


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
        self.declare_parameter('min_pwm', 57)               # Tuned: dead zone at PWM 52 + 5 margin. MUST match Arduino MIN_PWM.
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', False)         # EKF publishes odom->base_link TF
        self.declare_parameter('publish_rate', 20.0)        # Hz
        self.declare_parameter('imu_frame', 'imu_link')

        # --- Motion state thresholds ---
        self.declare_parameter('angular_deadband', 0.03)
        # [TUNING] Angular velocity below this → treated as zero (filters sensor jitter).
        # Range: 0.01 to 0.10 rad/s.
        # Lower → more responsive to small turn commands (risk: motor hum without motion).
        # Higher → filters more, but may ignore needed turn commands.

        self.declare_parameter('linear_deadband', 0.005)
        # [TUNING] Linear velocity below this → treated as zero.
        # Range: 0.001 to 0.02 m/s.
        # Keep very low so Nav2 approach commands pass through.

        # --- ROTATION state parameters ---
        self.declare_parameter('rotation_pwm', 65)
        # [TUNING] Base PWM for in-place rotation. Target PWM after soft-start completes.
        # Range: 57 to 130. Set above min_pwm to give soft-start a ramp range.
        # Soft-start ramps from min_pwm (57) to this value, so robot responds immediately.
        # At 65: ramp range is 57→65 (small, gentle). Robot physically spins at ~1.0 rad/s
        # regardless (min rotation speed is hardware-limited). Nav2 handles via closed-loop.
        # If robot doesn't rotate at all → increase (try 70, 75).
        # If rotation overshoots in Nav2 → decrease toward 60.

        self.declare_parameter('rotation_soft_start_steps', 5)
        # [TUNING] Number of control cycles (at 20Hz) to ramp from min_pwm to rotation_pwm.
        # Range: 1 to 15. Ramp is now min_pwm→rotation_pwm (57→65 = 8 PWM range).
        # At 5 steps and 20Hz → 250ms ramp. Robot moves from the FIRST cycle.
        # Higher → gentler acceleration. Lower → faster to full speed.
        # If rotation jerks at start → increase to 8.
        # If rotation feels sluggish → decrease to 3.

        self.declare_parameter('rotation_max_duration', 6.0)
        # [TUNING] Maximum seconds of continuous rotation before watchdog stops it.
        # Range: 2.0 to 15.0 seconds. Tuned down from 8.0: robot spins at ~1.0 rad/s
        # at minimum PWM, so 6s allows ~340° (covers U-turns with safety margin).
        # If robot needs slow multi-turn → increase to 8.0.
        # If robot spins out of control → decrease to 4.0.

        # --- LINEAR state parameters ---
        self.declare_parameter('linear_ramp_rate', 35)
        # [TUNING] Max PWM change per control cycle for linear motion.
        # Range: 10 to 80.
        # At 35 and 20Hz → 700 PWM/s → reaches full speed (PWM 255) in ~0.36s.
        # Lower → smoother acceleration (better for heavy robot). Higher → snappier.
        # If robot jerks when starting forward → decrease to 20.
        # If robot feels sluggish to accelerate → increase to 50.

        self.declare_parameter('linear_min_speed', 0.10)
        # [TUNING] Minimum linear speed when robot is supposed to be moving.
        # Range: 0.02 to 0.15 m/s.
        # Prevents Nav2 from commanding speeds below motor dead zone near goal.
        # If robot stalls near goal → increase (try 0.12, 0.15).
        # If robot overshoots goal → decrease (try 0.08).

        # --- ARC state parameters ---
        self.declare_parameter('arc_ramp_rate', 30)
        # [TUNING] Max PWM change per cycle during arc/curve following.
        # Range: 10 to 60.
        # Slightly slower than linear ramp because differential PWM causes heading changes.
        # If robot wobbles on curves → decrease to 20.
        # If curves feel sluggish → increase to 45.

        self.declare_parameter('arc_angular_scale', 0.90)
        # [TUNING] Scale factor for angular component during arc motion.
        # Range: 0.5 to 1.2. Tuned from 1.0: both left (+19°) and right (−41°)
        # over-steered in round 3. 0.90 reduces angular component by 10%.
        # If robot over-steers on curves → decrease (try 0.85, 0.80).
        # If robot under-steers (wide turns) → increase (try 0.95, 1.0).

        # --- Duty cycling for sub-minimum speeds ---
        self.declare_parameter('duty_cycle_enabled', True)
        # [TUNING] Enable duty-cycling for speeds below min_pwm threshold.
        # When True: alternates between min_pwm and 0 to achieve average speed below min.
        # When False: any speed below min_pwm is clamped to min_pwm (jumpy at low speeds).
        # Duty cycling gives smoother low-speed control but may cause slight pulsing.

        self.declare_parameter('duty_cycle_period', 6)
        # [TUNING] Number of control cycles per duty-cycle period.
        # Range: 2 to 10.
        # At 6 and 20Hz: period = 300ms. If target is 50% of min_pwm → 3 cycles ON, 3 OFF.
        # Higher → smoother average but more visible pulsing. Lower → jerkier but faster response.

        # --- Motor asymmetry compensation ---
        self.declare_parameter('motor_trim', 0.03)
        # [TUNING] Compensates for left/right motor speed asymmetry.
        # Range: -0.10 to 0.10.
        # Positive = reduce left PWM, boost right PWM (fixes rightward drift).
        # Negative = reduce right PWM, boost left PWM (fixes leftward drift).
        # Measured: robot drifts right ~3-5°/s during linear motion → left motor runs faster.
        # If robot drifts RIGHT during straight-line → increase (try 0.04, 0.05).
        # If robot drifts LEFT → set negative (try -0.02, -0.03).

        # Load all parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.wheel_sep = self.get_parameter('wheel_separation').value
        self.wheel_rad = self.get_parameter('wheel_radius').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        self.max_motor_speed = self.get_parameter('max_motor_speed').value
        self.min_pwm = self.get_parameter('min_pwm').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.imu_frame = self.get_parameter('imu_frame').value

        self.angular_deadband = self.get_parameter('angular_deadband').value
        self.linear_deadband = self.get_parameter('linear_deadband').value
        self.rotation_pwm = self.get_parameter('rotation_pwm').value
        self.rotation_soft_start_steps = self.get_parameter('rotation_soft_start_steps').value
        self.rotation_max_duration = self.get_parameter('rotation_max_duration').value
        self.linear_ramp_rate = self.get_parameter('linear_ramp_rate').value
        self.linear_min_speed = self.get_parameter('linear_min_speed').value
        self.arc_ramp_rate = self.get_parameter('arc_ramp_rate').value
        self.arc_angular_scale = self.get_parameter('arc_angular_scale').value
        self.duty_cycle_enabled = self.get_parameter('duty_cycle_enabled').value
        self.duty_cycle_period = self.get_parameter('duty_cycle_period').value
        self.motor_trim = self.get_parameter('motor_trim').value

        # Derived constants
        self.meters_per_tick = (2.0 * math.pi * self.wheel_rad) / self.ticks_per_rev
        self.pwm_per_mps = 255.0 / self.max_motor_speed

        # ========================== MOTION STATE ==========================
        self.motion_state = MotionState.IDLE
        self.rotation_start_time = None
        self.rotation_step_count = 0  # For soft-start ramping
        self.duty_cycle_counter = 0   # For low-speed duty cycling

        # ========================== ODOMETRY STATE ==========================
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v_linear = 0.0
        self.v_angular = 0.0

        self.last_left_ticks = None
        self.last_right_ticks = None
        self.last_odom_time = None

        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0

        # ========================== SERIAL CONNECTION ==========================
        self.ser = None
        self.serial_lock = threading.Lock()
        self.connect_serial()

        # ========================== ROS2 INTERFACES ==========================
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.odom_pub = self.create_publisher(Odometry, '/wheel/odom', qos)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', qos)

        self.tf_broadcaster = TransformBroadcaster(self)

        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self.update_callback)

        # Watchdog
        self.last_cmd_time = self.get_clock().now()
        self.cmd_timeout = 0.5

        self.last_left_pwm = 0
        self.last_right_pwm = 0
        self.pwm_log_counter = 0

        self.get_logger().info(
            f'DiffDriveNode [final-2 State Machine] started: port={self.serial_port}, '
            f'wheel_sep={self.wheel_sep}, wheel_rad={self.wheel_rad}'
        )
        self.get_logger().info(
            f'Motor params: max_speed={self.max_motor_speed} m/s, '
            f'min_pwm={self.min_pwm}, rotation_pwm={self.rotation_pwm}'
        )
        self.get_logger().info(
            f'State machine: soft_start={self.rotation_soft_start_steps} steps, '
            f'rot_watchdog={self.rotation_max_duration}s, '
            f'duty_cycle={"ON" if self.duty_cycle_enabled else "OFF"}'
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

    def _classify_motion(self, v, w):
        """Classify the commanded motion into a state."""
        has_linear = abs(v) >= self.linear_deadband
        has_angular = abs(w) >= self.angular_deadband

        if not has_linear and not has_angular:
            return MotionState.IDLE
        elif has_angular and not has_linear:
            return MotionState.ROTATING
        elif has_linear and not has_angular:
            return MotionState.LINEAR
        else:
            return MotionState.ARC

    def _transition_state(self, new_state):
        """Handle state transitions and reset state-specific variables."""
        if new_state == self.motion_state:
            return

        old_state = self.motion_state
        self.motion_state = new_state

        if new_state == MotionState.ROTATING:
            self.rotation_start_time = time.time()
            self.rotation_step_count = 0
        elif new_state == MotionState.IDLE:
            self.rotation_start_time = None
            self.rotation_step_count = 0

        if old_state != new_state:
            self.get_logger().debug(f'State: {old_state.name} → {new_state.name}')

    def _apply_duty_cycle(self, pwm):
        """Apply duty cycling for sub-minimum PWM values.

        Instead of clamping low PWM to min_pwm (which overshoots), alternate
        between min_pwm and 0 over a period to achieve a lower average PWM.
        """
        if not self.duty_cycle_enabled:
            # Simple clamping fallback
            if pwm == 0:
                return 0
            sign = 1 if pwm > 0 else -1
            return sign * max(abs(pwm), self.min_pwm)

        if pwm == 0:
            return 0

        sign = 1 if pwm > 0 else -1
        abs_pwm = abs(pwm)

        if abs_pwm >= self.min_pwm:
            # Above dead zone: no duty cycling needed
            return pwm

        # Below dead zone: duty cycle between min_pwm and 0
        # Calculate how many cycles should be ON in the period
        duty_ratio = abs_pwm / self.min_pwm  # 0.0 to 1.0
        on_cycles = max(1, int(round(duty_ratio * self.duty_cycle_period)))

        # Determine if this cycle should be ON or OFF
        cycle_pos = self.duty_cycle_counter % self.duty_cycle_period
        if cycle_pos < on_cycles:
            return sign * self.min_pwm
        else:
            return 0

    def cmd_vel_callback(self, msg: Twist):
        """Convert cmd_vel to motor PWM commands using motion state machine.

        Pipeline (Branch final-2 — Motion Profile State Machine):
        1. Deadband: Filter jitter
        2. State classification: IDLE / ROTATING / LINEAR / ARC
        3. State-specific PWM generation:
           - ROTATING: Fixed PWM with soft-start ramp + watchdog timeout
           - LINEAR: Velocity-proportional PWM with ramp + speed floor
           - ARC: Combined with angular scaling + moderate ramp
        4. Duty cycling: Sub-minimum speeds use ON/OFF alternation
        5. Send to Arduino (Arduino PID handles final speed tracking)
        """
        self.last_cmd_time = self.get_clock().now()
        self.duty_cycle_counter += 1

        v = msg.linear.x   # m/s
        w = msg.angular.z   # rad/s

        # --- Layer 1: Deadband ---
        if abs(w) < self.angular_deadband:
            w = 0.0
        if abs(v) < self.linear_deadband:
            v = 0.0

        # --- Layer 2: Classify and transition state ---
        new_state = self._classify_motion(v, w)
        self._transition_state(new_state)

        # --- Layer 3: State-specific PWM generation ---
        if self.motion_state == MotionState.IDLE:
            left_pwm = 0
            right_pwm = 0

        elif self.motion_state == MotionState.ROTATING:
            left_pwm, right_pwm = self._compute_rotation_pwm(w)

        elif self.motion_state == MotionState.LINEAR:
            left_pwm, right_pwm = self._compute_linear_pwm(v)

        elif self.motion_state == MotionState.ARC:
            left_pwm, right_pwm = self._compute_arc_pwm(v, w)

        # --- Layer 4: Clamp ---
        left_pwm = max(-255, min(255, left_pwm))
        right_pwm = max(-255, min(255, right_pwm))

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

    def _compute_rotation_pwm(self, w):
        """Compute PWM for pure in-place rotation with soft-start and watchdog.

        Soft-start ramps from min_pwm to rotation_pwm. Magnitude scaling
        provides proportional speed control: lower Nav2 commands → slower
        rotation → less coast overshoot. Sub-min PWM values are duty-cycled
        (alternating between min_pwm and 0) rather than clamped, giving
        actual proportional angular velocity control.

        Data-driven: diagnostic showed all cmd_w values produced identical
        ~1.0 rad/s rotation due to dead-zone clamping. With duty cycling:
          w=0.3 → ~0.5 rad/s, coast ~12°
          w=0.5 → ~0.7 rad/s, coast ~18°
          w=0.8 → ~1.0 rad/s, coast ~25°
        """
        # Watchdog: stop rotation after max duration
        if self.rotation_start_time is not None:
            elapsed = time.time() - self.rotation_start_time
            if elapsed > self.rotation_max_duration:
                self.get_logger().warn(
                    f'Rotation watchdog: {elapsed:.1f}s exceeded {self.rotation_max_duration}s limit'
                )
                return 0, 0

        # Soft-start: ramp from min_pwm to rotation_pwm
        self.rotation_step_count += 1
        ramp_fraction = min(1.0, self.rotation_step_count / max(1, self.rotation_soft_start_steps))
        target_pwm = int(self.min_pwm + (self.rotation_pwm - self.min_pwm) * ramp_fraction)

        # Direction: positive w = CCW = left backward, right forward
        w_sign = 1 if w > 0 else -1

        # Proportional scaling: higher commanded ω → more of the PWM range
        w_magnitude = abs(w)
        max_angular = 0.8  # rad/s — at this speed, use full rotation_pwm
        magnitude_scale = min(1.0, w_magnitude / max_angular)
        scaled_pwm = int(target_pwm * magnitude_scale)

        # Duty cycling: sub-min PWM alternates between min_pwm and 0
        # This replaces the old dead-zone clamp that forced everything to
        # min_pwm (making all rotation speeds identical at ~1.0 rad/s).
        left_raw = -w_sign * scaled_pwm
        right_raw = w_sign * scaled_pwm
        left_pwm = self._apply_duty_cycle(left_raw)
        right_pwm = self._apply_duty_cycle(right_raw)

        return left_pwm, right_pwm

    def _compute_linear_pwm(self, v):
        """Compute PWM for pure linear motion with speed floor and ramping."""
        # Enforce speed floor to prevent stalling near goal
        if abs(v) < self.linear_min_speed:
            v = math.copysign(self.linear_min_speed, v)

        # Convert to PWM with motor trim compensation
        left_raw = int(v * self.pwm_per_mps * (1.0 - self.motor_trim))
        right_raw = int(v * self.pwm_per_mps * (1.0 + self.motor_trim))

        # Apply duty cycling for sub-minimum speeds
        left_pwm = self._apply_duty_cycle(left_raw)
        right_pwm = self._apply_duty_cycle(right_raw)

        # Apply ramp
        left_pwm = self._ramp_pwm(left_pwm, self.last_left_pwm, self.linear_ramp_rate)
        right_pwm = self._ramp_pwm(right_pwm, self.last_right_pwm, self.linear_ramp_rate)

        return left_pwm, right_pwm

    def _compute_arc_pwm(self, v, w):
        """Compute PWM for combined linear + angular motion (curve following).

        Uses inverse kinematics but scales the angular component to prevent
        over-steering on curves. Motor trim is applied to the linear velocity
        only (before inverse kinematics) so it compensates straight-line drift
        without amplifying turn asymmetry.
        """
        # Scale angular component
        w_scaled = w * self.arc_angular_scale

        # Enforce speed floor
        if abs(v) < self.linear_min_speed:
            v = math.copysign(self.linear_min_speed, v)

        # Apply motor trim to linear velocity only (not angular)
        # This compensates motor asymmetry in the forward component
        # without worsening left/right turn asymmetry
        v_left_base = v * (1.0 - self.motor_trim)
        v_right_base = v * (1.0 + self.motor_trim)

        # Inverse kinematics: add angular differential to trimmed linear
        v_left = v_left_base - (w_scaled * self.wheel_sep / 2.0)
        v_right = v_right_base + (w_scaled * self.wheel_sep / 2.0)

        # Convert to PWM
        left_pwm = int(v_left * self.pwm_per_mps)
        right_pwm = int(v_right * self.pwm_per_mps)

        # Apply duty cycling for sub-minimum speeds
        left_pwm = self._apply_duty_cycle(left_pwm)
        right_pwm = self._apply_duty_cycle(right_pwm)

        # Apply arc ramp
        left_pwm = self._ramp_pwm(left_pwm, self.last_left_pwm, self.arc_ramp_rate)
        right_pwm = self._ramp_pwm(right_pwm, self.last_right_pwm, self.arc_ramp_rate)

        return left_pwm, right_pwm

    def _ramp_pwm(self, target, current, ramp_rate=None):
        """Limit PWM change per cycle for smooth acceleration.

        Stopping is IMMEDIATE — no delay when Nav2 says stop.
        """
        if ramp_rate is None:
            ramp_rate = self.linear_ramp_rate
        if target == 0:
            return 0
        # Direction reversal: stop first, then ramp in new direction
        if (target > 0 and current < 0) or (target < 0 and current > 0):
            return 0
        diff = target - current
        if abs(diff) <= ramp_rate:
            return target
        return current + ramp_rate * (1 if diff > 0 else -1)

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
