#!/usr/bin/env python3
"""
joystick_teleop_node.py — PS3-Style Gamepad Teleop for Tomas_bot

Converts joystick inputs from a generic PS3-style 2.4GHz wireless gamepad
into /cmd_vel (Twist) messages for proportional differential-drive control.

Controller: Generic PS3-style 2.4GHz wireless gamepad with USB dongle
  - Dual analog sticks (proportional), D-pad, shoulder buttons, face buttons
  - Connects via 2.4GHz USB wireless receiver → shows as /dev/input/js0

Axis Mapping (generic PS3 controller on Linux):
  Axis 0: Left Stick X   (-1.0 = left,  +1.0 = right)
  Axis 1: Left Stick Y   (-1.0 = up,    +1.0 = down)  ← INVERTED
  Axis 2: Right Stick X  (-1.0 = left,  +1.0 = right)
  Axis 3: Right Stick Y  (-1.0 = up,    +1.0 = down)  ← INVERTED

Button Mapping (generic PS3 controller on Linux):
  Button 0:  X / Cross          Button 4: L1
  Button 1:  Circle / A         Button 5: R1
  Button 2:  Square / B         Button 6: L2
  Button 3:  Triangle / Y       Button 7: R2
  Button 8:  Select / Back      Button 9: Start
  Button 10: L3 (Left Stick)    Button 11: R3 (Right Stick)

NOTE: Generic controllers vary. Use 'ros2 run joy joy_enumerate_devices' and
      'jstest /dev/input/js0' to verify YOUR specific mapping before driving.
      All axes/buttons are configurable via parameters.

Control Scheme:
  Left Stick Y  → Linear velocity  (forward/backward, proportional)
  Right Stick X → Angular velocity (turn left/right, proportional)
  L1 (hold)     → Enable driving   (deadman switch — REQUIRED to move)
  R1 (hold)     → Turbo mode       (full speed while held)
  Start         → Emergency stop   (zero velocity, clears turbo)

Speed Modes:
  Normal (L1 only):  50% of hardware max → ~0.24 m/s linear, ~1.25 rad/s angular
  Turbo  (L1 + R1):  100% of hardware max → ~0.47 m/s linear, ~2.50 rad/s angular

Safety:
  - Deadman switch: Robot stops instantly when L1 is released
  - Watchdog: If no Joy messages for >0.5s, commands zero velocity
  - Stick deadzone: Small deflections are ignored (configurable)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class JoystickTeleopNode(Node):
    def __init__(self):
        super().__init__('joystick_teleop_node')

        # ========================== PARAMETERS ==========================
        # --- Speed Limits (match robot hardware) ---
        self.declare_parameter('max_linear_speed', 0.47)       # m/s (130 RPM motor max)
        self.declare_parameter('max_angular_speed', 2.5)       # rad/s
        self.declare_parameter('normal_linear_scale', 0.5)     # 50% of max in normal mode
        self.declare_parameter('normal_angular_scale', 0.5)    # 50% of max in normal mode
        self.declare_parameter('turbo_linear_scale', 1.0)      # 100% of max in turbo mode
        self.declare_parameter('turbo_angular_scale', 1.0)     # 100% of max in turbo mode

        # --- Axis Mapping (generic PS3 2.4GHz controller defaults) ---
        self.declare_parameter('linear_axis', 1)               # Left Stick Y
        self.declare_parameter('angular_axis', 2)              # Right Stick X
        self.declare_parameter('linear_axis_inverted', True)   # Y-axis: up = -1.0 on most controllers
        self.declare_parameter('angular_axis_inverted', False) # X-axis: left = -1.0 (maps to positive angular = turn left)

        # --- Button Mapping (generic PS3 2.4GHz controller defaults) ---
        self.declare_parameter('enable_button', 4)             # L1 — deadman switch
        self.declare_parameter('turbo_button', 5)              # R1 — turbo mode
        self.declare_parameter('estop_button', 9)              # Start — emergency stop

        # --- Deadzone & Timing ---
        self.declare_parameter('stick_deadzone', 0.1)          # Ignore stick deflection below this
        self.declare_parameter('publish_rate', 20.0)           # Hz — cmd_vel publish rate
        self.declare_parameter('joy_timeout', 0.5)             # Seconds before watchdog stops robot

        # Read parameters
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.normal_lin_scale = self.get_parameter('normal_linear_scale').value
        self.normal_ang_scale = self.get_parameter('normal_angular_scale').value
        self.turbo_lin_scale = self.get_parameter('turbo_linear_scale').value
        self.turbo_ang_scale = self.get_parameter('turbo_angular_scale').value

        self.linear_axis = self.get_parameter('linear_axis').value
        self.angular_axis = self.get_parameter('angular_axis').value
        self.linear_inverted = self.get_parameter('linear_axis_inverted').value
        self.angular_inverted = self.get_parameter('angular_axis_inverted').value

        self.enable_btn = self.get_parameter('enable_button').value
        self.turbo_btn = self.get_parameter('turbo_button').value
        self.estop_btn = self.get_parameter('estop_button').value

        self.deadzone = self.get_parameter('stick_deadzone').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.joy_timeout = self.get_parameter('joy_timeout').value

        # ========================== STATE ==========================
        self.latest_joy = None
        self.last_joy_time = self.get_clock().now()
        self.estop_active = False
        self.enabled_prev = False  # Track enable transitions for logging

        # ========================== ROS2 INTERFACES ==========================
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer for periodic publishing
        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self.publish_cmd_vel)

        # Compute actual speeds for logging
        normal_lin = self.max_linear * self.normal_lin_scale
        normal_ang = self.max_angular * self.normal_ang_scale
        turbo_lin = self.max_linear * self.turbo_lin_scale
        turbo_ang = self.max_angular * self.turbo_ang_scale

        self.get_logger().info(
            f'Joystick Teleop started — '
            f'Normal: {normal_lin:.2f} m/s, {normal_ang:.2f} rad/s | '
            f'Turbo: {turbo_lin:.2f} m/s, {turbo_ang:.2f} rad/s'
        )
        self.get_logger().info(
            f'Controls: L1=enable, R1=turbo, Start=E-stop | '
            f'Left Stick Y=linear, Right Stick X=angular'
        )
        self.get_logger().info(
            f'Axes: linear={self.linear_axis}(inv={self.linear_inverted}), '
            f'angular={self.angular_axis}(inv={self.angular_inverted}) | '
            f'Buttons: enable={self.enable_btn}, turbo={self.turbo_btn}, estop={self.estop_btn}'
        )

    def joy_callback(self, msg: Joy):
        """Store latest joystick state and update timestamp."""
        self.latest_joy = msg
        self.last_joy_time = self.get_clock().now()

        # Check emergency stop: toggle on press
        if self._get_button(msg, self.estop_btn):
            if not self.estop_active:
                self.estop_active = True
                self.get_logger().warn('EMERGENCY STOP activated — press L1 to resume')
            # Publishing zero is handled in publish_cmd_vel

    def publish_cmd_vel(self):
        """Periodic callback: compute and publish velocity from joystick state."""
        twist = Twist()  # Default: all zeros (stop)

        now = self.get_clock().now()

        # Watchdog: if no joy messages received recently, stop
        if self.latest_joy is None:
            self.cmd_vel_pub.publish(twist)
            return

        dt_joy = (now - self.last_joy_time).nanoseconds / 1e9
        if dt_joy > self.joy_timeout:
            self.cmd_vel_pub.publish(twist)
            return

        joy = self.latest_joy

        # --- Check enable (deadman switch) ---
        enabled = self._get_button(joy, self.enable_btn)

        # Transition logging
        if enabled and not self.enabled_prev:
            if self.estop_active:
                self.estop_active = False
                self.get_logger().info('E-stop cleared — driving enabled')
            else:
                self.get_logger().info('Driving enabled (L1 held)')
        elif not enabled and self.enabled_prev:
            self.get_logger().info('Driving disabled (L1 released)')
        self.enabled_prev = enabled

        # If E-stop active or enable not held, publish zero
        if self.estop_active or not enabled:
            self.cmd_vel_pub.publish(twist)
            return

        # --- Read analog sticks ---
        raw_linear = self._get_axis(joy, self.linear_axis)
        raw_angular = self._get_axis(joy, self.angular_axis)

        # Apply inversion (most controllers: stick up = -1.0)
        if self.linear_inverted:
            raw_linear = -raw_linear
        if self.angular_inverted:
            raw_angular = -raw_angular

        # Apply deadzone
        raw_linear = self._apply_deadzone(raw_linear)
        raw_angular = self._apply_deadzone(raw_angular)

        # --- Determine speed scale (normal vs turbo) ---
        turbo = self._get_button(joy, self.turbo_btn)
        if turbo:
            lin_scale = self.turbo_lin_scale
            ang_scale = self.turbo_ang_scale
        else:
            lin_scale = self.normal_lin_scale
            ang_scale = self.normal_ang_scale

        # --- Compute velocity (proportional to stick deflection) ---
        twist.linear.x = raw_linear * self.max_linear * lin_scale
        twist.angular.z = raw_angular * self.max_angular * ang_scale

        self.cmd_vel_pub.publish(twist)

    def _get_axis(self, joy: Joy, index: int) -> float:
        """Safely read an axis value from Joy message."""
        if index < len(joy.axes):
            return joy.axes[index]
        return 0.0

    def _get_button(self, joy: Joy, index: int) -> bool:
        """Safely read a button state from Joy message."""
        if index < len(joy.buttons):
            return joy.buttons[index] == 1
        return False

    def _apply_deadzone(self, value: float) -> float:
        """Apply deadzone and rescale so output starts from 0.0 at deadzone edge."""
        if abs(value) < self.deadzone:
            return 0.0
        # Rescale: [deadzone..1.0] → [0.0..1.0]
        sign = 1.0 if value > 0 else -1.0
        rescaled = (abs(value) - self.deadzone) / (1.0 - self.deadzone)
        return sign * min(rescaled, 1.0)

    def destroy_node(self):
        """Clean shutdown: publish zero velocity."""
        self.get_logger().info('Shutting down joystick teleop — stopping robot')
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JoystickTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
