#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

class ThrusterTeleop(Node):
    def __init__(self):
        super().__init__('thruster_teleop')

        # Button indices (X/A/Y/B) — these usually work fine
        self.declare_parameters(
            namespace='',
            parameters=[
                ('front_left_button', 2),    # X
                ('front_right_button', 0),   # A
                ('rear_left_button', 3),     # Y
                ('rear_right_button', 1),    # B

                # D-pad preferred as AXES
                ('dpad_horizontal_axis', 6), # -1 left, +1 right
                ('dpad_vertical_axis', 7),   # -1 down, +1 up
                ('dpad_threshold', 0.5),

                # If your D-pad comes as buttons instead, set these to valid indices (>=0)
                ('dpad_up_button', -1),
                ('dpad_right_button', -1),
                ('dpad_left_button', -1),

                # Left Trigger (LT) preferred as AXIS (0..1 or -1..1 depending on driver)
                ('lt_axis', 2),
                ('lt_axis_threshold', 0.5),

                # If your LT comes as a button, set to valid index (>=0)
                ('negative_trigger_button', -1),

                # Output magnitude
                ('thruster_output', 200.0),
            ]
        )

        # Publishers (7 thrusters)
        self.pub_fl  = self.create_publisher(Float64, '/rov_ricketts/front_left_thrust',     10)
        self.pub_fr  = self.create_publisher(Float64, '/rov_ricketts/front_right_thrust',    10)
        self.pub_rl  = self.create_publisher(Float64, '/rov_ricketts/rear_left_thrust',      10)
        self.pub_rr  = self.create_publisher(Float64, '/rov_ricketts/rear_right_thrust',     10)
        self.pub_ufl = self.create_publisher(Float64, '/rov_ricketts/up_front_left_thrust',  10)
        self.pub_ufr = self.create_publisher(Float64, '/rov_ricketts/up_front_right_thrust', 10)
        self.pub_ur  = self.create_publisher(Float64, '/rov_ricketts/up_rear_thrust',        10)

        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

    # --- Helpers ---
    def _btn(self, idx, joy):
        return (0 <= idx < len(joy.buttons)) and (joy.buttons[idx] == 1)

    def _axis(self, idx, joy):
        return joy.axes[idx] if (0 <= idx < len(joy.axes)) else 0.0

    def _neg_modifier(self, joy):
        # Prefer LT as axis (>= threshold), else try button
        lt_axis = self.get_parameter('lt_axis').value
        th = float(self.get_parameter('lt_axis_threshold').value)
        if 0 <= lt_axis < len(joy.axes):
            val = self._axis(lt_axis, joy)
            # support either 0..1 or -1..1 ranges
            return (val >= th) or (val <= -th)
        # fallback: LT as button
        lt_btn = self.get_parameter('negative_trigger_button').value
        return self._btn(lt_btn, joy) if lt_btn >= 0 else False

    def _dpad_pressed(self, joy):
        """Return dict for D-pad virtual buttons: up/right/left (True/False)."""
        # First try AXES
        hx = self.get_parameter('dpad_horizontal_axis').value
        vy = self.get_parameter('dpad_vertical_axis').value
        thr = float(self.get_parameter('dpad_threshold').value)
        used_axes = False

        up = right = left = False
        if 0 <= hx < len(joy.axes) or 0 <= vy < len(joy.axes):
            used_axes = True
            if 0 <= vy < len(joy.axes):
                up = self._axis(vy, joy) >= thr
            if 0 <= hx < len(joy.axes):
                right = self._axis(hx, joy) >= thr
                left  = self._axis(hx, joy) <= -thr

        # If AXES not available, fall back to BUTTON indices
        if not used_axes:
            up_btn    = self.get_parameter('dpad_up_button').value
            right_btn = self.get_parameter('dpad_right_button').value
            left_btn  = self.get_parameter('dpad_left_button').value
            up    = self._btn(up_btn, joy) if up_btn >= 0 else False
            right = self._btn(right_btn, joy) if right_btn >= 0 else False
            left  = self._btn(left_btn, joy) if left_btn >= 0 else False

        return {'up': up, 'right': right, 'left': left}

    def joy_callback(self, msg: Joy):
        mag = float(self.get_parameter('thruster_output').value)
        value = -mag if self._neg_modifier(msg) else mag

        # X/A/Y/B (buttons)
        if self._btn(self.get_parameter('front_left_button').value, msg):
            self.pub_fl.publish(Float64(data=value))
        else:
            self.pub_fl.publish(Float64(data=0.0))

        if self._btn(self.get_parameter('front_right_button').value, msg):
            self.pub_fr.publish(Float64(data=value))
        else:
            self.pub_fr.publish(Float64(data=0.0))

        if self._btn(self.get_parameter('rear_left_button').value, msg):
            self.pub_rl.publish(Float64(data=value))
        else:
            self.pub_rl.publish(Float64(data=0.0))

        if self._btn(self.get_parameter('rear_right_button').value, msg):
            self.pub_rr.publish(Float64(data=value))
        else:
            self.pub_rr.publish(Float64(data=0.0))

        # D-pad (axes or buttons)
        dpad = self._dpad_pressed(msg)
        self.pub_ufl.publish(Float64(data=value if dpad['up']    else 0.0))
        self.pub_ufr.publish(Float64(data=value if dpad['right'] else 0.0))
        self.pub_ur.publish (Float64(data=value if dpad['left']  else 0.0))

def main(args=None):
    rclpy.init(args=args)
    node = ThrusterTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()