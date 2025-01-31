#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import py_trees
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int16
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState

from collections import deque
import numpy as np


class TeleopNode(Node):
    def __init__(self):
        super().__init__(node_name="teleop_node")
        # Loggers
        self.blackboard = py_trees.blackboard.Blackboard()
        self.info = lambda x: self.get_logger().info(f"{x}")
        self.warn = lambda x: self.get_logger().warn(f"{x}")
        self.err = lambda x: self.get_logger().error(f"{x}")

        # ROS Parameters
        self.ur_robot_prefix = self.declare_parameter("tf_prefix", value=Parameter.Type.STRING).get_parameter_value().string_value
        # self.max_speed = self.declare_parameter("max_speed", value=Parameter.Type.DOUBLE).get_parameter_value().double_value
        self.eef_max_linear_speed = 1
        self.eef_max_rot_speed = 2

        # Callback groups
        self.high_priority_cb_group = ReentrantCallbackGroup()

        # Subscriptions
        self._sub_joy_state = self.create_subscription(
            msg_type=Joy,
            topic="/joy",
            callback=self._sub_cb_joy_state,
            qos_profile=1,
            callback_group=self.high_priority_cb_group
        )

        # Publishers
        self._pub_servo = self.create_publisher(
            msg_type=TwistStamped,
            topic="/servo_node/delta_twist_cmds",
            qos_profile=QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST),
            callback_group=self.high_priority_cb_group
        )

        # Timers
        self._timer_pub_servo = self.create_timer(
            timer_period_sec=0.004,
            callback=self._timer_cb_pub_servo,
            callback_group=self.high_priority_cb_group
        )

        # Messages
        self.servo_msg = TwistStamped()
        # self.servo_msg.header.frame_id = f'{self.linear_slider_prefix}base_link'
        self.servo_msg.header.frame_id = f'{self.ur_robot_prefix}base_link'

        # Joystick mappings (XBox 1 controller)
        self.buttons = {
            "LB": 4,
            "RB": 5,
        }
        self.axes = {
            "LT": 2,
            "RT": 5,
            "left_joy_x": 0,
            "left_joy_y": 1,
            "right_joy_x": 3,
            "right_joy_y": 4,
        }
        self.remapped_velocities = {"LT": 0.0, "RT": 0.0, "left_joy_x": 0.0, "left_joy_y": 0.0, "right_joy_x": 0.0, "right_joy_y": 0.0, "T_sum": {}}
        self.scaled_velocities = np.zeros(3, dtype=float) # Array for keeping track of joystick -> world scaled values

        # self.cmd_vel_x = deque(np.zeros(15))
        return

    def _remap_trigger_values(self, msg: Joy) -> None:

        return

    def _remap_joystick_values(self, msg: Joy) -> None:
        """Remap joystick/trigger/button values to positive and negative velocities. Unpressed should have a velocity of 0.
        Joy input range for XBox controller: [1,-1]. Saves the remapped values to the class variable remapped_velocities.

        @param msg: Joy message containing the joystick state
        @return: None
        """
        # Map axes values to velocities
        for key, value in self.axes.items():
            # Map trigger values
            if key in {"LT", "RT"}:
                if key == "LT":
                    self.remapped_velocities[key] = self._scale(msg.axes[value], 1, -1, 0, -1 * self.eef_max_linear_speed) # +z
                else:
                    self.remapped_velocities[key] = self._scale(msg.axes[value], 1, -1, 0, self.eef_max_linear_speed) # -z
            # Map joystick values
            elif key in {"left_joy_x", "left_joy_y"}:
                self.remapped_velocities[key] = self._scale(msg.axes[value], 1, -1, -self.eef_max_linear_speed, self.eef_max_linear_speed) # +/- x,y
            elif key in {"right_joy_x", "right_joy_y"}:
                self.remapped_velocities[key] = self._scale(msg.axes[value], 1, -1, -self.eef_max_rot_speed, self.eef_max_rot_speed)


        # Map button values to max velocities
        for key, value in self.buttons.items():
            if key == "LB":
                self.remapped_velocities[key] = self._scale(msg.buttons[value], 0, 1, 0, -self.eef_max_rot_speed) # Rotate z-axis ccw
            else:
                self.remapped_velocities[key] = self._scale(msg.buttons[value], 0, 1, 0, self.eef_max_rot_speed)# Rotate z-axis cw


        return

    def _sum_trigger_values(self) -> None:
        """Sum the trigger values to get the raw twist commands"""
        self.remapped_velocities["z_sum"] = {
            "linear": self.remapped_velocities["LT"] + self.remapped_velocities["RT"],
            "angular": self.remapped_velocities["LB"] + self.remapped_velocities["RB"]
        }
        return


    def _scale_to_max_speed(self) -> None:
        self.scaled_velocities[0] = self.remapped_velocities["left_joy_x"]
        self.scaled_velocities[1] = self.remapped_velocities["left_joy_y"]
        self.scaled_velocities[2] = self.remapped_velocities["z_sum"]["linear"]
        speed = np.linalg.norm(self.scaled_velocities)
        if speed == 0:
            return
        if speed > self.eef_max_linear_speed:
            self.scaled_velocities = self.scaled_velocities / speed * self.eef_max_linear_speed
        return

    def _assign_twist_values(self) -> None:
        self.servo_msg.twist.linear.x = self.scaled_velocities[0]
        self.servo_msg.twist.linear.y = self.scaled_velocities[1]
        self.servo_msg.twist.linear.z = self.scaled_velocities[2]
        self.servo_msg.twist.angular.x = self.remapped_velocities["right_joy_x"]
        self.servo_msg.twist.angular.y = self.remapped_velocities["right_joy_y"]
        self.servo_msg.twist.angular.z = self.remapped_velocities["z_sum"]["angular"]
        return

    def _sub_cb_joy_state(self, msg: Joy):
        """Callback for the joystick state message. Remaps joystick values to linear and angular velocities for servo control.
        Does not publish the message, only updates the class variables.

        @param msg: Joy message containing the joystick state
        @return: None
        """
        self._remap_joystick_values(msg)
        self._sum_trigger_values()
        self._scale_to_max_speed()
        self._assign_twist_values()

        # self.info(self.remapped_z)
        # # sum both trigger values
        # vel_cmd = sum(self.remapped_z)

        # self.cmd_vel_x.popleft()
        # self.cmd_vel_x.append(vel_cmd)

        # #
        self.servo_msg.header.stamp = self.get_clock().now().to_msg()
        # self.servo_msg.twist.linear.x = np.mean(self.cmd_vel_x)

        # # self.warn(np.mean(self.cmd_vel_values))

        return

    def _timer_cb_pub_servo(self):
        if self.blackboard.get("teleop_control"):
            self._pub_servo.publish(self.servo_msg)
        return

    # def process_axis(self, axes_states):
    #     """TODO: replace the for loop above (similar to alex's?)"""

    #     return

    def _scale(self, val, amin, amax, bmin, bmax):
        """Scale a value from one range 'a' to range 'b' """
        if amin < amax:
            if val > amax or val < amin:
                raise ValueError(f"Input value {val} is out of bounds of input range {amin} to {amax}")
        else:
            if val < amax or val > amin:
                raise ValueError(f"Input value {val} is out of bounds of input range {amin} to {amax}")

        arange = amax - amin
        brange = bmax - bmin
        new_val = (((val - amin) * brange) / arange) + bmin
        return new_val


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    return


if __name__ == "__main__":
    main()