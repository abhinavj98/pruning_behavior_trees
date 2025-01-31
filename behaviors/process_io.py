import py_trees
import asyncio
from std_msgs.msg import Int16
from collections import namedtuple

from moveit_msgs.msg import RobotState
import numpy as np
import rclpy

from pruning_bt.utils.interrupts import MoveHomeInterrupt, ToggleServoInterrupt, ChangeBlackboardValueInterrupt, \
                                        UpdateCutpointInterrupt, UpdateCsvWithEndpoint
# Interrupt = namedtuple("Interrupt", ["joy_action", "callback", "async_run"])
#Import abstract class for Interrupts
from abc import ABC, abstractmethod
#Interrupts dont block the code or the main thread
#TODO: Implement a sepertate class for Interrupts
#Also give an option to add_done_callback to the future object


class ProcessIO(py_trees.behaviour.Behaviour):
    def __init__(self, name="Process IO", asyncio_loop=None):
        super().__init__(name=name)
        self.blackboard = py_trees.blackboard.Blackboard()

        self.interrupt_dict = {
            "move_home": MoveHomeInterrupt(1),
            "toggle_servo": ToggleServoInterrupt(2),
            "execute_action": ChangeBlackboardValueInterrupt(3, "execute_action", True, ChangeBlackboardValueInterrupt.ValueChange.TOGGLE),
            "reset_rl_model": ChangeBlackboardValueInterrupt(4, "reset_rl_model", True, ChangeBlackboardValueInterrupt.ValueChange.SET),
            "run_robot_w_joystick": ChangeBlackboardValueInterrupt(11, "teleop_control", True, ChangeBlackboardValueInterrupt.ValueChange.TOGGLE),
            "increment_cutpoint": UpdateCutpointInterrupt(18, "goal", None, UpdateCutpointInterrupt.ValueChange.SET),
            "decrement_cutpoint": UpdateCutpointInterrupt(-18, "goal", None, UpdateCutpointInterrupt.ValueChange.SET),
            "add_goal_to_csv": UpdateCsvWithEndpoint(12)
        }

        self.asyncio_loop = asyncio_loop  # Store the event asyncio_loop

        """
        xbox_controller = {
            "buttons": {
                0: "A",
                1: "B",
                2: "X",
                3: "Y",
                4: "LB",
                5: "RB",
                6: "view_button",
                7: "menu_button",
                8: "xbox_button",
                9: "left_joystick",
                10: "right_joystick",
                11: "share_button"
            },
            "axes": {
                0: "left_joy_x",
                1: "left_joy_y",
                2: "LT",
                3: "right_joy_x",
                4: "right_joy_y",
                5: "RT",
                6: "Dpad_x", #18,19
                7: "Dpad_y" #20,21
            }
        }
        """

    def setup(self, node):
        self.blackboard.set("joy_action", -999)
        self.node = node
        self.cb = rclpy.callback_groups.ReentrantCallbackGroup()
        self.joy_action_sub = self.node.create_subscription(
            Int16,
            'joy_action',
            self.joy_action_callback,
            10
        )

        for interrupt in self.interrupt_dict.values():
            interrupt.setup(node, self.cb, self.blackboard)

    def joy_action_callback(self, msg):
        self.blackboard.set("joy_action", msg.data)

    def process_interrupt(self, interrupt):
        """
        Handle interrupts asynchronously by scheduling them on the event asyncio_loop.
        """
        if interrupt.async_run:
            try:
                # Schedule the coroutine and get the Future
                future = asyncio.run_coroutine_threadsafe(interrupt.callback(), self.asyncio_loop)

                # Attach a callback to handle completion
                def on_complete(fut):
                    try:
                        result = fut.result()  # Fetch the result or handle exceptions
                        self.node.get_logger().info(f"Interrupt {interrupt.joy_action} completed with result: {result}")
                    except Exception as e:
                        self.node.get_logger().error(f"Interrupt {interrupt.joy_action} failed: {e}")
                    return result

                future.add_done_callback(on_complete)
            except Exception as e:
                self.node.get_logger().error(f"Error processing interrupt: {e}")
        else:
            return interrupt.callback()

    def update(self):
        """
        Synchronously tick the behavior while scheduling async callbacks.
        """
        for interrupt in self.interrupt_dict.values():
            if self.blackboard.get("joy_action") == interrupt.joy_action:
                self.node.get_logger().info(f"Processing interrupt {interrupt.joy_action}.")
                result = self.process_interrupt(interrupt)

        self.blackboard.set("joy_action", -999)

        return py_trees.common.Status.SUCCESS