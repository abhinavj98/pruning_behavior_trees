import py_trees
import asyncio
from std_msgs.msg import Int16
from collections import namedtuple
from move_group_server_interfaces.srv import MoveToState
from moveit_msgs.msg import RobotState
from controller_manager_msgs.srv import SwitchController, ListControllers
import numpy as np
from std_srvs.srv import Trigger
import rclpy
import re
from enum import Enum
Interrupt = namedtuple("Interrupt", ["joy_action", "callback", "async_run"])

#Interrupts dont block the code or the main thread
#TODO: Implement a sepertate class for Interrupts
#Also give an option to add_done_callback to the future object

class ResourceMode(Enum):
    DEFAULT = 0
    SERVO = 1


class ProcessIO(py_trees.behaviour.Behaviour):
    def __init__(self, name="Process IO", asyncio_loop=None):
        super().__init__(name=name)
        self.blackboard = py_trees.blackboard.Blackboard()

        self.interrupt_dict = {
            "move_home": Interrupt(1, self.move_home_callback, True),
            "toggle_servo": Interrupt(2, self.toggle_servo_callback, True),
            "execute_action": Interrupt(3, self.set_execution_var, False),
            "reset_rl_model": Interrupt(4, self.reset_rl_model, False)
        }

        self.asyncio_loop = asyncio_loop  # Store the event asyncio_loop
        self.base_ctrl_string = None
        self.servo_ctrl_string = None
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
                6: "Dpad_x",
                7: "Dpad_y"
            }
        }
        """

    def setup(self, node):
        self.node = node
        self.joy_action_sub = self.node.create_subscription(
            Int16,
            'joy_action',
            self.joy_action_callback,
            10
        )
        self.cb = rclpy.callback_groups.ReentrantCallbackGroup()
        
        # Interrupt 1: Move to home position
        self.move_to_state_client = self.node.create_client(MoveToState, "move_group_server/move_to_state", callback_group=self.cb)
        self.blackboard.set("joy_action", -999)
        
        # Interrupt 2: Toggle servo
        self.base_ctrl = self.node.declare_parameter("base_controller", ".*joint_trajectory_controller")
        self.servo_ctrl = self.node.declare_parameter("servo_controller", "forward_position_controller")
        self.enable_servo = self.node.create_client(Trigger, "/servo_node/start_servo", callback_group=self.cb)
        self.disable_servo = self.node.create_client(Trigger, "/servo_node/stop_servo", callback_group=self.cb)
        self.switch_ctrl = self.node.create_client(
            SwitchController, "/controller_manager/switch_controller", callback_group=self.cb
        )
        self.list_ctrl = self.node.create_client(
            ListControllers, "/controller_manager/list_controllers", callback_group=self.cb
        )

        self.get_ctrl_string_timer = self.node.create_timer(0.1, self.get_controller_names, callback_group=self.cb)
        self.resource_mode = ResourceMode.DEFAULT


    def joy_action_callback(self, msg):
        self.blackboard.set("joy_action", msg.data)

    async def move_to_joints(self, joint_values):
        request = MoveToState.Request()
        request.goal_state.joint_state.header.stamp = self.node.get_clock().now().to_msg()
        request.goal_state.joint_state.header.frame_id = 'base_link'
        request.goal_state.joint_state.position = joint_values
        request.goal_state.joint_state.name = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

        result = await self.move_to_state_client.call_async(request)
        return result
    
    async def move_home_callback(self):
        home_joint_angles = (-np.pi / 2, -np.pi * 2 / 3, np.pi * 2 / 3, -np.pi, -np.pi / 2,
                                  np.pi) 
        self.node.get_logger().info("Moving to home position.")
        await self.move_to_joints(home_joint_angles)
        return True

    async def toggle_servo_callback(self):
        # Implement toggle servo functionality here
        self.node.get_logger().info("Toggling servo.")
        await self.handle_resource_switch()

    def set_execution_var(self):
        """Toggle the execute_action variable."""
        execute_action = self.blackboard.get("execute_action")
        self.blackboard.set("execute_action", not execute_action)
        self.node.get_logger().info("Setting execute_action variable to {}.".format(self.blackboard.get("execute_action")))
        return True

    def reset_rl_model(self):
        """Reset the RL model."""
        self.blackboard.set("reset_rl_model", True)
        self.node.get_logger().info("Setting reset_rl_model variable to True.")

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

                future.add_done_callback(on_complete)
            except Exception as e:
                self.node.get_logger().error(f"Error processing interrupt: {e}")
        else:
            interrupt.callback()

    async def get_controller_names(self):
        if self.base_ctrl_string is not None:
            self.get_ctrl_string_timer.destroy()

        if not self.list_ctrl.service_is_ready():
            return

        rez = await self.list_ctrl.call_async(ListControllers.Request())

        for ctrl in rez.controller:
            if self.base_ctrl_string is None and re.match(self.base_ctrl.value, ctrl.name):
                self.base_ctrl_string = ctrl.name

            if self.servo_ctrl_string is None and re.match(self.servo_ctrl.value, ctrl.name):
                self.servo_ctrl_string = ctrl.name

        if bool(self.base_ctrl_string) ^ bool(self.servo_ctrl_string):
            print("Only was able to match one of the controllers! Not activating")
            self.base_ctrl_string = None
            self.servo_ctrl_string = None

        elif self.base_ctrl_string is not None:
            print("Located controllers! Base: {}, Servo: {}".format(self.base_ctrl_string, self.servo_ctrl_string))

        return
    
    async def handle_resource_switch(self):
        if self.base_ctrl_string is None or self.servo_ctrl_string is None:
            raise Exception("Controllers have not been identified yet!")

        if self.resource_mode != ResourceMode.DEFAULT:
            switch_ctrl_req = SwitchController.Request(
                activate_controllers=[self.base_ctrl_string], deactivate_controllers=[self.servo_ctrl_string]
            )

            
            self.disable_servo.call_async(Trigger.Request())
            self.switch_ctrl.call_async(switch_ctrl_req)
            self.resource_mode = ResourceMode.DEFAULT

        elif self.resource_mode != ResourceMode.SERVO:
            switch_ctrl_req = SwitchController.Request(
                activate_controllers=[self.servo_ctrl_string], deactivate_controllers=[self.base_ctrl_string]
            )
            
            self.enable_servo.call_async(Trigger.Request())
            self.switch_ctrl.call_async(switch_ctrl_req)
            self.resource_mode = ResourceMode.SERVO

        else:
            raise ValueError("Unknown resource mode {} specified!".format(self.resource_mode))
        # self.resource_ready = True
        self.node.get_logger().info("Resource switch completed to mode: {}".format(self.resource_mode))

        return

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