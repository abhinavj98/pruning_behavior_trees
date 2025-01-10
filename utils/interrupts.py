from abc import ABC, abstractmethod
import numpy as np
from move_group_server_interfaces.srv import MoveToState

from std_srvs.srv import Trigger
import re
from enum import Enum
from controller_manager_msgs.srv import SwitchController, ListControllers
class BaseInterrupt(ABC):
    def __init__(self, joy_action):
        self.joy_action = joy_action
        self.async_run = None
        self.node = None
        self.cb_group = None
    @abstractmethod
    def setup(self, node, callback_group, blackboard):
        pass

class AsyncInterrupt(BaseInterrupt):
    def __init__(self, joy_action):
        super().__init__(joy_action)
        self.async_run = True
    @abstractmethod
    async def callback(self):
        pass

class Interrupt(BaseInterrupt):
    def __init__(self, joy_action):
        super().__init__(joy_action)
        self.async_run = False

    @abstractmethod
    def callback(self):
        pass

class ChangeBlackboardValueInterrupt(Interrupt):
    class ValueChange(Enum):
        SET = 0
        TOGGLE = 1

    def __init__(self, joy_action, key, value, value_change):
        super().__init__(joy_action)
        self.key = key
        self.value = value
        self.value_change = value_change

    def setup(self, node, callback_group, blackboard):
        self.blackboard = blackboard
        self.node = node
        self.cb_group = callback_group

    def change_blackboard_value(self):
        if self.value_change == self.ValueChange.TOGGLE:
            self.value = not self.blackboard.get(self.key)

        self.blackboard.set(self.key, self.value)
        self.node.get_logger().info("Setting blackboard value {} to {}.".format(self.key, self.value))
        return True

    def callback(self):
        return self.change_blackboard_value()

class MoveHomeInterrupt(AsyncInterrupt):

    def __init__(self, joy_action):
        super().__init__(joy_action)


    def setup(self, node, cb_group, blackboard):
        self.node = node
        self.cb_group = cb_group
        self.blackboard = blackboard
        self.move_to_state_client = node.create_client(MoveToState, "move_group_server/move_to_state", callback_group=cb)

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

    async def callback(self):
        await self.move_home_callback()
        return True


class ToggleServoInterrupt(AsyncInterrupt):
    def __init__(self, joy_action):
        super().__init__(joy_action)
        self.base_ctrl_string = None
        self.servo_ctrl_string = None


    class ResourceMode(Enum):
        DEFAULT = 0
        SERVO = 1
    def setup(self, node, callback_group, blackboard):
        self.node = node
        self.cb_group = callback_group
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
        self.resource_mode = self.ResourceMode.DEFAULT

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

        if self.resource_mode != self.ResourceMode.DEFAULT:
            switch_ctrl_req = SwitchController.Request(
                activate_controllers=[self.base_ctrl_string], deactivate_controllers=[self.servo_ctrl_string]
            )

            self.disable_servo.call_async(Trigger.Request())
            self.switch_ctrl.call_async(switch_ctrl_req)
            self.resource_mode = self.ResourceMode.DEFAULT

        elif self.resource_mode != self.ResourceMode.SERVO:
            switch_ctrl_req = SwitchController.Request(
                activate_controllers=[self.servo_ctrl_string], deactivate_controllers=[self.base_ctrl_string]
            )

            self.enable_servo.call_async(Trigger.Request())
            self.switch_ctrl.call_async(switch_ctrl_req)
            self.resource_mode = self.ResourceMode.SERVO

        else:
            raise ValueError("Unknown resource mode {} specified!".format(self.resource_mode))
        # self.resource_ready = True
        self.node.get_logger().info("Resource switch completed to mode: {}".format(self.resource_mode))

        return

    async def callback(self):
        self.node.get_logger().info("Toggling servo.")
        await self.handle_resource_switch()
        return True

