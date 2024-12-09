import py_trees
import torch as th
from pruning_sb3.algo.PPOLSTMAE import RecurrentPPOAE
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3
import numpy as np
import rclpy


class ComputeRLAction(py_trees.behaviours.Behaviour):
    """Compute an action using RL logic."""
    def __init__(self, name, model_path):
        super().__init__(name)
        self.device = "cuda" if th.cuda.is_available() else "cpu"
        self.model = self.init_rl_model(model_path)
        self.blackboard = py_trees.blackboard.Blackboard()
        self.lstm_states = None
        self.episode_start = True

        self.key_order = ["achieved_goal", "desired_goal", "relative_distance", "achieved_or", "rgb",
                          "prev_rgb", "point_mask", "joint_angles", "prev_action_achieved"]
        self.pretend_action_scale = 1.0
        self.execution_timeout = 0.1
        self.cb = rclpy.callback_groups.ReentrantCallbackGroup()


    def setup(self, node):
        self.node = node
        self.vel_publisher = self.node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 1,
                                                    callback_group=self.cb)
        self.timer = None

    def init_rl_model(self, load_path):
        assert load_path is not None, "Model path not provided."
        custom_objects = {"n_envs": 1}
        self.model = RecurrentPPOAE.load(load_path, custom_objects=custom_objects)
        self.model.to(self.device)
        self.model.eval()
        self.model.policy.set_training_mode(False)
        self.get_logger().info(f"Loaded RL model from {load_path}")

    def make_observation(self):
        """Prepare the observation for the RL model."""
        observation = self.blackboard.get("observation")
        observation["prev_action_achieved"] = self.pretend_action(observation["prev_action_achieved"])
        ordered_observation = {key: observation[key] for key in self.key_order}
        return ordered_observation

    def reset(self):
        self.get_logger().info("Resetting RL model.")
        #get ee pose wrt base_link
        tf_ee_base = self.lookup_transform("fake_base", "endpoint")
        # self.lstm_states = None
        self.episode_start = True
        #TODO: End effector index during training should be the same here
        self.blackboard.set("end_position_init", tf_ee_base[:3, 3])


    def pretend_action(self, action):
        """To overcome latency issues, we pretend the action has been executed
        slower but compensate for that by executing it longer."""
        return action * self.pretend_action_scale

    def actual_action_from_pretend(self, pretend_action):
        """Reverse the pretend action scaling."""
        return pretend_action / self.pretend_action_scale

    def publish_velocity(self):
        action = self.pretend_action(self.blackboard.get("action"))
        twist = TwistStamped()
        twist.header.frame_id = "base_link"
        twist.header.stamp = self.node.get_clock().now().to_msg()
        twist.twist.linear = Vector3(x=float(action[0]), y=float(action[1]), z=float(action[2]))
        twist.twist.angular = Vector3(x=float(action[3]), y=float(action[4]), z=float(action[5]))
        self.vel_publisher.publish(twist)
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
        self.get_logger().info("Velocity command published. Starting execution timer.")
        self.timer = self.node.create_timer(self.execution_timeout, self.set_velocity_zero, callback_group=self.cb)

    def set_velocity_zero(self):
        twist = TwistStamped()
        twist.header.frame_id = "base_link"
        twist.header.stamp = self.node.get_clock().now().to_msg()
        twist.twist.linear = Vector3(x=0.0, y=0.0, z=0.0)
        twist.twist.angular = Vector3(x=0.0, y=0.0, z=0.0)
        self.vel_publisher.publish(twist)
        self.get_logger().info("Action executed. Set velocity to zero.")

    def update(self):
        execute_action = self.blackboard.get("execute_action") # Set via interrupts
        reset_rl_model = self.blackboard.get("reset_rl_model") # Set via interrupts
        if not execute_action:
            return py_trees.common.Status.SUCCESS
        if reset_rl_model:
            self.reset()
            self.get_logger().info("Resetting RL model.")
            self.blackboard.set("reset_rl_model", False)
            return py_trees.common.Status.SUCCESS

        if "observation" not in self.blackboard:
            return py_trees.common.Status.FAILURE

        observation = self.make_observation()
        # Placeholder: Replace with actual RL model inference logic
        action, self.lstm_states = self.model.predict(
            observation,  # type: ignore[arg-type]
            state=self.lstm_states,
            episode_start=self.episode_start, # Resets the LSTM state at the beginning of each episode.
            deterministic=True,
        )
        self.blackboard.action = action
        self.episode_start = False

        self.publish_velocity()
        self.logger.info(f"Computed action: {action}")
        return py_trees.common.Status.SUCCESS
