import py_trees
import torch as th
from pruning_bt.pruning_sb3.algo.PPOLSTMAE.ppo_recurrent_ae import RecurrentPPOAE
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3
import numpy as np
import rclpy
import os
import sys
from scipy.spatial.transform import Rotation
import asyncio
from tf2_ros import Buffer, TransformListener
import torch
class ComputeRLAction(py_trees.behaviour.Behaviour):
    """Compute an action using RL logic."""
    def __init__(self, name, model_path, load_timestep, load_folder, asyncio_loop):
        super().__init__(name)
        self.device = "cuda" if th.cuda.is_available() else "cpu"
        self.blackboard = py_trees.blackboard.Blackboard()
        self.lstm_states = None
        self.episode_start = True
        self.blackboard.set("execute_action", False)
        self.blackboard.set("reset_rl_model", True)
        self.waiting_for_tick = True
        self.blackboard.set("action", np.zeros(6))
        self.asyncio_loop = asyncio_loop
        self.model_path = model_path
        self.load_timestep = load_timestep
        self.load_folder = load_folder

        self.key_order = ["achieved_goal", "desired_goal", "relative_distance", "achieved_or", "rgb",
                          "prev_rgb", "point_mask", "joint_angles", "prev_action_achieved"]
        self.pretend_action_scale = 10
        self.execution_timeout = 1./2
        self.steps = 0

        self.blackboard.set("steps", self.steps)
        self.cb = rclpy.callback_groups.ReentrantCallbackGroup()
        


    def setup(self, node):
        self.node = node
        self.vel_publisher = self.node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 1,
                                                    callback_group=self.cb)
        self.vel_publisher_timer = self.node.create_timer(0.01, self.publish_velocity, callback_group=self.cb)
        self.timer = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.waiting_for_transform = False
        self.transform_result = None
        self.init_rl_model(self.model_path, self.load_folder, self.load_timestep)
        


    def init_rl_model(self, weights_path, load_folder, load_timestep):
        assert weights_path is not None, "Model path not provided."
        torch.set_float32_matmul_precision('high')
        

        load_path = os.path.join(weights_path, load_folder, f"model_{load_timestep}_steps")
        custom_objects = {"n_envs": 1}
        self.model = RecurrentPPOAE.load(load_path, custom_objects=custom_objects)
        
        self.model.policy.set_training_mode(False)

        #Dummy prediction
        observation = self.blackboard.get("observation")
        self.model.predict(observation=observation, 
                           state=self.lstm_states,
                            episode_start=self.episode_start, # Resets the LSTM state at the beginning of each episode.
                            deterministic=True)
        self.node.get_logger().info(f"Loaded RL model from {load_path}")

    def make_observation(self):
        """Prepare the observation for the RL model."""
        observation = self.blackboard.get("observation")
        # observation["prev_action_achieved"] = observation["prev_action_achieved"]
        # print("action achieved: ", observation["prev_action_achieved"])
        # print("Actual action: ", self.blackboard.get("action")*0.2)
        ordered_observation = {key: observation[key] for key in self.key_order}
        return ordered_observation
    
    async def lookup_transform(self, target_frame, source_frame):
        try:
            # self.node.get_logger().info(f"Looking up transform between {target_frame} and {source_frame}")
            future = self.tf_buffer.wait_for_transform_async(
                target_frame, source_frame, rclpy.time.Time()
            )
            await future
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time()
            )
            return transform
        except Exception as e:
            self.node.get_logger().error(f"Failed to lookup transform asynchronously: {e}")
            return None

    def wait_async_lookup_transform(self, target_frame, source_frame):
        """
        Handle interrupts asynchronously by scheduling them on the event asyncio_loop.
        """
        try:
            # Schedule the coroutine and get the Future
            self.waiting_for_transform = True
            future = asyncio.run_coroutine_threadsafe(self.lookup_transform(target_frame, source_frame), self.asyncio_loop)

            def on_complete(fut):
                try:
                    result = fut.result()  # Fetch the result or handle exceptions
                    self.waiting_for_transform = False
                    self.transform_result = result
                    # self.node.get_logger().info(f"Transform lookup completed with result: {result}")
                except Exception as e:
                    self.node.get_logger().error(f"Transform lookup failed: {e}")

            future.add_done_callback(on_complete)
            while self.waiting_for_transform:
                pass
            
            tf = self.transform_result
            tl = tf.transform.translation
            q = tf.transform.rotation
            mat = np.identity(4)
            mat[:3, 3] = [tl.x, tl.y, tl.z]
            mat[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            return mat

            # Attach a callback to handle completion
            
        except Exception as e:
            self.node.get_logger().error(f"Error processing interrupt: {e}")


    def reset(self):
        self.node.get_logger().info("Resetting RL model.")
        #get ee pose wrt base_link
        tf_ee_base = self.wait_async_lookup_transform("base_link", 'mock_pruner__endpoint')
        self.node.get_logger().info(f"EE pose wrt base_link: {tf_ee_base}")
        # self.lstm_states = None
        self.episode_start = True
        #TODO: End effector index during training should be the same here
        self.blackboard.set("end_position_init", tf_ee_base[:3, 3])
        self.blackboard.set("is_terminated", False)
        self.steps = 0

        self.blackboard.set("steps", self.steps)


    def pretend_action(self, action):
        """To overcome latency issues, we pretend the action has been executed
        slower but compensate for that by executing it longer."""
        return action * self.pretend_action_scale

    def actual_action_from_pretend(self, pretend_action):
        """Reverse the pretend action scaling."""
        return pretend_action / self.pretend_action_scale

    def publish_velocity(self):
        action = self.pretend_action(self.blackboard.get("action")) * 0.1
        #Just rotate action to be in base_link frame
        action_linear = action[:3]
        action_angular = action[3:]

        #Build the twist message
        twist = TwistStamped()
        twist.header.frame_id = "tool0"
        twist.header.stamp = self.node.get_clock().now().to_msg()
        twist.twist.linear = Vector3(x=float(action_linear[0]), y=float(action_linear[1]), z=float(action_linear[2]))
        twist.twist.angular = Vector3(x=float(action_angular[0]), y=float(action_angular[1]), z=float(action_angular[2]))
        if self.blackboard.get("execute_action"):
            self.vel_publisher.publish(twist)


    # def publish_velocity(self):
    #     tf_ee_base = self.wait_async_lookup_transform("base_link", "tool0")
    #     action = self.pretend_action(self.blackboard.get("action")) * 0.2
    #     action_linear = action[:3]
    #     action_angular = action[3:]
    #     twist = TwistStamped()
    #     twist.header.frame_id = "tool0"
    #     twist.header.stamp = self.node.get_clock().now().to_msg()
    #     twist.twist.linear = Vector3(x=float(action_linear[0]), y=float(action_linear[1]), z=float(action_linear[2]))
    #     twist.twist.angular = Vector3(x=float(action_angular[0]), y=float(action_angular[1]), z=float(action_angular[2]))
    #     if self.blackboard.get("execute_action"):
    #         self.vel_publisher.publish(twist)

        

    def set_velocity_zero(self):
        self.blackboard.set("action", np.zeros(6))
        self.waiting_for_tick = True

    def update(self):
        if not self.waiting_for_tick:
            return py_trees.common.Status.RUNNING

        execute_action = self.blackboard.get("execute_action") # Set via interrupts
        reset_rl_model = self.blackboard.get("reset_rl_model") # Set via interrupts
        is_terminated = self.blackboard.get("is_terminated")
        if reset_rl_model:
            self.reset()
            self.node.get_logger().info("Resetting RL model.")
            self.blackboard.set("reset_rl_model", False)
            return py_trees.common.Status.SUCCESS
        if not execute_action or is_terminated:
            return py_trees.common.Status.SUCCESS
       
        
        self.node.get_logger().info("Computing RL action.")

        if self.blackboard.get("observation") is None:
            return py_trees.common.Status.FAILURE

        observation = self.make_observation()

        action, self.lstm_states = self.model.predict(
            observation,  # type: ignore[arg-type]
            state=self.lstm_states,
            episode_start=self.episode_start, # Resets the LSTM state at the beginning of each episode.
            deterministic=True,
        )
        # self.blackboard.action = action
        self.episode_start = False
        # self.waiting_for_tick = True
        self.blackboard.set("action", action)   
        self.publish_velocity()
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
        # self.node.get_logger().info("Velocity command published. Starting execution timer.")
        # self.waiting_for_tick = False
        self.timer = self.node.create_timer(self.execution_timeout, self.set_velocity_zero, callback_group=self.cb)
        self.logger.info(f"Computed action: {action}")
        self.steps+=1
        self.blackboard.set("steps", self.steps)
        return py_trees.common.Status.SUCCESS
