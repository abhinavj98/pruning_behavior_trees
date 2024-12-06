import py_trees
import torch as th
from geometry_msgs.msg import TwistStamped, Vector3
from rclpy.callback_groups import ReentrantCallbackGroup

class PublishVelocity(py_trees.behaviours.Behaviour):
    """Publish velocity commands to the robot."""

    def __init__(self, name, node):
        super().__init__(name)
        self.device = "cuda" if th.cuda.is_available() else "cpu"
        self.node = node
        self.publisher = None
        self.callback_group = ReentrantCallbackGroup()  # Enables concurrency
        self.lstm_states = None  # For recurrent policies

    def setup(self, **kwargs):
        """
        Set up the publisher for velocity commands with a ReentrantCallbackGroup.
        """
        self.publisher = self.node.create_publisher(
            TwistStamped, "/servo_node/delta_twist_cmds", 1, callback_group=self.callback_group
        )
        self.logger.info("Velocity publisher initialized with ReentrantCallbackGroup.")

    def action_to_twist(self, action):
        twist = TwistStamped()
        twist.header.frame_id = "base_link"
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.twist.linear = Vector3(x=float(action[0]), y=float(action[1]), z=float(action[2]))
        twist.twist.angular = Vector3(x=float(action[3]), y=float(action[4]), z=float(action[5]))
        return twist


    def update(self):
        blackboard = py_trees.blackboard.Blackboard()
        if "action" not in blackboard:
            return py_trees.common.Status.FAILURE

        action = blackboard.action
        twist = self.action_to_twist(action)
        self.publisher.publish(twist)
        self.logger.info("Published velocity command.")
        return py_trees.common.Status.SUCCESS