import py_trees
import numpy as np
from utils.definitions import JointInfo
from sensor_msgs.msg import JointState, Image, Point
from cv_bridge import CvBridge
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node


class DataGatheringNode(Node):
    """Node to handle concurrent data gathering and update the py_trees blackboard.
    Use ROS2 with reentrant callback group to handle concurrent data gathering and
    update the py_trees blackboard."""

    def __init__(self):
        super().__init__('data_gathering_node')

        # Blackboard
        self.blackboard = py_trees.blackboard.Blackboard()
        self.joint_states = JointInfo()
        self.blackboard.joint_angles = None
        self.blackboard.camera_image = None
        self.blackboard.goal = None

        # Callback group for concurrent execution
        self.callback_group = ReentrantCallbackGroup()
        self.bridge = CvBridge()

        # Subscribers
        self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10, callback_group=self.callback_group
        )
        self.create_subscription(
            Image, '/camera/image_raw', self.camera_image_callback, 10, callback_group=self.callback_group
        )
        self.create_subscription(
            Point, '/goal', self.goal_callback, 10, callback_group=self.callback_group
        )

    def joint_states_callback(self, msg: JointState):
        self.joint_states.update(msg)
        self.blackboard.joint_angles = self.joint_states.get_joint_angles_ordered()
        self.blackboard.joint_velocities = self.joint_states.get_joint_velocities_ordered()
        self.get_logger().info("Updated joint_states on blackboard.")

    def camera_image_callback(self, msg: Image):
        self.blackboard.last_camera_image = self.blackboard.camera_image
        self.blackboard.camera_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        self.get_logger().info("Updated camera_image on blackboard.")

    def goal_callback(self, msg: Point):
        self.blackboard.goal = np.array([msg.x, msg.y, msg.z])
        self.get_logger().info("Updated goal on blackboard.")

