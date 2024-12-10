import py_trees
import numpy as np
from pruning_bt.utils.definitions import JointInfo
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
import rclpy


class DataGatheringNode(Node):
    """Node to handle concurrent data gathering and update the py_trees blackboard."""
    """Blackboards are shared between Nodes and Trees in the SAME process."""   

    def __init__(self):
        super().__init__('data_gathering_node')

        # Blackboard
        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.set(variable_name="joint_angles", value=None)
        self.blackboard.set(variable_name="camera_image", value=None)
        self.blackboard.set(variable_name="goal", value=None)
        self.blackboard.set(variable_name="last_camera_image", value=None)

        self.joint_states = JointInfo()

        # Callback group for concurrent execution
        self.callback_group = ReentrantCallbackGroup()
        self.bridge = CvBridge()

        # Subscribers
        self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10, callback_group=self.callback_group
        )
        self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.camera_image_callback, 10, callback_group=self.callback_group
        )

        self.create_subscription(
            Point, '/goal', self.goal_callback, 10, callback_group=self.callback_group
        )

        # Timer to display blackboard periodically
        # self.create_timer(0.1, self.display_blackboard)

    def display_blackboard(self):
        """Display the current blackboard content."""
        print(py_trees.display.unicode_blackboard())

    def joint_states_callback(self, msg: JointState):
        """Update joint states on the blackboard."""
        self.joint_states.update(msg)
        self.blackboard.set(variable_name="joint_angles", value=self.joint_states.get_joint_angles_ordered())
        self.blackboard.set(variable_name="joint_velocities", value=self.joint_states.get_joint_velocities_ordered())
        # self.get_logger().info(f"Updated joint_states on blackboard")

    def camera_image_callback(self, msg: Image):
        """Update the latest camera image on the blackboard."""
        self.blackboard.set(variable_name="last_camera_image", value=self.blackboard.get("camera_image"))
        self.blackboard.set(variable_name="camera_image", value=self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8"))
        # self.get_logger().info("Updated camera_image on blackboard.")

    def goal_callback(self, msg: Point):
        """Update the goal on the blackboard."""
        self.blackboard.goal = np.array([msg.x, msg.y, msg.z])
        self.get_logger().info(f"Updated goal on blackboard: {self.blackboard.goal}")


def main(args=None):
    
    
    rclpy.init(args=args)
    node = DataGatheringNode()
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.FATAL)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
