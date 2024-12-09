import rclpy
from rclpy.node import Node
from py_trees.blackboard import Blackboard
from std_msgs.msg import String

class BlackboardUpdater(Node):
    """
    ROS2 Node to update a blackboard value.
    """

    def __init__(self):
        super().__init__('blackboard_updater')

        # Shared blackboard
        self.blackboard = Blackboard()
        self.blackboard.hello_message = None

        # Publisher and Timer
        self.timer = self.create_timer(1.0, self.update_blackboard)
        self.publisher = self.create_publisher(String, '/hello_topic', 10)

    def update_blackboard(self):
        # Mock data to write to the blackboard
        message = "Hello from ROS2 Node!{}".format(self.get_clock().now().nanoseconds)
        self.blackboard.set("hello_message", message)
        self.get_logger().info(f"Updated blackboard: {message}")

        # Publish the same message (optional, for debugging)
        msg = String(data=message)
        self.publisher.publish(msg)
