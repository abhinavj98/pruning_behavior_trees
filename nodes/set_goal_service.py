import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from pruning_bt_interfaces.srv import SetGoal
from geometry_msgs.msg import Point
import py_trees

class SetGoalService(Node):
    def __init__(self):
        super().__init__('goal_service')

        # Create a publisher with transient local QoS for latched behavior
        qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        # self.publisher = self.create_publisher(Goal, '/goal', qos_profile)
        self.blackboard = py_trees.blackboard.Blackboard()
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()

        # Create the service
        self.srv = self.create_service(SetGoal, 'set_goal', self.handle_set_goal, callback_group=self.callback_group)

    def handle_set_goal(self, request, response):
        """Handle incoming requests to set a goal."""
        try:
            # Create and publish the goal message
            # goal_msg = Point()
            # goal_msg.x = request.x
            # goal_msg.y = request.y
            # goal_msg.z = request.z
            # self.publisher.publish(goal_msg)
            self.blackboard.set("goal", (request.x, request.y, request.z))

            # Log the published goal
            # self.get_logger().info(f"Published latched goal: {goal_msg}")
            self.get_logger().info(f"Writing goal to blackboard: ({request.x}, {request.y}, {request.z})")
            # Respond to the service call
            response.success = True
            response.message = f"Goal successfully published: ({request.x}, {request.y}, {request.z})"
        except Exception as e:
            # Handle any errors and log them
            self.get_logger().error(f"Failed to write goal: {e}")
            response.success = False
            response.message = f"Error: {e}"

        return response


def main(args=None):
    rclpy.init(args=args)
    node = SetGoalService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
