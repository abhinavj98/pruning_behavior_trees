import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from pruning_bt_interfaces.srv import SetGoal
from geometry_msgs.msg import Point
import py_trees
import csv

class SetGoalService(Node):
    def __init__(self, tree_type, tree_number, trial_number):
        super().__init__('goal_service')

        # Create a publisher with transient local QoS for latched behavior
        # qos_profile = QoSProfile(
        #     depth=1,
        #     durability=DurabilityPolicy.TRANSIENT_LOCAL
        # )
        # self.publisher = self.create_publisher(Goal, '/goal', qos_profile)
        self.blackboard = py_trees.blackboard.Blackboard()
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.filename = f"{tree_type}_{tree_number}"
        with open(f'results/{self.filename}.csv', 'a') as f:
            writer = csv.writer(f)
        # Create the service
        self.srv = self.create_service(SetGoal, 'set_goal', self.handle_set_goal, callback_group=self.callback_group)

    def handle_set_goal(self, request, response):
        """Handle incoming requests to set a goal."""
        try:
            self.blackboard.set("goal", (request.position.x, request.position.y, request.position.z))

            # Log the published goal
            # self.get_logger().info(f"Published latched goal: {goal_msg}")
            self.get_logger().info(f"Writing goal to blackboard: ({request.position.x}, {request.position.y}, {request.position.z})")
            #Append goal to csv
            with open(f'results/{self.filename}.csv', 'a') as f:
                f.write(f"{request.position.x},{request.position.y},{request.position.z}\n")
            
            # Respond to the service call
            response.success = True
            response.message = f"Goal successfully published: ({request.position.x}, {request.position.y}, {request.position.z})"
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
