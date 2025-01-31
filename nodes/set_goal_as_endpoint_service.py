# This file will get the transform from base_link to endpoint_link and set the goal as the endpoint service
# This goal will be set by the set_goal_service.py service

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from pruning_bt_interfaces.srv import SetGoal
from geometry_msgs.msg import Point
#import Trigger
from std_srvs.srv import Trigger

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class SetGoalFromEndpointService(Node):
    def __init__(self):
        super().__init__('set_goal_from_endpoint_service')
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.srv = self.create_service(Trigger, 'set_goal_from_endpoint', self.handle_set_goal_from_endpoint, callback_group=self.callback_group)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        #Create a service client for set_goal
        self.set_goal_client = self.create_client(SetGoal, 'set_goal') 
        #Wait for the service to be available
        while not self.set_goal_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_goal_service not available, waiting again...')
            
    async def handle_set_goal_from_endpoint(self, request, response):
        """ When triggered, get transfrom from base_link to endpoint_link and call set_goal_service with the new goal"""
        try:
            # Get the transform from base_link to endpoint_link
            transform = self.tf_buffer.lookup_transform('base_link', 'mock_pruner__endpoint', rclpy.time.Time())
            # Calculate the new goal
            new_goal = Point()
            new_goal.x = transform.transform.translation.x
            new_goal.y = transform.transform.translation.y
            new_goal.z = transform.transform.translation.z

            set_goal_msg = SetGoal.Request()
            set_goal_msg.position = new_goal

            # Call the set_goal_service with the new goal
            self.get_logger().info(f"Setting goal from endpoint: ({new_goal.x}, {new_goal.y}, {new_goal.z})")   

            future = self.set_goal_client.call_async(set_goal_msg)

            await future

            if future.result() is not None:
                response.success = True
                response.message = "Goal successfully set from endpoint."
            else:
                response.success = False
                response.message = "Error calling set_goal service"

        except Exception as e:
            self.get_logger().error(f"Failed to set goal from endpoint: {e}")
            response.success = False
            response.message = f"Error: {e}"
        return response
        #     response.success = True
        #     response.message = "Goal successfully set from endpoint."
        # except Exception as e:
        #     self.get_logger().error(f"Failed to set goal from endpoint: {e}")
        #     response.success = False
        #     response.message = f"Error: {e}"
        # return response

def main(args=None):
    rclpy.init(args=args)
    node = SetGoalFromEndpointService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()