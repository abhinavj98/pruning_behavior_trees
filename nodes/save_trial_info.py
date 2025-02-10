import rclpy
from pruning_bt_interfaces.srv import LogEndpointPose
from rclpy.node import Node
import argparse

class LogEndpointPoseClient(Node):

    def __init__(self):
        super().__init__('log_endpoint_pose_client')
        self.client = self.create_client(LogEndpointPose, 'log_endpoint_pose')

    def send_request(self, **kwargs):
        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        # Send the request with keyword arguments (tree_type, tree_number, trial_number, status)
        request = LogEndpointPose.Request()
        request.tree_type = kwargs.get('tree_type')  # Type of tree or experiment (e.g., "pruner")
        request.tree_number = kwargs.get('tree_number')  # Tree number (e.g., 1)
        request.trial_number = kwargs.get('trial_number')  # Trial number (e.g., "trial_1")
        request.type = kwargs.get('type')  # Status ('desired' or 'achieved')

        # Call the service asynchronously
        future = self.client.call_async(request)
        future.add_done_callback(self.callback)

    def callback(self, future):
        response = future.result()
        self.get_logger().info(f"Success: {response.success}, Message: {response.message}")


def main(args=None):
    rclpy.init(args=args)
    client = LogEndpointPoseClient()

    # Set up argparse to handle keyword arguments in the terminal
    parser = argparse.ArgumentParser(description="Log Endpoint Pose Client")
    parser.add_argument('--tree_type', type=str, required=True, help="Type of tree or experiment (e.g., 'pruner')")
    parser.add_argument('--tree_number', type=int, required=True, help="Tree number (e.g., '1')")
    parser.add_argument('--trial_number', type=int, required=True, help="Trial number (e.g., '1')")
    parser.add_argument('--type', type=str, choices=['d', 'a'], required=True, help="Status ('desired' or 'achieved')")

    # Parse the arguments
    args = parser.parse_args()


    # Send the request to log the pose using keyword arguments
    client.send_request(tree_type=args.tree_type, tree_number=args.tree_number, trial_number=args.trial_number, type=args.type)

    rclpy.spin(client)


if __name__ == '__main__':
    main()
