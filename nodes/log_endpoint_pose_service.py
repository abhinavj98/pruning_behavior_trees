import rclpy
from rclpy.node import Node
from pruning_bt_interfaces.srv import LogEndpointPose
from geometry_msgs.msg import Pose, Point
import csv
import time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class LogEndpointPoseService(Node):
    def __init__(self):
        super().__init__('log_endpoint_pose_service')
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.srv = self.create_service(LogEndpointPose, 'log_endpoint_pose', self.handle_log_endpoint_pose, callback_group=self.callback_group)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info("Running LogEndpointPoseService")

    def handle_log_endpoint_pose(self, request, response):
        """When triggered, get transform from base_link to endpoint_link and log the achieved pose"""
        
        tree_type = request.tree_type
        trial_number = request.trial_number
        tree_number = request.tree_number

        filename = f"results/{tree_type}_{tree_number}_{trial_number}_result.csv"  # CSV filename with tree type and trial number

        try:
            # Get the transform from base_link to mock_pruner__endpoint (achieved pose)
            transform = self.tf_buffer.lookup_transform('base_link', 'mock_pruner__endpoint', rclpy.time.Time())

            # Achieved pose
            position = transform.transform.translation
            orientation = transform.transform.rotation

            # Prepare data for CSV (status = 'achieved', followed by pose data)
            pose_data = [
                position.x,
                position.y,
                position.z,
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w
            ]
            log_data = [str(request.type)] + pose_data + [time.time()]  # Add timestamp

            # Open or create the CSV file
            try:
                with open(filename, 'a', newline='') as csvfile:
                    csv_writer = csv.writer(csvfile)
                    # If the file is empty (i.e., new), write the header
                    if csvfile.tell() == 0:
                        csv_writer.writerow(['type', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'timestamp'])  # Header
                    csv_writer.writerow(log_data)  # Write the achieved pose data

                response.success = True
                response.message = f"Achieved pose logged successfully to {filename}"

            except Exception as e:
                response.success = False
                response.message = f"Error writing to file {filename}: {str(e)}"

        except Exception as e:
            self.get_logger().error(f"Failed to get transform: {str(e)}")
            response.success = False
            response.message = f"Error: {str(e)}"

        return response


def main(args=None):
    rclpy.init(args=args)
    node = LogEndpointPoseService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
