import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker, InteractiveMarkerFeedback
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
import csv
import os


class ClickPruningPointSelection(Node):
    def __init__(self):
        super().__init__('pruning_point_selection_node')

        # Subscribe to the /clicked_point topic
        self.clicked_point_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10
        )
        self.get_logger().info("Subscribed to '/clicked_point'")

        # Interactive Marker Server
        self.marker_server = InteractiveMarkerServer(self, "interactive_marker_server")

        # Variables
        self.current_point = None
        self.csv_file_path = 'goal_log.csv'

        # Initialize the CSV file
        self.init_csv_file()

    def init_csv_file(self):
        """Create the CSV file with headers if it doesn't exist."""
        if not os.path.exists(self.csv_file_path):
            with open(self.csv_file_path, mode='w', newline='') as file:
                writer = csv.writer(file)
            self.get_logger().info(f"Created file: {self.csv_file_path}")

    def clicked_point_callback(self, msg):
        """Callback for the clicked point."""
        self.get_logger().info(f"Received Point: x={msg.point.x}, y={msg.point.y}, z={msg.point.z}")
        self.current_point = msg.point

        # Create an interactive marker at the received point
        self.create_interactive_marker(msg.point)

    def create_interactive_marker(self, point):
        """Create an interactive marker for the selected point."""
        # Initialize the interactive marker
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "world"  
        int_marker.name = "adjustable_marker"
        int_marker.description = "Drag to adjust the point"
        int_marker.scale = 0.5

        # Set the initial position of the marker
        int_marker.pose.position.x = point.x
        int_marker.pose.position.y = point.y
        int_marker.pose.position.z = point.z

        # Add controls for moving along x, y, z axes
        self.add_movement_controls(int_marker)

        # Add a context menu to save the adjusted position
        self.add_button_control(int_marker)

        # Add the marker visualization (sphere)
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(marker)
        int_marker.controls.append(control)

        # Insert the marker into the server
        self.marker_server.insert(marker=int_marker, feedback_callback = self.process_feedback)
        self.marker_server.applyChanges()

    def add_movement_controls(self, int_marker):
        """Add movement controls to the interactive marker."""
        # Define axis names and their corresponding quaternion orientations
        axes = {
            "x": (1.0, 1.0, 0.0, 0.0),  # (w, x, y, z)
            "y": (1.0, 0.0, 1.0, 0.0),
            "z": (1.0, 0.0, 0.0, 1.0)
        }

        for axis, orientation in axes.items():
            control = InteractiveMarkerControl()
            control.name = f"move_{axis}"
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

            # Set orientation dynamically
            control.orientation.w = orientation[0]
            control.orientation.x = orientation[1]
            control.orientation.y = orientation[2]
            control.orientation.z = orientation[3]
            control.always_visible = True

            # Append the control to the marker
            int_marker.controls.append(control)

    def add_button_control(self, int_marker):
        """Add a button control to the interactive marker."""
        button_control = InteractiveMarkerControl()
        button_control.interaction_mode = InteractiveMarkerControl.BUTTON
        button_control.name = "button_control"

        # Add a marker for visualization
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.5

        button_control.markers.append(marker)
        button_control.always_visible = True

        # Attach the button control to the interactive marker
        int_marker.controls.append(button_control)

    def process_feedback(self, feedback):
        """Process feedback from the interactive marker."""
        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            self.get_logger().info("Save Point clicked.")
            # Save the current position when "Save Point" is clicked
            self.save_to_csv(feedback.pose.position)
            #Erase marker
            self.marker_server.erase(feedback.marker_name)
            self.marker_server.applyChanges()
            self.get_logger().info(f"Marker {feedback.marker_name} erased.")
        else:
            self.get_logger().info(
                f"Adjusted Point: x={feedback.pose.position.x}, y={feedback.pose.position.y}, z={feedback.pose.position.z}"
            )

    def save_to_csv(self, position):
        """Save the adjusted position to the CSV file."""
        with open(self.csv_file_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([position.x, position.y, position.z])
        self.get_logger().info(f"Saved Point: x={position.x}, y={position.y}, z={position.z} to {self.csv_file_path}")


def main(args=None):
    rclpy.init(args=args)
    node = ClickPruningPointSelection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped cleanly.")
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
