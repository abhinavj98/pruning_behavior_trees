import py_trees
from visualization_msgs.msg import Marker, ColorRGBA
from geometry_msgs.msg import Point
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from sensor_msgs.msg import Image
class RVizMarkerPublisher(py_trees.behaviours.Behaviour):
    """
    A behavior that publishes markers to RViz.
    """

    def __init__(self, name, node, topic_name="/visualization_marker"):
        """
        :param name: Name of the behavior.
        :param node: ROS2 node for creating the publisher.
        :param topic_name: Topic to publish markers to.
        """
        super().__init__(name)
        self.node = node
        self.topic_name = topic_name
        self.velocity_marker_pub = self.node.create_publisher(Marker, "/velocity_marker", 10)
        self.goal_marker_pub = self.node.create_publisher(Marker, "/goal_marker", 10)
        self.image_pub = self.create_publisher(Image, '/point_mask_image', 10)
        self.blackboard = py_trees.blackboard.Blackboard()
        self.tf_buffer = Buffer()  # TF2 Buffer for storing transforms
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

    def lookup_transform(self, target_frame, source_frame, time=None, sync=True): #TODO: Make this a TFNode
        """
        Look up a transform between frames, with optional matrix conversion.
        We convert the source frame to the target frame.
        To be even more clear, we are looking at the source frame from the target frame.
        To look at frame B with respect to frame A (T_AB), A is the target frame and B is the source frame.
        """
        if time is None:
            time = rclpy.time.Time()

        if sync:
            future = self.tf_buffer.wait_for_transform_async(target_frame, source_frame, time)
            rclpy.spin_until_future_complete(self.node, future)

        tf = self.tf_buffer.lookup_transform(target_frame, source_frame, time)

        # Convert to a 4x4 transformation matrix
        tl = tf.transform.translation
        q = tf.transform.rotation
        mat = np.identity(4)
        mat[:3, 3] = [tl.x, tl.y, tl.z]
        mat[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        self.logger.info(f"Transform from {source_frame} to {target_frame}:\n{mat}")
        return mat

    def setup(self, **kwargs):
        """
        Initialize the publisher.
        """
        self.publisher = self.node.create_publisher(Marker, self.topic_name, 10)
        self.logger.info(f"Marker publisher initialized on topic: {self.topic_name}")

    def publish_velocity_marker(self):
        action = self.blackboard.action
        tf_base_end = self.lookup_transform("base_link", "endpoint")
        # Create and configure the marker
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Arrow scale and color
        marker.scale.x = 0.01  # Shaft diameter
        marker.scale.y = 0.01  # Arrowhead width
        marker.scale.z = 0.01  # Arrowhead height
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        # Define start point (end-effector position)
        start = Point(
            x=tf_base_end[0, 3],
            y=tf_base_end[1, 3],
            z=tf_base_end[2, 3]
        )

        # Define end point (end-effector position + action vector)
        end = Point(
            x=start.x + action[0],
            y=start.y + action[1],
            z=start.z + action[2]
        )

        # Assign points to the marker
        marker.points = [start, end]

        axis_colors = {
            'x': (1.0, 0.0, 0.0),  # Red for X-axis
            'y': (0.0, 1.0, 0.0),  # Green for Y-axis
            'z': (0.0, 0.0, 1.0)  # Blue for Z-axis
        }


        for i, axis in enumerate(['x', 'y', 'z']):
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.id = i  # Ensure unique ID for each axis marker

            # Set marker scale (shaft and head thickness)
            marker.scale.x = 0.02  # Shaft diameter
            marker.scale.y = 0.05  # Arrowhead width
            marker.scale.z = 0.05  # Arrowhead height

            # Set color based on axis
            marker.color.a = 1.0
            marker.color.r, marker.color.g, marker.color.b = axis_colors[axis]

            # Define end point based on angular velocity component
            end = Point(
                x=start.x + (action[3] if axis == 'x' else 0.0),
                y=start.y + (action[4] if axis == 'y' else 0.0),
                z=start.z + (action[5] if axis == 'z' else 0.0)
            )

            # Assign points to marker
            marker.points = [start, end]

            # Publish marker
            self.marker_pub.publish(marker)

        # Publish the marker
        self.marker_pub.publish(marker)
        self.logger.info("Arrow marker published.")

    def publish_goal_marker(self):
        tf_ee_sim = self.lookup_transform("base_link", "endpoint")
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        start = Point()
        start.x = tf_ee_sim[:3, 3][0]
        start.y = tf_ee_sim[:3, 3][1]
        start.z = tf_ee_sim[:3, 3][2]

        marker.pose.position = start
        self.endpoint_marker.publish(marker)
        return

    def publish_optical_flow_viz(self):
        pass

    def publish_point_mask(self):
        point_mask = self.bridge.cv2_to_imgmsg(self.blackboard.observation.mask, encoding='rgb8')
        # Publish the image
        self.image_pub.publish(point_mask)

    def update(self):
        """
        Publish a marker message to RViz.
        """
        self.publish_velocity_marker()
        self.publish_goal_marker()
        return py_trees.common.Status.SUCCESS


