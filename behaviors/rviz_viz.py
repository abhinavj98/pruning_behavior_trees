import py_trees
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import py_trees
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import numpy as np
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
import rclpy
import asyncio
from scipy.spatial.transform import Rotation

from std_msgs.msg import ColorRGBA

class TextMarkerPublisher():
    def __init__(self, node, keys):
        super().__init__()

        # Publisher for Marker messages
        self.node = node

        self.publisher = self.node.create_publisher(MarkerArray, 'robot_info', 10)

        # Create a timer to publish the marker periodically
        # self.timer = self.node.create_timer(1.0, self.publish_marker)
        self.keys = keys

        # Initialize a counter to show dynamic changes in the text
        self.counter = 0

        self.blackboard = py_trees.blackboard.Blackboard()

    def publish_marker(self):
        # Create a Marker message

        marker_array = MarkerArray()
        base_pos_max = np.array([1., -1., 2.])
        base_pose_min = np.array([1., -1., 1.])
        increment = (base_pos_max - base_pose_min)/len(self.keys)
        for i, key in enumerate(self.keys):
            marker = Marker()
            marker.header.stamp = self.node.get_clock().now().to_msg()
            marker.header.frame_id = "world"  # You can change this to the frame where you want the text to appear
            marker.ns = "text_marker"  # Namespace for the marker
            marker.id = i  # Unique ID for the marker (use a unique number for each marker)

            # Set the type of the marker to TEXT_VIEW_FACING
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD

            pos = base_pose_min + i*increment
            # Set the position where the text will appear in the 3D space
            marker.pose.position = Point(x=pos[0], y = pos[1], z = pos[2])  # Adjust position as needed

            # Set the text content of the marker
            marker.text = f"{key}:{self.blackboard.get(key)}"

            # Set the color of the text (RGBA format)
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red color, fully opaque

            # Set the scale of the text
            marker.scale.z = 0.1  # Font size (z-scale controls text size)

            # Publish the marker
            marker_array.markers.append(marker)

            # Increment counter for dynamic text
            self.counter += 1

        self.publisher.publish(marker_array )

class RVizVisualization(py_trees.behaviour.Behaviour):
    """
    A PyTree behavior that publishes visualization markers and images to RViz.
    """

    def __init__(self, name, asyncio_loop):
        super().__init__(name)
        
        self.asyncio_loop = asyncio_loop
        # Blackboard for shared data
        self.blackboard = py_trees.blackboard.Blackboard()
        self.bridge = CvBridge()


    def setup(self, node):
        self.node = node
        # Publishers
        self.info_display = TextMarkerPublisher(node, ['teleop_control', 'controller_mode', 'execute_action'])
        self.marker_pub = self.node.create_publisher(MarkerArray, "/visualization_marker", 10)
        self.image_pub = self.node.create_publisher(Image, "/point_mask_image", 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.waiting_for_transform = False
        self.transform_result = None

    async def lookup_transform(self, target_frame, source_frame):
        try:
            # self.node.get_logger().info(f"Looking up transform between {target_frame} and {source_frame}")
            future = self.tf_buffer.wait_for_transform_async(
                target_frame, source_frame, rclpy.time.Time()
            )
            await future
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time()
            )
            return transform
        except Exception as e:
            self.node.get_logger().error(f"Failed to lookup transform asynchronously: {e}")
            return None

    def wait_async_lookup_transform(self, target_frame, source_frame):
        """
        Handle interrupts asynchronously by scheduling them on the event asyncio_loop.
        """
        try:
            # Schedule the coroutine and get the Future
            self.waiting_for_transform = True
            # print("Getting transform between {} and {}".format(target_frame, source_frame))
            future = asyncio.run_coroutine_threadsafe(self.lookup_transform(target_frame, source_frame), self.asyncio_loop)

            def on_complete(fut):
                try:
                    result = fut.result()  # Fetch the result or handle exceptions
                    self.waiting_for_transform = False
                    self.transform_result = result
                    # self.node.get_logger().info(f"Transform lookup completed with result: {result}")
                except Exception as e:
                    self.node.get_logger().error(f"Transform lookup failed: {e}")

            future.add_done_callback(on_complete)
            while self.waiting_for_transform:
                pass
            
            tf = self.transform_result
            tl = tf.transform.translation
            q = tf.transform.rotation
            mat = np.identity(4)
            mat[:3, 3] = [tl.x, tl.y, tl.z]
            mat[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            return mat

            # Attach a callback to handle completion
            
        except Exception as e:
            self.node.get_logger().error(f"Error processing interrupt: {e}")

    def get_goal_marker(self):
        """
        Publish a spherical marker at the goal position.
        """
        goal = self.blackboard.get("goal")

        if goal is None:
            self.node.get_logger().warn("Goal not set on blackboard. Skipping goal marker.")
            return

        marker = Marker()
        marker.ns = 'goal'
        marker.id = 0
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Set goal position
        marker.pose.position.x = goal[0]
        marker.pose.position.y = goal[1]
        marker.pose.position.z = goal[2]

        # self.node.get_logger().info("Published goal marker at position: {}".format(goal))

        return marker
    
    def adj_multiply(self, mat, vec):
        """
        Multiply a 4x4 matrix with a 3x1 vector.
        """
        return np.dot(mat[:3, :3], vec) + mat[:3, 3]
        
    
    def get_action_marker(self):
        """Executed action visualization"""
        tf_base_end = self.wait_async_lookup_transform("base_link", 'mock_pruner__endpoint')
        if tf_base_end is None:
            self.node.get_logger().warn("Failed to lookup transform. Skipping action marker.")
            return
        
        action_base = self.blackboard.get("action")
        tf_b_t = self.wait_async_lookup_transform("base_link", "tool0")
        #Multiply action to get in base_link frame
        # action = np.dot(tf_b_t, action_base) $TODO: Check if this is correct
        action = action_base

        if action is None:
            self.node.get_logger().warn("Action not set on blackboard. Skipping action marker.")
            return
        translation_marker = Marker()
        translation_marker.ns = 'action'
        translation_marker.id = 0

        translation_marker.header.frame_id = 'base_link'
        translation_marker.header.stamp = self.node.get_clock().now().to_msg()
        translation_marker.type = Marker.ARROW
        translation_marker.action = Marker.ADD

        # Arrow scale and color
        translation_marker.scale.x = 0.01  # Shaft diameter
        translation_marker.scale.y = 0.01  # Arrowhead width
        translation_marker.scale.z = 0.01  # Arrowhead height
        translation_marker.color.a = 1.0
        translation_marker.color.r = 1.0
        translation_marker.color.g = 1.0
        translation_marker.color.b = 0.0

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
        translation_marker.points = [start, end]

        axis_colors = {
            'x': (1.0, 0.0, 0.0),  # Red for X-axis
            'y': (0.0, 1.0, 0.0),  # Green for Y-axis
            'z': (0.0, 0.0, 1.0)  # Blue for Z-axis
        }

        angular_marker = (Marker(), Marker(), Marker())
        angular_marker[0].id = 1
        angular_marker[0].ns = 'action'
        angular_marker[1].id = 2
        angular_marker[1].ns = 'action'
        angular_marker[2].id = 3
        angular_marker[2].ns = 'action'

        for i, axis in enumerate(['x', 'y', 'z']):
            marker = angular_marker[i]
            marker.header.frame_id = 'base_link'
            marker.header.stamp = self.node.get_clock().now().to_msg()
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            # marker.id = i  # Ensure unique ID for each axis marker

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
        return translation_marker, angular_marker


    def publish_point_mask(self):
        """
        Publish the point mask as an image to RViz.
        """
        point_mask = self.blackboard.get("point_mask")
        if point_mask is None:
            # self.node.get_logger().warn("Point mask not set on blackboard. Skipping mask visualization.")
            return

        try:
            # self.node.get_logger().log("Publishing point mask, shape: {}".format(point_mask.shape))
            point_mask = (point_mask * 255).astype(np.uint8)
            #Change axis to get 1st channel to last
            point_mask = np.moveaxis(point_mask, 0, -1)
            img_msg = self.bridge.cv2_to_imgmsg(point_mask, encoding="mono8")
            self.image_pub.publish(img_msg)
        except Exception as e:
            self.node.get_logger().error(f"Failed to publish point mask image: {e}")
            
    def publish_blackboard_text(self):
        """
        Read blackboard stuff from 
        """
    def update(self):
        """
        Periodically publish markers and images to RViz.
        """
        goal_marker = self.get_goal_marker()
        # trans_markers, angular_markers = self.get_action_marker()
        # print(len(angular_markers))
        self.marker_pub.publish(MarkerArray(markers=[goal_marker]))
        self.info_display.publish_marker()  
        self.publish_point_mask()
        return py_trees.common.Status.SUCCESS

