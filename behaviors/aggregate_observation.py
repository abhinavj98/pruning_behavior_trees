import py_trees
import numpy as np
import rclpy
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from scipy.spatial.transform import Rotation
from follow_the_leader_msgs.srv import Jacobian
from utils.definitions import Observation
from skimage.draw import disk
import cv2

class AggregateObservation(py_trees.behaviours.Behaviour):
    """Aggregate sensor data into an observation."""

    def __init__(self, name, node, pretend_action_scale=1.0, cam_height=240, cam_width=424, algo_height=240, algo_width=424):
        """
        :param node: The ROS2 node to use for tf2 buffer and transform listener.
        :param target_frame: The frame to transform into (e.g., "base_link").
        :param source_frame: The frame to transform from (e.g., "end_effector").
        """
        super().__init__(name)
        self.node = node  # ROS2 node that will be used for tf2 buffer and transform listener
        self.tf_buffer = Buffer()  # TF2 Buffer for storing transforms
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.blackboard = py_trees.blackboard.Blackboard()  # Blackboard for storing shared data
        self.pretend_action_scale = pretend_action_scale
        self.cam_height = cam_height
        self.cam_width = cam_width
        self.algo_height = algo_height
        self.algo_width = algo_width

    def lookup_transform(self, target_frame, source_frame, time=None, sync=True):
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

    def get_current_pose(self):
        """
        Get the current position of the robot's end effector.
        Return x, y, z position and orientation as a 3x3 rotation matrix.
        """
        #TODO: Pass frame names using ROS parameters
        tf_fb_end = self.lookup_transform(target_frame='fake_base',
                                          source_frame='endpoint')  # Looking at endpoint from fake_base
        pos_fb_end = tf_fb_end[:3, 3]
        orientation_fb_end = tf_fb_end[:3, :3]
        return pos_fb_end, orientation_fb_end

    def transform_goal_to_fake_base(self, goal):
        """
        Transform the goal position to the fake base frame.
        """
        # TODO: Pass frame names using ROS parameters
        tf_fb_goal = self.lookup_transform(target_frame='fake_base',
                                           source_frame='base_link')  # Looking at base_link from fake_base
        goal_fb = tf_fb_goal @ np.array([goal[0], goal[1], goal[2], 1])  # Transform goal to fake_base
        return goal_fb[:3]

    def create_goal_point_mask(self, point):
        """
        Project a 3D point onto the camera image as a white circle.

        :param height: The height of the resulting mask.
        :param width: The width of the resulting mask.
        :param point: The 3D point to project.
        :return: A resized point mask.
        """
        # Transform from base to camera frame
        height, width = self.algo_height, self.algo_width
        try:
            tf_base_cam = self.lookup_transform(
                'base_link', self.camera.tf_frame
            )
            tf_cam_base = np.linalg.inv(tf_base_cam)
        except Exception as e:
            self.get_logger().error(f"Failed to compute transform: {e}")
            return np.zeros((height, width), dtype=np.float32)

        # Initialize mask
        cam_height = (self.cam_height, self.cam_width)
        point_mask = np.zeros(cam_height, dtype=np.float32)

        if self.goal_sim is None:
            self.get_logger().warn("Goal is not set. Returning empty mask.")
            return np.zeros((height, width), dtype=np.float32)

        # Transform the point to the camera frame
        point_in_cam_frame = self.mul_homog(tf_cam_base, point)

        # Project the point onto the image plane
        projection = self.camera.project3dToPixel(point_in_cam_frame)
        row = int(cam_height[0] - projection[1])  # Adjust for image coordinate system
        col = int(cam_height[1] - projection[0])

        # Check if the projected point is within bounds
        if 0 <= row < cam_height[0] and 0 <= col < cam_height[1]:
            radius = 5  # Adjustable radius
            rr, cc = disk((row, col), radius, shape=cam_height)
            point_mask[rr, cc] = 1
        else:
            self.get_logger().warn(
                f"Projected point out of bounds. Row: {row}, Col: {col}, Bounds: {cam_height}"
            )

        # Resize the mask to the algorithm dimensions
        resized_mask = cv2.resize(point_mask, dsize=(width, height))
        return np.expand_dims(resized_mask, axis=0)

    @staticmethod
    def mul_homog(mat, pt):
        pt = np.array(pt)
        pt_homog = np.ones((*pt.shape[:-1], pt.shape[-1] + 1))
        pt_homog[..., : pt.shape[-1]] = pt
        return (mat @ pt_homog.T).T[..., : pt.shape[-1]]
    def encode_joint_angles(self, joint_angles):
        """
        Encode joint angles by using sin and cos.
        """
        return np.hstack([np.sin(joint_angles), np.cos(joint_angles)])

    def get_tool0_velocity(self):
        # Get jacobian matrix
        request = Jacobian.Request()
        future = self.jacobian_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        if future.result() is None:
            self.logger.warning("Service call failed.")
            return None
        else:
            response = future.result()
        jacobian = np.array(response.jacobian).reshape(6, 6)
        joint_velocities = self.blackboard.joint_velocities.reshape(6, 1)
        tool0_velocity = jacobian @ joint_velocities
        return tool0_velocity

    def pretend_action(self, action):
        """To overcome latency issues, we pretend the action has been executed
        slower but compensate for that by executing it longer."""
        return action * self.pretend_action_scale

    def actual_action_from_pretend(self, pretend_action):
        """Reverse the pretend action scaling."""
        return pretend_action / self.pretend_action_scale

    def update(self):
        if (
                self.blackboard.joint_states is None
                or self.blackboard.camera_image is None
                or self.blackboard.goal is None
        ):
            return py_trees.common.Status.RUNNING  # Wait for all data to be available, next tick will check again

        # Example observation aggregation logic
        observation = Observation()

        end_position, end_orientation = self.get_current_pose()
        fb_goal = self.transform_goal_to_fake_base(self.blackboard.goal)
        tool0_velocity = self.get_tool0_velocity()

        observation.achieved_goal = end_position - self.blackboard.end_position_init #TODO: On reset, end_position_init is set to the current end effector position on blackboard
        observation.achieved_or = end_orientation[:3, :2].reshape(6, )
        observation.desired_goal = fb_goal - self.blackboard.end_position_init
        observation.joint_angles = self.encode_joint_angles(self.blackboard.joint_states["angles"])
        observation.prev_action_achieved = self.pretend_action(tool0_velocity).reshape(6, )
        observation.mask = self.create_goal_point_mask(fb_goal)
        observation.relative_distance = end_position - fb_goal
        observation.rgb = self.blackboard.camera_image
        observation.prev_rgb = self.blackboard.last_camera_image

        # Update blackboard with aggregated observation
        self.blackboard.observation = observation._asdict()
        self.logger.info("Observation aggregated.")
        return py_trees.common.Status.SUCCESS
