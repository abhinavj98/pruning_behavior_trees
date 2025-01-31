import py_trees
import numpy as np
import rclpy
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from scipy.spatial.transform import Rotation
from move_group_server_interfaces.srv import GetJacobian
from pruning_bt.utils.definitions import Observation
from skimage.draw import disk
import cv2
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo


class AggregateObservation(py_trees.behaviour.Behaviour):
    """Aggregate sensor data into an observation."""

    def __init__(self, name, pretend_action_scale=1.0, cam_info_topic = "/camera/camera/color/camera_info", algo_height=240, algo_width=424):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()
        self.pretend_action_scale = pretend_action_scale
        self.algo_height = algo_height
        self.algo_width = algo_width
        self.blackboard.set("observation", None)
        self.blackboard.set("end_position_init", [0., 0., 0.]) #Whenever robot is reset this gets updated
        self.blackboard.set("prev_camera_image", None)
        self.blackboard.set("camera_image", None)
        self.blackboard.set("point_mask", None)
        self.blackboard.set("goal", (0., -1., 0.))
        self.camera = PinholeCameraModelNP()
        self.cam_info_topic = cam_info_topic
        # Results placeholder for callbacks
        self.lookup_transform_result = None
        self.jacobian_result = None

    def setup(self, node):
        """Assign the ROS2 node during setup."""
        self.node = node
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.jacobian_client = self.node.create_client(GetJacobian, 'move_group_server/get_jacobian', callback_group=self.callback_group)
        while not self.jacobian_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Jacobian service not available, waiting again...')
        if self.cam_info_topic is not None:
            self._cam_info_sub = self.node.create_subscription(CameraInfo, self.cam_info_topic, self._handle_cam_info, 1)
        self.blackboard.set("goal", (0., -1., 0.)) #TODO: This is in setup again because probably the interrupt is changing this to a different value? Investigate
        # while self.camera.height is None:
        #     self.node.get_logger().info('Cam not available, waiting again...')
        dummy_obs = self.get_dummy_obs()
        self.blackboard.set("observation", dummy_obs._asdict())
        return True
    
    def _handle_cam_info(self, msg: CameraInfo):
        self.camera.fromCameraInfo(msg)
        return

    def lookup_transform(self, target_frame, source_frame):
       
        try:
            self.transform_ready = False
            # self.node.get_logger().info(f"Looking for transform from {source_frame} to {target_frame}...")
            future = self.tf_buffer.wait_for_transform_async(target_frame, source_frame, rclpy.time.Time())
            future.add_done_callback(self.lookup_transform_callback)
            while not self.transform_ready:
                print("Waiting for transform between frames...", source_frame, target_frame)

            tf = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            tl = tf.transform.translation
            q = tf.transform.rotation
            mat = np.identity(4)
            mat[:3, 3] = [tl.x, tl.y, tl.z]
            mat[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            return mat
            
        except Exception as e:
            self.node.get_logger().error(f"Failed to initiate transform lookup: {e}")
            

    def lookup_transform_callback(self, future):
        """Callback to handle the transform lookup result."""
        if future.result():
            self.transform_ready = True

    def get_current_pose(self):
        """Non-blocking get current pose."""
        tf = self.lookup_transform('base_link', 'mock_pruner__endpoint')
        endpoint_position, endpoint_orientation = tf[:3, 3],tf[:3, :3]
        return endpoint_position, endpoint_orientation


    def update_jacobian(self):
        """Non-blocking Jacobian service call using add_done_callback."""
        if not self.jacobian_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error("Jacobian service not available.")
            return
        self.jacobian_result = None
        request = GetJacobian.Request()
        future = self.jacobian_client.call_async(request)
        future.add_done_callback(self.jacobian_callback)
        while self.jacobian_result is None:
            pass
            # self.node.get_logger().info("Waiting for Jacobian...")
        return self.jacobian_result

    def jacobian_callback(self, future):
        """Callback to handle the Jacobian service response."""
        try:
            response = future.result()
            if response is None:
                self.node.get_logger().error("Jacobian service call failed.")
                self.jacobian_result = None
            else:
                self.jacobian_result = np.array(response.jacobian).reshape(6, 6)
                # self.node.get_logger().info(f"Jacobian retrieved successfully")
        except Exception as e:
            self.node.get_logger().error(f"Jacobian service call failed: {e}")
            self.jacobian_result = None

    def make_adjoint_from_transform(self, transform):
        """
        Compute the adjoint matrix from a homogeneous transform.
        """
        rotation_matrix = transform[:3, :3]
        translation = transform[:3, 3]
        p_skew = self.skew_symmetric(translation)
        adjoint = np.block([
            [rotation_matrix, np.zeros((3, 3))],
            [np.dot(p_skew, rotation_matrix), rotation_matrix]
        ])
        return adjoint
    
    def skew_symmetric(self, vector):
        """
        Compute the skew-symmetric matrix of a vector.
        :param vector: np.array, shape (3,)
        :return: np.array, shape (3, 3)
        """
        x, y, z = vector
        return np.array([
            [0, -z, y],
            [z, 0, -x],
            [-y, x, 0]
        ])

    def get_tool0_velocity_and_tool0_goal(self):
        """Compute tool velocity."""
        jacobian = self.update_jacobian()
        joint_velocities = self.blackboard.get("joint_velocities").reshape(6, 1)
        tool0_velocity_base_link = jacobian @ joint_velocities

        #Get transform from base_link to tool0
        tf_tool0_base = self.lookup_transform('tool0', 'base_link')
        adjoint_tool0_base = self.make_adjoint_from_transform(tf_tool0_base)
        tool0_velocity_tool0 = np.dot(adjoint_tool0_base, tool0_velocity_base_link)
        
        goal = self.blackboard.get("goal")
        tool0_goal = self.mul_homog(tf_tool0_base, goal)

        return tool0_velocity_tool0, tool0_goal
    
    
    def encode_joint_angles(self, joint_angles):
        """
        Encode joint angles by using sin and cos.
        """
        return np.hstack([np.sin(joint_angles), np.cos(joint_angles)])
    
    def create_goal_point_mask(self, point):
        """
        Project a 3D point onto the camera image as a white circle.

        :param height: The height of the resulting mask.
        :param width: The width of the resulting mask.
        :param point: The 3D point to project.
        :return: A resized point mask.
        """
        # Transform from base to camera frame
        tf_cam_base = self.lookup_transform('camera_color_optical_frame', 'base_link')

        # Initialize mask
        cam_image_dim = (self.camera.height, self.camera.width)
        point_mask = np.zeros(cam_image_dim, dtype=np.float32)

        # Transform the point to the camera frame
        point_in_cam_frame = self.mul_homog(tf_cam_base, point)
        #Publish point in cam as a marker

        # Project the point onto the image plane
        projection = self.camera.project3dToPixel(point_in_cam_frame)
        col = projection[0]  # Adjust for image coordinate system
        row = projection[1]  # Adjust for image coordinate system

        # Check if the projected point is within bounds
        if 0 <= row < cam_image_dim[0] and 0 <= col < cam_image_dim[1]:
            radius = 20  # Adjustable radius
            rr, cc = disk((row, col), radius, shape=cam_image_dim)
            point_mask[rr, cc] = 1
        # else:
        #     self.node.get_logger().warn(
        #         f"Projected point out of bounds. Row: {row}, Col: {col}, Bounds: {cam_image_dim}"
        #     )

        # Resize the mask to the algorithm dimensions
        resized_mask = cv2.resize(point_mask, dsize=(self.algo_width, self.algo_height))
        return np.expand_dims(resized_mask, axis=0)
    
    @staticmethod
    def mul_homog(mat, vec):
        """Multiply a homogeneous matrix by a vector."""
        vec = np.array(vec)
        vec_homog = np.ones((vec.shape[0] + 1,))
        vec_homog[:vec.shape[0]] = vec

        return (mat @ vec_homog)[:3]
    
    
    def get_dummy_obs(self):
        achieved_goal = np.zeros((3,), dtype=np.float32)
        achieved_or = np.zeros((6,), dtype=np.float32)
        desired_goal = np.zeros((3,), dtype=np.float32)
        joint_angles = np.zeros((12,), dtype=np.float32)
        prev_action_achieved = np.zeros((6,), dtype=np.float32)
        point_mask = np.zeros((1, self.algo_height, self.algo_width), dtype=np.float32)
        relative_distance = np.zeros((3,), dtype=np.float32)
        rgb = np.zeros((3, self.algo_height, self.algo_width), dtype=np.float32)
        prev_rgb = np.zeros((3, self.algo_height, self.algo_width), dtype=np.float32)

        observation = Observation(
            achieved_goal=achieved_goal,
            achieved_or=achieved_or,
            desired_goal=desired_goal,
            joint_angles=joint_angles,
            prev_action_achieved=prev_action_achieved,
            point_mask=point_mask,
            relative_distance=relative_distance,
            rgb=rgb,
            prev_rgb=prev_rgb,
        )

        return observation
        




    def update(self):
        """Non-blocking observation aggregation."""
        if not self.camera.height:
            return py_trees.common.Status.RUNNING
        # self.node.get_logger().info("Aggregating observation...")
        if (
            self.blackboard.get("joint_angles") is None
            or self.blackboard.get("camera_image") is None
            or self.blackboard.get("goal") is None
        ):
            self.node.get_logger().info("Waiting for all data to be available.")
            return py_trees.common.Status.RUNNING

      
       
        endpoint_position, endpoint_orientation =  self.get_current_pose() #wrt base_link
        goal = np.array(self.blackboard.get("goal")) #Convert goal to base_link frame
        tool0_velocity, tool0_goal = self.get_tool0_velocity_and_tool0_goal()
        #Get tool0 velocity wrt base_link rotation only (TODO: Use adjoints)
        # tf_ee_base = self.lookup_transform('base_link', 'tool0')
        # tool0_velocity[:3] = np.dot(tf_ee_base[:3, :3].T, tool0_velocity[:3])
        # tool0_velocity[3:] = np.dot(tf_ee_base[:3, :3].T, tool0_velocity[3:])

        endpoint_position_init = self.blackboard.get("end_position_init")
    
        achieved_goal = endpoint_position - endpoint_position_init #end position wrt base_link
        desired_goal_tool = tool0_goal
        
        achieved_or = endpoint_orientation[:3, :2].reshape(6, )
        joint_angles = self.encode_joint_angles(self.blackboard.get("joint_angles"))
        prev_action_achieved = tool0_velocity.reshape(6, ) #In tool0 frame
        mask = self.create_goal_point_mask(self.blackboard.get("goal"))
        relative_distance = endpoint_position - goal

        self.blackboard.set("point_mask", mask)
        rgb = self.blackboard.get("camera_image")
        prev_rgb = self.blackboard.get("prev_camera_image")
        if prev_rgb is None:
            prev_rgb = np.zeros_like(rgb)

        observation = Observation(
            achieved_goal=achieved_goal,
            achieved_or=achieved_or,
            desired_goal=desired_goal_tool,
            joint_angles=joint_angles,
            prev_action_achieved=prev_action_achieved,
            point_mask=mask,
            relative_distance=relative_distance,
            rgb=rgb,
            prev_rgb=prev_rgb,
        )

        #Print shape of the observation
        # for key, value in observation._asdict().items():
        #     self.node.get_logger().info(f"Key: {key}, Value shape: {value.shape}")
        self.blackboard.set("observation", observation._asdict())
       
        
        return py_trees.common.Status.SUCCESS


class PinholeCameraModelNP(PinholeCameraModel):
    """
    Modifications to the PinholeCameraModel class to make them operate with Numpy.
    """
    def project3dToPixel(self, pts):
        pts = np.array(pts)
        pts_homog = np.ones((*pts.shape[:-1], pts.shape[-1] + 1))
        pts_homog[..., :3] = pts
        x, y, w = np.array(self.P) @ pts_homog.T
        return np.array([x / w, y / w]).T

    def getDeltaU(self, deltaX, Z):
        fx = self.P[0, 0]
        return fx * deltaX / Z
