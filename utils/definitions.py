from collections import namedtuple
import numpy as np
from sensor_msgs.msg import JointState

Joint = namedtuple("Joint", ["angle", "velocity"])
Observation = namedtuple("Observation", ["achieved_goal", "achieved_or", "desired_goal", "joint_angles",
                                         "prev_action_achieved", "relative_distance", "rgb", "prev_rgb", "mask"])
class JointInfo():
    def __init__(self):
        self.control_joints = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint",
                               "wrist_2_joint", "wrist_3_joint"]
        self.joint_angle_vel = {joint: Joint(None, None) for joint in self.control_joints}

    def update(self, msg: JointState):
        for i, joint in enumerate(msg.name):
            self.joint_angle_vel[joint] = Joint(msg.position[i], msg.velocity[i])

    def get_joint_angles_ordered(self):
        #Collect the joint angles in the order of the control joints
        return np.array([self.joint_angle_vel[joint].angle for joint in self.joint_angle_vel.keys()])

    def get_joint_velocities_ordered(self):
        #Collect the joint velocities in the order of the control joints
        return np.array([self.joint_angle_vel[joint].velocity for joint in self.joint_angle_vel.keys()])

#Do dictionaries enumerate in insertion order?
# https://stackoverflow.com/questions/39980323/do-dictionaries-enumerate-in-insertion-order