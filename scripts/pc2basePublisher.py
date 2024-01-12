#!/usr/bin/env python
import tf
import rospy
import numpy as np
from copy import deepcopy
from pcPubBase import pcPubBase
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R

import rospkg
rospack = rospkg.RosPack()

class pc2basePublisher(pcPubBase):

    def __init__(self):
        super().__init__()
        self.obj_name = rospy.get_param('~mj_obj_name', 'object')
        self.obj_base_frame = rospy.get_param('~obj_base_frame', 'base_link')
        
        rospy.Subscriber("/robot_base_pose", Pose, self.base_pose_callback, queue_size=1)
        rospy.Subscriber("/mujoco_joint_states", JointState, self.callback, queue_size=1)

        self.base_pose = np.eye(4)
        self.base2obj = np.eye(4)

        self.br = tf.TransformBroadcaster()


    def base_pose_callback(self, msg):
        robot_base_pose = np.eye(4)
        robot_base_pose[:3, 3] = np.array([msg.position.x, msg.position.y, msg.position.z])
        robot_base_pose[:3, :3] = R.from_quat([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]).as_matrix()
        self.base_pose = np.linalg.inv(robot_base_pose)


    def callback(self, msg):
        idx = np.array([i for i, name in enumerate(msg.name) if self.obj_name in name])
        assert len(idx) == 7, "[pc2basePublisher] Object joint states not found"

        msg.position = np.array(msg.position)
        # Convert quaternion to rotation matrix
        transform = np.eye(4)
        transform[:3, 3] = msg.position[idx[:3]]
        transform[:3, :3] = R.from_quat(np.concatenate((msg.position[idx[4:7]], msg.position[idx[3]:idx[3]+1]))).as_matrix()
        self.base2obj = self.base_pose @ transform

        translation = self.base2obj[:3, 3]
        rotation = self.base2obj[:3, :3]
        quat = R.from_matrix(rotation).as_quat()
        self.br.sendTransform((translation[0], translation[1], translation[2]),
                              (quat[0], quat[1], quat[2], quat[3]),
                              rospy.Time.now(),
                              self.obj_name,
                              self.obj_base_frame)


    def pcd_trans(self, pcd):
        pcd = deepcopy(pcd).transform(self.base2obj)
        return pcd


def main():
    pub = pc2basePublisher()
    pub.execute()

if __name__ == "__main__":
    main()