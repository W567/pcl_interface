#!/usr/bin/env python
import tf
import rospy
import numpy as np
from copy import deepcopy
from pcPubBase import pcPubBase
from scipy.spatial.transform import Rotation as R

import rospkg
rospack = rospkg.RosPack()

class pcTransPublisher(pcPubBase):

    def __init__(self):
        super().__init__()
        self.child_frame = rospy.get_param("~child_frame", "object")
        self.parent_frame = rospy.get_param("~parent_frame", "base")
        self.tf_listener.waitForTransform(self.parent_frame, self.child_frame, rospy.Time(), rospy.Duration(5.0))

        self.palm_rot = np.eye(4)
        self.palm_rot[:3, :3] = R.from_euler('x', -90, degrees=True).as_matrix()

        self.br = tf.TransformBroadcaster()

    def get_tf_mat(self):
        (trans,rot) = self.tf_listener.lookupTransform(self.parent_frame, self.child_frame, rospy.Time(0))
        tf_mat = np.eye(4)
        tf_mat[:3, :3] = tf.transformations.quaternion_matrix(rot)[:3, :3]
        tf_mat[:3, 3] = trans
        return np.linalg.inv(tf_mat)

    def pcd_trans(self, pcd):
        tf_mat = self.get_tf_mat()
        pcd = deepcopy(pcd).transform(tf_mat)
        center = -pcd.get_center()
        center[2] = -pcd.get_min_bound()[2]
        trans = np.eye(4)
        trans[:3, 3] = center
        trans = self.palm_rot @ trans
        pcd = pcd.transform(trans)

        trans = np.linalg.inv(trans @ tf_mat)
        translation = trans[:3, 3]
        rotation = trans[:3, :3]
        quat = R.from_matrix(rotation).as_quat()
        self.br.sendTransform((translation[0], translation[1], translation[2]),
                              (quat[0], quat[1], quat[2], quat[3]),
                              rospy.Time.now(),
                              "obj_bot",
                              self.parent_frame)

        return pcd

def main():
    pub = pcTransPublisher()
    pub.execute()

if __name__ == "__main__":
    main()