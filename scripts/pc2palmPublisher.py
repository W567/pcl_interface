#!/usr/bin/env python
import tf
import rospy
import numpy as np
from copy import deepcopy
from pcPubBase import pcPubBase
from scipy.spatial.transform import Rotation as R

import rospkg
rospack = rospkg.RosPack()

class pc2palmPublisher(pcPubBase):
    # It is supposed in this script that the robot base (x-o-y plane)
    # is always parallel to the ground/table
    def __init__(self):
        super().__init__()
        self.base_frame = rospy.get_param('~base_frame', 'base')

        # tf from base to palm
        self.palm_rot = np.eye(4)
        self.palm_rot[:3, :3] = R.from_euler('x', -90, degrees=True).as_matrix()

        self.br = tf.TransformBroadcaster()

    def pcd_process(self, pcd):
        # pcd under base coordinate system
        center = -pcd.get_center()
        center[2] = -pcd.get_min_bound()[2]
        # tf from object_bot to base
        trans = np.eye(4)
        trans[:3, 3] = center
        # tf from object_bot to palm
        trans = self.palm_rot @ trans
        # pcd under object_bot/palm coordinate system
        pcd = deepcopy(pcd).transform(trans)

        # tf from base to object_bot
        trans = np.linalg.inv(trans)
        translation = trans[:3, 3]
        rotation = trans[:3, :3]
        quat = R.from_matrix(rotation).as_quat()
        self.br.sendTransform((translation[0], translation[1], translation[2]),
                              (quat[0], quat[1], quat[2], quat[3]),
                              rospy.Time.now(),
                              "obj_bot",  # TODO add to rosparam
                              self.base_frame)

        return pcd


def main():
    pub = pc2palmPublisher()
    pub.execute()

if __name__ == "__main__":
    main()