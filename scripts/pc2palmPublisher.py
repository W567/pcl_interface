#!/usr/bin/env python
import tf
import rospy
import numpy as np
import open3d as o3d
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
        self.desk_frame = rospy.get_param('~desk_frame', 'desk_frame')

        # tf from base to palm
        self.palm_rot = np.eye(4)
        self.palm_rot[:3, :3] = R.from_euler('x', -90, degrees=True).as_matrix()

        self.br = tf.TransformBroadcaster()

        min_inf = np.finfo(np.float32).min
        max_inf = np.finfo(np.float32).max
        self.min_bound = np.array([min_inf, 0.0, min_inf])
        self.max_bound = np.array([max_inf, max_inf, max_inf])

    def pcd_trans(self, pcd):

        (desk_t, desk_r) = self.tf_listener.lookupTransform(self.base_frame, self.desk_frame,
                                                            rospy.Time(0))
        desk_z = desk_t[2]

        # pcd under base coordinate system
        center = -pcd.get_center()
        obj_min = pcd.get_min_bound()[2]
        if obj_min < desk_z:
            center[2] = -desk_z
        else:
            center[2] = -obj_min
        # tf from object_bot to base
        trans = np.eye(4)
        trans[:3, 3] = center
        # tf from object_bot to palm
        trans = self.palm_rot @ trans
        # pcd under object_bot/palm coordinate system
        pcd = deepcopy(pcd).transform(trans)

        # pcd has been transformed to palm coordinate system
        points_np = np.asarray(pcd.points)
        mask = np.all((self.min_bound <= points_np) & (points_np <= self.max_bound), axis=1)
        cropped_points_np = points_np[mask]
        pcd.points = o3d.utility.Vector3dVector(cropped_points_np)
        if pcd.has_colors:
            colors_np = np.asarray(pcd.colors)
            if colors_np.size != 0:
                cropped_colors_np = colors_np[mask]
                pcd.colors = o3d.utility.Vector3dVector(cropped_colors_np)
        if pcd.has_normals:
            normals_np = np.asarray(pcd.normals)
            if normals_np.size != 0:
                cropped_normals_np = normals_np[mask]
                pcd.normals = o3d.utility.Vector3dVector(cropped_normals_np)

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