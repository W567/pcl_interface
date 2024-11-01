#!/usr/bin/env python
import os
import tf
import rospy
import numpy as np
from copy import deepcopy
from pcPubBase import pcPubBase
from scipy.spatial.transform import Rotation as R

import rospkg
rospack = rospkg.RosPack()

class obj2palmPublisher(pcPubBase):
    # It is supposed in this script that the robot base (x-o-y plane)
    # is always parallel to the ground/table
    def __init__(self):
        super().__init__()
        self.robot_base_frame = rospy.get_param('robot_base_frame')

        # tf from base to palm
        self.palm2robot_base = np.eye(4)
        self.palm2robot_base[:3, :3] = (R.from_euler('y', -90, degrees=True).as_matrix() @
                                        R.from_euler('x', -90, degrees=True).as_matrix())

        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

        self.pc_frames_orig = deepcopy(self.pc_frames)
        palm_frame = rospy.get_param('palm_frame', 'ik_palm')

        self.pc_topics = [f"palm_{pc_topic}" for pc_topic in self.pc_topics]
        self.pc_frames = [palm_frame for _ in self.pc_topics]

    def init_file_topic_frame(self):
        self.with_aff = rospy.get_param("with_aff", False)
        ext = rospy.get_param("aff_ext", "_aff0")
        while True:
            if rospy.has_param("free_body_names"):
                pc_filenames = rospy.get_param("free_body_names")
                self.pc_filenames = []
                self.pc_frames = []
                for pc_filename in pc_filenames:
                    self.pc_filenames.append(pc_filename)
                    self.pc_frames.append(pc_filename)
                    if self.with_aff:
                        self.pc_filenames.append(f"{pc_filename}{ext}")
                        self.pc_frames.append(pc_filename)

                self.pc_topics = self.pc_filenames.copy()
                package_path = os.path.join(rospack.get_path('obj_models'), 'obj')
                self.pc_file_paths = [f"{package_path}/{filename}.pcd" for filename in self.pc_filenames]
                break
            else:
                rospy.sleep(0.016)

    def pcd_process(self, pcd, i):
        frame = self.pc_frames_orig[i]
        try:
            (pos,rot) = self.listener.lookupTransform(self.robot_base_frame, frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn(f"[obj2palmPublisher] Failed to get transform from {frame} to {self.robot_base_frame}")
            return None

        frame2robot_base = np.eye(4)
        frame2robot_base[:3, 3] = pos
        frame2robot_base[:3, :3] = R.from_quat(rot).as_matrix()
        pcd_robot_base = deepcopy(pcd).transform(frame2robot_base)

        # pcd under base coordinate system
        center = -pcd_robot_base.get_center()
        center[2] = -pcd_robot_base.get_min_bound()[2]
        # tf from object_bot to base
        robot_base2obj_base = np.eye(4)
        robot_base2obj_base[:3, 3] = center
        # tf from object_bot to palm
        palm2obj_base = self.palm2robot_base @ robot_base2obj_base
        # pcd under object_bot/palm coordinate system
        pcd_palm = deepcopy(pcd_robot_base).transform(palm2obj_base)

        if self.with_aff and i % 2 == 1:
            pass
        else:
            # bot frame orientation is same to that of the palm frame
            frame2obj_base = np.linalg.inv(palm2obj_base @ frame2robot_base)
            pos = frame2obj_base[:3, 3]
            quat = R.from_matrix(frame2obj_base[:3, :3]).as_quat()
            self.br.sendTransform((pos[0], pos[1], pos[2]),
                                  (quat[0], quat[1], quat[2], quat[3]),
                                  rospy.Time.now(),
                                  f"{frame}_bot",
                                  frame)

        return pcd_palm


def main():
    pub = obj2palmPublisher()
    pub.execute()

if __name__ == "__main__":
    main()