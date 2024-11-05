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

        # tf from base to palm
        self.world2palm = np.eye(4)
        self.world2palm[:3, :3] = (R.from_euler('y', -90, degrees=True).as_matrix() @
                                        R.from_euler('x', -90, degrees=True).as_matrix())

        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

        self.pc_frames_orig = deepcopy(self.pc_frames)
        palm_frame = rospy.get_param('palm_frame', 'ik_palm')

        self.pc_topics = [f"palm_{pc_topic}" for pc_topic in self.pc_topics]
        self.pc_frames = [palm_frame for _ in self.pc_topics]

        self.frame2world = np.eye(4)
        self.palm2obj_bot = np.eye(4)

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


    def common_process(self, pcd, frame):
        try:
            (_,rot) = self.listener.lookupTransform("world", frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn_once(f"[obj2palmPublisher] Failed to get tf from /world to {frame}")
            return None

        # pcd read from file is under world coordinate system by default
        # rotate the point cloud to met the actual orientation (object self frame)
        self.frame2world = np.eye(4)
        self.frame2world[:3, :3] = R.from_quat(rot).as_matrix()
        pcd_world = deepcopy(pcd).transform(self.frame2world)

        # pcd under world coordinate system
        center = -pcd_world.get_center()
        center[2] = -pcd_world.get_min_bound()[2]
        # tf from world to object_bot
        world2obj_bot = np.eye(4)
        world2obj_bot[:3, 3] = center
        # translate above z axis and rotate to palm frame
        self.palm2obj_bot = self.world2palm @ world2obj_bot
        # pcd under object_bot/palm coordinate system
        pcd_palm = deepcopy(pcd_world).transform(self.palm2obj_bot)

        bot_pos = np.linalg.inv(self.frame2world[:3, :3]) @ -center
        # rotate to have the orientation of the object bottom frame to be same with palm frame
        bot_quat = R.from_matrix(np.linalg.inv(self.world2palm[:3, :3] @ self.frame2world[:3, :3])).as_quat()
        self.br.sendTransform(bot_pos, bot_quat, rospy.Time.now(), f"{frame}_bot", frame)

        return pcd_palm


    def pcd_process(self, pcd, i):
        frame = self.pc_frames_orig[i]
        if self.with_aff:
            if i % 2 == 0:
                pcd_palm = self.common_process(pcd, frame)
            else:
                # Use same transformation between full cloud and partial cloud
                pcd_world = deepcopy(pcd).transform(self.frame2world)
                pcd_palm = deepcopy(pcd_world).transform(self.palm2obj_bot)
        else:
            pcd_palm = self.common_process(pcd, frame)

        return pcd_palm


def main():
    pub = obj2palmPublisher()
    pub.execute()

if __name__ == "__main__":
    main()