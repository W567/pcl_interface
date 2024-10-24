#!/usr/bin/env python
import rospy
from pcPubBase import pcPubBase

import rospkg
rospack = rospkg.RosPack()

class wsPcPublisher(pcPubBase):
    def __init__(self):
        super().__init__()

    def init_file_topic_frame(self):
        if rospy.has_param("~pc_filenames"):
            self.pc_filenames = rospy.get_param("~pc_filenames")
            self.pc_file_paths = self.pc_filenames.copy()
            package_path = rospack.get_path('ranspreg')
            self.pc_file_paths = [f"{package_path}/{filename}" for filename in self.pc_file_paths]
        else:
            rospy.logerr("[pcPubBase] No pc_filenames")

        pc_topic_prefix = rospy.get_param("~pc_topic_prefix", "ws_pc_")
        self.pc_topics = [pc_topic_prefix + str(i) for i in range(len(self.pc_filenames))]

        palm_frame = rospy.get_param('palm_frame', 'ik_palm')
        self.pc_frames = [palm_frame for _ in self.pc_topics]

def main():
    pub = wsPcPublisher()
    pub.execute()

if __name__ == "__main__":
    main()