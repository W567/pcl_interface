#!/usr/bin/env python
import tf
import rospy
from pcl_interface.srv import poseTF, poseTFResponse
from geometry_msgs.msg import TransformStamped

class tfPubServer():
    def __init__(self):
        rospy.init_node("palm_candidate_pub", anonymous=True)
        rate = rospy.get_param("~rate", 10)
        self.rate = rospy.Rate(rate)

        if rospy.has_param("~parent_frame"):
            self.parent_frame = rospy.get_param('~parent_frame')
        else:
            self.parent_frame = rospy.get_param('robot_base_frame')
            rospy.logwarn("[tfPubServer] No parent_frame given, using robot_base_frame")
        self.child_frame = rospy.get_param('~child_frame')

        self.server = rospy.Service('palm_tf_pub', poseTF, self.poseCB)
        self.br = tf.TransformBroadcaster()

        self.transform_stamped = None
        
    def poseCB(self, req):
        pose = req.pose
        self.transform_stamped = TransformStamped()
        self.transform_stamped.header.frame_id = self.parent_frame
        self.transform_stamped.child_frame_id = self.child_frame
        self.transform_stamped.transform.translation.x = pose.position.x
        self.transform_stamped.transform.translation.y = pose.position.y
        self.transform_stamped.transform.translation.z = pose.position.z
        self.transform_stamped.transform.rotation.x = pose.orientation.x
        self.transform_stamped.transform.rotation.y = pose.orientation.y
        self.transform_stamped.transform.rotation.z = pose.orientation.z
        self.transform_stamped.transform.rotation.w = pose.orientation.w
        return poseTFResponse(True)

    def execute(self):
        while not rospy.is_shutdown():
            if self.transform_stamped is not None:
                self.transform_stamped.header.stamp = rospy.Time.now()
                self.br.sendTransformMessage(self.transform_stamped)

            self.rate.sleep()


def main():
    publisher = tfPubServer()
    publisher.execute()

if __name__ == "__main__":
    main()