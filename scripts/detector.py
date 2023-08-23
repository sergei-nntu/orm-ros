#!/usr/bin/env python
# license removed for brevity
import rospy
from fiducial_msgs.msg import FiducialTransformArray


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.transforms)

def listener():
    rospy.init_node('aruco_tf')
    rospy.Subscriber('fiducial_transforms', FiducialTransformArray, callback=callback)
    rospy.spin()


if __name__ == '__main__':
    listener()