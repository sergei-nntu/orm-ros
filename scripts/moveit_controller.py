#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import copy
from moveit_msgs.msg import DisplayTrajectory, PlanningScene
import geometry_msgs.msg
from fiducial_msgs.msg import FiducialTransformArray
from moveit_commander import MoveGroupCommander

moveit_commander.roscpp_initialize(sys.argv)
group = MoveGroupCommander("arm")

def detect_tf(data):
    for m in data.transforms:
        id = m.fiducial_id
        trans = m.transform.translation
        rot = m.transform.rotation
        rospy.loginfo(rospy.get_caller_id() + "\n%s", trans)

        pose_goal = geometry_msgs.msg.Pose()

        pose_goal.orientation.w = 1.0
        pose_goal.position.x = round(trans.x / 3, 6)
        pose_goal.position.y = round(trans.z / 3, 6)
        pose_goal.position.z = round(trans.y / 3, 6)

        print(pose_goal.position)

        group.set_pose_target(pose_goal)

        success = group.go()
        print("Does it work?", success)

        group.clear_pose_targets()


def main():
    rospy.init_node('moveit_controller')
    rospy.Subscriber('fiducial_transforms', FiducialTransformArray, callback=detect_tf, queue_size=1)
    rospy.spin() 

if __name__ == '__main__':
    try:
        main()
        moveit_commander.roscpp_shutdown()
    except rospy.ROSInterruptException:
        print("Something went wrong!")


