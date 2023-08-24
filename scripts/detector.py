#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from fiducial_msgs.msg import FiducialTransformArray
from moveit_commander import MoveGroupCommander

def detect_tf(data):
    for m in data.transforms:
        id = m.fiducial_id
        trans = m.transform.translation
        rot = m.transform.rotation
        rospy.loginfo(rospy.get_caller_id() + "\n%s", trans)
        print(trans.x)

def detector():
    rospy.init_node('aruco_tf')
    rospy.Subscriber('fiducial_transforms', FiducialTransformArray, callback=detect_tf)
    rospy.spin()

def main():
    rospy.init_node('moveit_controller')

    moveit_commander.roscpp_initialize(sys.argv)
    group = MoveGroupCommander("arm")
    
    group.set_start_state_to_current_state()
    group.set_position_target([0.22, 0.1, 0.3])

    plan_success, plan, planning_time, error_code = group.plan()

    group.execute(plan)

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        # main()
        detector()
    except rospy.ROSInterruptException:
        print("Something went wrong!")