#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.logging import get_logger
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState # type: ignore


def move_robot():
    we_r2_arm = MoveItPy(node_name="moveit_py")
    we_r2_arm_arm = we_r2_arm.get_planning_component("arm")
    # we_r2_arm_gripper = we_r2_arm.get_planning_component("gripper")
    
    arm_state = RobotState(we_r2_arm.get_robot_model())
    # gripper_state = RobotState(we_r2_arm.get_robot_model())
    
    arm_state.set_joint_group_positions("arm", np.array([1.17, 0.0, 0.0, 0.14]))
    # gripper_state.set_joint_group_positions("gripper", np.array([-0.7, 0.7]))
    
    we_r2_arm_arm.set_start_state_to_current_state()
    # we_r2_arm_gripper.set_start_state_to_current_state()
    
    we_r2_arm_arm.set_goal_state(robot_state=arm_state)
    # we_r2_arm_gripper.set_goal_state(robot_state=gripper_state)
    arm_plan_result = we_r2_arm_arm.plan()
    # gripper_plan_result = we_r2_arm_gripper.plan()
    
    if arm_plan_result:
        get_logger("rclpy").info("Planner SUCCEED, moving the arm and the gripper")
        we_r2_arm.execute(arm_plan_result.trajectory, controllers=[])
        # we_r2_arm.execute(gripper_plan_result.trajectory, controllers=[])
    else:
        get_logger("rclpy").error("One or more planners failed!")


def main():
    rclpy.init()
    move_robot()
    rclpy.shutdown()


if __name__ == "__main__":
    main()