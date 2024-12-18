#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.logging import get_logger
from moveit.core.robot_state import RobotState # type: ignore
from robot_planner import RobotPlanner
from utils import joints_state_validation


class GripperController():
    _gripper = None
    _gripper_state = None
    _gripper_plan_result = None
    _moveit_instance = None

    def __init__(self, moveit_planner):
        self._moveit_instance = moveit_planner
        self._gripper = moveit_planner.get_planning_component("gripper")
        self._gripper_state = RobotState(moveit_planner.get_robot_model())

    def plan_and_execute(self):
        if self.plan():
            self.execute()

    def plan(self):
        self._gripper_plan_result = self._gripper.plan()
        return self._gripper_plan_result

    def execute(self):
        if self._gripper_plan_result:
            get_logger("gripper_controller").info("Planner SUCCEED, moving the gripper...")
            self._moveit_instance.execute(self._gripper_plan_result.trajectory, controllers=[])
        else:
            get_logger("gripper_controller").error("The gripper planner failed!")

    def set_joints_state(self, joint_positions):
        joints_state_validation(joint_positions, 2)

        self._gripper_state.set_joint_group_positions("gripper", np.array(joint_positions))

        self._gripper.set_start_state_to_current_state()
        self._gripper.set_goal_state(robot_state=self._gripper_state)


if __name__ == "__main__":
    rclpy.init()
    
    logger = get_logger("gripper_controller")
    
    try:

        logger.info('Starting gripper controller...')

        planner = RobotPlanner()
        moveit_planner = planner.get_moveit_planner()

        arm_controller = GripperController(moveit_planner)
        arm_controller.set_joints_state([-0.7, 0.7])
        arm_controller.plan_and_execute()

    except KeyboardInterrupt:
        logger.info("Application interrupted by user.")
    except Exception as e:
        logger.error('Arm controller failed...', e)
    finally:
        rclpy.shutdown()
