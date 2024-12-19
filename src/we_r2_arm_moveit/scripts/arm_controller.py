#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.logging import get_logger
from moveit.core.robot_state import RobotState # type: ignore
from robot_planner import RobotPlanner
from utils import joints_state_validation


class ArmController():
    _arm = None
    _arm_state = None
    _arm_plan_result = None
    _moveit_instance = None

    def __init__(self, moveit_planner):
        self._moveit_instance = moveit_planner
        self._arm = moveit_planner.get_planning_component("arm")
        self._arm_state = RobotState(moveit_planner.get_robot_model())

    def plan_and_execute(self):
        if self.plan():
            self.execute()

    def plan(self):
        self._arm_plan_result = self._arm.plan()
        return self._arm_plan_result

    def execute(self):
        if self._arm_plan_result:
            get_logger("arm_controller").info("Planner SUCCEED, moving the arm...")
            self._moveit_instance.execute(self._arm_plan_result.trajectory, controllers=[])
        else:
            get_logger("arm_controller").error("The arm planner failed!")

    def set_joints_state(self, joint_positions):
        joints_state_validation(joint_positions, 4)

        self._arm_state.set_joint_group_positions("arm", joint_positions)

        self._arm.set_start_state_to_current_state()
        self._arm.set_goal_state(robot_state=self._arm_state)


if __name__ == "__main__":
    rclpy.init()
    
    logger = get_logger("arm_controller")
    
    try:

        logger.info('Starting arm controller...')

        planner = RobotPlanner()
        moveit_planner = planner.get_moveit_planner()

        arm_controller = ArmController(moveit_planner)
        arm_controller.set_joints_state([1.17, 0.0, 0.0, 0.14])
        arm_controller.plan_and_execute()

    except KeyboardInterrupt:
        logger.info("Application interrupted by user.")
    except Exception as e:
        logger.error('Arm controller failed...', e)
    finally:
        rclpy.shutdown()
