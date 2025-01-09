#!/usr/bin/env python3
import rclpy
from rclpy.logging import get_logger
from robot_planner import RobotPlanner

from controller import Controller


class ArmController(Controller):
    def __init__(self, moveit_planner):
        super().__init__(moveit_planner, 'arm', 4)


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
