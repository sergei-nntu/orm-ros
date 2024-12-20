#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.logging import get_logger
from we_r2_arm_interfaces.action import MoveJointGroup
from arm_controller import ArmController
from gripper_controller import GripperController
from robot_planner import RobotPlanner


class MoveJointGroupServer(Node):
    _planner = None
    _controller = None
    _moveit_planner = None

    def __init__(self):
        super().__init__('move_joint_group_server')
        self.get_logger().info('Initializing MoveJointGroupServer...')

        self._planner = RobotPlanner()
        self._moveit_planner = self._planner.get_moveit_planner()
        self._action_server = ActionServer(
            self,
            MoveJointGroup,
            'move_joint_group',
            self.execute_callback
        )

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        result = MoveJointGroup.Result()
        # feedback_msg = MoveJointGroup.Feedback()

        joints_position = goal_handle.request.joints_position
        joint_position_group = goal_handle.request.joint_position_group

        self.set_controller_based_on_group(joint_position_group)

        if not self._controller:
            result.success = False
            result.message = f"Error, the {joint_position_group} wasn't recognized"
            goal_handle.abort()

            return result

        self._controller.set_joints_state(joints_position)
        self._controller.plan_and_execute()

        result.success = True
        result.message = "The goal was executed successfully!"
        goal_handle.succeed()

        return result
    
    def set_controller_based_on_group(self, joint_position_group):
        if joint_position_group == 'arm':
            self._controller = ArmController(self._moveit_planner)
        elif joint_position_group == 'gripper':
            self._controller = GripperController(self._moveit_planner)
        else:
            self._controller = None


if __name__ == '__main__':
    rclpy.init()
    
    logger = get_logger("move_joint_group_server")

    node = None

    try:
        logger.info('Starting move joint group server...')

        node = MoveJointGroupServer()

        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info("Application interrupted by user.")
    except Exception as e:
        logger.error('Arm controller failed...', e)
    finally:
        if node is not None:
            node.destroy_node()

        rclpy.shutdown()
    
