#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.logging import get_logger
from we_r2_arm_interfaces.action import MoveJointGroup


class MoveJointGroupServer(Node):
    def __init__(self):
        super().__init__('move_joint_group_server')
        self._action_server = ActionServer(
            self,
            MoveJointGroup,
            'move_joint_group',
            self.execute_callback
        )

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        feedback_msg = MoveJointGroup.Feedback()

        result = MoveJointGroup.Result()

        joints_position = goal_handle.request.joints_position
        joint_position_group = goal_handle.request.joint_position_group

        self.get_logger().info(f'Feedback: {joints_position} {joint_position_group}')

        feedback_msg.current_position = [0, 0, 0, 0]
        feedback_msg.finished = True
        feedback_msg.in_progress = False
        goal_handle.publish_feedback(feedback_msg)

        result.success = True
        result.message = "Hello, world!"
        goal_handle.succeed()

        return result


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
    
