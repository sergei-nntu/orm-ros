import rclpy
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from example_interfaces.action import Fibonacci
from rclpy.action import ActionServer

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self.action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = Fibonacci.Feedback()
        result = Fibonacci.Result()
        sequence = [0, 1]
        for i in range(1, goal_handle.request.order):
            sequence.append(sequence[i] + sequence[i - 1])
            feedback_msg.partial_sequence = sequence
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {sequence}')
        result.sequence = sequence
        goal_handle.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)
    node = FibonacciActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
