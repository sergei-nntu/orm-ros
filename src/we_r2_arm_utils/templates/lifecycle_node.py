import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from std_msgs.msg import String

class MyLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__('my_lifecycle_node')

        # Declare parameters (optional)
        self.declare_parameter('example_param', 'default_value')

        # Create a lifecycle publisher
        self.lifecycle_publisher = self.create_lifecycle_publisher(
            msg_type=String,  # Replace with the actual message type
            topic_name='lifecycle_topic',
            qos_profile=10
        )

        self.get_logger().info("Lifecycle node created.")

    # Callback for the "configure" transition
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f'Configuring... Current state: {state.label}')
        # Initialize resources, parameters, etc.
        self.get_logger().info('Configuration complete.')
        return TransitionCallbackReturn.SUCCESS

    # Callback for the "activate" transition
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f'Activating... Current state: {state.label}')
        # Start publishing or any main operational logic
        self.lifecycle_publisher.on_activate()  # Enable the publisher
        self.get_logger().info('Activation complete.')
        return TransitionCallbackReturn.SUCCESS

    # Callback for the "deactivate" transition
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f'Deactivating... Current state: {state.label}')
        # Stop publishing or pause operations
        self.lifecycle_publisher.on_deactivate()  # Disable the publisher
        self.get_logger().info('Deactivation complete.')
        return TransitionCallbackReturn.SUCCESS

    # Callback for the "cleanup" transition
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f'Cleaning up... Current state: {state.label}')
        # Free resources, reset parameters, etc.
        self.get_logger().info('Cleanup complete.')
        return TransitionCallbackReturn.SUCCESS

    # Callback for the "shutdown" transition
    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f'Shutting down... Current state: {state.label}')
        # Cleanup before exiting
        self.get_logger().info('Shutdown complete.')
        return TransitionCallbackReturn.SUCCESS


def main(args=None):
    rclpy.init(args=args)
    lifecycle_node = MyLifecycleNode()

    # Use an executor to spin the node
    executor = rclpy.executors.MultiThreadedExecutor()
    try:
        executor.add_node(lifecycle_node)
        lifecycle_node.get_logger().info('Lifecycle node is spinning...')
        executor.spin()
    finally:
        lifecycle_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
