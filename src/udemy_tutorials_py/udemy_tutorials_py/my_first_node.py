#!/usr/bin/env python3
""" Hello World ROS2 Node """
import sys
import rclpy
from rclpy.node import Node


class MyNode(Node):
    """ MyNode class """
    def __init__(self):
        super().__init__("py_test")
        self.logger = self.get_logger()
        self.logger.info("Hello World from MyNode!")
        self.create_timer(
            0.5,  # Run at 2 Hz (0.5 seconds)
            self.timer_callback,  # Call timer_callback function
        )
        self.counter = 0

    def timer_callback(self):
        """ Timer callback function """
        self.counter += 1
        self.logger.info(f"Hello World from timer callback! ({self.counter})")


def main(args=None):
    """ Main function """
    rclpy.init(args=args)  # This is the first thing to do in all ROS2 nodes
    node = MyNode()  # Create MyNode instance

    try:
        rclpy.spin(node)  # Keep the node running until it is stopped
    except KeyboardInterrupt:
        node.get_logger().warning("Keyboard Interrupt (SIGINT)")
    finally:
        # Close ROS2 node communication
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
