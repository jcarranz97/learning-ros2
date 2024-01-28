#!/usr/bin/env python3
""" Hello World ROS2 Node """
import sys
import rclpy
from rclpy.node import Node


class MyNode(Node):
    """ MyNode class """
    def __init__(self):
        super().__init__("py_test")
        self.get_logger().info("Hello World from MyNode!")


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
