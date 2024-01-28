#!/usr/bin/env python3
"""Example of ROS2 subscriber.

This scripts creates an 'smartphone' node which subscribes to the
'robot_news_station' topic and prints the messages to the console.

"""
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String


class SmartphoneNode(Node):
    """Smartphone node class."""
    def __init__(self):
        super().__init__('smartphone')
        # Create subscriber
        self.subscriber = self.create_subscription(
            msg_type=String,
            topic='robot_news',
            callback=self.callback_robot_news,
            qos_profile=10,  # Queue size (Same as robot_news_station)
        )
        # Show message that node is running
        self.logger.info('Smartphone node been started.')

    @property
    def logger(self):
        """Return logger."""
        return self.get_logger()

    def callback_robot_news(self, msg):
        """Handle received messages."""
        self.logger.info(f"Message received: {msg.data}")


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    smartphone_node = SmartphoneNode()
    rclpy.spin(smartphone_node)
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
