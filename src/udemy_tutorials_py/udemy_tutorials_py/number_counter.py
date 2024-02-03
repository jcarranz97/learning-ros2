#!/usr/bin/env python3
"""Python node that takes a number and counts from 0 to that number.

    Publishers:
        number_count (std_msgs/Int64) - Publishes the current count.

    Subscribers:
        number (std_msgs/Int64) - Subscribes to get the value that needs to be
        added to the counter.
"""
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64


class NumberCount(Node):
    """Node that counts from 0 by adding a number to the count."""
    def __init__(self):
        super().__init__("number_counter")
        self.__count = 0
        self.subscriber = self.create_subscription(
            msg_type=Int64,
            topic="number",
            callback=self.callback_number_received,
            qos_profile=10,
        )
        self.publisher = self.create_publisher(
            msg_type=Int64,
            topic="number_count",
            qos_profile=10,
        )

    @property
    def logger(self):
        """Logger for the node."""
        return self.get_logger()

    def callback_number_received(self, msg):
        """Callback function that is called when a number is received."""
        self.__count += msg.data
        self.logger.info(
            f"Received number: {msg.data}. Current count: {self.__count}")
        self.publish_count()

    def publish_count(self):
        """Publishes the current count."""
        msg = Int64()
        msg.data = self.__count
        self.publisher.publish(msg)
        self.logger.info(f"Published count: {msg.data}")


def main(args=None):
    """Main function that creates the node and spins it."""
    rclpy.init(args=args)
    node = NumberCount()
    rclpy.spin(node)
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
