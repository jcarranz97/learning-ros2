#!/usr/bin/env python3
"""Create a ROS2 node that publisheds a number to a topic.

    Publishers:
        number (std_msgs.msg.Int64): Publishes a number to the topic "number".
"""
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64


class NumberPublisher(Node):
    """ROS2 node that publishes a number to a topic."""
    def __init__(self):
        """Initialize the node and create a publisher."""
        super().__init__("number_publisher")
        self.__publisher = self.create_publisher(
            msg_type=Int64,
            topic="number",
            qos_profile=10,  # Queue size
        )
        # Publish a number to the topic every 1 second
        self.create_timer(
            timer_period_sec=1,
            callback=self.publish_number,
        )
        # Log a message to indicate that the node has been initialized
        self.logger.info("Number publisher node has been initialized.")

    @property
    def logger(self):
        """rclpy.logging.Logger: Node's logger."""
        return self.get_logger()

    def publish_number(self):
        """Publish a number to the topic."""
        number = 1
        # Publish the number to the topic. In this case, the number is 1, which
        # corresponds to the data field of the Int64 message.
        self.__publisher.publish(Int64(data=number))


def main():
    """Create and run the number publisher node."""
    rclpy.init(args=sys.argv)
    node = NumberPublisher()
    rclpy.spin(node)
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
