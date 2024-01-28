#!/usr/bin/env python3
"""Example of using ROS2 publisher to a topic with Python."""
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String


class RobotNewsStationNode(Node):
    """Node class where Robot News are published."""
    def __init__(self):
        # It is always recommended to call the node as the file name
        super().__init__("robot_news_station")
        # Create publisher
        self.__publisher = self.create_publisher(
            msg_type=String,
            topic="robot_news",
            qos_profile=10,  # Queue size. Normally 10 is enough.
        )
        # We are going to create a Timer to publish news
        self.create_timer(
            timer_period_sec=0.5,  # In seconds
            callback=self.publish_news,
        )
        # Show message that th node is alive
        self.logger.info("Robot News Station has been started.")

    @property
    def logger(self):
        """Get logger."""
        return self.get_logger()

    def publish_news(self):
        """Publish news to the topic."""
        # Create message
        msg = String()
        msg.data = "Hi, this is Robot News Station!"
        # Publish message
        self.__publisher.publish(msg)


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    node = RobotNewsStationNode()
    rclpy.spin(node)
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
