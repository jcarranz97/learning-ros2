#!/usr/bin/env python3
"""Create a ROS2 node that publishes to a topic using the new custom message
   for the udemy_tutorials_interfaces package.
"""
import sys
import rclpy
from rclpy.node import Node
from udemy_tutorials_interfaces.msg import HardwareStatus


class HardwareStatusPublisher(Node):
    """Create a node that publishes to the hw_status topic using the
       custom HardwareStatus message from the udemy_tutorials_interfaces
    """
    def __init__(self):
        """Intialize the node and create a publisher to the topic."""
        super().__init__("hw_status_publisher")
        self.__publisher = self.create_publisher(
            msg_type=HardwareStatus,
            topic="hw_status",
            qos_profile=10,
        )
        # Create a timer to publish the message every 1 second
        self.create_timer(
            timer_period_sec=1.0,
            callback=self.publish_hw_status,
        )
        # Log a message to indicate the node is running
        self.logger.info("Hardware Status Publisher has been started.")

    @property
    def logger(self):
        """Return the logger for the node."""
        return self.get_logger()

    def publish_hw_status(self):
        """Publish the hardware status message to the topic."""
        hw_status_msg = HardwareStatus()
        hw_status_msg.temperature = 45
        hw_status_msg.are_motors_ready = True
        hw_status_msg.debug_message = "All systems are go!"
        # Publish the message to the topic
        self.__publisher.publish(hw_status_msg)


def main(args=None):
    """Main entry point for the node."""
    rclpy.init(args=args)
    hw_status_publisher = HardwareStatusPublisher()
    rclpy.spin(hw_status_publisher)
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
