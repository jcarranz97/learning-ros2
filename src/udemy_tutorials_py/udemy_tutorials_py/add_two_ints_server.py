#!/usr/bin/env python3
"""ROS2 service server node example."""
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsServerNode(Node):
    """ROS2 service server node."""
    def __init__(self):
        """Class constructor to set up the node."""
        super().__init__('add_two_ints_server')
        self.create_service(
            srv_type=AddTwoInts,  # service type
            srv_name='add_two_ints',  # service name
            callback=self.callback_add_two_ints  # callback function
        )
        self.logger.info('Add Two Ints Server has been started.')

    @property
    def logger(self):
        """Logger function to log messages."""
        return self.get_logger()

    def callback_add_two_ints(self, request, response):
        """Callback function to handle the service request."""
        response.sum = request.a + request.b
        self.logger.info(f'Request: a={request.a}, b={request.b}')
        self.logger.info(f'Response: {response.sum}')
        return response


def main(args=None):
    """Main function to set up the node."""
    rclpy.init(args=args)
    add_two_ints_server = AddTwoIntsServerNode()
    rclpy.spin(add_two_ints_server)
    rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
