#!/usr/bin/env python3
"""Example to communicate with the add_two_ints service without OOP."""
import sys
from functools import partial
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsClientNode(Node):
    """Client node with OOP."""
    def __init__(self):
        """Create an instance of the client node."""
        super().__init__("add_two_ints_client_oop")
        self.call_service(AddTwoInts.Request(a=3, b=4))

    def call_service(self, request):
        """Call the service."""
        client = self.create_client(AddTwoInts, "add_two_ints")
        self.get_logger().info("Client node has been created.")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Service not available, waiting again...")
        self.get_logger().info("Service is available.")
        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_service, request=request)
        )

    @property
    def logger(self):
        """Return the logger."""
        return self.get_logger()

    def callback_call_service(self, future, request):
        """Callback function for the service call."""
        response = future.result()
        self.logger.info(f"Request: {request.a} + {request.b}")
        self.logger.info(f"Result: {response.sum}")

def main(args=None):
    """Client node without OOP."""
    rclpy.init(args=args)
    # Create a node
    node = AddTwoIntsClientNode()
    rclpy.spin(node)
    # Destroy the node explicitly
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
