#!/usr/bin/env python3
"""Example to communicate with the add_two_ints service without OOP."""
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


def main(args=None):
    """Client node without OOP."""
    rclpy.init(args=args)
    # Create a node
    node = Node("add_two_ints_client_no_oop")
    client = node.create_client(AddTwoInts, "add_two_ints")
    node.get_logger().info("Client node has been created.")
    # Wait for the service to be available
    node.get_logger().info("Waiting for the service to be available...")
    while not client.wait_for_service(timeout_sec=1.0):  # Check every second
        node.get_logger().warn("Service not available, waiting again...")
    node.get_logger().info("Service is available.")
    # Create a request
    request = AddTwoInts.Request()
    request.a = 3
    request.b = 4
    # Call the service
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        node.get_logger().info(f"Result: {future.result().sum}")
    else:
        node.get_logger().error("Service call failed.")
    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
