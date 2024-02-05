#!/usr/bin/env python3
"""ROS2 service server node that computer rectangle area"""
import sys
import rclpy
from rclpy.node import Node
from udemy_tutorials_interfaces.srv import ComputeRectangleArea


class ComputeRectangleAreaServerNode(Node):
    """ROS2 service server node which computes RectangleArea"""
    def __init__(self):
        super().__init__("compute_rectangle_area_server")
        self.create_service(
            srv_type=ComputeRectangleArea,
            srv_name="compute_rectangle_area",
            callback=self.callback_compute_rectangle_area,
        )
        self.logger.info("ComputeRectangleAreaServerNode is ready!!!")

    @property
    def logger(self):
        """Return ROS2 logger"""
        return self.get_logger()

    def callback_compute_rectangle_area(self, request, response):
        """Callback function to handle the service request."""
        self.logger.info("Calculating RectangleArea:")
        self.logger.info(f"  length: {request.length}")
        self.logger.info(f"   width: {request.width}")
        response.area = request.length * request.width
        self.logger.info(f"    Area: {response.area}")
        return response


def main(args=None):
    """Main method to setup and spin the ROS2 node"""
    rclpy.init(args=args)
    compute_rectangle_area_server = ComputeRectangleAreaServerNode()
    rclpy.spin(compute_rectangle_area_server)
    rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
