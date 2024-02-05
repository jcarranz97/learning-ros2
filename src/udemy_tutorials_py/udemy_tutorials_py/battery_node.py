#!/usr/bin/env python3
"""Battery node used for activity-004 of ros2-for-begginers Udemy course"""
import sys
from functools import partial
from time import sleep
import rclpy
from rclpy.node import Node
from udemy_tutorials_interfaces.srv import SetLedStatus


class BatteryNode(Node):
    """Battery node"""
    def __init__(self):
        super().__init__("battery")
        self.led_status_client = self.create_client(SetLedStatus, "set_led")
        while not self.led_status_client.wait_for_service(timeout_sec=1.0):
            self.logger.warn("'set_led' service is not available ...")
        self.logger.info("client service found!!")
        self.__baterry_level = 100

        # Set some battery levels
        while True:
            self.logger.info("Setting battery level to 100%")
            self.battery_level = 100
            sleep(5)
            self.logger.info("Setting battery level to 50%")
            self.battery_level = 50
            sleep(5)
            self.logger.info("Setting battery level to 10%")
            self.battery_level = 10
            sleep(5)

    @property
    def battery_level(self):
        """Getters for battery level"""
        return self.__baterry_level

    @battery_level.setter
    def battery_level(self, value):
        """Setters for battery level"""
        self.__baterry_level = value
        if self.__baterry_level < 15:
            self.set_led_status(1, True)
        else:
            self.set_led_status(1, False)

    @property
    def logger(self):
        """Get node logger"""
        return self.get_logger()

    def set_led_status(self, led_number, state):
        """Call service to set led status"""
        request = SetLedStatus.Request(led_number=led_number, state=state)
        led_state = "ON" if state else "OFF"
        self.logger.info(f"Setting led {led_number} to '{led_state}'")
        future = self.led_status_client.call_async(request)
        future.add_done_callback(
            partial(self.callback_set_led_status_done, request=request)
        )

    def callback_set_led_status_done(self, future, request):
        """Callback when led status is done"""
        response = future.result()
        if not response.success:
            self.logger.error(
                f"Something went wrong changing the led {request.led_number} status")
            self.logger.error(response.message)
            return
        self.logger.info("Led status change done!!!")


def main(args=None):
    """Start Battery Node """
    rclpy.init(args=args)
    # Create node and spin it!
    node = BatteryNode()
    rclpy.spin(node)
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
