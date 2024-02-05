#!/usr/bin/env python3
"""ROS2 Led Panne Node.

This module is not only to punlish the led pannel state to the 'led_panel_state'
topic, but it is also to create the service that will modifying the panel state.

"""
import sys
import rclpy
from rclpy.node import Node
from udemy_tutorials_interfaces.srv import SetLedStatus
from udemy_tutorials_interfaces.msg import LedPanelState


class LedPanelnode(Node):
    """ROS2 node that controls the led panel.

    This node is responsible for publishing the led panel state to the
    'led_panel_state' topic and for creating the service that will modify the
    panel state.

    """
    def __init__(self):
        super().__init__('led_panel_node')
        # All leds are turned off by default
        self.led_panel_status = [
            False,
            False,
            False,
        ]
        self.create_service(
            srv_type=SetLedStatus,
            srv_name='set_led',
            callback=self.callback_set_led_state,
        )
        self.led_panel_state_publisher = self.create_publisher(
            msg_type=LedPanelState,
            topic='led_panel_state',
            qos_profile=10,
        )
        # Publish the led panel state every second
        self.create_timer(1.0, self.publish_led_panel_state)

    def callback_set_led_state(self, request, response):
        """Callback for the 'set_led' service.

        This callback is called when a request is made to the 'set_led' servi-
        ce. The request contains the led number and the new state of the led.
        The response is a boolean that indicates if the request was successful.
        """
        led_number = request.led_number
        new_state = request.state
        if led_number < 1 or led_number > len(self.led_panel_status):
            response.success = False
            response.message = 'Invalid led number'
            return response
        self.led_panel_status[led_number - 1] = new_state
        response.success = True
        response.message = 'Led state updated'
        return response

    def publish_led_panel_state(self):
        """Publish the led panel state to the 'led_panel_state' topic."""
        panel_state = LedPanelState()
        panel_state.led1 = self.led_panel_status[0]
        panel_state.led2 = self.led_panel_status[1]
        panel_state.led3 = self.led_panel_status[2]
        self.led_panel_state_publisher.publish(panel_state)


def main(args=None):
    """Start Led Panel Node."""
    rclpy.init(args=args)
    # Create node and spin it!
    node = LedPanelnode()
    rclpy.spin(node)
    rclpy.shutdown()
    return 0
