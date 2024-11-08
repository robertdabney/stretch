#! /usr/bin/env python3

# Adapted from the simple commander demo examples on 
# https://github.com/ros-planning/navigation2/blob/galactic/nav2_simple_commander/nav2_simple_commander/demo_security.py


from dabner_interfaces.srv import RequestNumber
from std_srvs.srv import Trigger

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
import json
import sys

class Logic(Node):
    def __init__(self):
        
        super().__init__('logic')
        node_name = self.get_name()
        self.get_logger().info("{0} started".format(node_name))
        # set up jec_nav waypoint, busy and speech server clients
        self.main()
    def main(self):
        done = 0
        # while(1):
        #     if jev_nav is busy:
        #         continue
        #     if done == 0:
        #         speech.say_arrived(location)
        #         done = 1
        #     text = get_speech()
        #     location = get_location_from_text(text)
        #     request_waypoint(location)
        #     done = 0
            
            

def main(args=None):
    rclpy.init()
    node = Logic()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('interrupt received, so shutting down')

        
    if node is not None:
        node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()