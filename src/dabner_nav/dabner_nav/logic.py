#! /usr/bin/env python3

from dabner_interfaces.srv import RequestNumber
from std_srvs.srv import Trigger

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
import sys

# This Node is the general logic driver for JEC navigation
# if the navigator is not busy, request speech input, request waypoint if speech is recieved

class Logic(Node):
    def __init__(self):
        
        super().__init__('logic')
        node_name = self.get_name()
        self.get_logger().info("{0} started".format(node_name))
        # set up jec_nav waypoint, free and speech clients
        self.free_cli = self.create_client(Trigger, 'is_free')
        while not self.free_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('free service not available, waiting again...')
        self.waypoint_cli = self.create_client(RequestNumber,'request_waypoint')
        while not self.waypoint_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waypoint service not available, waiting again...')
        self.speech_cli = self.create_client(Trigger, 'collect_speech')
        while not self.speech_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('speech service not available, waiting again...')
        self.main()

    def call_speech(self):
        self.future = self.speech_cli.call_async(self.speech_req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    def is_free(self):
        self.future = self.free_cli.call_async(self.nav_req)
        rclpy.spin_until_future_complete(self,self.future)
        return self.future.result()
    def request_waypoint(self,waypoint):
        self.waypoint_req.number = waypoint
        self.future = self.waypoint_cli.call_async(self.waypoint_req)
        rclpy.spin_until_future_complete(self,self.future)
        return self.future.result()

    def main(self):
        self.nav_req = Trigger.Request()
        self.speech_req = Trigger.Request()
        self.waypoint_req = RequestNumber.Request()
        while True:
            free_response = self.is_free()
            if not free_response.success:
                continue
            speech_result = self.call_speech()
            self.get_logger().info("{0}{1} recieved".format(speech_result.success,speech_result.message))
            if not speech_result.success:
                continue
            if not speech_result.message.isdigit():
                continue
            waypoint_number = int(speech_result.message)
            self.request_waypoint(waypoint_number)
            
            

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
