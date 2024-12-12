#! /usr/bin/env python3

# Adapted from the simple commander demo examples on 
# https://github.com/ros-planning/navigation2/blob/galactic/nav2_simple_commander/nav2_simple_commander/demo_security.py


from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from stretch_nav2.robot_navigator import BasicNavigator, TaskResult
from dabner_interfaces.srv import RequestNumber
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory
import os

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
import sys
import pandas as pd

# This node is the main navigation node for JEC navigation
# It hosts two services for the logic node to check if navigation is free and to request a waypoint

class JecNav(Node):
    def __init__(self):
        
        super().__init__('jec_nav')
        node_name = self.get_name()
        self.get_logger().info("{0} started".format(node_name))
        self.waypoint_srv = self.create_service(RequestNumber, 'request_waypoint', self.request_waypoint_callback)
        self._srv = self.create_service(Trigger, 'is_free', self.request_free_callback)

	# change to a different waypoint file if needed
        filename = '/home/hello-robot/dabner_ws/src/dabner_nav/maps/simulated_waypoints.csv'
        self.df = pd.read_csv(filename) 
        self.navigator = BasicNavigator()
        
        self.navigator.waitUntilNav2Active()
        # send initial pose of the origin, all other navigation logic is in service callbacks
        self.send_initial_pose_origin()
        self.get_logger().info("Nav2 active and recognized jec_nav!")
        
        
    def request_free_callback(self,request,response):
        response.success = self.navigator.isTaskComplete()
        return response
    
    # unused for now
    def request_pose_callback(self, request, response):
        response.result = self.navigator.isTaskComplete()
        if response.result:
            gopose = PoseStamped()
            gopose.header.frame_id = 'map'
            gopose.header.stamp = self.navigator.get_clock().now().to_msg()
            gopose.pose.position.x = request.position.x
            gopose.pose.position.y = request.position.y
            gopose.pose.orientation.z = request.orientation.z
            self.navigator.goToPose(gopose)
        return response
        
    def make_pose(self,x,y,z):
        gopose = PoseStamped()
        gopose.header.frame_id = 'map'
        gopose.header.stamp = self.navigator.get_clock().now().to_msg()
        gopose.pose.position.x = float(x)
        gopose.pose.position.y = float(y)
        gopose.pose.orientation.z = float(z)
        return gopose
    
    def waypoint_to_pose(self,waypoint):
        x = self.df[self.df['Number']==waypoint]['X']
        y = self.df[self.df['Number']==waypoint]['Y']
        name = self.df[self.df['Number']==waypoint]['Name']
        self.get_logger().info("Going to {0}".format(name))
        return self.make_pose(x,y,0)
                
    def request_waypoint_callback(self, request, response):
        response.result = self.navigator.isTaskComplete()
        if response.result:
            self.navigator.goToPose(self.waypoint_to_pose(request.number))
        return response
            
    def send_initial_pose_origin(self):
        initial_pose = self.make_pose(0,0,0)
        self.navigator.setInitialPose(initial_pose)
        self.get_logger().info("Sending initial pose: {0}".format(initial_pose))


def main(args=None):
    rclpy.init()
    node = JecNav()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('interrupt received, so shutting down')

        
    if node is not None:
        node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
