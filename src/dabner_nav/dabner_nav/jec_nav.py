#! /usr/bin/env python3

# Adapted from the simple commander demo examples on 
# https://github.com/ros-planning/navigation2/blob/galactic/nav2_simple_commander/nav2_simple_commander/demo_security.py


from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from stretch_nav2.robot_navigator import BasicNavigator, TaskResult
from dabner_interfaces.srv import RequestNumber
from std_srvs.srv import Trigger

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
import json
import sys

class JecNav(Node):
    def __init__(self):
        
        super().__init__('jec_nav')
        node_name = self.get_name()
        self.get_logger().info("{0} started".format(node_name))

        self.waypoint_srv = self.create_service(RequestNumber, 'request_waypoint', self.request_waypoint_callback)
        self.busy_srv = self.create_service(Trigger, 'is_busy', self.request_busy_callback)
        # self.pose_srv = self.create_service(Pose, 'request_pose', self.request_pose_callback)
        # self.declare_parameter('route_file', "")
        # self.route_file = self.get_parameter('route_file').value

        self.navigator = BasicNavigator()

        # self.file_path = '/home/hello-robot/stretch_user/navigation_config/'
        # try:
        #     saved_file = open(self.file_path + self.route_file)
        #     self.pose_dict = json.load(saved_file)
        #     saved_file.close()
        # except:
        #     self.pose_dict = {}
        #     self.get_logger().info("Empty pose dict")
        # self.get_logger().info("Pose dict: {0}".format(self.pose_dict))

        # self.get_logger().info("Starting up the waypoint navigator!")
        # self.get_logger().info("Pose dict first pose: {0}".format(self.pose_dict[list(self.pose_dict.keys())[0]]))
        # self.send_initial_pose(self.pose_dict['origin'])
        # self.pose_dict = self.pose_dict['Poses']

        # Wait for navigation to fully activate
        
        self.navigator.waitUntilNav2Active()
        self.send_initial_pose_origin()
        self.get_logger().info("Nav2 active and recognized jec_nav!")
        
        

        # self.audio_cue_client = ActionClient(self, MercerAudio, 'mercer_audio_server')
        # self.audio_file_path = '/home/hello-robot/Documents/MercerX Lab Greeter Audio/'

        # self.ready_to_move = False

        # self.main()
    def request_busy_callback(self,request,response):
        response.success = self.navigator.isTaskComplete()
        return response
    
    def request_pose_callback(self, request, response):
        response.result = self.navigator.isTaskComplete()
        if response.result:
            # parse request number into pose
            gopose = PoseStamped()
            gopose.header.frame_id = 'map'
            gopose.header.stamp = self.navigator.get_clock().now().to_msg()
            gopose.pose.position.x = request.position.x
            gopose.pose.position.y = request.position.y
            gopose.pose.orientation.z = request.orientation.z
            # gopose.pose.orientation.w = 0.0
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
        match waypoint:
            case 0:
                self.get_logger().info("Requested origin")
                return self.make_pose(0,0,0)
            case 1:
                self.get_logger().info("Requested in front ef elevator")
                return self.make_pose(-2.5,-25,math.pi/2)
            case 2:
                self.get_logger().info("Requested close to origin 1")
                return self.make_pose(0,-1,0)
            case 3:
                self.get_logger().info("Requested close to origin 2")
                return self.make_pose(-1,-1,0)
                
    def request_waypoint_callback(self, request, response):
        response.result = self.navigator.isTaskComplete()
        if response.result:
            self.navigator.goToPose(self.waypoint_to_pose(request.number))
        return response
            
    def send_initial_pose_origin(self):
        initial_pose = self.make_pose(0,0,0)
        self.navigator.setInitialPose(initial_pose)
        self.get_logger().info("Sending initial pose: {0}".format(initial_pose))

    # def send_initial_pose(self, pose):
    #     initial_pose = PoseStamped()
    #     initial_pose.header.frame_id = 'map'
    #     initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
    #     initial_pose.pose.position.x = pose['x']
    #     initial_pose.pose.position.y = pose['y']
    #     initial_pose.pose.orientation.z = pose['z']
    #     initial_pose.pose.orientation.w = pose['w']
    #     self.navigator.setInitialPose(initial_pose)
    #     self.get_logger().info("Sending initial pose: {0}".format(initial_pose))


    # def parse_pose_dict_to_poses(self):
    #     self.route_poses = []
    #     pose = PoseStamped()
    #     pose.header.frame_id = 'map'
    #     pose.header.stamp = self.navigator.get_clock().now().to_msg()
    #     pose.pose.orientation.w = 1.0
    #     for key, ps in self.pose_dict.items():
    #         pose.pose.position.x = ps['x']
    #         pose.pose.position.y = ps['y']
    #         pose.pose.orientation.z = ps['z']
    #         self.route_poses.append({
    #             'id' : ps['id'], 
    #             'pose' : deepcopy(pose),
    #             'delay' : ps['delay']})


    # def go_to_waypoint(self, pose, goal_id=""):
    #     nav_start = self.navigator.get_clock().now()

        
    #     # self.status_pub.publish(status_msg)

    #     self.navigator.goToPose(pose)
    #     # self.status = NavStatus.PENDING

        

    #     self.result = self.navigator.getResult()
    #     # publish message to status topic


    # def get_result_callback(self, future):
    #     self.audio_result = future.result().result
    #     self.ready_to_move = True
    #     if self.audio_result.finished == MercerAudio.Result.SUCCESS:
    #         self.get_logger().info("Audio played successfully")
    #     else:
    #         self.get_logger().info("Audio failed to play")

    # def main(self):
        # self.parse_pose_dict_to_poses()

        # self.audio_result = None
        # self.audio_msg = MercerAudio.Goal()
        # self.audio_msg.file_path = self.audio_file_path + 'origin.wav'
        # self.audio_future = self.audio_cue_client.send_goal_async(self.audio_msg)
        # self.audio_future.add_done_callback(self.audio_response_callback)
        # while self.audio_result is None:
            # rclpy.spin_once(self, timeout_sec=1.0)
        # self.get_logger().info("Audio finished playing!")

        # for pose in self.route_poses:
        #     self.get_logger().info("Pose: {0}".format(pose['id']))
        #     self.audio_result = None
        #     self.result = TaskResult.UNKNOWN


        #     self.go_to_waypoint(pose['pose'])

        #     i = 0
        #     while not self.navigator.isTaskComplete():
        #         i += 1
        #         feedback = self.navigator.getFeedback()
        #         if feedback and i % 5 == 0:
        #             self.get_logger().info('Executing current waypoint')
            
        #     self.get_logger().info("Result: {0}".format(self.result))
        #     if self.result == TaskResult.SUCCEEDED:
        #         self.get_logger().info("Navigation to waypoint successful!")
        #     else:
        #         if self.result == TaskResult.UNKNOWN:
        #             while not self.navigator.isTaskComplete():
        #                 rclpy.spin_once(self, timeout_sec=1.0)
        #                 self.get_logger().info("Still spinning {0}".format(pose['id']))

        #     self.audio_msg = MercerAudio.Goal()
        #     self.audio_msg.file_path = self.audio_file_path + pose['id'] + '.wav'
        #     self.audio_future = self.audio_cue_client.send_goal_async(self.audio_msg)
        #     self.audio_future.add_done_callback(self.audio_response_callback)
        #     while self.audio_result is None:
        #         # self.get_logger().info("Waiting for audio to finish playing...")
        #         rclpy.spin_once(self, timeout_sec=0.2)

        #     server_reached = self.audio_cue_client.wait_for_server(timeout_sec=60.0)
        #     if not server_reached:
        #         self.get_logger().error('Unable to connect to audio action server. Timeout exceeded.')
        #         sys.exit()

        #     self.get_logger().info("Audio finished playing!")


        # self.audio_msg = MercerAudio.Goal()
        # self.audio_msg.file_path = self.audio_file_path + 'end.wav'
        # self.audio_future = self.audio_cue_client.send_goal_async(self.audio_msg)
        # self.audio_future.add_done_callback(self.audio_response_callback)
        # self.get_logger().info("Navigation complete!")

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