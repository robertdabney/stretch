#! /usr/bin/env python3

# Adapted from the simple commander demo examples on 
# https://github.com/ros-planning/navigation2/blob/galactic/nav2_simple_commander/nav2_simple_commander/demo_security.py



import math
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import json
import sys
import speech_recognition as sr
import pyttsx3 

class Speech(Node):
    def __init__(self):
        
        super().__init__('speech')
        node_name = self.get_name()
        self.get_logger().info("{0} started".format(node_name))
        self.r = sr.Recognizer() 
        self.speech_srv = self.create_service(Trigger, 'collect_speech', self.request_speech_callback)
        
        
    def SpeakText(self,command):
        # Initialize the engine
        engine = pyttsx3.init()
        engine.say(command) 
        engine.runAndWait()
    def request_speech_callback(self,request,response):
        try:
        
            # use the microphone as source for input.
            with sr.Microphone() as source2:
                
                # wait for a second to let the recognizer
                # adjust the energy threshold based on
                # the surrounding noise level 
                self.r.adjust_for_ambient_noise(source2, duration=0.2)
                
                #listens for the user's input 
                audio2 = self.r.listen(source2)
                
                # Using google to recognize audio
                MyText = self.r.recognize_google(audio2)
                MyText = MyText.lower()
                response.success = True
                response.message = MyText
                self.get_logger().info("Captured {0}".format(MyText))
                return response
                        
        except sr.RequestError as e:
            self.get_logger().warning("Could not request results; {0}".format(e))
            
        except sr.UnknownValueError:
            self.get_logger().warning("unknown error occurred")
            


def main(args=None):
    rclpy.init()
    node = Speech()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('interrupt received, so shutting down')

        
    if node is not None:
        node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()

