import math
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import sys
import speech_recognition as sr

# This is the speech recognition node for JEC navigation
# it is a service server that when called, responds with speech recognition for 5 seconds

class Speech(Node):
    def __init__(self):
        
        super().__init__('speech')
        node_name = self.get_name()
        self.get_logger().info("{0} started".format(node_name))
        self.r = sr.Recognizer() 
        self.speech_srv = self.create_service(Trigger, 'collect_speech', self.request_speech_callback)
        self.num_words = [
            "zero", "one", "two", "three", "four", "five", "six", "seven", "eight",
            "nine", "ten", "eleven", "twelve", "thirteen", "fourteen", "fifteen",
            "sixteen", "seventeen", "eighteen", "nineteen",
        ]
            
    def request_speech_callback(self,request,response):
        try:
        
            # use the microphone as source for input.
            with sr.Microphone() as source2:
                # wait for a second to let the recognizer
                # adjust the energy threshold based on
                # the surrounding noise level 
                self.r.adjust_for_ambient_noise(source2, duration=0.2)
                self.get_logger().info("Listening for input")
                #listens for the user's input 
                audio2 = self.r.listen(source2,timeout=5)
                self.get_logger().info("Parsing Recorded Input")
                # Using google to recognize audio
                MyText = self.r.recognize_google(audio2)
                MyText = MyText.lower()
                if MyText in self.num_words:
                    MyText = str(self.num_words.index(MyText))
                response.success = True
                response.message = MyText
                self.get_logger().info("Captured {0}".format(MyText))
                return response
                        
        except sr.RequestError as e:
            self.get_logger().warning("Could not request results; {0}".format(e))
            response.success = False
            response.message = ""
            return response
        except sr.UnknownValueError:
            self.get_logger().warning("unknown error occurred")
            response.success = False
            response.message = ""
            return response

            


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

