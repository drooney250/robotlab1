#!/usr/bin/env python
import rospy
from balboa_core.msg import balboaMotorSpeeds
from geometry_msgs.msg import Twist

class TheNode(object):

    def handleKeyPress(self, msg):
        #Interpret message from keyboard, flag new message received
        self.msg_received = True
        linear = msg.linear.x
        angular = msg.angular.z
        
        #Translate into motor speeds
        self.message = balboaMotorSpeeds()
        self.message.left = (linear * 7.5) - (angular * 2.5)
        self.message.right = (linear * 7.5) + (angular * 2.5)

    def __init__(self):
        #Initialize the node
        rospy.init_node( 'teleop_robot', anonymous=True )
        
        #Message to send to motorSpeeds topic and flag for sending
        self.message = balboaMotorSpeeds()
        self.msg_received = False
        
        #Start listening and set up subscriber
        rospy.Subscriber("/turtle1/cmd_vel", Twist, self.handleKeyPress)
        self.publisher = rospy.Publisher("motorSpeeds", balboaMotorSpeeds, queue_size = 1 )
    
    def main_loop(self):
        #Create rate object for main loop
        rate = rospy.Rate(20) #20 Hz
        
        while not rospy.is_shutdown():
            #If keyboard message present, push through, otherwise, stop
            if self.msg_received:
                self.publisher.publish(self.message)
                self.msg_received = False
            else:
                self.message = balboaMotorSpeeds()
                self.message.left = 0
                self.message.right = 0
                self.publisher.publish(self.message)
            
            #Sleep until next loop
            rate.sleep()
    
if __name__ == '__main__':
    a = TheNode()
    a.main_loop()
