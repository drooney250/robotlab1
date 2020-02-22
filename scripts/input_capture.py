#!/usr/bin/env python
import rospy
from balboa_core.msg import balboaMotorSpeeds
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class TheNode(object):
    def handle_vel_msg(self, msg):
        #Read linear and angular from new message
        linear = msg.linear.x
        angular = msg.angular.z
        
	#read in constants from param server
        linear_input_scaling = rospy.get_param("linear_input_scaling", 0.15)
    	angular_input_scaling = rospy.get_param("angular_input_scaling", 1.0)

        #Aggregate into setpoints
        self.linear_setpoint = self.linear_setpoint + (linear * linear_input_scaling)
        self.angular_setpoint = self.angular_setpoint + (angular * angular_input_scaling)
        
        #Create messages and publish to setpoint topics
        dist_message = Float64(self.linear_setpoint)
        self.dist_publisher.publish(dist_message)
        ang_message = Float64(self.angular_setpoint)
        self.ang_publisher.publish(ang_message)
        

    def __init__(self):
        #Initialize the node
        rospy.init_node( 'input_capture', anonymous=True )
        
        #Set up publishers and subscribers
        rospy.Subscriber("/turtle1/cmd_vel", Twist, self.handle_vel_msg)
        self.ang_publisher = rospy.Publisher("ang_setpoint", Float64, queue_size = 1 )
        self.dist_publisher = rospy.Publisher("dist_setpoint", Float64, queue_size = 1 )
        
        #Initialize setpoints
        self.linear_setpoint = rospy.wait_for_message("dist_state", Float64).data
        self.angular_setpoint = rospy.wait_for_message("ang_state", Float64).data
        
        #Create messages and publish to setpoint topics
        dist_message = Float64(self.linear_setpoint)
        self.dist_publisher.publish(dist_message)
        ang_message = Float64(self.angular_setpoint)
        self.ang_publisher.publish(ang_message)

    def main_loop(self):
        rospy.spin()

if __name__ == '__main__':
    a = TheNode()
    a.main_loop()
