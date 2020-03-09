#!/usr/bin/env python
import rospy
import math
from balboa_core.msg import balboaLL
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

UPDATE_RATE = 30
MAX_DRIVE_PER_SECOND = 3.0
MAX_DRIVE_PER_UPDATE = MAX_DRIVE_PER_SECOND / UPDATE_RATE
ALLOWABLE_ERROR = 2.0

class TheNode(object):
    def __init__(self):
        #Initialize the node
        rospy.init_node( 'drive_and_sample', anonymous=True )
        
    	#Wait for first range message before initializing node
    	self.range = rospy.wait_for_message('sensor/range', Float64).data
    	self.current_dist_setpoint = rospy.wait_for_message('sensor/range', Float64).data
        
        #Set up publishers and subscribers
        self.cmd_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size = 10 )
    	rospy.Subscriber("sensor/range", Float64, self.handle_range_msg)
    	
    def handle_range_msg(self, msg):
        self.range = msg.data
        
    def main_loop(self):
        #read in parameters for goal distance from object
        goal_range = rospy.get_param("reactive_control/goal_range", 24.0) #default is 24in or about 61 cm
        
        #before sending commands, make sure pid node is up and ready
        rospy.wait_for_message("dist_pid_output", Float64)
        rospy.wait_for_message("ang_pid_output", Float64)

        #check distance at 30 Hz
        rate = rospy.Rate(UPDATE_RATE)
        
        while not rospy.is_shutdown():
            #get error between desired range and current position
            error = self.range - goal_range

            #if error is positive, move setpoint forward, else move backwards
            x = 0
            if error > ALLOWABLE_ERROR:
                x += MAX_DRIVE_PER_UPDATE
            elif error < (-1*ALLOWABLE_ERROR):
                x -= MAX_DRIVE_PER_UPDATE

            #send message to update setpoint
            msg = Twist()
            msg.linear.x = x
            msg.angular.z = 0
            self.cmd_publisher.publish( msg )
            
            #sleep for next iteration
            rate.sleep()

if __name__ == '__main__':
    a = TheNode()
    a.main_loop()
