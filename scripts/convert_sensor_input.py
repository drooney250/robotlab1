#!/usr/bin/env python
import rospy
from balboa_core.msg import balboaLL
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

TICKS_PER_DEGREE = 973.275
TICKS_PER_INCH = 120.0

class TheNode(object):
    def handle_balboa_msg(self, msg):
        #Read linear from new message
        linear_dist = ( msg.distanceLeft + msg.distanceRight ) / 2
        
        #Convert to useful units
        ang_state = msg.angleX / TICKS_PER_DEGREE
        lin_state = linear_dist / TICKS_PER_INCH
        
        #Create and publish messages
        ang_state_msg = Float64(ang_state)
        self.ang_publisher.publish(ang_state_msg)
        
        dist_state_msg = Float64(lin_state)
        self.dist_publisher.publish(dist_state_msg)


    def __init__(self):
        #Initialize the node
        rospy.init_node( 'convert_sensor_input', anonymous=True )
        
        #Set up publishers and subscribers
        rospy.Subscriber("/balboaLL", balboaLL, self.handle_balboa_msg)
        self.ang_publisher = rospy.Publisher("ang_state", Float64, queue_size = 1 )
        self.dist_publisher = rospy.Publisher("dist_state", Float64, queue_size = 1 )

    def main_loop(self):
        rospy.spin()

if __name__ == '__main__':
    a = TheNode()
    a.main_loop()
