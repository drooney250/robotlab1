#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

#list of distances and angles to drive to write cs
cs_cmds = [(20, 0), (0, 90), (20, 0), (0, 90), (20, 0), (20, 0), (0, 90), (10, 0), (0, 90), (20, 0), (0, -90), (10, 0), (0, -90), (20, 0) ]

curve_cmds = [(40, 180), (0, 0)]

drive_example = [(12, 0), (-12, 0)]

rotate_example = [(0, 90), (0, 180), (0, 360)]

class TheNode(object):
    def __init__(self):
        #Initialize the node
        rospy.init_node( 'write_with_PID', anonymous=True )
        
        #Set up publishers and subscribers
        self.cmd_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size = 10 )
        
    def main_loop(self):
    
        #before sending commands, make sure pid node is up and ready
        rospy.wait_for_message("dist_pid_output", Float64)
        
        #send a command once every 3.5 seconds
        rate = rospy.Rate( 1 / 3.0 )
                
        #loop through each command and publish them
        for dist, angle in cs_cmds:
            msg = Twist()
            msg.linear.x = dist
            msg.angular.z = angle
            self.cmd_publisher.publish( msg )
            
            #sleep until next message
            rate.sleep()
            

if __name__ == '__main__':
    a = TheNode()
    a.main_loop()
