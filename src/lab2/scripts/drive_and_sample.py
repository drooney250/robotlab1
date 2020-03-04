#!/usr/bin/env python
import rospy
import math
from balboa_core.msg import balboaLL
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

SAMPLE_LENGTH = ( 2.52 / 2.0 )
SAMPLE_WIDTH = SAMPLE_LENGTH * 4.0

MAX_ANG_ERROR  = 2.0
MAX_DIST_ERROR = 0.75

ASCII_GRAY_SCALE = " .:-=+*#%@"

MAX_SENSOR_VALUE = 2500.0

class TheNode(object):
    def __init__(self):
        #Initialize the node
        rospy.init_node( 'drive_and_sample', anonymous=True )
        
        #Set up publishers and subscribers
        self.cmd_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size = 10 )
        rospy.Subscriber("/balboaLL", balboaLL, self.handle_balboa_msg)
    	rospy.Subscriber("dist_state", Float64, self.handle_dist_state_msg)
    	rospy.Subscriber("dist_setpoint", Float64, self.handle_dist_setpoint_msg)
    	rospy.Subscriber("ang_state", Float64, self.handle_ang_state_msg)
    	rospy.Subscriber("ang_setpoint", Float64, self.handle_ang_setpoint_msg)
    	
    	#set initial state and setpoint
    	self.dist_state = 0.0
    	self.dist_setpoint = 0.0
    	self.ang_state = 0.0
    	self.ang_setpoint = 0.0
    	
    	#set variables for sensor values
    	self.ref_sensor1 = 0
    	self.ref_sensor3 = 0
    	self.ref_sensor5 = 0
    	
    def handle_balboa_msg(self, msg):
        self.ref_sensor1 = msg.reflectance1
    	self.ref_sensor3 = msg.reflectance3
    	self.ref_sensor5 = msg.reflectance5

    def handle_dist_state_msg(self, msg):
        self.dist_state = msg.data    
    
    def handle_dist_setpoint_msg(self, msg):
        self.dist_setpoint = msg.data
        
    def handle_ang_state_msg(self, msg):
        self.ang_state = msg.data    
    
    def handle_ang_setpoint_msg(self, msg):
        self.ang_setpoint = msg.data
        
    def get_sample(self, pass_num, should_append ):
        #add values to array
        rospy.loginfo("{}, {}, {}".format(self.ref_sensor1, self.ref_sensor3, self.ref_sensor5))
        if should_append:
            self.ref_vals[(pass_num*3)].append(self.ref_sensor1)
            self.ref_vals[(pass_num*3)+1].append(self.ref_sensor3)
            self.ref_vals[(pass_num*3)+2].append(self.ref_sensor5)
        else:
            self.ref_vals[(pass_num*3)].insert(0, self.ref_sensor5)
            self.ref_vals[(pass_num*3)+1].insert(0, self.ref_sensor3)
            self.ref_vals[(pass_num*3)+2].insert(0, self.ref_sensor1)
        
    def wait_for_error(self):
        rate = rospy.Rate(5) #check at 5 Hz
        rate.sleep()
        
        #wait for angular and linear to converge
        while not rospy.is_shutdown():
            dist_err = abs(self.dist_state - self.dist_setpoint)
            ang_err = abs(self.ang_state - self.ang_setpoint)
            
            if dist_err < MAX_DIST_ERROR and ang_err < MAX_ANG_ERROR:
                rate.sleep()
                dist_err = abs(self.dist_state - self.dist_setpoint)
                ang_err = abs(self.ang_state - self.ang_setpoint)
                
                if dist_err < MAX_DIST_ERROR and ang_err < MAX_ANG_ERROR:
                    break

            rate.sleep()
        
    def main_loop(self):
        #read in parameters for square size
        sweep_width = rospy.get_param("sweep_width", 18.0)
        sweep_length = rospy.get_param("sweep_length", 18.0)
        
        #round up width and length to make divisible by width of the robot
        num_passes = int(math.ceil( sweep_width / SAMPLE_WIDTH ))
        num_samples_sweep = int(math.ceil( sweep_length / SAMPLE_LENGTH ))
        
        #set up array for storing values
        self.ref_vals = []
        for i in range(0, num_passes):
            self.ref_vals.append([])
            self.ref_vals.append([])
            self.ref_vals.append([])
        
        #whether we turn left or right
        should_turn_right = True
        
        #before sending commands, make sure pid node is up and ready
        rospy.wait_for_message("dist_pid_output", Float64)
        rospy.wait_for_message("ang_pid_output", Float64)
        
        #go through each pass
        for i in range(0, num_passes):
            for k in range(0, num_samples_sweep):
                #get reading from current position
                self.get_sample( i, should_turn_right )
                
                #create and publish drive message
                msg = Twist()
                msg.linear.x = SAMPLE_LENGTH
                msg.angular.z = 0
                self.cmd_publisher.publish( msg )
                
                #wait for error below threshold
                self.wait_for_error()
                
                #check for rospy shutdown
                if rospy.is_shutdown():
                    break
             
            #get reading from end position
            self.get_sample(i, should_turn_right)

            #turn 90, drive, turn 90
            turn_msg = Twist()
            turn_msg.linear.x = 0
            turn_msg.angular.z = 90 * (-1 if should_turn_right else 1)
            self.cmd_publisher.publish( turn_msg )
            self.wait_for_error()

            drive_msg = Twist()
            drive_msg.linear.x = SAMPLE_WIDTH
            drive_msg.angular.z = 0
            self.cmd_publisher.publish( drive_msg )
            self.wait_for_error()

            self.cmd_publisher.publish( turn_msg )
            self.wait_for_error()
            
            #toggle turn direction
            should_turn_right = not should_turn_right
            
            #check for rospy shutdown
            if rospy.is_shutdown():
                break
        
        #print out reflectance values
        for i in range(0, len(self.ref_vals)):
            ASCII_GRAY_SCALE
            line = ""
            for k in range(0, len(self.ref_vals[i])):
                fill = 1 - (self.ref_vals[i][k] / MAX_SENSOR_VALUE)
                index = int(fill * (len(ASCII_GRAY_SCALE) -1))
                line += ASCII_GRAY_SCALE[ index ]
            rospy.loginfo(line)
            

if __name__ == '__main__':
    a = TheNode()
    a.main_loop()
