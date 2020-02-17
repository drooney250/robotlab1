#!/usr/bin/env python
import rospy
from balboa_core.msg import balboaMotorSpeeds
from std_msgs.msg import Float64

MOTOR_SPEED_CAP = 20
LINEAR_SPEED_SCALE = 0.5
ANGULAR_SPEED_SCALE = 0.25

class PIDNode(object):
    def __init__(self):
        #initialize the node
        rospy.init_node('adder', anonymous=True)

        #Subscribe to topic for retrieving angle pid
        rospy.Subscriber("ang_pid_output", Float64, self.handle_angle_pid_msg)

        #Subscribe to topic for retrieving dist pid
        rospy.Subscriber("dist_pid_output", Float64, self.handle_dist_pid_msg)

        #Create publisher to publish combined motor speeds
        self.publisher = rospy.Publisher('motorSpeeds', balboaMotorSpeeds, queue_size = 1)

        #Create variables for storing pid control
        self.angle_pid = 0.0
        self.dist_pid = 0.0

    #Read properties off PID message
    def handle_angle_pid_msg(self, msg):
        self.angle_pid = msg.data
        
        #calc new motor speeds and publish
        self.calc_motor_speeds()

    #Read properties off PID message
    def handle_dist_pid_msg(self, msg):
        self.dist_pid = msg.data
        
        #calc new motor speeds and publish
        self.calc_motor_speeds()

    #Calculate motor speeds and publish
    def calc_motor_speeds(self):
        message = balboaMotorSpeeds()
        
        message.left = (self.dist_pid * LINEAR_SPEED_SCALE) - (self.angle_pid * ANGULAR_SPEED_SCALE)
        if message.left > MOTOR_SPEED_CAP:
            message.left = MOTOR_SPEED_CAP
        elif message.left < -1*MOTOR_SPEED_CAP:
            message.left = -1 * MOTOR_SPEED_CAP
        
        message.right = (self.dist_pid * LINEAR_SPEED_SCALE) + (self.angle_pid * ANGULAR_SPEED_SCALE)
        if message.right > MOTOR_SPEED_CAP:
            message.right = MOTOR_SPEED_CAP
        elif message.right < -1*MOTOR_SPEED_CAP:
            message.right = -1 * MOTOR_SPEED_CAP
        
        self.publisher.publish(message)

    #Node main loop
    def main_loop(self):
        rospy.spin()

if __name__ == '__main__':
    a = PIDNode()
    a.main_loop()
