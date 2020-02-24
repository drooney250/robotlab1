#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

class PIDNode(object):
    def __init__(self):
        #initialize the node
        rospy.init_node('PID', anonymous=True)

        #Subscribe to topic for retrieving goals
        rospy.Subscriber("generic_setpoint", Float64, self.goal_collector)

        #Subscribe to topic for retrieving robot state
        rospy.Subscriber("current_state", Float64, self.state_collector)

        #Create publisher to publish PID output
        self.publisher = rospy.Publisher('pid_output', Float64, queue_size = 1)

        #Get PID parameters and necessary values
        self.p = rospy.get_param("p_term", 0.0)
        self.i = rospy.get_param("i_term", 0.0)
        self.d = rospy.get_param("d_term", 0.0)
        self.goal = rospy.wait_for_message("generic_setpoint", Float64).data
        self.state = rospy.wait_for_message("current_state", Float64).data
        self.prev_error = 0
        self.integral = 0
        self.command = Float64()

    #Retrive current goal of robot
    def goal_collector(self, goal):
        self.goal = goal.data

    #Retrieve current state of robot
    def state_collector(self, state):
        self.state = state.data

    #Calculate command value from PID formula
    def PID(self):
        #Get values from param server
        self.p = rospy.get_param("p_term", 0.0)
        self.i = rospy.get_param("i_term", 0.0)
        self.d = rospy.get_param("d_term", 0.0)

        #Calculate error
        error = self.goal - self.state

        #Calculate integral for I term, capping at 100 to prevent windup error
        self.integral = self.integral + error
        if self.integral > 100:
            self.integral = 100
        elif self.integral < -100:
            self.integral = -100
            
        #Calculate output
        self.command = Float64( self.p*(error) + self.i*(self.integral) + self.d*(error - self.prev_error) )
        self.prev_error = error
        self.publisher.publish(self.command)

    #Node main loop
    def main_loop(self):
        rate = rospy.Rate(30) #30Hz

        while not rospy.is_shutdown():
            self.PID()
            rate.sleep()

if __name__ == '__main__':
    a = PIDNode()
    a.main_loop()
