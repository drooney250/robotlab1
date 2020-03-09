#!/usr/bin/env python
import rospy
from balboa_core.msg import balboaLL
from std_msgs.msg import Float64

V_REF          = 5.0            #ADC reference voltage
MAX_ADC        = float(2 ** 10) #10 bit ADC maximum value
MIN_MEASURE_CM = 15.0
MAX_MEASURE_CM = 150.0

CM_PER_IN = 2.54

'''
Below are values from the datasheet for the range sensor (voltage, centimeters)
{ {2.75, 15}, {2.52, 20}, {2, 30}, {1.53, 40}, {1.28, 50}, {1.05, 60}, {0.94, 70}, {0.85, 80}, {0.73, 90}, {0.65, 100}, {0.59, 110}, {0.55, 120}, {0.505, 130}, {0.475, 140}, {0.44, 150} }

The best fit curve based on the above points, used to estimate range based on voltage input
263.271 - 342.824 x + 172.931 x^2 - 29.6235 x^3
'''

class TheNode(object):
    def handle_balboa_msg(self, msg):
        #Read range finder ADC measurement from message
        range_adc = msg.range

        #convert and put in array
        self.recent_ranges.pop(0)
        self.recent_ranges.append( self.convert_to_inches(range_adc) )

    def convert_to_inches(self, adc_value):
        #Convert from ADC units to voltage
        range_volt = ( adc_value / MAX_ADC ) * V_REF

        #Convert from voltage to distance, using formula described at top of file
        range_cm = 263.271 - (342.82*range_volt) + (172.931*(range_volt**2)) - (29.6235*(range_volt**3))
        range_cm = MIN_MEASURE_CM if range_cm < MIN_MEASURE_CM else range_cm
        range_cm = MAX_MEASURE_CM if range_cm > MAX_MEASURE_CM else range_cm
        range_inch = range_cm / CM_PER_IN
        
        #rospy.loginfo("Got {} from ADC, converted to {} in".format(adc_value, range_inch))
        
        return range_inch

    def __init__(self):
        #Initialize the node
        rospy.init_node( 'convert_sensor_input', anonymous=True )

        #set up initial estimation of range by reading first message
        msg = rospy.wait_for_message('/balboaLL', balboaLL)
        self.recent_ranges = [self.convert_to_inches(msg.range), self.convert_to_inches(msg.range), self.convert_to_inches(msg.range)]

        #Set up publishers and subscribers
        rospy.Subscriber("/balboaLL", balboaLL, self.handle_balboa_msg)
        self.publisher = rospy.Publisher("sensor/range", Float64, queue_size = 1 )

    def main_loop(self):
        rate = rospy.Rate(30) #publish range updates at 30Hz

        #post range messages based on balboa input
        while not rospy.is_shutdown():
            #average most recent range samples
            avg = 0
            for i in range(0, len(self.recent_ranges)):
                avg += self.recent_ranges[i]
            avg /= len(self.recent_ranges)

            #create and post message to sensor/range topic
            msg = Float64(avg)
            self.publisher.publish(msg)

            #sleep until next loop
            rate.sleep()

if __name__ == '__main__':
    a = TheNode()
    a.main_loop()
