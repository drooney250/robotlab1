#!/usr/bin/env python
import struct
import serial
import sys
import rospy
from std_msgs.msg import UInt8

class TheNode(object):
  '''This class will hold the rospy logic for polling and paring a serial
  packet and than publishing it to byte message'''

  def __init__(self):

    rospy.init_node('read_serial')

    # Make sure to do chmod 777 /dev/TTYACM0 first to make it r/w
    port_file = '/dev/ttyACM0'
    baud = 9600

    self.port = serial.Serial(port=port_file, baudrate=baud)
    self.publisher = rospy.Publisher('rx', UInt8, queue_size=10)

  def main_loop(self):
    '''main loop get a packet from the port and parse it and publish it'''
    r = rospy.Rate(400)

    # Publish each byte as it comes in
    while not rospy.is_shutdown():
      c = self.port.read()
      print c
      msg= UInt8()
      msg.data = ord(c)
      self.publisher.publish(msg)
      r.sleep()

if __name__ == '__main__':
  a = TheNode()
  a.main_loop()


