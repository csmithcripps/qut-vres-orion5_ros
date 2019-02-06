#!/usr/bin/env python

# Author: Quang Tran 2018
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import serial.tools.list_ports
from math import pi
from math import degrees
# import Orion5
import orion5
from orion5 import orion5_math
import time

def talker():
    pub = rospy.Publisher('joint_vals', JointState, queue_size=10)
    rospy.init_node('orion5_joint_state_publisher')
    rate = rospy.Rate(10) # 0.5 Hz
    # Create a JointState variable
    pose = JointState()
    pose.header = Header()
    pose.header.stamp = rospy.Time.now()
    pose.name = ['base', 'shoulder', 'elbow', 'wrist', 'claw']
    pose.velocity = []
    pose.effort = []
    # Initial pose
    pose.position = orion.getAllJointsPosition()
    # Publish message and wait 5 second for the robot to run
    pub.publish(pose)
    rospy.sleep(1)
    while not rospy.is_shutdown():
      while True:
          pose.header.stamp = rospy.Time.now()
          pose.position = orion.getAllJointsPosition()
          pub.publish(pose)
          print(pose)
          rate.sleep()

if __name__ == '__main__':
    try:

        global orion
        orion = orion5.Orion5()
        print('running Publisher')

        talker()
    except rospy.ROSInterruptException:
        pass
