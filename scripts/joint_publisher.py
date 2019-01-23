#!/usr/bin/env python

# Author: Quang Tran 2018

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def talker():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('joint_state_publisher')
    rate = rospy.Rate(0.5) # 0.5 Hz
    # Create a JointState variable
    pose = JointState()
    pose.header = Header()
    pose.header.stamp = rospy.Time.now()
    pose.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4']
    pose.velocity = []
    pose.effort = []
    # Initial pose
    pose.position = [6.0, 1.8, 1.1, 3.7, 0.025]
    # Publish message and wait 5 second for the robot to run
    pub.publish(pose)
    rospy.sleep(5)
    while not rospy.is_shutdown():
      # Rotate the base_servo by 0.5 radians 
      while True:
          pose.position[0]+=0.5
          pub.publish(pose)
          rate.sleep()
          if pose.position[0]>=6.0:
              break
      while True:
          pose.position[0]-=0.5
          pub.publish(pose)
          rate.sleep()
          if pose.position[0]<=0.0:
              break

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
