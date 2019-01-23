#!/usr/bin/env python

# Author: Quang Tran 2018

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def talker():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('joint_state_publisher')
    rate = rospy.Rate(0.2) # 0.25 Hz
    # Create a JointState variable
    pose = JointState()
    pose.header = Header()
    pose.header.stamp = rospy.Time.now()
    pose.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4']
    pose.velocity = []
    pose.effort = []
    # Initial pose
    pose.position = [0.0, 1.63, 3.14, 3.14, 0.01]
    # Publish message and wait 5 second for the robot to run
    pub.publish(pose)
    rospy.sleep(7)
    while not rospy.is_shutdown():
      	# Rotate the base_servo by 0.5 radians 
  	pose.position = [0.80, 1.55, 0.79, 2.34, 0.03]
  	pub.publish(pose)
  	rate.sleep()

	pose.position = [5.5, 1.55, 0.79, 2.34, 0.01]
  	pub.publish(pose)
  	rate.sleep()
	
	pose.position = [0.0, 1.63, 3.14, 3.14, 0.01]
  	pub.publish(pose)
  	rate.sleep()
          

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
