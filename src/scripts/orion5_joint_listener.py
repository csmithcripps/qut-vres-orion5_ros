#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import serial.tools.list_ports
from math import pi
from math import degrees, radians
# import Orion5
import orion5
from orion5 import orion5_math
import time


NUM_JOINTS = 5
joints = {'base_servo':0, 'shoulder':1, 'elbow': 2, 'wrist': 3, 'claw': 4}


class joint_state_publisher():
    pub = rospy.Publisher('joint_vals', JointState, queue_size=10)
    # Create a JointState variable
    pose = JointState()
    pose.header = Header()
    pose.name = ['base_servo', 'shoulder', 'elbow', 'wrist', 'claw']
    pose.velocity = []
    pose.effort = []
    # Publish message and wait 5 second for the robot to run
    def publishJointState(self):
        joint_values = orion.getAllJointsPosition()
        for i in range(len(joints)):
            joint_values[i] = radians(joint_values[i])
        joint_values[joints['claw']] = (joint_values[joints['claw']])/10000 + 0.01
        self.pose.position = joint_values
        self.pose.header.stamp = rospy.Time.now()
        self.pub.publish(self.pose)

def pub_callback(data):
    publisher.publishJointState()


def callback(data):

    r = rospy.Rate(1) # 10hz

    try:
        # fires when the subscribed node hears the message

        names = data.name
        position = data.position


        # Joint Messages are given in radians, however the orion5 Server needs
        # degrees, so these are converted
        base_servo_position = degrees(position[joints['base_servo']])
        shoulder_servo_position = degrees(position[joints['shoulder']])
        elbow_servo_position = degrees(position[joints['elbow']])
        wrist_servo_position = degrees(position[joints['wrist']])

        # As this Joint is treated as prismatic, it is given a value in metres
        # This is an approximate translation to degrees.
        claw_servo_position = 70 + 5000*position[joints['claw']]


        jointValues = [
                    base_servo_position,
                    shoulder_servo_position,
                    elbow_servo_position,
                    wrist_servo_position,
                    claw_servo_position
                  ]

        orion.setAllJointsPosition(jointValues)
    except KeyboardInterrupt:

        rospy.signal_shutdown('Keyboard interrupt')
        quit()

def listener():
    while not rospy.is_shutdown():
        try:
            rospy.init_node('orion5_joint_listener', anonymous=True)

            # Controls the Arm when given a goal position on the topic 'joint_goal'
            rospy.Subscriber('joint_goal', JointState, callback)

            # Publishes the current joint state at 10 Hz (Ever 0.1s)
            rospy.Timer(rospy.Duration(0.1), pub_callback)

            # Don't exit the
            rospy.spin()

        except KeyboardInterrupt:

            rospy.signal_shutdown('Keyboard interrupt')
            quit()


if __name__ == '__main__':


    global orion
    global publisher
    print("Initialising")
    time.sleep(5)

    orion = orion5.Orion5()
    publisher = joint_state_publisher()
    print('running listener')

    listener()
