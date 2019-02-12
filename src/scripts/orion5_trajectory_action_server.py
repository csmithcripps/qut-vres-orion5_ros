#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import moveit_msgs.msg
from std_msgs.msg import Header
import serial.tools.list_ports
from math import pi
from math import degrees, radians
# import Orion5
import orion5
from orion5 import orion5_math
import time


arriveThreshold = 0.01
waitTime = 0.5
index = 0

def arrived(desired, actual, threshold):
    diff = orion5_math.absdiff(desired, actual)
    diff[0] = abs((diff[0] + 180) % 360 - 180)
    return max(diff) < threshold

# Scrapes the data from the goal message, publishes goal position
class goal_handler():
    goal_points = []
    positions = []
    goal_index = 0
    current_goal_position = None
    current_joint_state = None
    pub = rospy.Publisher('/joint_goal', JointState, queue_size=10)

    # Create a JointState variable
    pose = JointState()
    pose.header = Header()
    pose.name = ['base_servo', 'shoulder', 'elbow', 'wrist', 'claw']
    pose.velocity = []
    pose.effort = []

    def points_to_positions(self):
        points = self.goal_points
        self.positions = []
        for point in points:
            self.positions.append(point.positions)
        # print(self.positions)

    def publishGoal(self):
        joints = self.current_goal_position
        for i in range(4):
            self.pose.position[0:3] = joints
        self.pose.position[4] = 0.4
        self.pose.header.stamp = rospy.Time.now()
        self.pub.publish(self.pose)

    def reset(self):
        self.goal_points = []
        self.positions = []
        self.goal_index = 0
        self.current_goal_position = None
        self.current_joint_state = None

# Begins handling goal when recieved
def goal_callback(goal):
    print("Received Goal")
    handler.goal_index = 0
    handler.goal_points = goal.goal.trajectory.joint_trajectory.points
    handler.points_to_positions()
    handler.current_goal_position = handler.positions[0]

# Checks the current joint state against the desired and decides what to publish
def jointState_callback(data):
    handler.current_joint_state = data.position
    if handler.current_goal_position is not None:
        if not arrived(handler.current_goal_position, handler.current_joint_state, arriveThreshold):
            handler.publishGoal()
            print(handler.current_goal_position)
        else:
            if handler.current_goal_position == handler.positions[len(handler.positions)-1]:
                handler.reset()
                print('Goal Achieved')
            else:
                handler.goal_index += 1
                handler.points_to_positions()
                handler.current_goal_position = handler.positions[handler.goal_index]
                # print(handler.current_goal_position)
    # else:
    #     print('No Current Goal')


def run():
    while not rospy.is_shutdown():
        try:
            rospy.init_node('orion5_trajectory_action_server', anonymous=True)
            rospy.Subscriber('execute_trajectory/goal', moveit_msgs.msg.ExecuteTrajectoryActionGoal, goal_callback)
            rospy.Subscriber('joint_states', JointState, jointState_callback)
            rospy.spin()

        except KeyboardInterrupt:

            rospy.signal_shutdown('Keyboard interrupt')
            quit()


if __name__ == '__main__':
    global handler
    handler = goal_handler()
    print('Running Action Server')

    run()
