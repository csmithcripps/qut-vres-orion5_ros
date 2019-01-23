#!/usr/bin/env python

# Author: Quang Tran 2018

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import getch

# Amount to move end effector each update
INCREMENT = 0.05


print("============ Starting moveit commander")
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('controller_movegroup_interface', anonymous=False)

robot = moveit_commander.RobotCommander()
# scene = moveit_commander.PlanningSceneInterface()

'''
Instantiate a MoveGroupCommander object. This object is an interface to one group of joints. In this case the group is the joints in the left arm. This interface can be used to plan and execute motions on the left arm.
'''
group = moveit_commander.MoveGroupCommander("manipulator")
# group.clear_pose_targets()
rate = rospy.Rate(0.5)


print "============ Reference frame: %s" % group.get_planning_frame()
print "============ End effector: %s" % group.get_end_effector_link()

print("Setting initial pose...")

xyz =[0.129,-0.00,0.250]
group.set_position_target(xyz, end_effector_link=group.get_end_effector_link())
plan1 = group.plan()
print group.get_current_joint_values()
group.go(wait=True)

while not rospy.is_shutdown():
	a = getch.getch()
	if a == "i":
		msg = group.get_current_pose()
		xyz = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
		xyz[2]+=INCREMENT
		print xyz
		group.set_position_target(xyz, end_effector_link=group.get_end_effector_link())
		plan1 = group.plan()
		# print group.get_current_joint_values()
		group.go(wait=True)
		rate.sleep()
	elif a == "k":
		msg = group.get_current_pose()
		xyz = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
		xyz[2]-=INCREMENT
		print xyz
		group.set_position_target(xyz, end_effector_link=group.get_end_effector_link())
		plan1 = group.plan()
		# print group.get_current_joint_values()
		group.go(wait=True)
		rate.sleep()
	elif a == "w":
		msg = group.get_current_pose()
		xyz = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
		xyz[0]+=INCREMENT
		print xyz
		group.set_position_target(xyz, end_effector_link=group.get_end_effector_link())
		plan1 = group.plan()
		print group.get_current_joint_values()
		group.go(wait=True)
		rate.sleep()
	elif a == "s":
		msg = group.get_current_pose()
		xyz = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
		xyz[0]-=INCREMENT
		print xyz
		group.set_position_target(xyz, end_effector_link=group.get_end_effector_link())
		plan1 = group.plan()
		print group.get_current_joint_values()
		group.go(wait=True)
		rate.sleep()
	elif a == "d":
		msg = group.get_current_pose()
		xyz = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
		xyz[1]+=INCREMENT
		print xyz
		group.set_position_target(xyz, end_effector_link=group.get_end_effector_link())
		plan1 = group.plan()
		print group.get_current_joint_values()
		group.go(wait=True)
		rate.sleep()
	elif a == "a":
		msg = group.get_current_pose()
		xyz = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
		xyz[1]-=INCREMENT
		print xyz
		group.set_position_target(xyz, end_effector_link=group.get_end_effector_link())
		plan1 = group.plan()
		print group.get_current_joint_values()
		group.go(wait=True)
		rate.sleep()
	elif a == "q":
		break
	elif a == "j":
		array = group.get_current_joint_values()
		array[0]-=0.5 #base_servo angle
		group.set_joint_value_target(array)
		print group.get_current_joint_values()
		print group.get_current_pose()
		plan1 = group.plan()
		group.go(wait=True)
		rate.sleep()
	elif a == "l":
		array = group.get_current_joint_values()
		array[0]+=0.5
		group.set_joint_value_target(array)
		print group.get_current_joint_values()
		print group.get_current_pose()
		plan1 = group.plan()
		group.go(wait=True)
		rate.sleep()

	# array = group.get_current_joint_values()
	# print array[0]
    #
	# group.set_position_target(xyz, end_effector_link=group.get_end_effector_link())
	# plan1 = group.plan()
	# group.go(wait=True)
	# rate.sleep()
