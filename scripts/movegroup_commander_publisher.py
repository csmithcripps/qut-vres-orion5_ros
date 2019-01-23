#!/usr/bin/env python

# Author: Jenna Riseley 2017

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import controller_simulation
#import ps2ctrl

# Amount to move end effector each update
INCREMENT = 0.01


def shutdown_hook():
  print "Shutting down movegroup interface for psx!"


print("============ Starting moveit commander/ psx controller node")
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('controller_movegroup_interface', anonymous=False)



robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

'''
Instantiate a MoveGroupCommander object. This object is an interface to one group of joints. In this case the group is the joints in the left arm. This interface can be used to plan and execute motions on the left arm.
'''
group = moveit_commander.MoveGroupCommander("manipulator")
#group.clear_pose_targets()
rate = rospy.Rate(0.5)


# Get an interface to the ps2 controller

ps2ctrl = controller_simulation.ps2ctrltest()

print "============ Reference frame: %s" % group.get_planning_frame()
print "============ End effector: %s" % group.get_end_effector_link()

# Set initial pose


#First initialize moveit_commander and rospy.
#print "============ Please wait - setting initial pose..."



#group.allow_replanning(True)
group.set_goal_position_tolerance(0.001)
#group.set_goal_orientation_tolerance(6.2)


#start_pose = group.get_current_pose()
#group.set_pose_target(start_pose)
#plan0 = group.plan()
#group.go(wait=True)

#print(start_pose)
#pose_target = start_pose
#print(type(start_pose))[
#print(type(pose_target))
print("Setting initial pose...")
xyz =[0.129,-0.00,0.250]
group.set_position_target(xyz, end_effector_link=group.get_end_effector_link())

plan1 = group.plan()

group.go(wait=True)


while not rospy.is_shutdown():
    ps2ctrl.get_controls()
    #print(ps2ctrl.btn_states[ps2ctrl.DPAD_UP])
    if ps2ctrl.btn_states[ps2ctrl.DPAD_UP]==True:
        xyz[2] += INCREMENT
    elif ps2ctrl.btn_states[ps2ctrl.DPAD_DOWN]==True:
        xyz[2] -= INCREMENT

    if ps2ctrl.stick_states[ps2ctrl.RIGHT_X] > 130:
        xyz[0] += INCREMENT
    elif ps2ctrl.stick_states[ps2ctrl.RIGHT_X] < 120:
        xyz[0] -= INCREMENT

    if ps2ctrl.stick_states[ps2ctrl.RIGHT_Y] > 130:
        xyz[1] += INCREMENT
    elif ps2ctrl.stick_states[ps2ctrl.RIGHT_Y] < 120:
        xyz[1] -= INCREMENT

    #print(ps2ctrl.stick_states[ps2ctrl.RIGHT_X])
    # TODO: add analog (X, Y plane)

    group.set_position_target(xyz, end_effector_link=group.get_end_effector_link())

    plan1 = group.plan()

    group.go(wait=True)

    rate.sleep()

rospy.on_shutdown(shutdown_hook)
