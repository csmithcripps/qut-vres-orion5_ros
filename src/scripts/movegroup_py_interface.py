#!/usr/bin/env python

# Author: Quang Tran 2018

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import controller_simulation
import tf

'''
First initialize moveit_commander and rospy.
'''
print ("============ Starting MoveGroupCommander")
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

'''
Instantiate a RobotCommander object. This object is an interface to the robot as a whole.
'''
robot = moveit_commander.RobotCommander()

'''
Instantiate a PlanningSceneInterface object. This object is an interface to the world surrounding the robot.
'''
scene = moveit_commander.PlanningSceneInterface()

'''
Instantiate a MoveGroupCommander object. This object is an interface to one group of joints. In this case the group is the joints in the left arm. This interface can be used to plan and execute motions on the left arm.
'''
group = moveit_commander.MoveGroupCommander("manipulator")

'''
We create this DisplayTrajectory publisher which is used below to publish trajectories for RVIZ to visualize.
'''
#display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)

'''
Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
'''


#print "============ Reference frame: %s" % group.get_planning_frame()

#print "============ Reference frame: %s" % group.get_end_effector_link()

#print "============ Robot Groups:"
#print robot.get_group_names()arry out, specifically setting joint or pose goals, creating motion plans, moving the robot,

'''
q = tf.transformations.quaternion_from_euler(3.14, -0.38, 0.77)

print "============ Generating plan 1"
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.x = q[0]
pose_target.orientation.y = q[1]
pose_target.orientation.z = q[2]
pose_target.orientation.w = q[3]
pose_target.position.x = 0.22
pose_target.position.y = 0.21
pose_target.position.z = 0.1
group.set_pose_target(pose_target, end_effector_link=group.get_end_effector_link())
'''

'''
#rpy = [0.0, 0.0, 0.0]
#group.set_rpy_target(rpy, end_effector_link=group.get_end_effector_link())
xyz =[0.25,0.2,0.2]
group.set_position_target(xyz, end_effector_link=group.get_end_effector_link())

#print xyz

#print pose_target

plan1 = group.plan()
rospy.sleep(5)
group.go(wait=True)
'''

'''
print "============ Printing robot state"
print robot.get_current_state()
print "============"
'''

'''
print "============ Printing robot current pose"
print group.get_current_pose(end_effector_link=group.get_end_effector_link())
print "============"



print "============ Printing robot current rpy"
print group.get_current_rpy(end_effector_link=group.get_end_effector_link())
print "============"
'''


'''group.clear_pose_targets()
group_variable_values = group.get_current_joint_values()
print "============ Joint values: ", group_variable_values
'''


while not rospy.is_shutdown():
    xyz = [0.1,-0.1,0.250]
    group.set_position_target(xyz, end_effector_link=group.get_end_effector_link())
    plan1 = group.plan()
    group.go(wait=True)
    rospy.sleep(5)

    print "============ Printing robot current pose"
    print group.get_current_pose(end_effector_link=group.get_end_effector_link())
    print "============"

    print "============ Printing robot current rpy"
    print group.get_current_rpy(end_effector_link=group.get_end_effector_link())
    print "============"

    xyz = [-0.1,-0.1,0.250]
    group.set_position_target(xyz, end_effector_link=group.get_end_effector_link())
    plan1 = group.plan()
    group.go(wait=True)
    rospy.sleep(5)

    print "============ Printing robot current pose"
    print group.get_current_pose(end_effector_link=group.get_end_effector_link())
    print "============"

    print "============ Printing robot current rpy"
    print group.get_current_rpy(end_effector_link=group.get_end_effector_link())
    print "============"

'''
    xyz = [-0.15,-0.2,0.250]
    group.set_position_target(xyz, end_effector_link=group.get_end_effector_link())
    plan1 = group.plan()
    group.go(wait=True)
    rospy.sleep(5)

    xyz = [-0.2,-0.2,0.250]
    group.set_position_target(xyz, end_effector_link=group.get_end_effector_link())
    plan1 = group.plan()
    group.go(wait=True)
    rospy.sleep(5)
'''
