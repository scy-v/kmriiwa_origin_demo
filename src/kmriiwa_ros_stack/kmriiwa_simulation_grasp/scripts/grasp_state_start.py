#!/usr/bin/env python
import rospy
import moveit_commander

rospy.init_node('grasp_object')
arm = moveit_commander.MoveGroupCommander("kmriiwa_manipulator")
grp = moveit_commander.MoveGroupCommander("gripper")
result  = rospy.get_param('/kmriiwa/grasp_state/load_real_world')

if result:
    arm.set_named_target('grasp_state_real')
else:
    arm.set_named_target('grasp_state_sim')

arm.go(wait=True)
