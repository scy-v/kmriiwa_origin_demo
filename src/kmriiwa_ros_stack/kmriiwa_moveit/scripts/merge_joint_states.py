#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 该例程将订阅/joint_states话题，消息类型为sensor_msgs/JointState

import rospy
from sensor_msgs.msg import JointState

name_kmriiwa = [] 
velocity_kmriiwa = [] 
effort_kmriiwa = [] 
position_kmriiwa = [] 

name_RG2FT = [] 
velocity_RG2FT = [] 
effort_RG2FT = [] 
position_RG2FT = [] 

def kmriiwa_jointStateCallback(msg):
   global name_kmriiwa
   name_kmriiwa = msg.name
   global velocity_kmriiwa
   velocity_kmriiwa = msg.velocity
   global effort_kmriiwa
   effort_kmriiwa = msg.effort
   global position_kmriiwa
   position_kmriiwa = msg.position
    
def RG2FT_jointStateCallback(msg):
   global name_RG2FT
   name_RG2FT = msg.name
   global velocity_RG2FT
   velocity_RG2FT = msg.velocity
   global effort_RG2FT
   effort_RG2FT = msg.effort
   global position_RG2FT
   position_RG2FT = msg.position

def jointStateSubscriber():
    rospy.init_node('joint_state_subscriber', anonymous=True)

    rospy.Subscriber("/kmriiwa/arm/joint_states", JointState, kmriiwa_jointStateCallback)
    rospy.Subscriber("/joint_states", JointState, RG2FT_jointStateCallback)
    pub = rospy.Publisher('/kmriiwa/merge/joint_states', JointState,queue_size=50)
    joint_state_msg = JointState()
    rate = rospy.Rate(10)  
    rospy.sleep(2)
    # joint_state_msg.name = ["kmriiwa_joint_1","kmriiwa_joint_2","kmriiwa_joint_3","kmriiwa_joint_4","kmriiwa_joint_5","kmriiwa_joint_6","kmriiwa_joint_7","finger_joint"]
    while not rospy.is_shutdown():
        merge_name_kmriiwa = name_kmriiwa + name_RG2FT
        joint_state_msg.name = merge_name_kmriiwa
        # merge_velocity_kmriiwa = velocity_kmriiwa + velocity_RG2FT
        merge_effort_kmriiwa = effort_kmriiwa
        merge_position_kmriiwa = position_kmriiwa + position_RG2FT

        # joint_state_msg.velocity = merge_velocity_kmriiwa
        joint_state_msg.effort = merge_effort_kmriiwa
        joint_state_msg.position = merge_position_kmriiwa
        joint_state_msg.header.stamp = rospy.Time.now()

        pub.publish(joint_state_msg)  
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    jointStateSubscriber()