#!/usr/bin/env python
import rospy 
from std_msgs.msg import String 

while True:
    rospy.init_node('pubname', anonymous=True) 
    pub = rospy.Publisher('topic', String, queue_size=10) 
    msg = String() 
    msg.data = "hello" 
    pub.publish(msg) 

