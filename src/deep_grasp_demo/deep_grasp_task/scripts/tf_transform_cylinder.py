#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
def object_position_pose(t,o):

    pub = rospy.Publisher('/objection_position_pose',Pose,queue_size=10)
    p = Pose()
    rate = rospy.Rate(5)
    p.position.x = t[0]
    p.position.y = t[1]
    p.position.z = t[2]

    p.orientation.x = o[0]
    p.orientation.y = o[1]
    p.orientation.z = o[2]
    p.orientation.w = o[3]
    pub.publish(p)
    rate.sleep()



def tf_callback(msg):
    frame_names = msg.transforms
    for transform in frame_names:
        frame_id = transform.child_frame_id
        if frame_id.startswith("object"):
            global frame_object_id
            frame_object_id = frame_id
            rospy.loginfo(f"Found frame with name starting with 'object': {frame_object_id}")
            global exit
            exit = True
            subscriber.unregister()



if __name__ == '__main__':

    rospy.init_node('tf_listener',anonymous=True)
    listener = tf.TransformListener() 
    subscriber = rospy.Subscriber('/tf', TransformStamped, tf_callback)
    exit = False
    frame_object_id = "object"
    while True:
        if exit == True:
            break
    rate = rospy.Rate(10.0)
    rospy.sleep(2)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/kmriiwa_base_link', frame_object_id, rospy.Time(0))
            print("trans:")
            print(trans)
            print("rot:")
            print(rot)
            object_position_pose(trans,rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("exception")
            #break
            continue
        rate.sleep()
    
