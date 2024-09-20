#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

def move_to_goal(x, y):
    rospy.init_node('move_to_goal_node')
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'  # 坐标系为地图
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0  # 朝向不变

    client.send_goal(goal)
    client.wait_for_result()

    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("成功到达目标点！")
    else:
        rospy.loginfo("无法到达目标点。")

if __name__ == '__main__':
    try:
        x_goal = 2  # 设置目标点的x坐标
        y_goal = 1.3  # 设置目标点的y坐标
        move_to_goal(x_goal, y_goal)
    except rospy.ROSInterruptException:
        pass
