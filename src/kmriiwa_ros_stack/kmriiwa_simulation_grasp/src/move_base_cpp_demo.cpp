#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void move_to_goal(double x, double y) {
    ros::NodeHandle nh;
    MoveBaseClient client("move_base", true);
    client.waitForServer();

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";  // 坐标系为地图
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.w = 1.0;  // 朝向不变

    client.sendGoal(goal);
    client.waitForResult();

    if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("成功到达目标点！");
    } else {
        ROS_INFO("无法到达目标点。");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "move_to_goal_node");
    double x_goal = 1.0;  // 设置目标点的x坐标
    double y_goal = 2.0;  // 设置目标点的y坐标
    move_to_goal(x_goal, y_goal);
    return 0;
}