#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_interface/planning_interface.h>
#include <pluginlib/class_loader.hpp>
#include <ros/ros.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
// MoveIt
#include <moveit_msgs/PlanningScene.h>
#include <boost/scoped_ptr.hpp>
// Moveit
#include <ros/ros.h>
#include <memory>
#include <std_msgs/String.h>
#include <onrobot_rg2ft_msgs/RG2FTCommand.h>
// MoveitCpp
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>

#include <geometry_msgs/PointStamped.h>
const double tau = 2 * M_PI;


void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL open_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.04;
  posture.points[0].positions[1] = 0.04;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL closed_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.00;
  posture.points[0].positions[1] = 0.00;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}


void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
  // BEGIN_SUB_TUTORIAL pick1
  // Create a vector of grasps to be attempted, currently only creating single grasp.
  // This is essentially useful when using a grasp generator to generate and test multiple grasps.
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  // Setting grasp pose
  // ++++++++++++++++++++++
  // This is the pose of panda_link8. |br|
  // Make sure that when you set the grasp_pose, you are setting it to be the pose of the last link in
  // your manipulator which in this case would be `"panda_link8"` You will have to compensate for the
  // transform from `"panda_link8"` to the palm of the end effector.
  grasps[0].grasp_pose.header.frame_id = "kmriiwa_base_footprint";
  tf2::Quaternion orientation;  
  orientation.setRPY(0, 0, 0);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = 1.39517;
  grasps[0].grasp_pose.pose.position.y = -0.253699;
  grasps[0].grasp_pose.pose.position.z = 0.745499;

  // Setting pre-grasp approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  // grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
  // /* Direction is set as positive x axis */
  // grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
  // grasps[0].pre_grasp_approach.min_distance = 0.095;
  // grasps[0].pre_grasp_approach.desired_distance = 0.115;

  // // Setting post-grasp retreat
  // // ++++++++++++++++++++++++++
  // /* Defined with respect to frame_id */
  // grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
  // /* Direction is set as positive z axis */
  // grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  // grasps[0].post_grasp_retreat.min_distance = 0.1;
  // grasps[0].post_grasp_retreat.desired_distance = 0.25;

  // Setting posture of eef before grasp
  // +++++++++++++++++++++++++++++++++++
  // openGripper(grasps[0].pre_grasp_posture);
  // END_SUB_TUTORIAL

  // BEGIN_SUB_TUTORIAL pick2
  // Setting posture of eef during grasp
  // +++++++++++++++++++++++++++++++++++
  // closedGripper(grasps[0].grasp_posture);
  // END_SUB_TUTORIAL

  // BEGIN_SUB_TUTORIAL pick3
  // Set support surface as table1.
  // Call pick to pick up the object using the grasps given
  move_group.pick("none", grasps);
  // END_SUB_TUTORIAL
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grasp_test_node");
  ros::NodeHandle node_handle;

  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(2);
  spinner.start();

  static const std::string PLANNING_GROUP = "kmriiwa_manipulator";
  static const std::string LOGNAME = "test_grasp";
  
  ros::Duration(1.0).sleep();

  ROS_INFO_STREAM_NAMED(LOGNAME, "Starting MoveIt...");

  auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(node_handle);
  moveit_cpp_ptr->getPlanningSceneMonitorNonConst()->providePlanningSceneService();

  auto planning_components = std::make_shared<moveit_cpp::PlanningComponent>(PLANNING_GROUP, moveit_cpp_ptr);
  auto robot_model_ptr = moveit_cpp_ptr->getRobotModel();
  auto robot_start_state = planning_components->getStartState();
  auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);

  planning_components->setStartStateToCurrentState();
  // The first way to set the goal of the plan is by using geometry_msgs::PoseStamped ROS message type as follow
  geometry_msgs::PoseStamped target_pose1;
  target_pose1.header.frame_id = "kmriiwa_link_0";
  target_pose1.pose.orientation.w = 1.0;
  target_pose1.pose.position.x = 0.15;
  target_pose1.pose.position.y = -0.6;
  target_pose1.pose.position.z = 0.75;
  planning_components->setGoal(target_pose1, "kmriiwa_link_ee");
  // Now, we call the PlanningComponents to compute the plan and visualize it.
  // Note that we are just planning
  auto plan_solution1 = planning_components->plan();
  if (plan_solution1){
     planning_components->execute();
  }

    ros::Publisher pub = node_handle.advertise<onrobot_rg2ft_msgs::RG2FTCommand>("/command", 20);
    onrobot_rg2ft_msgs::RG2FTCommand msg;
    msg.TargetForce = 200;
    msg.TargetWidth = 1000;
    msg.Control = 0x0001;
    while(true)
    {
        ros::Duration(0.1).sleep();
        pub.publish(msg);
        ros::Duration(0.1).sleep();
        if (pub.getNumSubscribers() == 1)
        {
            pub.publish(msg);
            break;
        }
    }
    ros::Duration(0.5).sleep();

    msg.TargetForce = 200;
    msg.TargetWidth = 10;
    msg.Control = 0x0001;
    while(true)
    {
        ros::Duration(0.1).sleep();
        pub.publish(msg);
        ros::Duration(0.1).sleep();
        if (pub.getNumSubscribers() == 1)
        {
            pub.publish(msg);
            break;
        }
    }
    ros::Duration(0.5).sleep();

    msg.TargetForce = 200;
    msg.TargetWidth = 700;
    msg.Control = 0x0001;
    while(true)
    {
        ros::Duration(0.1).sleep();
        pub.publish(msg);
        ros::Duration(0.1).sleep();
        if (pub.getNumSubscribers() == 1)
        {
            pub.publish(msg);
            break;
        }
    }

  sleep(2000);
  // robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  // const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  // // planning_scene::PlanningScene planning_scene(kinematic_model);
  // planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(kinematic_model));
  // static const std::string PLANNING_GROUP = "kmriiwa_manipulator";

  // // The :planning_interface:`MoveGroupInterface` class can be easily
  // // setup using just the name of the planning group you would like to control and plan for.
  // moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // const moveit::core::JointModelGroup* joint_model_group =
  //     move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  // ROS_INFO("Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

 
  // // ROS_INFO_NAMED("Available Planning Groups:");

  // // #2.89517 -0.253699 0.745499
  // std::copy(move_group_interface.getJointModelGroupNames().begin(),
  //           move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));



  // // /* Set the state in the planning scene to the final state of the last plan */
  // move_group_interface.getCurrentState()->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  // planning_scene->setCurrentState(*move_group_interface.getCurrentState().get());

  // pick(move_group_interface);
  // geometry_msgs::Pose target_pose1;
  // target_pose1.orientation.w = 0.604;
  // target_pose1.orientation.x = -0.293;
  // target_pose1.orientation.y = -0.435;
  // target_pose1.orientation.z = -0.628;
  
  // target_pose1.position.x = 1.050;
  // target_pose1.position.y = 0.027;
  // target_pose1.position.z = 1.809;
  // move_group_interface.setPoseReferenceFrame("kmriiwa_link0");
  // move_group_interface.setPoseTarget(target_pose1);
  // moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  // move_group_interface.execute(my_plan);
}
