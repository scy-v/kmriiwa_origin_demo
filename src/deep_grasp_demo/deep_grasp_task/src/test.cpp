/*********************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2020 PickNik Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Henning Kayser, Simon Goldstein, Boston Cleek
   Desc:   A demo to show MoveIt Task Constructor using a deep learning based
           grasp generator
*/

// ROS
#include <ros/ros.h>
#include <moveit_task_constructor_gpd/grasp_detection.h>
// MTC demo implementation
#include <deep_grasp_task/deep_pick_place_task.h>

#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <tf2_ros/transform_broadcaster.h>

#include <iostream>
#include "std_msgs/String.h"
#include <geometric_shapes/shape_operations.h>
#include <grasping_msgs/GraspPlanningAction.h>
#include <actionlib/client/simple_action_client.h>

void doMsg(const geometry_msgs::Pose::ConstPtr& msg_p){
    ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!:%f",msg_p->position.x);
    ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!:%f",msg_p->position.y);
    ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!:%f",msg_p->position.z);
    ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!:");
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "deep_grasp_demo2");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

moveit::planning_interface::PlanningSceneInterface psi;
  ros::NodeHandle pnh("~");

ros::Subscriber sub = nh.subscribe<geometry_msgs::Pose>("/objection_position_pose",10,doMsg);
  ros::waitForShutdown();
  return 0;
}
