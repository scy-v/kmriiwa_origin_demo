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
// MTC demo implementation
#include <deep_grasp_task/deep_pick_place_task.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <tf2_ros/transform_broadcaster.h>

#include <iostream>
#include "std_msgs/String.h"
#include <geometric_shapes/shape_operations.h>
#include <grasping_msgs/GraspPlanningAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

constexpr char LOGNAME[] = "deep_grasp_demo";

void spawnObject(moveit::planning_interface::PlanningSceneInterface& psi, const moveit_msgs::CollisionObject& object)
{
  if (!psi.applyCollisionObject(object))
    throw std::runtime_error("Failed to spawn object: " + object.id);
}

moveit_msgs::CollisionObject createTable()
{
  ros::NodeHandle pnh("~");
  std::string table_name, table_reference_frame;
  std::vector<double> table_dimensions;
  geometry_msgs::Pose pose;

  std::size_t errors = 0;
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_name", table_name);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_reference_frame", table_reference_frame);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_dimensions", table_dimensions);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_pose", pose);
  rosparam_shortcuts::shutdownIfError(LOGNAME, errors);

  moveit_msgs::CollisionObject object;
  object.id = table_name;
  object.header.frame_id = table_reference_frame;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  object.primitives[0].dimensions = table_dimensions;
  pose.position.z -= 0.5 * table_dimensions[2];  // align surface with world
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::CollisionObject::ADD;

  return object;
}

moveit_msgs::CollisionObject createcaffetable()
{
  ros::NodeHandle pnh("~");
  std::string caffe_table_name, table_reference_frame;
  std::vector<double> caffe_table_dimensions;
  geometry_msgs::Pose caffe_pose;
  std::size_t errors = 0;
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "caffe_table_name", caffe_table_name);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_reference_frame", table_reference_frame);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "caffe_table_dimensions", caffe_table_dimensions);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, "caffe_table_pose", caffe_pose);
  rosparam_shortcuts::shutdownIfError(LOGNAME, errors);

  moveit_msgs::CollisionObject object;
  object.id = caffe_table_name;
  object.header.frame_id = table_reference_frame;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  object.primitives[0].dimensions = caffe_table_dimensions;
  caffe_pose.position.z -= 0.5 * caffe_table_dimensions[2];  // align surface with world
  object.primitive_poses.push_back(caffe_pose);
  object.operation = moveit_msgs::CollisionObject::ADD;

  return object;
}


// void doMsg(const geometry_msgs::Pose::ConstPtr& msg_p){
//     ROS_INFO("x: %f",msg_p->position.x);
//     ROS_INFO("y: %f",msg_p->position.y);
//     ROS_INFO("z: %f",msg_p->position.z);
//     ros::NodeHandle pnh("~");
//     std::string cylinder_name, object_reference_frame;
//     std::vector<double> cylinder_dimensions;
//     moveit::planning_interface::PlanningSceneInterface psi;
//     geometry_msgs::Pose pose;
//     std::size_t error = 0;
//     error += !rosparam_shortcuts::get(LOGNAME, pnh, "cylinder_name", cylinder_name);
//     error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_reference_frame", object_reference_frame);
//     error += !rosparam_shortcuts::get(LOGNAME, pnh, "cylinder_dimensions", cylinder_dimensions);
//     rosparam_shortcuts::shutdownIfError(LOGNAME, error);
//     pose.position.x = msg_p->position.x;
//     pose.position.y = msg_p->position.y;
//     pose.position.z = msg_p->position.z;

//     moveit_msgs::CollisionObject object;
//     object.id = cylinder_name;
//     object.header.frame_id = object_reference_frame;
//     object.primitives.resize(1);
//     object.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
//     object.primitives[0].dimensions = cylinder_dimensions;
//     pose.position.z += 0.5 * cylinder_dimensions[0];
//     object.primitive_poses.push_back(pose);
//     object.operation = moveit_msgs::CollisionObject::ADD;
//     spawnObject(psi, object);
// }

moveit_msgs::CollisionObject createCylinder()
{
  ros::NodeHandle pnh("~");
  std::string cylinder_name, object_grasp_reference_frame;
  std::vector<double> cylinder_dimensions;
  geometry_msgs::Pose pose;
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "cylinder_name", cylinder_name);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_grasp_reference_frame", object_grasp_reference_frame);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "cylinder_dimensions", cylinder_dimensions);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "cylinder_pose", pose);
  rosparam_shortcuts::shutdownIfError(LOGNAME, error);

  moveit_msgs::CollisionObject object;
  object.id = cylinder_name;
  object.header.frame_id = object_grasp_reference_frame;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = cylinder_dimensions;
  pose.position.z += 0.5 * cylinder_dimensions[0];
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::CollisionObject::ADD;

  return object;
}

void sub_cylinder()
{
  boost::shared_ptr<geometry_msgs::Pose const>  msg = ros::topic::waitForMessage<geometry_msgs::Pose>("/objection_position_pose", ros::Duration(5));
  ROS_INFO("x: %f",msg->position.x);
  ROS_INFO("y: %f",msg->position.y);
  ROS_INFO("z: %f",msg->position.z);
  ros::NodeHandle pnh("~");
  std::string cylinder_name, object_grasp_reference_frame;
  std::vector<double> cylinder_dimensions;
  moveit::planning_interface::PlanningSceneInterface psi;
  geometry_msgs::Pose pose;
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "cylinder_name", cylinder_name);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_grasp_reference_frame", object_grasp_reference_frame);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "cylinder_dimensions", cylinder_dimensions);
  rosparam_shortcuts::shutdownIfError(LOGNAME, error);
  pose.position.x = msg->position.x;
  pose.position.y = msg->position.y;
  pose.position.z = msg->position.z;

  moveit_msgs::CollisionObject object;
  object.id = cylinder_name;
  object.header.frame_id = object_grasp_reference_frame;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = cylinder_dimensions;
  // pose.position.z += 0.5 * cylinder_dimensions[0];
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::CollisionObject::ADD;
  spawnObject(psi, object);
}

moveit_msgs::CollisionObject createCube()
{
  ros::NodeHandle pnh("~");
  std::string cube_name, object_grasp_reference_frame;
  std::vector<double> cube_dimensions;
  geometry_msgs::Pose cube_pose;
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "cube_name", cube_name);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_grasp_reference_frame", object_grasp_reference_frame);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "cube_dimensions", cube_dimensions);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "cube_pose", cube_pose);
  rosparam_shortcuts::shutdownIfError(LOGNAME, error);

  moveit_msgs::CollisionObject object;
  object.id = cube_name;
  object.header.frame_id = object_grasp_reference_frame;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  object.primitives[0].dimensions = cube_dimensions;
  cube_pose.position.z += 0.5 * cube_dimensions[2];
  object.primitive_poses.push_back(cube_pose);
  object.operation = moveit_msgs::CollisionObject::ADD;

  return object;
}

moveit_msgs::CollisionObject createCamera()
{
  ros::NodeHandle pnh("~");
  std::string camera_name, camera_reference_frame, camera_mesh_file;
  geometry_msgs::Pose pose;
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "camera_name", camera_name);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "camera_mesh_file", camera_mesh_file);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "camera_reference_frame", camera_reference_frame);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "camera_pose", pose);
  rosparam_shortcuts::shutdownIfError(LOGNAME, error);

  shapes::Mesh* obj_mesh = shapes::createMeshFromResource(camera_mesh_file);

  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape(obj_mesh, mesh_msg);
  shape_msgs::Mesh mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

  moveit_msgs::CollisionObject object;
  object.id = camera_name;
  object.header.frame_id = camera_reference_frame;
  object.meshes.emplace_back(mesh);
  object.mesh_poses.emplace_back(pose);
  object.operation = moveit_msgs::CollisionObject::ADD;

  return object;
}

moveit_msgs::CollisionObject createObjectMesh()
{
  ros::NodeHandle pnh("~");
  std::string cylinder_name, object_grasp_reference_frame, object_mesh_file;
  std::vector<double> cylinder_dimensions;
  geometry_msgs::Pose pose;
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "cylinder_name", cylinder_name);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_mesh_file", object_mesh_file);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_grasp_reference_frame", object_grasp_reference_frame);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "cylinder_dimensions", cylinder_dimensions);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "cylinder_pose", pose);
  rosparam_shortcuts::shutdownIfError(LOGNAME, error);

  shapes::Mesh* obj_mesh = shapes::createMeshFromResource(object_mesh_file);

  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape(obj_mesh, mesh_msg);
  shape_msgs::Mesh mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

  moveit_msgs::CollisionObject object;
  object.id = cylinder_name;
  object.header.frame_id = object_grasp_reference_frame;
  object.meshes.emplace_back(mesh);
  object.mesh_poses.emplace_back(pose);
  object.operation = moveit_msgs::CollisionObject::ADD;

  // moveit_msgs::CollisionObject object;
  // object.id = cylinder_name;
  // object.header.frame_id = object_reference_frame;
  // object.primitives.resize(1);
  // object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  // object.primitives[0].dimensions = cylinder_dimensions;
  // pose.position.z += 0.5 * cylinder_dimensions[0];
  // object.primitive_poses.push_back(pose);
  // object.operation = moveit_msgs::CollisionObject::ADD;

  return object;
}
void move_base_to_target()
{
  ros::NodeHandle pnh("~");
  std::string move_base_reference_frame;
  double move_base_x, move_base_y, move_base_yaw;
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "move_base_reference_frame", move_base_reference_frame);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "move_base_x", move_base_x);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "move_base_y", move_base_y);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "move_base_yaw", move_base_yaw);

  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  double x_goal = move_base_x; 
  double y_goal = move_base_y;  
  MoveBaseClient client("move_base", true);
  client.waitForServer();
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = move_base_reference_frame;  
  goal.target_pose.pose.position.x = x_goal;
  goal.target_pose.pose.position.y = y_goal;
  tf2::Quaternion quat;
  quat.setRPY(0, 0, move_base_yaw);  
  goal.target_pose.pose.orientation = tf2::toMsg(quat); 

  client.sendGoal(goal);
  client.waitForResult();

  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("succeed to target");
  } else {
      ROS_INFO("failed to target");
  }
}


int main(int argc, char** argv)
{
  ROS_INFO_NAMED(LOGNAME, "Init deep_grasp_demo");
  ros::init(argc, argv, "deep_grasp_demo");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Wait for ApplyPlanningScene service
  ros::Duration(1.0).sleep();

  // Add table and object to planning scene
  moveit::planning_interface::PlanningSceneInterface psi;
  ros::NodeHandle pnh("~");
  if (pnh.param("spawn_table", true))
  {
    spawnObject(psi, createTable());
  }

  // Add camera to planning scene
  if (pnh.param("spawn_camera", true))
  {
    spawnObject(psi, createCamera());
  }

  // Add object to planning scene either as mesh or geometric primitive
  if (pnh.param("spawn_cylinder", true))
  {
    sub_cylinder(); 
    // spawnObject(psi, createCylinder());
  }


    if (pnh.param("spawn_cube", true))
  {
    spawnObject(psi, createCube());
  }

    if(pnh.param("spawn_mesh", true))
  {
    spawnObject(psi, createObjectMesh());
  }
      if(pnh.param("spawn_caffe_table", true))
  {
    spawnObject(psi, createcaffetable());
  }

  deep_grasp_task::DeepPickPlaceTask deep_pick_place_task_grasp_move_place("deep_pick_place_task_grasp_move_place", nh);
  while(true)
  {
        // Construct and run task
      deep_pick_place_task_grasp_move_place.loadParameters();
      
      deep_pick_place_task_grasp_move_place.grasp_cylinder();
      if (deep_pick_place_task_grasp_move_place.plan())
    {
      ROS_INFO_NAMED(LOGNAME, "Planning succeded");
      if (pnh.param("execute", true))
      {
        deep_pick_place_task_grasp_move_place.execute();
        ROS_INFO_NAMED(LOGNAME, "Execution complete");
        break;
      }
      else
      {
        ROS_INFO_NAMED(LOGNAME, "Execution disabled");
      }
    }
    else
    {
      ROS_INFO_NAMED(LOGNAME, "Planning failed");
    }
  }
  
    while(true)
  { 
      deep_pick_place_task_grasp_move_place.grasp_state();
      if (deep_pick_place_task_grasp_move_place.plan())
    {
      ROS_INFO_NAMED(LOGNAME, "Planning succeded");
      if (pnh.param("execute", true))
      {
        deep_pick_place_task_grasp_move_place.execute();
        ROS_INFO_NAMED(LOGNAME, "Execution complete");
        break;
      }
      else
      {
        ROS_INFO_NAMED(LOGNAME, "Execution disabled");
      }
    }
    else
    {
      ROS_INFO_NAMED(LOGNAME, "Planning failed");
    }
  }
   move_base_to_target();
    while(true)
  { 
      deep_pick_place_task_grasp_move_place.place_cylinder();
      if (deep_pick_place_task_grasp_move_place.plan())
    {
      ROS_INFO_NAMED(LOGNAME, "Planning succeded");
      if (pnh.param("execute", true))
      {
        deep_pick_place_task_grasp_move_place.execute();
        ROS_INFO_NAMED(LOGNAME, "Execution complete");
        break;
      }
      else
      {
        ROS_INFO_NAMED(LOGNAME, "Execution disabled");
      }
    }
    else
    {
      ROS_INFO_NAMED(LOGNAME, "Planning failed");
    }
  }
  // Keep introspection alive
  ros::waitForShutdown();
  return 0;
}
