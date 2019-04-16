/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017-2018, AUBO Robotics
 * All rights reserved.
 *
 *
 *  Author: zhaoyu
 *  email : zhaoyu@aubo-robotics.cn
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */




#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf/LinearMath/Quaternion.h>
#include <unistd.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_externAxisWithRobot_kinetic");
  ros::NodeHandle node_handle;

  // Start a thread
  ros::AsyncSpinner spinner(1);
  spinner.start();


  // Define the planning group name
  static const std::string PLANNING_GROUP = "move_arm";


  // Create a planning group interface object and set up a planning group
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);


  // Create a planning scene interface object
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


  // Create a robot model information object
  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


  // Create an object of the visualization class
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();


  // Load remote control tool
  visual_tools.loadRemoteControl();


  // Create text
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.2;
  visual_tools.publishText(text_pose, "AUBO Demo", rvt::RED, rvt::XLARGE);
  // Text visualization takes effect
  visual_tools.trigger();


  // Get the coordinate system of the basic information
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());


  // Get the end of the basic information
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());


  // Visual terminal prompt (blocking)
  visual_tools.prompt("Press 'next'1 in the RvizVisualToolsGui window to start the demo");

  //***************************************************************************************************  Home Position

  std::vector<double> home_position;
  home_position.push_back(-0.001255);
  home_position.push_back(-0.148822);
  home_position.push_back(-1.406503);
  home_position.push_back(0.311441);
  home_position.push_back(-1.571295);
  home_position.push_back(-0.002450);
  move_group.setJointValueTarget(home_position);
  move_group.move();



   //**************************************************************************************************    Tenth example: random motion (screened, users need to be able to open it themselves. Tip: random motion is uncertain, users need to hold the emergency stop to pay attention to safety)



   visual_tools.deleteAllMarkers();
   visual_tools.publishText(text_pose, "AUBO Random Move Exalmple 10", rvt::RED, rvt::XLARGE);
   visual_tools.trigger();

   geometry_msgs::PoseStamped pose;

   while(ros::ok())
   {

     move_group.setRandomTarget();

     move_group.move();

//     pose = move_group.getPoseTarget("wrist3_Link");

//     ROS_INFO("position :%f  %f  %f",pose.pose.position.x,pose.pose.position.y,pose.pose.position.z);

     sleep(0.1);
   }






   // END_TUTORIAL
   ros::shutdown();
   return 0;
}
