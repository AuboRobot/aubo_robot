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
#include <moveit/collision_detection_fcl/collision_robot_fcl.h>



#define Enable_Path_Constraint false
#define Enable_Cartesian_Path_Motion false

int main(int argc, char** argv)
{
  ros::init(argc, argv, "AuboTest_kinetic");
  ros::NodeHandle node_handle;

  // Start a thread
  ros::AsyncSpinner spinner(1);
  spinner.start();


  // Define the planning group name
  static const std::string PLANNING_GROUP = "manipulator_i5";


  // Create a planning group interface object and set up a planning group
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);


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
  text_pose.translation().z() = 1.3;
  visual_tools.publishText(text_pose, "AUBO Demo", rvt::RED, rvt::XLARGE);
  visual_tools.trigger();


  // Get the coordinate system of the basic information
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());


  // Get the end of the basic information
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // Visual terminal prompt (blocking)
  visual_tools.prompt("Press 'next'0 in the RvizVisualToolsGui window to move the position of home");

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

  // Visual terminal prompt (blocking)
  visual_tools.prompt("Press 'next'1 in the RvizVisualToolsGui window to motioning");


  //**************************************************************************************************   First example: planning and moving to a target pose (use trajectory plan)
  // Set the target pose , RPY mode (rotation around the reference axis X, Y, Z)
  tf::Quaternion q;
  q.setRPY(3.14,0,-1.57);       //radian

  geometry_msgs::Pose target_pose1;
  target_pose1.position.x = -0.3;
  target_pose1.position.y = -0.15;
  target_pose1.position.z = 0.30;
  target_pose1.orientation.x = q.x();
  target_pose1.orientation.y = q.y();
  target_pose1.orientation.z = q.z();
  target_pose1.orientation.w = q.w();

  geometry_msgs::Pose target_pose2;
  target_pose2.position.x = -0.6;
  target_pose2.position.y = -0.15;
  target_pose2.position.z = 0.30;
  target_pose2.orientation.x = q.x();
  target_pose2.orientation.y = q.y();
  target_pose2.orientation.z = q.z();
  target_pose2.orientation.w = q.w();

  geometry_msgs::Pose target_pose3;
  target_pose3.position.x = -0.6;
  target_pose3.position.y = 0.15;
  target_pose3.position.z = 0.30;
  target_pose3.orientation.x = q.x();
  target_pose3.orientation.y = q.y();
  target_pose3.orientation.z = q.z();
  target_pose3.orientation.w = q.w();

  geometry_msgs::Pose target_pose4;
  target_pose4.position.x = -0.3;
  target_pose4.position.y = 0.15;
  target_pose4.position.z = 0.30;
  target_pose4.orientation.x = q.x();
  target_pose4.orientation.y = q.y();
  target_pose4.orientation.z = q.z();
  target_pose4.orientation.w = q.w();

  int  tmp_count = 0;
  bool success = false;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  move_group.setMaxAccelerationScalingFactor(0.01);


  while(ros::ok())
  {
      robot_state::RobotState Cur_State(*move_group.getCurrentState());

      move_group.setStartState(Cur_State);

      tmp_count++;

      if(5 == tmp_count)
      {
          tmp_count = 1;
      }

      switch(tmp_count)
      {
        case 1: move_group.setPoseTarget(target_pose1);
                visual_tools.deleteAllMarkers();
                visual_tools.publishAxisLabeled(target_pose1, "pose");
                break;
        case 2: move_group.setPoseTarget(target_pose2);
                visual_tools.deleteAllMarkers();
                visual_tools.publishAxisLabeled(target_pose2, "pose");
                break;
        case 3: move_group.setPoseTarget(target_pose3);
                visual_tools.deleteAllMarkers();
                visual_tools.publishAxisLabeled(target_pose3, "pose");
                break;
        case 4: move_group.setPoseTarget(target_pose4);
                visual_tools.deleteAllMarkers();
                visual_tools.publishAxisLabeled(target_pose4, "pose");
                break;
      }


      // Call the planner for planning calculations Note: This is just planning
      success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "Success" : "FAILED");


      // visual planning path in Rviz
      visual_tools.publishText(text_pose, "AUBO Pose Goal Example1", rvt::RED, rvt::XLARGE);
      // Parameter 1 (trajectory_): path information
      // Parameter 2 (JointModelGroup): Joint angle information and arm model information of the initial pose
      visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
      visual_tools.trigger();

      // Perform planning actions
      move_group.execute(my_plan);


      // Move back the home point position
      move_group.setJointValueTarget(home_position);
      move_group.move();

      //visual_tools.prompt("Press 'next'2 in the RvizVisualToolsGui window to motioning");
//      fcl::DistanceResult dist_result;
//      double d = fcl::distance(o1, o2, fcl::DistanceRequest(true), dist_result);
//      std::cout << "d: " << d << std::endl <<
//                   "p1: " << dist_result.nearest_points[0] << std::endl <<
//                   "p2: " << dist_result.nearest_points[1] << std::endl;
      usleep(1000*200);
  }



  // **************************************************************************************************    The second example, the joint 1 is rotated 90 degrees based on the home position. (use trajectory plan)

  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  

  // Get the joint value and model information of the current group
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);


  // Modify the value of joint 1
  joint_group_positions[0] = -1.57;  // radians
  move_group.setJointValueTarget(joint_group_positions);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "success" : "FAILED");

  // Visual display in RVIZ
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "AUBO Joint Space Goal Example2", rvt::RED, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  // Perform planning actions
  move_group.execute(my_plan);

  // Move back the home point position
  joint_group_positions[0] = 0;  // radians
  move_group.setJointValueTarget(joint_group_positions);
  move_group.move();

  visual_tools.prompt("Press 'next'3 in the RvizVisualToolsGui window to motioning");





  // **************************************************************************************************   The third example: the robot arm moves from point A to point B according to the set path constraint. (use trajectory plan)

  q.setRPY(3.14,0,-1.57);
  geometry_msgs::Pose  start_pose2;
  // Set initial position
  start_pose2.position.x = -0.4;
  start_pose2.position.y = 0.05;
  start_pose2.position.z = 0.54;
  start_pose2.orientation.x = q.x();
  start_pose2.orientation.y = q.y();
  start_pose2.orientation.z = q.z();
  start_pose2.orientation.w = q.w();

  if(Enable_Path_Constraint)
  {
    // Define the path constraint
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "wrist3_Link";
    ocm.header.frame_id = "base_link";

    // Set the pose to be constrained at the end (consistent with base_link according to the settings)
    ocm.orientation.w = q.w();
    ocm.orientation.x = q.x();
    ocm.orientation.y = q.y();
    ocm.orientation.z = q.z();
    ocm.absolute_x_axis_tolerance = 0.2;   //(Specify the tolerance of the axis)
    ocm.absolute_y_axis_tolerance = 0.2;
    ocm.absolute_z_axis_tolerance = 0.2;
    ocm.weight = 1.0;


    // Add path constraints
    moveit_msgs::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    move_group.setPathConstraints(test_constraints);


    // the robot must first move to the starting position
    move_group.setPoseTarget(start_pose2);
    move_group.move();


    robot_state::RobotState start_state(*move_group.getCurrentState());
    // Reset the joint_model_group of the starting position for visual track display
    start_state.setFromIK(joint_model_group, start_pose2);
    move_group.setStartState(start_state);


    // Set the target pose
    geometry_msgs::Pose target_pose3_1;
    target_pose3_1.position.x = -0.4;
    target_pose3_1.position.y = -0.19;
    target_pose3_1.position.z = 0.41;
    target_pose3_1.orientation.x = q.x();
    target_pose3_1.orientation.y = q.y();
    target_pose3_1.orientation.z = q.z();
    target_pose3_1.orientation.w = q.w();
    move_group.setPoseTarget(target_pose3_1);


    // The default time for the kinematics solver calculation plan is 5s. Increasing the time can increase the success rate.
    move_group.setPlanningTime(20.0);

    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "success" : "FAILED");


    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(start_pose2, "start");
    visual_tools.publishAxisLabeled(target_pose3_1, "goal");
    visual_tools.publishText(text_pose, "AUBO Constrained Goal Example3", rvt::RED, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();


    // Perform planning actions
    move_group.execute(my_plan);


    // Move back the home point position and Clear path constraint
    move_group.clearPathConstraints();
    move_group.setPoseTarget(start_pose2);
    move_group.move();

    visual_tools.prompt("Press 'next'4 in the RvizVisualToolsGui window to motioning");
  }





  //**************************************************************************************************   Fourth example: planning and moving a Cartesian interpolation path (use trajectory plan)



  if(Enable_Cartesian_Path_Motion)
  {
      //Add three waypoints
      std::vector<geometry_msgs::Pose> waypoints;
      waypoints.push_back(start_pose2);

      geometry_msgs::Pose target_pose3 = start_pose2;

      target_pose3.position.z -= 0.2;
      waypoints.push_back(target_pose3);  // down

      target_pose3.position.y -= 0.15;
      waypoints.push_back(target_pose3);  // right

      target_pose3.position.z += 0.2;
      target_pose3.position.y += 0.2;
      target_pose3.position.x -= 0.2;
      waypoints.push_back(target_pose3);  // up and left


      // Reduce the speed of the robot arm by the scaling factor of the maximum speed of each joint. Please note that this is not the speed of the final effector.
      move_group.setMaxVelocityScalingFactor(0.5);



      // We want the Cartesian path to be interpolated at a resolution of 1 cm.
      moveit_msgs::RobotTrajectory trajectory;
      const double jump_threshold = 0.0;           //(The jump threshold is set to 0.0)
      const double eef_step = 0.01;                //(interpolation step)



      // Calculate Cartesian interpolation path: return path score (0~1, -1 stands for error)
      double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
      ROS_INFO_NAMED("tutorial", "Visualizing plan  (Cartesian path) (%.2f%% acheived)", fraction * 100.0);


      // Visualize the plan in RViz
      visual_tools.deleteAllMarkers();
      visual_tools.publishText(text_pose, "AUBO Joint Space Goal Example4", rvt::RED, rvt::XLARGE);
      visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
      for (std::size_t i = 0; i < waypoints.size(); ++i)
      {
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
      }
      visual_tools.trigger();


      // Move to the home point position
      my_plan.trajectory_= trajectory;
      move_group.execute(my_plan);


      // Clear path constraint
      move_group.setJointValueTarget(home_position);
      move_group.move();
  }

  visual_tools.prompt("Press 'next'5 in the RvizVisualToolsGui window to motioning");
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();


   // END_TUTORIAL
   ros::shutdown();
   return 0;
}
