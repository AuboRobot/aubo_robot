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



int main(int argc, char** argv)
{
  ros::init(argc, argv, "MoveGroupInterface_To_Kinetic");
  ros::NodeHandle node_handle;

  // Start a thread
  ros::AsyncSpinner spinner(1);
  spinner.start();


  // Define the planning group name
  static const std::string PLANNING_GROUP = "manipulator_i5";


  // Create a planning group interface object and set up a planning group
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  move_group.setPoseReferenceFrame("base_link");


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


  //**************************************************************************************************   First example: planning and moving to a target pose
  // Set the target pose , RPY mode (rotation around the reference axis X, Y, Z)
  tf::Quaternion q;
  q.setRPY(3.14,0,-1.57);       //radian

  geometry_msgs::Pose target_pose1;
  target_pose1.position.x = -0.4;
  target_pose1.position.y = -0.3;
  target_pose1.position.z = 0.30;
  target_pose1.orientation.x = q.x();
  target_pose1.orientation.y = q.y();
  target_pose1.orientation.z = q.z();
  target_pose1.orientation.w = q.w();

  move_group.setPoseTarget(target_pose1);

  // Call the planner for planning calculations Note: This is just planning
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "Success" : "FAILED");


  // visual planning path in Rviz
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "AUBO Pose Goal Example1", rvt::RED, rvt::XLARGE);
  // Parameter 1 (trajectory_): path information
  // Parameter 2 (JointModelGroup): Joint angle information and arm model information of the initial pose
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();


  // Perform planning actions
  move_group.execute(my_plan);


  // Move to the home point position
  move_group.setJointValueTarget(home_position);
  move_group.move();

  visual_tools.prompt("Press 'next'2 in the RvizVisualToolsGui window to continue the demo");







  // **************************************************************************************************    The second example, the joint 1 is rotated 90 degrees based on the home position.

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

  // Move to the home point position
  joint_group_positions[0] = 0;  // radians
  move_group.setJointValueTarget(joint_group_positions);
  move_group.move();

  visual_tools.prompt("Press 'next'3 in the RvizVisualToolsGui window to continue the demo");








  // **************************************************************************************************   第三实例：机器臂根据设置的路径约束从起始点运动到初始点
  // **************************************************************************************************   The third example: the robot arm moves from point A to point B according to the set path constraint.

  q.setRPY(3.14,0,-1.57);

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



  // Set initial position
  robot_state::RobotState start_state(*move_group.getCurrentState());
  geometry_msgs::Pose     start_pose2;
  start_pose2.position.x = -0.4;
  start_pose2.position.y = 0.05;
  start_pose2.position.z = 0.54;
  start_pose2.orientation.x = q.x();
  start_pose2.orientation.y = q.y();
  start_pose2.orientation.z = q.z();
  start_pose2.orientation.w = q.w();

  // 机械臂首先要运动到起始位置
  // the robot must first move to the starting position
  move_group.setPoseTarget(start_pose2);
  move_group.move();


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


  // Move to the home point position
  move_group.setPoseTarget(start_pose2);
  move_group.move();

  visual_tools.prompt("next step 4");


  // Clear path constraint
  move_group.clearPathConstraints();







  // **************************************************************************************************   Fourth example: planning and moving a Cartesian interpolation path

  //  Add three waypoints
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

  visual_tools.prompt("Press 'next'5  ADD a OBject in the RvizVisualToolsGui window to continue the demo");








  // **************************************************************************************************   Fifth example: adding obstacles

  // Define a collision object
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();


  // Set the ID of the object
  collision_object.id = "box1";


  // ********************************Defining a box (obstacle) added to the world
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.4;
  primitive.dimensions[1] = 0.05;
  primitive.dimensions[2] = 0.4;

  // 设置盒子(障碍物)的位置和姿态
  // Set the box (obstacle) pose
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = -0.3;
  box_pose.position.y = 0.2;
  box_pose.position.z = 0.54;


  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;


  // *********************************Defining a box (desktop) added in the world
  moveit_msgs::CollisionObject collision_object2;
  collision_object2.header.frame_id = move_group.getPlanningFrame();

  // Set the ID of the object
  collision_object2.id = "box2";


  // Set length, width and height
  shape_msgs::SolidPrimitive primitive2;
  primitive2.type = primitive.BOX;
  primitive2.dimensions.resize(3);
  primitive2.dimensions[0] = 1.7;
  primitive2.dimensions[1] = 1.7;
  primitive2.dimensions[2] = 0.05;


  // Set the box (Desktop) pose
  geometry_msgs::Pose box_pose2;
  box_pose2.orientation.w = 1.0;
  box_pose2.position.x = 0.0;
  box_pose2.position.y = 0.0;
  box_pose2.position.z = 0.0;

  collision_object2.primitives.push_back(primitive2);
  collision_object2.primitive_poses.push_back(box_pose2);
  collision_object2.operation = collision_object2.ADD;


  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  collision_objects.push_back(collision_object2);

  planning_scene_interface.addCollisionObjects(collision_objects);


  // Show text in RViz of status
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "AUBO Add object Example5", rvt::RED, rvt::XLARGE);
  visual_tools.trigger();

  visual_tools.prompt("Press 'next'6 in the RvizVisualToolsGui window to once the collision object appears in RViz");



  // **************************************************************************************************   Sixth example: obstacle avoidance movement


  // Add a track to avoid obstacle movement
  q.setRPY(1.77,-0.59,-1.79 );                       // radian

  move_group.setStartState(*move_group.getCurrentState());
  geometry_msgs::Pose another_pose;
  another_pose.orientation.x = q.x();
  another_pose.orientation.y = q.y();
  another_pose.orientation.z = q.z();
  another_pose.orientation.w = q.w();
  another_pose.position.x = -0.37;
  another_pose.position.y = 0.6;
  another_pose.position.z = 0.4;
  move_group.setPoseTarget(another_pose);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (pose goal move around cuboid) %s", success ? "" : "FAILED");


  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "AUBO Obstacle Goal Exalmple6", rvt::RED, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();


  // Perform planning actions
  move_group.execute(my_plan);


  // Move to the home point position
  move_group.setJointValueTarget(home_position);
  move_group.move();
  visual_tools.prompt("next step 7 attach the collision object to the robot");



  // **************************************************************************************************   Seventh example: Simulated collision

  ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
  move_group.attachObject(collision_object.id);

  visual_tools.publishText(text_pose, "AUBO Object attached to robot Example7", rvt::RED, rvt::XLARGE);
  visual_tools.trigger();

  visual_tools.prompt("Press 'next'8 in the RvizVisualToolsGui window to once the collision object attaches to the "
                      "robot");



  // **************************************************************************************************   Eighth example: Simulated no collision

  ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
  move_group.detachObject(collision_object.id);

  visual_tools.publishText(text_pose, "AUBO Object dettached from robot Example8", rvt::RED, rvt::XLARGE);
  visual_tools.trigger();

  visual_tools.prompt("Press 'next'9 in the RvizVisualToolsGui window to once the collision object detaches to the "
                      "robot");




  //**************************************************************************************************    Ninth example: removing obstacles


  // Remove obstacles
  ROS_INFO_NAMED("tutorial", "Remove the object from the world");
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "AUBO Object1 removed EXample9", rvt::RED, rvt::XLARGE);
  visual_tools.trigger();



   //**************************************************************************************************    Tenth example: random motion (screened, users need to be able to open it themselves. Tip: random motion is uncertain, users need to hold the emergency stop to pay attention to safety)



//   visual_tools.deleteAllMarkers();
//   visual_tools.publishText(text_pose, "AUBO Random Move Exalmple 10", rvt::RED, rvt::XLARGE);
//   visual_tools.trigger();

//   for(int i=0;i<30;i++)
//   {

//     move_group.setRandomTarget();

//     move_group.move();

//     ROS_INFO_NAMED("tutorial", "Random Moving %d:",i);

//   }
//   visual_tools.prompt("Press 'next'10 : Robot Random Moving");


   // Remove the desktop
   ROS_INFO_NAMED("tutorial", "Remove the desktop from the world");
   object_ids.push_back(collision_object2.id);
   planning_scene_interface.removeCollisionObjects(object_ids);
   // Show text in RViz of status
   visual_tools.deleteAllMarkers();
   visual_tools.publishText(text_pose, " Finish ", rvt::RED, rvt::XLARGE);
   visual_tools.trigger();


   // END_TUTORIAL
   ros::shutdown();
   return 0;
}
