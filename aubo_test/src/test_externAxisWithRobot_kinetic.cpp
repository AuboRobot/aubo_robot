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
 */

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
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


  //***************************************************************************************************  Home Position

  robot_state::RobotState get_position(*move_group.getCurrentState());
  move_group.setStartState(get_position);

//  std::vector<double> home_position;
//  home_position.push_back(-0.001255);
//  home_position.push_back(-0.148822);
//  home_position.push_back(-1.406503);
//  home_position.push_back(0.311441);
//  home_position.push_back(-1.571295);
//  home_position.push_back(-0.002450);
//  move_group.setJointValueTarget(home_position);
//  move_group.move();

   //**************************************************************************************************     example: random motion (screened, users need to be able to open it themselves. Tip: random motion is uncertain, users need to hold the emergency stop to pay attention to safety)

   int  time = 0;
   bool isSuccess=false;
   moveit::planning_interface::MoveGroupInterface::Plan my_plan;
   geometry_msgs::Pose pose = move_group.getCurrentPose().pose;
   ROS_INFO("The Robot Position:%f  %f  %f",pose.position.x,pose.position.y,pose.position.z);

   move_group.setMaxVelocityScalingFactor(1);
   move_group.setMaxAccelerationScalingFactor(1);

   while(ros::ok())
   {
     robot_state::RobotState get_position1(*move_group.getCurrentState());

     move_group.setStartState(get_position1);
     move_group.setGoalTolerance(0.001);

     geometry_msgs::PoseStamped pose = move_group.getRandomPose("wrist3_Link");

     if(pose.pose.position.z < 2.0)
     {
       ROS_ERROR_STREAM_NAMED("test","Z: "<< pose.pose.position.z << "  Need restart compute !";);
     }
     else
     {
       ROS_ERROR_STREAM_NAMED("test","Z: "<< pose.pose.position.z << "  doing restart compute !!!!!!!!!!!!!!";);
       move_group.setPoseTarget(pose);

       isSuccess = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
       if(true == isSuccess)
       {
         ROS_WARN("move before");

         isSuccess = (move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
         if(true == isSuccess)
         {
           ROS_WARN("move after");
           time++;
           ROS_ERROR("set pose time :%d",time);
           sleep(0.5);
           ROS_INFO("waiting ***************");
         }
         else
         {
           ROS_ERROR("move failed");
         }

       }
       else
       {
         ROS_ERROR("plan failed");
       }
     }
   }

   // END_TUTORIAL
   ros::shutdown();
   return 0;
}
