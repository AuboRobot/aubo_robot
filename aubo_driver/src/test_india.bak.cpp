/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017-2018, AUBO Robotics
 * All rights reserved.
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
//#include <ros/ros.h>

//int main(int argc, char **argv)
//{
//  ros::init(argc, argv, "test_india");
//  ros::NodeHandle nh;

//  ROS_INFO("Hello world!");
//}


/* Author: Anirudh, Akash
Priting current end effector pose w.r.t world frame */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/collision_detection/collision_matrix.h>
#include <geometric_shapes/shape_operations.h>

#include <ros/console.h>
#include <ros/console.h>

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include "tf/LinearMath/Transform.h"

#include "octomap_msgs/conversions.h"
#include "geometry_msgs/Quaternion.h"
#include <tf/transform_datatypes.h>
#include <moveit/robot_state/conversions.h>

#define EXECUTION_ITERATIONS                  (1)

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_movement_with_constraint");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    ros::console::notifyLoggerLevelsChanged();

    static const std::string PLANNING_GROUP = "move_arm";     //might vary for different arm models.
    ROS_INFO_NAMED("arm_movement_with_constraint","STARTING MOVEMENT SCRIPT");

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;


    ROS_INFO("Reference frame: %s", move_group.getPlanningFrame().c_str());

    ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());

    move_group.setPlannerId("RRTConnectkConfigDefault");  // DO not change this.

    move_group.setPlanningTime(10);

    move_group.setMaxVelocityScalingFactor(1);
    move_group.setMaxAccelerationScalingFactor(1);

    move_group.setPoseReferenceFrame( move_group.getPlanningFrame());

    bool success ;
    moveit_msgs::MoveItErrorCodes pogp_success;

    std::vector< double > test_point_1 ;
    test_point_1.push_back(0.3);
    test_point_1.push_back(3.045322);
    test_point_1.push_back(0.544908);
    test_point_1.push_back(-1.884432);
    test_point_1.push_back(0.749672);
    test_point_1.push_back(1.410019);
    test_point_1.push_back(0.000000);

    std::vector< double > test_point_2 ;
    test_point_2.push_back(-0.3);
    test_point_2.push_back(3.045322);
    test_point_2.push_back(0.544908);
    test_point_2.push_back(-1.884432);
    test_point_2.push_back(0.749672);
    test_point_2.push_back(1.410019);
    test_point_2.push_back(0.0);

    int plan_number = 0 ;

    move_group.setGoalTolerance(0.001);
    move_group.setJointValueTarget(test_point_1);
    pogp_success = move_group.move();
    if(pogp_success.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      robot_state::RobotState ds(*move_group.getCurrentState());
      const double j1=test_point_2[0];
      const double j2=test_point_2[1];
      const double j3=test_point_2[2];
      const double j4=test_point_2[3];
      const double j5=test_point_2[4];
      const double j6=test_point_2[5];
      const double j7=test_point_2[6];

      ds.setJointPositions("arm_mount_joint",&j1);
      ds.setJointPositions("shoulder_joint",&j2);
      ds.setJointPositions("upperArm_joint",&j3);
      ds.setJointPositions("foreArm_joint",&j4);
      ds.setJointPositions("wrist1_joint",&j5);
      ds.setJointPositions("wrist2_joint",&j6);
      ds.setJointPositions("wrist3_joint",&j7);

      moveit_msgs::RobotState dummy_state;
      robotStateToRobotStateMsg(ds,dummy_state,true);
      dummy_state.is_diff=true;
      move_group.setStartState(dummy_state);

      moveit::planning_interface::MoveGroupInterface::Plan test_plan_1 ;
      moveit::planning_interface::MoveGroupInterface::Plan test_plan_2 ;


      move_group.setGoalTolerance(0.001);
      move_group.setJointValueTarget(test_point_1);
      pogp_success = move_group.plan(test_plan_1);
      if(pogp_success.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
      {
        ROS_INFO("Plan successfull for plan 1!!!");

        robot_state::RobotState ds(*move_group.getCurrentState());
        const double j1=test_point_1[0];
        const double j2=test_point_1[1];
        const double j3=test_point_1[2];
        const double j4=test_point_1[3];
        const double j5=test_point_1[4];
        const double j6=test_point_1[5];
        const double j7=test_point_1[6];

        ds.setJointPositions("arm_mount_joint",&j1);
        ds.setJointPositions("shoulder_joint",&j2);
        ds.setJointPositions("upperArm_joint",&j3);
        ds.setJointPositions("foreArm_joint",&j4);
        ds.setJointPositions("wrist1_joint",&j5);
        ds.setJointPositions("wrist2_joint",&j6);
        ds.setJointPositions("wrist3_joint",&j7);

        moveit_msgs::RobotState dummy_state;
        robotStateToRobotStateMsg(ds,dummy_state,true);
        dummy_state.is_diff=true;
        move_group.setStartState(dummy_state);

        move_group.setGoalTolerance(0.001);
        move_group.setJointValueTarget(test_point_2);
        pogp_success = move_group.plan(test_plan_2);
        if(pogp_success.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
          ROS_INFO("Plan successfull for plan 2!!!");

          while(ros::ok() || (plan_number < EXECUTION_ITERATIONS))
          {
            robot_state::RobotState get_position1(*move_group.getCurrentState());
            std::cout << "get_position1: "<< *get_position1.getJointPositions("arm_mount_joint")<<std::endl;
            move_group.setStartState(get_position1);
            move_group.setGoalTolerance(0.001);
            move_group.setJointValueTarget(test_point_2);
            pogp_success = move_group.plan(test_plan_2);
            pogp_success = move_group.execute(test_plan_2);

            if(pogp_success.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
            {
              ROS_INFO("Execution success for plan 1!!!!");

              ros::Duration(0.1).sleep();

              robot_state::RobotState get_position2(*move_group.getCurrentState());
              std::cout << "get_position2: "<< *get_position2.getJointPositions("arm_mount_joint")<<std::endl;
              move_group.setStartState(get_position2);
              move_group.setGoalTolerance(0.001);
              move_group.setJointValueTarget(test_point_1);
              pogp_success = move_group.plan(test_plan_1);
              pogp_success = move_group.execute(test_plan_1);

              if(pogp_success.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
              {
                ROS_INFO("Execution success for plan 2!!!!");
                plan_number += 1 ;
                ros::Duration(0.1).sleep();
              }
              else
              {
                ROS_ERROR("Execution failed for plan 2!!!");
                break ;
              }
            }
            else
            {
              ROS_ERROR("Execution failed for plan 1!!!");
              break ;
            }
          }
        }
        else
        {
          ROS_ERROR("Plan failed for plan 2!!!");
        }
      }
      else
      {
        ROS_ERROR("Plan failed for plan 1!!!");
      }
    }
    else
    {
      ROS_ERROR("Failure in planning and execution of home plan");
    }

  ros::shutdown();
  return 0;

}


