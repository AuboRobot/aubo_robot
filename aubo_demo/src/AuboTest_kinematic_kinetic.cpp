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
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf/LinearMath/Quaternion.h>
#include <moveit_msgs/RobotState.h>
#include <sys/time.h>


#define PI 3.1415926

#define AUBO_i3 0
#define AUBO_i5 1
#define AUBO_i10 0

const double I3_PoseZ_Limit = 0.7;
const double I5_PoseZ_Limit = 1.0;
const double I10_PoseZ_Limit = 1.2;

void GetCurrentPoseInfo(moveit::planning_interface::MoveGroupInterface *move_group)
{
  geometry_msgs::PoseStamped Cur_Pos;
  std::vector<double>        Cur_Joint;
  Cur_Joint.resize(6);

  Cur_Pos = move_group->getCurrentPose("wrist3_Link");

  Cur_Joint =  move_group->getCurrentJointValues();


  ROS_INFO_STREAM_NAMED("test","Current " <<
                               " X:"<<Cur_Pos.pose.position.x <<
                               " Y:"<<Cur_Pos.pose.position.y <<
                               " Z:"<<Cur_Pos.pose.position.z <<
                               " W:"<<Cur_Pos.pose.orientation.w <<
                               " X:"<<Cur_Pos.pose.orientation.x <<
                               " Y:"<<Cur_Pos.pose.orientation.y <<
                               " Z:"<<Cur_Pos.pose.orientation.z);

  ROS_INFO_STREAM_NAMED("test","Current " <<
                               " joint1:"<< Cur_Joint.at(0) <<
                               " joint2:"<< Cur_Joint.at(1) <<
                               " joint3:"<< Cur_Joint.at(2) <<
                               " joint4:"<< Cur_Joint.at(3) <<
                               " joint5:"<< Cur_Joint.at(4) <<
                               " joint6:"<< Cur_Joint.at(5));
}





int main(int argc, char** argv)
{
  ros::init(argc, argv, "Test_Aubokinematic");
  ros::NodeHandle node_handle;

  // Start a thread
  ros::AsyncSpinner spinner(1);
  spinner.start();

  geometry_msgs::PoseStamped random_pose;
  geometry_msgs::Pose target_pose;

  // Define the planning group name
#if AUBO_i3
  static const std::string PLANNING_GROUP = "manipulator_i3";
#endif

#if AUBO_i5
  static const std::string PLANNING_GROUP = "manipulator_i5";
#endif

#if AUBO_i10
  static const std::string PLANNING_GROUP = "manipulator_i10";
#endif

  struct timeval tv;
  char buf[64];
  gettimeofday(&tv, NULL);//获取系统时间
  strftime(buf, sizeof(buf)-1, "%Y-%m-%d %H:%M:%S", localtime(&tv.tv_sec));
  std::cout << "time: "<< buf <<"."<<(int)(tv.tv_usec / 1000)  << std::endl;


  // Create a planning group interface object and set up a planning group
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);


  move_group.setPoseReferenceFrame("world");


  // Create a robot model information object
  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  // Get the coordinate system of the basic information
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // Get the end of the basic information
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");

//  visual_tools.prompt(" move init position by joint ");
  usleep(1000*1000);



  //*****************************************************************  Init Position
  //AUBO
  std::vector<double> init_position;
  init_position.push_back(1.57);
  init_position.push_back(1.57);
  init_position.push_back(1.57);
  init_position.push_back(1.57);
  init_position.push_back(1.57);
  init_position.push_back(1.57);
  move_group.setJointValueTarget(init_position);
  move_group.move();


  ROS_INFO(" ");
  ROS_ERROR("----------init_position-----------------");
  GetCurrentPoseInfo(&move_group);
  ROS_ERROR("----------init_position-----------------");
  ROS_INFO(" ");

  // Visual terminal prompt (blocking)
  visual_tools.prompt(" move target position by joint ");
  usleep(1000*1000);

  /** *********************************************************************************************** **/







  //*****************************************************************  Init1 Position
  //AUBO
  std::vector<double> init_position1;
  init_position1.push_back(1.7635);
  init_position1.push_back(-1.28312);
  init_position1.push_back(-1.74926);
  init_position1.push_back(0.109193);
  init_position1.push_back(0.192703);
  init_position1.push_back(-1.57078);
  move_group.setJointValueTarget(init_position1);
  move_group.move();

  ROS_INFO(" ");
  ROS_ERROR("----------Target_position-----------------");
  GetCurrentPoseInfo(&move_group);
  ROS_ERROR("----------Target_position-----------------");
  ROS_INFO(" ");

  // Visual terminal prompt (blocking)
//  visual_tools.prompt(" move init position by Pose ");
  usleep(1000*1000);

  /** *********************************************************************************************** **/

  //Call the planner for planning calculations Note: This is just planning
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

//  tf::Quaternion q_t;
//  q_t.setRPY(PI,-PI/2,0);       //radian

//  //AUBO
//  geometry_msgs::Pose init_pose;
//  init_pose.position.x = 0.199963;
//  init_pose.position.y = 0.0986473;
//  init_pose.position.z = 0.547127;
//  init_pose.orientation.w = 0.688845;
//  init_pose.orientation.x = 0.0774001;
//  init_pose.orientation.y = 0.65757;
//  init_pose.orientation.z = 0.295133;


  //设置位置（单位：米）姿态（单位：弧度）允许误差  (与运动学逆解有关的容差,容差越大，离目标点越大)
  move_group.setGoalPositionTolerance(0.001);
  move_group.setGoalOrientationTolerance(0.001);
  move_group.setPlanningTime(5.0);

  while(ros::ok())
  {
    move_group.setStartState(*move_group.getCurrentState());

//  move_group.setPoseTarget(init_pose);
    random_pose = move_group.getRandomPose("wrist3_Link");
    target_pose.position.x = random_pose.pose.position.x;
    target_pose.position.y = random_pose.pose.position.y;
    target_pose.position.z = random_pose.pose.position.z;
    target_pose.orientation.w = random_pose.pose.orientation.w;
    target_pose.orientation.y = random_pose.pose.orientation.y;
    target_pose.orientation.z = random_pose.pose.orientation.z;
    target_pose.orientation.x = random_pose.pose.orientation.x;

    if(target_pose.position.z > I5_PoseZ_Limit)
    {
      ROS_INFO(" ");
      ROS_WARN("----------plan init_pose-----------------");

      gettimeofday(&tv, NULL);//获取系统时间
      strftime(buf, sizeof(buf)-1, "%Y-%m-%d %H:%M:%S", localtime(&tv.tv_sec));
      std::cout << "           time: "<< buf <<"."<<(int)(tv.tv_usec / 1000) << std::endl;

      ROS_INFO_STREAM_NAMED("test","Generate Random Pose "
                                   " X: "<<target_pose.position.x <<
                                   " Y: "<<target_pose.position.y <<
                                   " Z: "<<target_pose.position.z <<
                                   " W: "<<target_pose.orientation.w <<
                                   " X: "<<target_pose.orientation.x <<
                                   " Y: "<<target_pose.orientation.y <<
                                   " Z: "<<target_pose.orientation.z);
      move_group.setPoseTarget(target_pose);

      bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      if(true == success)
      {
        ROS_INFO_NAMED("tutorial", " plan init pose goal %s", success ? "Success" : "FAILED");

        // Perform planning actions
        move_group.execute(my_plan);

        usleep(1000*100);

        GetCurrentPoseInfo(&move_group);
        ROS_WARN("----------plan init_pose-----------------");
        ROS_INFO(" ");

        //Visual terminal prompt (blocking)
//      visual_tools.prompt(" move Target position by Pose");
      }
      else
      {
          ROS_ERROR_NAMED("tutorial", "init pose goal %s", success ? "Success" : "FAILED");
          return 0;
      }
    }
    else
    {
      ROS_ERROR_STREAM_NAMED("test"," target_pose.position.z < 0.7 "<<" |   target_pose.position.z:"<<target_pose.position.z);
    }
    usleep(1000*50);
  }
  // END_Test
  ros::shutdown();
  return 0;
}
