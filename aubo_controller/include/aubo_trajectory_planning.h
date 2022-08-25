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
#ifndef AUBO_TRAJECTORY_PLANNING_H
#define AUBO_TRAJECTORY_PLANNING_H

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32MultiArray.h>
#include <queue>
#include <pthread.h>
namespace aubo_trajectory_planning {
class aubo_Trajectory_Planning_Class
{
public:
  aubo_Trajectory_Planning_Class(ros::NodeHandle *NH);
  bool isMotion();
  void stop_interpolation();

  void trajectoryCallBackFromMoveit(const trajectory_msgs::JointTrajectory::ConstPtr &msg);
  void Order_trajectoryPoints(trajectory_msgs::JointTrajectory *m_inPoint, trajectory_msgs::JointTrajectory &m_outPoint);
  void enqueue_trajectoryPlanPoints(trajectory_msgs::JointTrajectory *point);
  void interpolationTrajectory();
  void checkDriverMemory();
  void jointStatePublic(double *joint_Position);
  void ribStatusCallback(const std_msgs::Int32MultiArray::ConstPtr &msg);

private:
  ros::NodeHandle nodehandle;
  ros::Subscriber joint_path_sub;
  ros::Subscriber rib_status_sub;
  ros::Publisher  joint_interpolation_pub_;

  std::vector<std::string> Joint_names_from_ParamServer;
  std::queue<trajectory_msgs::JointTrajectoryPoint> motion_buffer_;

  pthread_mutex_t  mutex_motion_buffer;
  pthread_mutex_t  mutex_driver_buffer;
  pthread_mutex_t  mutex_log_sdk_status;

  int  driver_buffer_size_;
  bool log_sdk_status_;
};
}
#endif // AUBO_TRAJECTORY_PLANNING_H
