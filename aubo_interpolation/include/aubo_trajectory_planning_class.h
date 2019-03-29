#ifndef AUBO_TRAJECTORY_PLANNING_CLASS_H
#define AUBO_TRAJECTORY_PLANNING_CLASS_H

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <queue>
#include <thread>
#include <mutex>

class aubo_Trajectory_Planning_Class
{
public:
  aubo_Trajectory_Planning_Class(ros::NodeHandle *NH);
  void trajectoryCallBackFromMoveit(const trajectory_msgs::JointTrajectory::ConstPtr &msg);
  void Order_trajectoryPoints(trajectory_msgs::JointTrajectory *m_inPoint, trajectory_msgs::JointTrajectory &m_outPoint);
  void enqueue_trajectoryPlanPoints(trajectory_msgs::JointTrajectory *point);
  void interpolationTrajectory();
  void jointStatePublic(double *joint_Position);

private:
  ros::NodeHandle nodehandle;
  ros::Subscriber joint_path_sub;
  ros::Publisher  joint_interpolation_pub_;
  std::vector<std::string> Joint_names_from_ParamServer;
  std::queue<trajectory_msgs::JointTrajectoryPoint> motion_buffer_;
  std::thread *calculation_thread;
  std::mutex  motion_buffer_mutex;
};

#endif // AUBO_TRAJECTORY_PLANNING_CLASS_H
