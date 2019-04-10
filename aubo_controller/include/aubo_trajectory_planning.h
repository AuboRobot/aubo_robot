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
