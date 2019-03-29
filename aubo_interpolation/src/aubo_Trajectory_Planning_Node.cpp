#include <ros/ros.h>
#include <aubo_trajectory_planning_class.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aubo_Trajectory_Planning_Node");
  ros::NodeHandle nh;

  //start three thread to publish or subcribe topic
  ros::AsyncSpinner spinner(4);
  spinner.start();

  std::cout << "start run aubo_Trajectory_Planning_Node " <<std::endl;

  aubo_Trajectory_Planning_Class *object = new aubo_Trajectory_Planning_Class(&nh);


  ros::waitForShutdown();

  ROS_INFO("Hello world!");
}
