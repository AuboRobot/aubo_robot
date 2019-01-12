#include "ros/ros.h"
#include <aubo_msgs/ExtMove.h>
#include <aubo_msgs/ToolDynamicParam.h>
#include <cstdlib>

/**
 * Terminal command:
 * stop command:   ./testExtServer 0
 * move command:   ./testExtServer 1
 * reset command:   ./testExtServer 2
 *
 */

int main(int argc, char **argv)
{
  ros::init(argc, argv, "testExtServer");

  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<aubo_msgs::ExtMove>("/aubo_driver/ExtAxle_Driver");

  aubo_msgs::ExtMove srv;
  srv.request.maxAcc.resize(2);
  srv.request.maxVelc.resize(2);
  srv.request.ext_id = 1;
  srv.request.ext_status = atoll(argv[1]);
  srv.request.maxAcc[0] = 10;
  srv.request.maxAcc[1] = 10;
  srv.request.maxVelc[0] = 0.02;//0.02  0.5
  srv.request.maxVelc[1] = 0.02;
  srv.request.dirtion = false;



  ROS_INFO("Ext_status:%d",srv.request.ext_status);

  if (client.call(srv))
  {
    if(true == srv.response.result)
    {
      ROS_INFO("result: %s", "true");
    }
    else
    {
      ROS_INFO("result: %s", "failed");
    }
  }
  else
  {
    ROS_ERROR("Failed to call service ExtAxle_Driver");
    return 1;
  }

  return 0;
}
