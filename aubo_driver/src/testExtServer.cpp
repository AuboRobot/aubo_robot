#include "ros/ros.h"
#include <aubo_msgs/ExtMove.h>
#include <aubo_msgs/ClearExternError.h>
#include <aubo_msgs/ToolDynamicParam.h>
#include <std_msgs/Bool.h>
#include <cstdlib>

#include <industrial_msgs/RobotStatus.h>

/**
 * Terminal command:
 * stop command:   ./testExtServer 0
 * move command:   ./testExtServer 1
 * reset command:   ./testExtServer 2
 *
 */

int ExtAxis_errStatus = 0;

void test_alarm(const industrial_msgs::RobotStatus::ConstPtr &msg)
{
  ROS_INFO("alarm: %d",msg->in_error.val);
  ExtAxis_errStatus = msg->in_error.val;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "testExtServer");

  ros::NodeHandle nh;

  ros::Subscriber ExtAxis_Alarm = nh.subscribe("robot_status",100,&test_alarm);

  ros::ServiceClient clearExtErr_test = nh.serviceClient<aubo_msgs::ClearExternError>("/aubo_driver/server/ClearExternError");

  aubo_msgs::ClearExternError srv_clearExtErr;
  srv_clearExtErr.request.ExternAxis_id = 1;


// ros::ServiceClient client = nh.serviceClient<aubo_msgs::ExtMove>("/aubo_driver/ExtAxle_Driver");
//  aubo_msgs::ExtMove srv;
//  srv.request.maxAcc.resize(2);
//  srv.request.maxVelc.resize(2);
//  srv.request.ext_id = 1;
//  srv.request.ext_status = atoll(argv[1]);
//  srv.request.maxAcc[0] = 10;
//  srv.request.maxAcc[1] = 10;
//  srv.request.maxVelc[0] = 0.02;//0.02  0.5
//  srv.request.maxVelc[1] = 0.02;
//  srv.request.dirtion = false;

//  ROS_INFO_STREAM("Ext_status:  " << srv.request.ext_status);

//  if (client.call(srv))
//  {
//    if(true == srv.response.result)
//    {
//      ROS_INFO("result: %s", "true");
//    }
//    else
//    {
//      ROS_INFO("result: %s", "failed");
//    }
//  }
//  else
//  {
//    ROS_ERROR("Failed to call service ExtAxle_Driver");
//    return 1;
//  }

  ros::Rate loop_rate(1);
  while(ros::ok())
  {
    if(2 == ExtAxis_errStatus)
    {
      clearExtErr_test.call(srv_clearExtErr);

      ROS_INFO_STREAM_NAMED("test","result:"<<srv_clearExtErr.response.result);

      if(0 == srv_clearExtErr.response.result)
      {
        ROS_INFO("ClearExternError result: %s", "true");
      }
      else
      {
        ROS_INFO("ClearExternError result: %s", "failed");
      }
    }

    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
