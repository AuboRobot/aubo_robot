#include "aubo_driver/aubo_driver.h"

#include <string>
#include <cstdlib>

int pin = 3;
int fun = 3;
int state = 1;
void IOCallback(const aubo_msgs::IOState::ConstPtr &msg)
{
    std::cout<<"pin"<<fun<<"state:"<<msg->digital_out_states[fun].state<<std::endl;
    ROS_INFO("Sum: %d", msg->digital_out_states[fun].state);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "testIO");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<aubo_msgs::SetIO>("/aubo_driver/set_io");
  ros::Subscriber io_subs = n.subscribe("/aubo_driver/io_states", 100, IOCallback);
  aubo_msgs::SetIO srv;
  std::string s1, s2, s3;
  ros::Rate loop_rate(1);
  while (ros::ok())
  {
      ros::param::get("/aubo_driver/io_fun", s1);
      if(s1 != "")
        fun = std::atoi(s1.c_str());

      ros::param::get("/aubo_driver/io_pin", s2);
      if(s2 != "")
        pin = std::atoi(s2.c_str());

      ros::param::get("/aubo_driver/io_state", s3);
      if(s3 != "")
        state = std::atoi(s3.c_str());

      srv.request.fun = fun;
      srv.request.pin = pin;
      srv.request.state = state;
      if (client.call(srv))
      {
        ROS_INFO("set io: %ld", (long int)srv.response.success);
      }
      else
      {
        ROS_ERROR("Failed to set io");
        return -1;
      }
      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}
