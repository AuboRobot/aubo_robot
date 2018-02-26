#include "aubo_driver/aubo_driver.h"

#include <string>
#include <cstdlib>

int i = 2;
void IOCallback(const aubo_msgs::IOState::ConstPtr &msg)
{
    std::cout<<"pin"<<i<<"state:"<<msg->digital_out_states[i].state<<std::endl;
    ROS_INFO("Sum: %d", msg->digital_out_states[i].state);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aubo_io");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<aubo_msgs::SetIO>("aubo_driver/set_io");
  ros::Subscriber io_subs = n.subscribe("aubo_driver/io_state", 10, IOCallback);
  aubo_msgs::SetIO srv;
  srv.request.fun = 2;
  srv.request.pin = i;
  srv.request.state = 1;
  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.success);
  }
  else
  {
    ROS_ERROR("Failed to set io");
    return 1;
  }

  return 0;
}
