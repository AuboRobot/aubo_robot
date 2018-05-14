#include "aubo_driver/aubo_driver.h"
#include <string>
#include <sys/timeb.h>
#include "aubo_driver.cpp"

using namespace aubo_driver;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "aubo_driver");
    ros::NodeHandle n;

    AuboDriver robot_driver;
    robot_driver.run();

    ros::AsyncSpinner spinner(6);
    spinner.start();

    ros::Rate loop_rate(robot_driver.UPDATE_RATE_);
    while(ros::ok())
    {
        robot_driver.updateControlStatus();
        loop_rate.sleep();
        ros::spinOnce();
    }
    ROS_WARN("Exiting robot_driver");
    return(0);
}


