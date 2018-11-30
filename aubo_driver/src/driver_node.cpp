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

#include "aubo_driver/aubo_driver.h"
#include <string>
#include <sys/timeb.h>
#include "aubo_driver.cpp"

using namespace aubo_driver;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "aubo_driver");
    ros::NodeHandle n;

    int num = 0;
    ros::param::get("/aubo_driver/external_axis_number", num);
    int N = 3;
    while(num == 0 && N < 0)
    {
      sleep(1);
      ros::param::get("/aubo_driver/external_axis_number", num);
      N--;
    }

    ROS_INFO("aubo_driver/external_axis_number: %s", std::to_string(num).c_str());
    AuboDriver robot_driver(num);
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


