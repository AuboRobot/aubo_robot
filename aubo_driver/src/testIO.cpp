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
#include <cstdlib>

#define PI 3.141592654

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
  ros::ServiceClient client_ik = n.serviceClient<aubo_msgs::GetIK>("/aubo_driver/get_ik");
  ros::ServiceClient client_fk = n.serviceClient<aubo_msgs::GetFK>("/aubo_driver/get_fk");
  ros::Subscriber io_subs = n.subscribe("/aubo_driver/io_states", 100, IOCallback);
  aubo_msgs::SetIO srv;
  aubo_msgs::GetFK get_fk;
  aubo_msgs::GetIK get_ik;
  std::string s1, s2, s3;
  ros::Rate loop_rate(1);
  while (ros::ok())
  {
      get_fk.request.joint.resize(6);
      get_fk.request.joint[0] = 0;
      get_fk.request.joint[1] = PI/2;
      get_fk.request.joint[2] = 0;
      get_fk.request.joint[3] = 0;
      get_fk.request.joint[4] = 0;
      get_fk.request.joint[5] = 0;
      if (client_fk.call(get_fk))
      {
        ROS_INFO("get_fk:pos: %lf, %lf, %lf, ori: %lf, %lf, %lf, %lf", (double)get_fk.response.pos[0], (double)get_fk.response.pos[1], (double)get_fk.response.pos[2],
                                              (double)get_fk.response.ori[0], (double)get_fk.response.ori[1], (double)get_fk.response.ori[2], (double)get_fk.response.ori[3]);
      }
      else
      {
        ROS_ERROR("Failed to get_fk");
        return -1;
      }

      get_ik.request.ref_joint.resize(6);
      get_ik.request.ref_joint[0] = 0.1;
      get_ik.request.ref_joint[1] = PI/2;
      get_ik.request.ref_joint[2] = 0.1;
      get_ik.request.ref_joint[3] = 0;
      get_ik.request.ref_joint[4] = 0;
      get_ik.request.ref_joint[5] = 0;

      get_ik.request.pos.resize(3);
      get_ik.request.ori.resize(4);
      get_ik.request.pos[0] = -0.886500; get_ik.request.pos[1] = -0.215500; get_ik.request.pos[2] = 0.098500;
      get_ik.request.ori[0] = 0.5; get_ik.request.ori[1] = 0.5; get_ik.request.ori[2] = -0.5; get_ik.request.ori[3] = 0.5;
      if (client_ik.call(get_ik))
      {
        ROS_INFO("get_ik:joint: %lf, %lf, %lf, %lf, %lf, %lf", (double)get_ik.response.joint[0], (double)get_ik.response.joint[1], (double)get_ik.response.joint[2],
                                              (double)get_ik.response.joint[3], (double)get_ik.response.joint[4], (double)get_ik.response.joint[5]);
      }
      else
      {
        ROS_ERROR("Failed to get_ik");
        return -1;
      }

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
