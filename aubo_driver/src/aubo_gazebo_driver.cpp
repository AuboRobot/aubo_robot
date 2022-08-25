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
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

#define ARM_DOF 6

float joint_states[ARM_DOF];

void Get_Joint_States(const sensor_msgs::JointStateConstPtr& msg)
{
    for(int i=0;i<ARM_DOF;i++)
    {
        //ROS_ERROR_STREAM_NAMED("gazebo"," : "<<msg->name[i] <<" : " << msg->position[i]);
        joint_states[i] = msg->position[i];
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aubo_gazebo_driver");
    ros::NodeHandle nh;

    memset(joint_states,0.0,ARM_DOF);

    ros::Subscriber sub_Joint_state_ = nh.subscribe("/joint_states",10,&Get_Joint_States);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::string robot_name;
    ros::param::get("/robot_name", robot_name);

    std::string shoulder_joint_command_name="/shoulder_joint_position_controller/command";
    std::string upperArm_joint_command_name="/upperArm_joint_position_controller/command";
    std::string foreArm_joint_command_name="/foreArm_joint_position_controller/command";
    std::string wrist1_joint_command_name="/wrist1_joint_position_controller/command";
    std::string wrist2_joint_command_name="/wrist2_joint_position_controller/command";
    std::string wrist3_joint_command_name="/wrist3_joint_position_controller/command";


    std::string shoulder_command_topic = robot_name+shoulder_joint_command_name;
    std::string upperArm_command_topic = robot_name+upperArm_joint_command_name;
    std::string foreArm_command_topic  = robot_name+foreArm_joint_command_name;
    std::string wrist1_command_topic   = robot_name+wrist1_joint_command_name;
    std::string wrist2_command_topic   = robot_name+wrist2_joint_command_name;
    std::string wrist3_command_topic   = robot_name+wrist3_joint_command_name;

    //ROS_ERROR_STREAM_NAMED("test_Topic_name",shoulder_command_topic);

    ros::Publisher  pub_gazebo_shoulder_joint = nh.advertise<std_msgs::Float64>(shoulder_command_topic,1000);
    ros::Publisher  pub_gazebo_upperArm_joint = nh.advertise<std_msgs::Float64>(upperArm_command_topic,1000);
    ros::Publisher  pub_gazebo_foreArm_joint =  nh.advertise<std_msgs::Float64>(foreArm_command_topic,1000);
    ros::Publisher  pub_gazebo_wrist1_joint =   nh.advertise<std_msgs::Float64>(wrist1_command_topic,1000);
    ros::Publisher  pub_gazebo_wrist2_joint =   nh.advertise<std_msgs::Float64>(wrist2_command_topic,1000);
    ros::Publisher  pub_gazebo_wrist3_joint =   nh.advertise<std_msgs::Float64>(wrist3_command_topic,1000);

    std_msgs::Float64 shoulder_joint;
    shoulder_joint.data = 0.0;

    std_msgs::Float64 upperArm_joint;
    upperArm_joint.data = 0.0;

    std_msgs::Float64 foreArm_joint;
    foreArm_joint.data = 0.0;

    std_msgs::Float64 wrist1_joint;
    wrist1_joint.data = 0.0;

    std_msgs::Float64 wrist2_joint;
    wrist2_joint.data = 0.0;

    std_msgs::Float64 wrist3_joint;
    wrist3_joint.data = 0.0;

    while(ros::ok())
    {
        shoulder_joint.data = joint_states[0];
        upperArm_joint.data = joint_states[1];
        foreArm_joint.data = joint_states[2];
        wrist1_joint.data = joint_states[3];
        wrist2_joint.data = joint_states[4];
        wrist3_joint.data = joint_states[5];

        pub_gazebo_shoulder_joint.publish(shoulder_joint);
        pub_gazebo_upperArm_joint.publish(upperArm_joint);
        pub_gazebo_foreArm_joint.publish(foreArm_joint);
        pub_gazebo_wrist1_joint.publish(wrist1_joint);
        pub_gazebo_wrist2_joint.publish(wrist2_joint);
        pub_gazebo_wrist3_joint.publish(wrist3_joint);

        ros::spinOnce();

        usleep(1000*100);
    }

    spinner.stop();
}
