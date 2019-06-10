/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017-2018, AUBO Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <ros/ros.h>
#include <ros/console.h>
#include <pluginlib/class_loader.h>
#include <time.h>
#include <string.h>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

bool plan_finished = false;
int plan_type = 0;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "aubo_planner");

    ros::NodeHandle n;
    ros::Publisher target_pose_pub = n.advertise<geometry_msgs::Pose>("/aubo_driver/target_pose", 100);
    ros::Publisher target_joint_pub = n.advertise<std_msgs::Float32MultiArray>("/aubo_driver/target_joint_value", 100);
    ros::Publisher cartesian_path_pub = n.advertise<geometry_msgs::PoseArray>("/aubo_driver/cartesian_path_list", 100);

    std::string s1, s2;
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        ros::param::get("/aubo_driver/plan_finished", s1);
        if(s1 != "")
            plan_finished = (s1 == "0")? false : true;

        if(!plan_finished)
        {
            ros::param::get("/aubo_driver/plan_type", s2);
            if(s2 != "")
                plan_type = std::atoi(s2.c_str());
            if(plan_type == 0)
            {
                geometry_msgs::Pose pose;
                pose.position.x = -0.335007;
                pose.position.y = -0.564855;
                pose.position.z = 0.388566;
                pose.orientation.w = 0.029874;
                pose.orientation.x = 0.458695;
                pose.orientation.y = 0.887734;
                pose.orientation.z = -0.025182;
                target_pose_pub.publish(pose);
            }
            else if(plan_type == 1)
            {
                std_msgs::Float32MultiArray joints;
                joints.data.resize(6);
                joints.data[0] = -1.033696;
                joints.data[1] = -0.231935;
                joints.data[2] = 1.425622;
                joints.data[3] = 0.164112;
                joints.data[4] = 1.559689;
                joints.data[5] = -1.651097;
                target_joint_pub.publish(joints);
            }
            else
            {
                geometry_msgs::PoseArray pose_array;
                geometry_msgs::Pose pose;
                //pose1
                pose.position.x = -0.335007;
                pose.position.y = -0.564855;
                pose.position.z = 0.388566;
                pose.orientation.w = 0.029874;
                pose.orientation.x = 0.458695;
                pose.orientation.y = 0.887734;
                pose.orientation.z = -0.025182;
                pose_array.poses.push_back(pose);
                //pose2
                pose.position.x = 0.196444;
                pose.position.y = -0.391179;
                pose.position.z = 0.382422;
                pose.orientation.w = 0.029870;
                pose.orientation.x = 0.458695;
                pose.orientation.y = 0.887736;
                pose.orientation.z = -0.025148;
                pose_array.poses.push_back(pose);
                //pose3
                pose.position.x = -0.374711;
                pose.position.y = -0.389879;
                pose.position.z = 0.384175;
                pose.orientation.w = 0.029880;
                pose.orientation.x = 0.458692;
                pose.orientation.y = 0.887736;
                pose.orientation.z = -0.025151;
                pose_array.poses.push_back(pose);
                //pose4
                pose.position.x = -0.375145;
                pose.position.y = -0.399046;
                pose.position.z = 0.155094;
                pose.orientation.w = 0.029896;
                pose.orientation.x = 0.458698;
                pose.orientation.y = 0.887733;
                pose.orientation.z = -0.025142;
                pose_array.poses.push_back(pose);
                cartesian_path_pub.publish(pose_array);
            }
            plan_finished = true;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

    return 0;
}

