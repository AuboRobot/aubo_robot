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
#include <ros/console.h>
#include <pluginlib/class_loader.h>
#include <time.h>
#include <string.h>
#include <vector>
#include <condition_variable>
#include <boost/scoped_ptr.hpp>

#include <moveit/move_group_interface/move_group.h>
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

namespace aubo_controller {
class AuboRobotPlannerNode
{
protected:
    ros::NodeHandle nh_;
    ros::Publisher display_publisher_;
    ros::Subscriber target_pose_subs_;
    ros::Subscriber target_joints_subs_;
    moveit_msgs::DisplayTrajectory display_trajectory_;
    moveit::planning_interface::MoveGroup::Plan my_plan_;
    moveit::planning_interface::MoveGroup *group_;
    robot_model::RobotModelPtr robot_model;
    moveit::planning_interface::PlanningSceneInterface *planning_scene_interface_;
    // We will use the :planning_scene_interface:`PlanningSceneInterface` class to deal directly with the world.

    geometry_msgs::Pose target_pose_;
    std::vector<double> group_variable_values_;
    std::vector<std::string> joint_names_;
    int num_joints_;
    int plan_type_;
    bool plan_flag_;

public:
    AuboRobotPlannerNode(const std::string &group_name);
    ~AuboRobotPlannerNode();
    void setTargetPose(const geometry_msgs::Pose &msg);
    void setTargetJointValue(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void planner();
    void run();
    void abort();
};

AuboRobotPlannerNode::AuboRobotPlannerNode(const std::string &group_name)
{
    //         ros::NodeHandle node_handle("~");
    //         robot_model_loader::RobotModelLoader robot_model_loader(group_name, true);
    //         robot_model = robot_model_loader.getModel();

    // The :move_group_interface:`MoveGroup` class can be easily setup using just the name of the group you would like to control and plan for.
    //waiting for the Rviz to come up before initiate the group.
    //        const moveit::planning_interface::MoveGroup::Options opt(group_name);
    group_ = new moveit::planning_interface::MoveGroup(group_name);
    //        planning_scene_interface_ = new moveit::planning_interface::PlanningSceneInterface();

    // (Optional) Create a publisher for visualizing plans in Rviz.
    display_publisher_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    target_pose_subs_ = nh_.subscribe("target_pose", 50, &AuboRobotPlannerNode::setTargetPose, this);
    target_joints_subs_ = nh_.subscribe("target_joint_value", 50, &AuboRobotPlannerNode::setTargetJointValue, this);

    // Getting Basic Information
    ROS_INFO("Base Reference frame: %s", group_->getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO("Tool Reference frame: %s", group_->getEndEffectorLink().c_str());

    plan_flag_ = false;
    plan_type_ = -1;

    joint_names_ = group_->getJointNames();
    num_joints_ = joint_names_.size();
    if(num_joints_ <= 0)
        ROS_WARN("The planning group is not correct!");

    //  group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
}

AuboRobotPlannerNode::~AuboRobotPlannerNode()
{
    if(group_ != NULL)
    {
        delete group_;
        group_ = NULL;
    }
}


void AuboRobotPlannerNode::setTargetPose(const geometry_msgs::Pose &msg)
{
    if(!plan_flag_)
    {
        target_pose_.orientation.w = msg.orientation.w;
        target_pose_.orientation.x = msg.orientation.x;
        target_pose_.orientation.y = msg.orientation.y;
        target_pose_.orientation.z = msg.orientation.z;
        target_pose_.position.x = msg.position.x;
        target_pose_.position.y = msg.position.y;
        target_pose_.position.z = msg.position.z;

        plan_type_ = 0;
    }
    else
    {
        ROS_WARN("There is a planning already!");
    }
}

void AuboRobotPlannerNode::setTargetJointValue(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    if(!plan_flag_)
    {
        for(int i = 0; i < num_joints_; i++)
            group_variable_values_[i] = msg->data[i];

        plan_type_ = 1;
    }
    else
    {
        ROS_WARN("There is a planning already!");
    }
}


void AuboRobotPlannerNode::planner()
{
    if(0 == plan_type_)     // Planning to a Pose goal
    {
        ROS_INFO("Planning to a Pose goal");
        // We can plan a motion for this group to a desired pose for the end-effector.
        plan_flag_ = true;
        group_->setPoseTarget(target_pose_);
        // Now, we call the planner to compute the plan and visualize it.
        // Note that we are just planning, not asking move_group to actually move the robot.
        bool success = group_->plan(my_plan_);

        /* Sleep to give Rviz time to visualize the plan. */
        sleep(1.0);
        // Moving to a pose goal
        // Moving to a pose goal is similar to the step above
        // except we now use the move() function. Note that
        // the pose goal we had set earlier is still active
        // and so the robot will try to move to that goal. We will
        // not use that function in this tutorial since it is
        // a blocking function and requires a controller to be active
        // and report success on execution of a trajectory.

        /* Uncomment below line when working with a real robot*/
        if(success)
        {
            ROS_INFO("Plan Succeed, Now Move!!!");
            group_->move();
        }
        else
        {
            ROS_ERROR("Plan Failed, Cann't Move!!!");
        }
        plan_flag_ = false;
    }
    else if(1 == plan_type_)        // Planning to a joint-space goal
    {
        plan_flag_ = true;
        ROS_INFO("Planning to a joint-space goal");
        // First get the current set of joint values for the group.
        // Then, let's modify one of the joints, plan to the new joint
        // space goal and visualize the plan.
        group_->setJointValueTarget(group_variable_values_);

        bool success = group_->plan(my_plan_);
        sleep(1.0);

        if(success)
        {
            ROS_INFO("Plan Succeed, Now Move!!!");
            group_->move();
        }
        else
        {
            ROS_ERROR("Plan Failed, Cann't Move!!!");
        }
        plan_flag_ = false;
    }

    ROS_WARN("Exiting move_group_interface");
}

void AuboRobotPlannerNode::run()
{
    ros::spin();
}

void AuboRobotPlannerNode::abort()
{
    ros::shutdown();
}
}

using aubo_controller::AuboRobotPlannerNode;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "aubo_robot_planner");
    AuboRobotPlannerNode planner("manipulator");
    planner.run();
    return 0;
}
