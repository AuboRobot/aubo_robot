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

#ifndef MOTION_CONTROLLER_SIMULATOR_H
#define MOTION_CONTROLLER_SIMULATOR_H

#include <ros/ros.h>
#include <ros/console.h>
#include <time.h>
#include <string.h>
#include <vector>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <iostream>
#include <pthread.h>


#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

#include "sensor_msgs/JointState.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "actionlib/server/action_server.h"
#include "actionlib/server/server_goal_handle.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "std_msgs/String.h"
#include <controller_manager/controller_manager.h>
#include <realtime_tools/realtime_publisher.h>

namespace aubo_robot_simulator
{
#define ARM_DOF 8
class MotionControllerSimulator
{
    /*
     * This class simulates an Aubo robot controller.  The simulator
     * adheres to the ROS-Industrial robot driver specification:
     *
     * http://www.ros.org/wiki/Industrial/Industrial_Robot_Driver_Spec
     *
     * TODO: Currently the simulator only supports the bare minimum motion interface.
     *
     * TODO: Interfaces to add:
     * Joint streaming
     * All services
    */
public:
    int rib_buffer_size_;
    bool controller_connected_flag_;
    boost::mutex mutex_;

protected:
    ros::NodeHandle nh_;
    double update_rate_;
    std::queue<trajectory_msgs::JointTrajectoryPoint> motion_buffer_;
    std::string joint_names_[ARM_DOF];
    double joint_positions_[ARM_DOF];
    std::string initial_joint_state_;

    bool position_updated_flag_;
    bool sig_shutdown_;
    bool sig_stop_;

    const int MINIMUM_BUFFER_SIZE_ = 250;
    const int JOINT_STATE_PUB_RATE = 50;
    bool controller_enable_flag_;
    MotionControllerSimulator *motion_ctrl_;

    ros::Publisher joint_state_pub_;
    ros::Publisher joint_feedback_pub_;
    ros::Subscriber joint_path_sub_;
    ros::Subscriber plan_type_sub_;
    ros::Publisher moveit_joint_state_pub_;
    ros::Subscriber update_joint_state_subs_;
    ros::Timer timer_;

public:
    MotionControllerSimulator(int num_joints, double update_rate, std::string *jointnames);
    ~MotionControllerSimulator();

    void updateJointStateCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void addMotionWaypoint(trajectory_msgs::JointTrajectoryPoint point);
    void getJointPositions(double *joint_position);
    void motionWorker();
    bool isInMotion();
    void shutdown();
    void stop();
    void jointStatePublisher();
    void moveToTarget(int dur, double *point);
    void publishWorker(const ros::TimerEvent& e);
    void trajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr &msg);
    void ribStatusCallback(const std_msgs::Int32MultiArray::ConstPtr &msg);

private:
    int num_joints_;
    int external_axis_number_;
    int pub_rate_;
    double velocity_scale_factor_;

};
}
#endif // MOTION_CONTROLLER_SIMULATOR_H

