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
#include <time.h>
#include <string.h>
#include <vector>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <thread>
#include <algorithm>
#include <cmath>
#include <chrono>


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


#define ARM_DOF 8
namespace aubo_controller {

class MotionControllerSimulator
{
public:
    int rib_buffer_size_;
    bool controller_connected_flag_;
    boost::mutex mutex_;

protected:
    ros::NodeHandle nh_;
    double update_rate_;
    std::queue<trajectory_msgs::JointTrajectoryPoint> motion_buffer_;
//    int num_joints_;
    std::string joint_names_[ARM_DOF];
    double joint_positions_[ARM_DOF];
    std::string initial_joint_state_;

    bool position_updated_flag_;
    bool sig_shutdown_;
    bool sig_stop_;

    const int MINIMUM_BUFFER_SIZE_ = 250;

    ros::Publisher moveit_joint_state_pub_;
    ros::Subscriber update_joint_state_subs_;
    std::thread* motion_thread_;

public:
    MotionControllerSimulator(int num_joints, double update_rate, std::string *jointnames)
    {
        //Motion loop update rate (higher update rates result in smoother simulated motion)
        update_rate_ = update_rate;
        ROS_INFO("Setting motion update rate (hz): %f", update_rate_);

        for(int i = 0; i < ARM_DOF; i++)
            joint_names_[i] = jointnames[i];

//        Initialize motion buffer (contains joint position lists)
//        motion_buffer_.
        std::string def_joint_names[] = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
//        joint_names_ = ros::param::get("controller_joint_names",def_joint_names);
        double joint_state[num_joints];
//        initial_joint_state_ = ros::param::get("initial_joint_state", joint_state);


        rib_buffer_size_ = 0;
        controller_connected_flag_ = false;
        position_updated_flag_ = false;
        sig_shutdown_ = false;
        sig_stop_ = false;


        moveit_joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("moveit_controller_cmd", 2000);
        update_joint_state_subs_ = nh_.subscribe("real_pose", 50, &MotionControllerSimulator::updateJointStateCallback, this);

        motion_thread_ = new std::thread(boost::bind(&MotionControllerSimulator::motionWorker, this));

    }

    void updateJointStateCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
    {
        if(isInMotion())
        {
            for(int i = 0; i < ARM_DOF; i++)
            {
                joint_positions_[i] = msg->data[i];
            }
            ROS_DEBUG("Update current joint state successfully!");
        }
    }

    void addMotionWaypoint(trajectory_msgs::JointTrajectoryPoint point)
    {
        //When add new trajectory into the buffer, here need to handle the accelerations!!
        motion_buffer_.push(point);
    }

    void getJointPositions(double *joint_position)
    {
        mutex_.lock();
        memcpy(joint_position, joint_positions_, ARM_DOF*sizeof(double));
        mutex_.unlock();
    }


    void motionWorker()
    {
        double move_duration, update_duration, T, T2, T3, T4, T5, tt, ti, t1, t2, t3, t4, t5;
        double a1[ARM_DOF], a2[ARM_DOF], a3[ARM_DOF], a4[ARM_DOF], a5[ARM_DOF], h[ARM_DOF], intermediate_goal_point[ARM_DOF];
        ros::param::set("/driver_start","1");
//        self.positionUpdatedFlag = rospy.get_param('/IsRobotConnected', '0')
//               while self.positionUpdatedFlag == '0':
//                   rospy.sleep( self.update_rate/400)
//                   self.positionUpdatedFlag = rospy.get_param('/IsRobotConnected', '0')
        ROS_INFO("Starting motion worker in motion controller simulator");
        if(update_rate_ > 0)
            update_duration = 1.0 / update_rate_;
        trajectory_msgs::JointTrajectoryPoint last_goal_point, current_goal_point;

        mutex_.lock();
        memcpy(&last_goal_point.positions[0], joint_positions_, ARM_DOF*sizeof(double));
        mutex_.unlock();

        while(!sig_shutdown_)
        {
            try
            {
                current_goal_point = motion_buffer_.front();
                motion_buffer_.pop();
                if(current_goal_point.time_from_start <= last_goal_point.time_from_start)
                    move_duration = current_goal_point.time_from_start.toSec();
                else
                {
                    T = current_goal_point.time_from_start.toSec() - last_goal_point.time_from_start.toSec();
                    memcpy(a1, &last_goal_point.velocities[0], ARM_DOF*sizeof(double));
                    T2 = T * T;
                    T3 = T2 * T;
                    T4 = T3 * T;
                    T5 = T4 * T;
                    for(int i = 0; i < ARM_DOF; i++)
                    {
                        a2[i] = 0.5 * last_goal_point.accelerations[i];
                        h[i] = current_goal_point.positions[i] - last_goal_point.positions[i];
                        a3[i] =0.5 / T3 * (20*h[i] - (8*current_goal_point.velocities[i] + 12*last_goal_point.velocities[i])*T - (3*last_goal_point.accelerations[i] -current_goal_point.accelerations[i])*T2);
                        a4[i] =0.5 / T4 * (-30*h[i] + (14*current_goal_point.velocities[i] + 16*last_goal_point.velocities[i])*T + (3*last_goal_point.accelerations[i] - 2*current_goal_point.accelerations[i])*T2);
                        a5[i] =0.5 / T5 * (12*h[i] - 6*(current_goal_point.velocities[i] + last_goal_point.velocities[i])*T + (current_goal_point.accelerations[i] - last_goal_point.accelerations[i])*T2);
                    }
                    if(update_rate_ > 0)
                    {
                        tt = last_goal_point.time_from_start.toSec();
                        ti = tt;
                        while (tt < current_goal_point.time_from_start.toSec())
                        {
                            t1 = tt - ti;
                            t2 = t1 * t1;
                            t3 = t2 * t1;
                            t4 = t3 * t1;
                            t5 = t4 * t1;
                            for(int i = 0; i < ARM_DOF; i++)
                                intermediate_goal_point[i] = last_goal_point.positions[i] +a1[i]*t1+a2[i]*t2 +a3[i]*t3 +a4[i]*t4 +a5[i]*t5;
                            tt += update_duration;
                            moveToTarget((int)(update_duration*1000), intermediate_goal_point);
                            jointStatePublisher();
                        }
                        move_duration = current_goal_point.time_from_start.toSec() - tt;
                    }
                }
                moveToTarget((int)(move_duration*1000), &current_goal_point.positions[0]);
                jointStatePublisher();
                last_goal_point = current_goal_point;

            }
            catch(...)
            {
                ROS_ERROR("Unexpected exception in generating control data!");
            }
        }
        ROS_DEBUG("Shutting down motion controller!");
    }

    bool isInMotion()
    {
        return !motion_buffer_.empty();
    }

    void shutdown()
    {
        sig_shutdown_ = true;
        ROS_DEBUG("Motion_Controller shutdown signaled!");
    }

    void stop()
    {
        ROS_DEBUG("Motion_Controller stop signaled!");
        mutex_.lock();
        while (!motion_buffer_.empty())
            motion_buffer_.pop();
        sig_stop_ = true;
        mutex_.unlock();
    }

    void jointStatePublisher()
    {
        try
        {
            sensor_msgs::JointState joint_state;
            joint_state.header.stamp = ros::Time::now();
            joint_state.name.resize(ARM_DOF);
            joint_state.position.resize(ARM_DOF);
            mutex_.lock();
            for(int i = 0; i < ARM_DOF; i++)
            {
                joint_state.name[i] = joint_names_[i];
                joint_state.position[i] = joint_positions_[i];
            }
            mutex_.unlock();
            moveit_joint_state_pub_.publish(joint_state);
        }
        catch (...)
        {
             ROS_ERROR("Unexpected exception in joint state publisher!!!");
        }
    }

    void moveToTarget(int dur, double *point)
    {
        while(rib_buffer_size_ > MINIMUM_BUFFER_SIZE_)
        {
            ROS_INFO("generate control data too fast!");
            std::this_thread::sleep_for(std::chrono::milliseconds(dur));
        }

        if(rib_buffer_size_ == 0 && controller_connected_flag_ == false)    //motion start or no robot connected!
            std::this_thread::sleep_for(std::chrono::milliseconds(dur));

        mutex_.lock();
        if(!sig_stop_)
        {
            memcpy(joint_positions_, &point[0], ARM_DOF*sizeof(double));
            ROS_DEBUG("Moved to target position!");
        }
        else
        {
            ROS_INFO("Stopping motion immediately, clearing stop signal");
            sig_stop_ = false;
        }
        mutex_.unlock();

    }

private:

};


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
class AuboRobotSimulatorNode
{
public:

protected:
    const int JOINT_STATE_PUB_RATE = 50;
    ros::NodeHandle nh_;
    double pub_rate_;
    bool controller_enable_flag_;
    MotionControllerSimulator *motion_ctrl_;

    std::string joint_names_[ARM_DOF];

    ros::Publisher joint_state_pub_;
    ros::Publisher joint_feedback_pub_;
    ros::Subscriber joint_path_sub_;
    ros::Subscriber plan_type_sub_;

    ros::Timer timer_;


    std::queue<trajectory_msgs::JointTrajectoryPoint> motion_buffer_;
    double joint_positions_[];
    std::string initial_joint_state_;
    int rib_buffer_size_;
    bool controller_connected_flag_;
    bool position_updated_flag_;
    bool sig_shutdown_;
    bool sig_stop_;

public:
    AuboRobotSimulatorNode()
    {
        controller_connected_flag_ = true;
        std::string joint_names_[ARM_DOF];
        std::string def_joint_names[] = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
        std::vector<std::string> jointNames;
        ros::param::get("controller_joint_names",jointNames);
        motion_ctrl_ = new MotionControllerSimulator(6, 200, &jointNames[0]);

        joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 100);
        joint_feedback_pub_ = nh_.advertise<control_msgs::FollowJointTrajectoryFeedback>("feedback_states", 100);
        joint_path_sub_ = nh_.subscribe("joint_path_command", 100, &AuboRobotSimulatorNode::trajectoryCallback, this);

        plan_type_sub_ = nh_.subscribe("rib_status", 100, &AuboRobotSimulatorNode::ribStatusCallback, this);

//        joint_positions = new double[ARM_DOF];

        timer_ = nh_.createTimer(ros::Duration(1.0 / JOINT_STATE_PUB_RATE),&AuboRobotSimulatorNode::publishWorker,this);
        timer_.start();
    }

    ~AuboRobotSimulatorNode()
    {
        if(timer_.isValid())
            timer_.stop();
        if(motion_ctrl_ != NULL)
        {
            delete motion_ctrl_;
            motion_ctrl_ = NULL;
        }
    }

    void publishWorker(const ros::TimerEvent& e)
    {
        if(controller_enable_flag_ == true /*&& motion_ctrl_.positionUpdatedFlag == true*/)
        {
            try
            {
                control_msgs::FollowJointTrajectoryFeedback joint_fb_msg;
                motion_ctrl_->mutex_.lock();
                joint_fb_msg.header.stamp = ros::Time::now();
                joint_fb_msg.joint_names.resize(ARM_DOF);
                joint_fb_msg.actual.positions.resize(ARM_DOF);
                double joint_position[ARM_DOF];
                motion_ctrl_->getJointPositions(joint_position);
                for(int i = 0; i<ARM_DOF; i++)
                {
                    joint_fb_msg.joint_names[i] = joint_names_[i];
                    joint_fb_msg.actual.positions[i] = joint_position[i];
                }
                joint_feedback_pub_.publish(joint_fb_msg);
                motion_ctrl_->mutex_.unlock();
            }
            catch(...)
            {
                ROS_INFO("Unexpected exception in joint feedback state publisher!");
            }
        }
    }

    void trajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
    {
        if(msg->points.size() == 0 || controller_enable_flag_ == false)
        {
           // if the JointTrajectory is null or the robot is controlled by other controller, then do nothing.
        }
        else
        {
            ROS_DEBUG("handle joint_path_command message");
            try
            {
                ROS_INFO("Received trajectory with %d points, executing callback", msg->points.size());
                if(motion_ctrl_->isInMotion())
                {
                    if(msg->points.size() > 0)
                        ROS_ERROR("Received trajectory while still in motion, trajectory splicing not supported");
                    else
                        ROS_DEBUG("Received empty trajectory while still in motion, stopping current trajectory");
                    motion_ctrl_->stop();
                }
                else
                {
                    for(int i = 0; i < msg->points.size(); i++)
                    {
                        // first remaps point to controller joint order, the add the point to the controller.
                        trajectory_msgs::JointTrajectoryPoint point = msg->points[i];
                        for(int j = 0; j < ARM_DOF; j++)
                        {
                            for(int k = 0; k < ARM_DOF; k++)
                            {
                                if(joint_names_[j] == msg->joint_names[k])
                                {
                                    point.positions[j] = msg->points[i].positions[k];
                                    point.velocities[j] = msg->points[i].velocities[k];
                                    point.accelerations[j] = msg->points[i].accelerations[k];
                                    break;
                                }
                            }
                        }
                        motion_ctrl_->addMotionWaypoint(point);
                    }
                }
            }
            catch(...)
            {
                ROS_DEBUG("Unexpected exception while adding control point to the controller!");
            }
        }
        ROS_DEBUG("Exiting trajectory callback");
    }

    void ribStatusCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
    {
        try
        {
            if(msg->data[1] == 1)
            {
                controller_connected_flag_ = true;
                ROS_DEBUG("The robot is controlled by the ros controller!");
            }
            else
            {
                controller_connected_flag_ = false;
                 ROS_DEBUG("The robot is not controlled by the ros controller!");
            }
            motion_ctrl_->rib_buffer_size_ = msg->data[0];
            motion_ctrl_->controller_connected_flag_ = msg->data[2];
        }
        catch(...)
        {
            ROS_DEBUG("Unexpected exception while parsing the aubo driver message!");
        }
    }

private:

};
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aubo_simulator_controller");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(3);
    spinner.start();

    ros::waitForShutdown();

    exit(0);
}
