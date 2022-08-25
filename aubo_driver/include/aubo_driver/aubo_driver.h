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
#ifndef AUBO_DRIVER_H_
#define AUBO_DRIVER_H_

#include <thread>
#include <string>
#include <sys/timeb.h>
#include <queue>

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>
#include <aubo_msgs/SetIO.h>
#include <aubo_msgs/SetPayload.h>
#include <aubo_msgs/SetIORequest.h>
#include <aubo_msgs/SetIOResponse.h>
#include <aubo_msgs/GetFK.h>
#include <aubo_msgs/GetFKRequest.h>
#include <aubo_msgs/GetFKResponse.h>
#include <aubo_msgs/GetIK.h>
#include <aubo_msgs/GetIKRequest.h>
#include <aubo_msgs/GetIKResponse.h>
#include <aubo_msgs/IOState.h>
#include <aubo_msgs/Digital.h>
#include <aubo_msgs/Analog.h>
#include <aubo_msgs/JointPos.h>
#include <industrial_msgs/RobotStatus.h>
#include "aubo_driver/AuboRobotMetaType.h"
#include "aubo_driver/serviceinterface.h"
#include "sensor_msgs/JointState.h"
#include <control_msgs/FollowJointTrajectoryFeedback.h>

#include "otg/otgnewslib.h"

#define MINIMUM_BUFFER_SIZE 300
#define ARM_DOF 8               //support at most 8 axes
#define MAXALLOWEDDELAY 50
#define server_port 8899
#define BIG_MODULE_RATIO 2 * M_PI / 60.0 / 121
#define SMALL_MODULE_RATIO 2 * M_PI / 60.0 / 101
#define VMAX 3000
//#define AMAX 10000
#define AMAX 10000
//#define JMAX 40000
#define JMAX 40000
#define STOP_DELAY_CLEAR_TIMES 100
//#define LOG_INFO_DEBUG

namespace aubo_driver
{
    struct PlanningState
    {
        double joint_vel_[ARM_DOF];
        double joint_acc_[ARM_DOF];
        double joint_pos_[ARM_DOF];
    };
    enum ROBOT_CONTROLLER_MODE
    {
        ROBOT_CONTROLLER=0, //
        ROS_CONTROLLER
    };
    enum ControlOption
    {
        AuboAPI = 0,
        RosMoveIt
    };
    enum ControMode
    {
        Teach = 0,
        SendTargetGoal,
        SynchronizeWithRealRobot
    };

    struct RobotState
    {
        aubo_robot_namespace::JointStatus joint_status_[ARM_DOF];
        aubo_robot_namespace::wayPoint_S wayPoint_;
        aubo_robot_namespace::RobotDiagnosis robot_diagnosis_info_;
        bool IsRealRobotExist;
        bool isRobotControllerConnected;
        ROBOT_CONTROLLER_MODE robot_controller_;
        aubo_robot_namespace::RobotState state_;
        aubo_robot_namespace::RobotErrorCode code_;
    };

    class AuboDriver
    {
        public:
            AuboDriver(int num);
            ~AuboDriver();
            bool roadPointCompare(double *point1, double *point2);

            double* getCurrentPosition();
            void setCurrentPosition(double *target);
            double* getTagrtPosition();
            void setTagrtPosition(double *target);

            void updateControlStatus();
            void run();
            bool connectToRobotController();
            bool setIO(aubo_msgs::SetIORequest& req, aubo_msgs::SetIOResponse& resp);
            bool getFK(aubo_msgs::GetFKRequest& req, aubo_msgs::GetFKResponse& resp);
            bool getIK(aubo_msgs::GetIKRequest& req, aubo_msgs::GetIKResponse& resp);

            const int UPDATE_RATE_ = 500;
            const int TIMER_SPAN_ = 50;
            const double THRESHHOLD = 0.000001;

        public:
            static std::string joint_name_[ARM_DOF];
            double joint_ratio_[ARM_DOF];
            int axis_number_;
            int buffer_size_;
            ServiceInterface robot_send_service_;      //send
            ServiceInterface robot_receive_service_;     //receive

            RobotState rs;
//            std::thread* mb_publish_thread_;

            std::queue<PlanningState>  buf_queue_;
            aubo_msgs::JointPos cur_pos;
            ros::Publisher joint_states_pub_;
            ros::Publisher joint_feedback_pub_;
            ros::Publisher joint_target_pub_;
            ros::Publisher robot_status_pub_;
            ros::Subscriber teach_subs_;
            ros::Subscriber moveAPI_subs_;
            ros::Subscriber moveit_controller_subs_;
            ros::Subscriber trajectory_execution_subs_;
            ros::Subscriber robot_control_subs_;
            ros::Publisher io_pub_;

        private:
            void moveItPosCallback(const trajectory_msgs::JointTrajectoryPoint::ConstPtr &msg);
            void trajectoryExecutionCallback(const std_msgs::String::ConstPtr &msg);
            void robotControlCallback(const std_msgs::String::ConstPtr &msg);
            void AuboAPICallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
            void teachCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
            void timerCallback(const ros::TimerEvent& e);
            bool setRobotJointsByMoveIt();
            void controllerSwitchCallback(const std_msgs::Int32::ConstPtr &msg);
            void publishIOMsg();

            bool reverse_connected_;
            double last_recieve_point_[ARM_DOF];   /** To avoid joining the same waypoint to the queue **/
            int control_option_;
            bool emergency_stopped_;
            bool protective_stopped_;
            bool normal_stopped_;
            bool data_recieved_;
            int data_count_;
            bool real_robot_exist_;
            bool controller_connected_flag_;
            bool start_move_;
            double current_joints_[ARM_DOF];
            double target_point_[ARM_DOF];
            JointTrajectoryInput jti;
            JointTrajectoryOutput jto;

            ros::NodeHandle nh_;
            ros::Publisher  rib_pub_;
            ros::Publisher  cancle_trajectory_pub_;
            ros::Subscriber controller_switch_sub_;
            ros::Timer timer_;
            ros::Timer io_publish_timer;

            ros::ServiceServer io_srv_;
            ros::ServiceServer ik_srv_;
            ros::ServiceServer fk_srv_;
            std::thread* mb_publish_thread_;

            double io_flag_delay_;
            std::string server_host_;
            int rib_buffer_size_;
            int control_mode_;
            int collision_class_;
            std_msgs::Int32MultiArray rib_status_;
            industrial_msgs::RobotStatus robot_status_;

            int delay_clear_times;
    };
}

#endif /* AUBO_DRIVER_H_ */
