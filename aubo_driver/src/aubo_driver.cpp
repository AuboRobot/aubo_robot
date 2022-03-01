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

namespace aubo_driver {

std::string AuboDriver::joint_name_[ARM_DOF] = {"shoulder_joint","upperArm_joint","foreArm_joint","wrist1_joint","wrist2_joint","wrist3_joint"};

AuboDriver::AuboDriver(int num = 0):delay_clear_times(0),buffer_size_(400),io_flag_delay_(0.02),data_recieved_(false),data_count_(0),real_robot_exist_(false),emergency_stopped_(false),protective_stopped_(false),normal_stopped_(false),
    controller_connected_flag_(false),start_move_(false),control_mode_ (aubo_driver::SendTargetGoal),rib_buffer_size_(0),jti(ARM_DOF,1.0/200),jto(ARM_DOF),collision_class_(6)
{
    axis_number_ = 6 + num;
    /** initialize the parameters **/
    for(int i = 0; i < axis_number_; i++)
    {
        current_joints_[i] = 0;
        target_point_[i] = 0;
        if(i < 3)
        {
            joint_ratio_[i] = BIG_MODULE_RATIO;
        }
        else if(i < 6)
        {
            joint_ratio_[i] = SMALL_MODULE_RATIO;
        }
        else
        {
            joint_ratio_[i] = 2 * M_PI / 10.05309632; //adjust by the real application
        }
        jti.maxVelocity[i] = VMAX * joint_ratio_[i];
        jti.maxAcceleration[i] = AMAX * joint_ratio_[i];
        jti.maxJerk[i] = JMAX * joint_ratio_[i];
    }
    rs.robot_controller_ = ROBOT_CONTROLLER;
    rib_status_.data.resize(3);

    /** publish messages **/
    joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 300);
    joint_feedback_pub_ = nh_.advertise<control_msgs::FollowJointTrajectoryFeedback>("feedback_states", 100);
    joint_target_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/aubo_driver/real_pose", 50);
    robot_status_pub_ = nh_.advertise<industrial_msgs::RobotStatus>("robot_status", 100);
    io_pub_ = nh_.advertise<aubo_msgs::IOState>("/aubo_driver/io_states", 10);
    rib_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("/aubo_driver/rib_status", 100);
    cancle_trajectory_pub_ = nh_.advertise<std_msgs::UInt8>("aubo_driver/cancel_trajectory",100);
    io_srv_ = nh_.advertiseService("/aubo_driver/set_io",&AuboDriver::setIO, this);
    ik_srv_ = nh_.advertiseService("/aubo_driver/get_ik",&AuboDriver::getIK, this);
    fk_srv_ = nh_.advertiseService("/aubo_driver/get_fk",&AuboDriver::getFK, this);

    /** subscribe topics **/
    trajectory_execution_subs_ = nh_.subscribe("trajectory_execution_event", 10, &AuboDriver::trajectoryExecutionCallback,this);
    robot_control_subs_ = nh_.subscribe("robot_control", 10, &AuboDriver::robotControlCallback,this);
    moveit_controller_subs_ = nh_.subscribe("moveItController_cmd", 2000, &AuboDriver::moveItPosCallback,this);
    teach_subs_ = nh_.subscribe("teach_cmd", 10, &AuboDriver::teachCallback,this);
    moveAPI_subs_ = nh_.subscribe("moveAPI_cmd", 10, &AuboDriver::AuboAPICallback, this);
    controller_switch_sub_ = nh_.subscribe("/aubo_driver/controller_switch", 10, &AuboDriver::controllerSwitchCallback, this);
}

AuboDriver::~AuboDriver()
{
    /** leave Tcp2CanbusMode, surrender the control to the robot-controller**/
    if(control_option_ == aubo_driver::RosMoveIt)
        robot_send_service_.robotServiceLeaveTcp2CanbusMode();
    /** log out　**/
    robot_send_service_.robotServiceLogout();
    robot_receive_service_.robotServiceLogout();
}

void AuboDriver::timerCallback(const ros::TimerEvent& e)
{
    if(controller_connected_flag_)
    {
        /** Query the states of robot joints **/
        int ret = robot_receive_service_.robotServiceGetCurrentWaypointInfo(rs.wayPoint_);      /** this method upates the joint states more quickly **/
        if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
        {
            double joints[ARM_DOF];
            for(int i = 0; i < 6; i++)
                joints[i] = rs.wayPoint_.jointpos[i];
            setCurrentPosition(joints);  // update the current robot joint states to ROS Controller

            /** Get the buff size of thr rib **/
            robot_receive_service_.robotServiceGetRobotDiagnosisInfo(rs.robot_diagnosis_info_);
            rib_buffer_size_ = rs.robot_diagnosis_info_.macTargetPosDataSize;

//            robot_receive_service_.robotServiceGetRobotCurrentState(rs.state_);            // this is controlled by Robot Controller
//            robot_receive_service_.getErrDescByCode(rs.code_);
            if(real_robot_exist_)
            {
                // publish robot_status information to the controller action server.
                robot_status_.mode.val            = (int8)rs.robot_diagnosis_info_.orpeStatus;
                robot_status_.e_stopped.val       = (int8)(rs.robot_diagnosis_info_.softEmergency || emergency_stopped_);
                robot_status_.drives_powered.val  = (int8)rs.robot_diagnosis_info_.armPowerStatus;
                robot_status_.motion_possible.val = (int)(!start_move_);
                robot_status_.in_motion.val       = (int)start_move_;
                robot_status_.in_error.val        = (int)protective_stopped_;   //used for protective stop.
                robot_status_.error_code          = (int32)rs.robot_diagnosis_info_.singularityOverSpeedAlarm;
            }
        }
        else if(ret == aubo_robot_namespace::ErrCode_SocketDisconnect)
        {
            /** Here we check the connection to satisfy the ROS-I specification **/
            /** Try to connect with the robot again **/
            if(!connectToRobotController())
            {
                ROS_ERROR("Cann't connect to the robot controller!");
            }
        }

        //publish the rib_status to the controller simulator
        rib_status_.data[0] = buf_queue_.size();
        rib_status_.data[1] = control_mode_;
        rib_status_.data[2] = controller_connected_flag_;
    }
    else
    {
        /** maintain the ros-controller states from the ros environment **/
        setCurrentPosition(target_point_);      //return back immediately
    }

    robot_status_pub_.publish(robot_status_);
    rib_pub_.publish(rib_status_);

    if(control_mode_ == aubo_driver::SynchronizeWithRealRobot /*|| rs.robot_controller == ROBOT_CONTROLLER*/)
    {
        if(controller_connected_flag_)
        {
            sensor_msgs::JointState joint_state;
            joint_state.header.stamp = ros::Time::now();
            joint_state.name.resize(axis_number_);
            joint_state.position.resize(axis_number_);
            for(int i = 0; i<axis_number_; i++)
            {
                joint_state.name[i] = joint_name_[i];
                joint_state.position[i] = current_joints_[i];
            }
            //            joint_states_pub_.publish(joint_state);

            memcpy(last_recieve_point_, current_joints_, sizeof(double) * axis_number_);
            memcpy(target_point_, current_joints_, sizeof(double) * axis_number_);
        }
        else
        {
            ROS_INFO("There is no to the robot controller!");
        }
    }
    else if(control_mode_ == aubo_driver::SendTargetGoal)
    {
        sensor_msgs::JointState joint_state;
        control_msgs::FollowJointTrajectoryFeedback joint_feedback;

        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(axis_number_);
        joint_feedback.joint_names.resize(axis_number_);
        joint_state.position.resize(axis_number_);
        joint_feedback.actual.positions.resize(axis_number_);
        for(int i = 0; i<axis_number_; i++)
        {
            joint_state.name[i] = joint_name_[i];
            if(controller_connected_flag_)
                joint_state.position[i] = current_joints_[i];
            else
                joint_state.position[i] = target_point_[i];

            joint_feedback.joint_names[i] = joint_name_[i];
            joint_feedback.actual.positions[i] = joint_state.position[i];
        }
        joint_states_pub_.publish(joint_state);
        joint_feedback_pub_.publish(joint_feedback);

        /** If the controller is robot-controller, then synchronize the ros-controller states **/
        if(control_option_ == aubo_driver::AuboAPI)
        {
            memcpy(target_point_, current_joints_, sizeof(double) * axis_number_);
            std_msgs::Float32MultiArray joints;
            joints.data.resize(axis_number_);
            for(int i = 0; i<axis_number_; i++)
            {
                joints.data[i] = target_point_[i];
            }
            joint_target_pub_.publish(joints);
        }
    }
}

bool AuboDriver::roadPointCompare(double *point1, double *point2)
{
    /** If there is a enough difference, then it will return true. **/
    bool ret = false;
    for(int i = 0; i < axis_number_;i++)
    {
        if(fabs(point1[i] - point2[i]) >= THRESHHOLD)
        {
            ret = true;
            break;
        }
    }
    return ret;
}

double* AuboDriver::getCurrentPosition()
{
    return current_joints_;
}

void AuboDriver::setCurrentPosition(double *target)
{
    for(int i = 0; i < axis_number_;i++)
    {
        current_joints_[i] = target[i];
    }
}

double* AuboDriver::getTagrtPosition()
{
    return target_point_;
}

void AuboDriver::setTagrtPosition(double *target)
{
    for(int i = 0; i < axis_number_;i++)
    {
        target_point_[i] = target[i];
    }
}

bool AuboDriver::setRobotJointsByMoveIt()
{
    int ret = 0;
    // First check if the buf_queue_ is Empty
    if(!buf_queue_.empty())
    {
        PlanningState ps = buf_queue_.front();
        buf_queue_.pop();
     

        if(controller_connected_flag_)      // actually no need this judgment
        {
            if(emergency_stopped_)
            {
                //clear the buffer, there will be a jerk
                start_move_ = false;
                while(!buf_queue_.empty())
                    buf_queue_.pop();
            }
            else if(protective_stopped_ || normal_stopped_)
            {
                //cancle.data will be set 0 in the aubo_robot_simulator.py when clear this one trajectory data

                std_msgs::UInt8 cancle;
                cancle.data = 1;
                cancle_trajectory_pub_.publish(cancle);

                //first slow down, until the velocity to 0.
                memcpy(&jti.currentPosition[0], ps.joint_pos_, axis_number_*sizeof(double));
                memcpy(&jti.currentVelocity[0], ps.joint_vel_, axis_number_*sizeof(double));
                memcpy(&jti.currentAcceleration[0], ps.joint_acc_, axis_number_*sizeof(double));
                memset(&jti.targetVelocity[0], 0, axis_number_*sizeof(double));
                bool update = otgVelocityModeParameterUpdate(jti);
                int resultValue = 0;
                while(resultValue != 1)
                {
                   resultValue = otgVelocityModeResult(1, jto);
                   double jointAngle[] = {jto.newPosition[0],jto.newPosition[1],jto.newPosition[2],jto.newPosition[3],jto.newPosition[4],jto.newPosition[5]};
                   ret = robot_send_service_.robotServiceSetRobotPosData2Canbus(jointAngle);
                   //std::cout<<jointAngle[0]<<","<<jointAngle[1]<<","<<jointAngle[2]<<","<<jointAngle[3]<<","<<jointAngle[4]<<","<<jointAngle[5]<<","<<std::endl;

                }
                //clear the buffer
                start_move_ = false;
                while(!buf_queue_.empty())
                    buf_queue_.pop();

                //clear the flag
                if(normal_stopped_)
                {
                    normal_stopped_ = false;
                    delay_clear_times = STOP_DELAY_CLEAR_TIMES;
                }
            }
            else
            {
                ret = robot_send_service_.robotServiceSetRobotPosData2Canbus(ps.joint_pos_);
            }
#ifdef LOG_INFO_DEBUG
            //            struct timeb tb;
            //            ftime(&tb);
            //            std::cout<<tb.millitm<<std::endl;
            //std::cout<<ps.joint_pos_[0]<<","<<ps.joint_pos_[1]<<","<<ps.joint_pos_[2]<<","<<ps.joint_pos_[3]<<","<<ps.joint_pos_[4]<<","<<ps.joint_pos_[5]<<std::endl;
#endif
        }
        setTagrtPosition(ps.joint_pos_);
    }
    else
    {
        if(start_move_)
            start_move_ = false;
    }
}

void AuboDriver::controllerSwitchCallback(const std_msgs::Int32::ConstPtr &msg)
{
    //update the control option
    int controller_type = msg->data;
    if(controller_type == control_option_)
    {
        std::string conType = (control_option_== aubo_driver::AuboAPI)?"robot-controller":"ros-controller";
        ROS_INFO("The controller type is: %s", conType.c_str());
        return;
    }

    int ret = aubo_robot_namespace::InterfaceCallSuccCode;
    if(controller_type == aubo_driver::AuboAPI)
    {
        if(start_move_)
        {
            ROS_WARN("The robot is controlled by the ros-controller now, try to switch the controller after the robot comes to stop!");
            return;
        }
        ret = robot_send_service_.robotServiceLeaveTcp2CanbusMode();
        if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
        {
            ROS_INFO("Switches to robot-controller successfully");
            control_option_ = aubo_driver::AuboAPI;
        }
        else
            ROS_INFO("Failed to switch to robot-controller");
    }
    else if(controller_type == aubo_driver::RosMoveIt)
    {
        ret = robot_send_service_.robotServiceEnterTcp2CanbusMode();
        if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
        {
            ROS_INFO("Switches to ros-controller successfully");
            control_option_ = aubo_driver::RosMoveIt;
        }
        else
            ROS_INFO("Failed to switch to ros-controller, make sure there is no other controller which is controlling the robot to move.");
    }
    else
    {
        ROS_INFO("Undefined controller type, failed to switch the controller!");
    }
}

void AuboDriver::moveItPosCallback(const trajectory_msgs::JointTrajectoryPoint::ConstPtr &msg)
{
    double jointAngle[axis_number_];
    for(int i = 0; i < axis_number_; i++)
        jointAngle[i] = msg->positions[i];

    if(controller_connected_flag_)
    {
        /** The need a buffer to connect to the RIB to confirm the REAL TIME**/
        if(roadPointCompare(jointAngle, last_recieve_point_))
        {
            //            data_recieved_ = true;
            ROS_DEBUG("Add new waypoint to the buffer.");
            data_count_ = 0;
            PlanningState ps;

            memcpy(ps.joint_pos_, jointAngle, sizeof(double) * axis_number_);
            memcpy(ps.joint_vel_, &msg->velocities[0], sizeof(double) * axis_number_);
            memcpy(ps.joint_acc_, &msg->accelerations[0], sizeof(double) * axis_number_);
            memcpy(last_recieve_point_, jointAngle, sizeof(double) * axis_number_);
            buf_queue_.push(ps);
            if(buf_queue_.size() > buffer_size_ && !start_move_)
                start_move_ = true;
        }
    }
    else
    {
        setTagrtPosition(jointAngle);
        rib_buffer_size_ = 0;
    }
}

void AuboDriver::trajectoryExecutionCallback(const std_msgs::String::ConstPtr &msg)
{
    if(msg->data == "stop")
    {
        ROS_INFO("trajectory execution status: stop");
        normal_stopped_ = true;
    }
}

void AuboDriver::robotControlCallback(const std_msgs::String::ConstPtr &msg)
{
    if(msg->data == "powerOn")
    {
        int ret = aubo_robot_namespace::InterfaceCallSuccCode;
        aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
        memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));
        aubo_robot_namespace::ROBOT_SERVICE_STATE result;
        ret = robot_send_service_.rootServiceRobotStartup(toolDynamicsParam/**工具动力学参数**/,
                                                   collision_class_        /*碰撞等级*/,
                                                   true     /*是否允许读取位姿　默认为true*/,
                                                   true,    /*保留默认为true */
                                                   1000,    /*保留默认为1000 */
                                                   result); /*机械臂初始化*/
        if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
            ROS_INFO("Initial sucess.");
        else
            ROS_ERROR("Initial failed.");
    }
}

void AuboDriver::updateControlStatus()
{
    data_count_++;
    /** The max delay time is MAXALLOWEDDELAY * robot_driver.UPDATE_RATE_ = 50 * 0.002 = 0.1s **/
    if(data_count_ == MAXALLOWEDDELAY)
    {
        data_count_ = 0;
        //        data_recieved_ = false;
        /** If the total size of the trajectory is less than buffer_size_, then it will move after 0.1s **/
        if(buf_queue_.size() > 0 && !start_move_ )
            start_move_ = true;
    }

    if(delay_clear_times > 0)
    {
      while(!buf_queue_.empty())
          buf_queue_.pop();
      start_move_ = false;
      delay_clear_times--;
    }

    if(start_move_ && rib_buffer_size_ < MINIMUM_BUFFER_SIZE)
    {
        setRobotJointsByMoveIt();
    }
}

void AuboDriver::teachCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    if(control_mode_ == aubo_driver::Teach)
    {
        double type = msg->data[0];
        double teachMode = msg->data[1];
        if(type == 0)
        {
            //0 for Aubo API;
            if(controller_connected_flag_)
            {
                // robot_send_service_.robotServiceJointMove(joints, false);
            }
            else
            {
                //update the current joint angle directly
                //                memcpy(current_joints_, joints, sizeof(double) * axis_number_);
            }
        }
        else
        {
            //1 for MoveIt
        }
    }
}

void AuboDriver::AuboAPICallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    if(control_mode_ == aubo_driver::SendTargetGoal)
    {
        //ROS_INFO("goal=[%f,%f,%f,%f,%f,%f]",msg->data[0],msg->data[1],msg->data[2],msg->data[3],msg->data[4],msg->data[5]);
        double type = msg->data[0];
        if(type == 0)
        {
            /** move to a joint target **/
            double joints[6] = {msg->data[1],msg->data[2],msg->data[3],msg->data[4],msg->data[5],msg->data[6]};
            if(roadPointCompare(joints,target_point_))
            {
                memcpy(target_point_, joints, sizeof(double) * axis_number_);
                if(controller_connected_flag_)
                {
                    int ret = robot_send_service_.robotServiceJointMove(joints, true);
                    ROS_INFO("move to the goal with API! %d", ret);
                    if(ret == 0)
                    {
                        ROS_INFO("move to the goal with API!");
                    }
                }
            }
        }
        else
        {
            /** move to a pose target **/
        }

    }
}

bool AuboDriver::connectToRobotController()
{
    int ret1 = aubo_robot_namespace::InterfaceCallSuccCode;
    int ret2 = aubo_robot_namespace::InterfaceCallSuccCode;

    std::string s;
    ros::param::get("/aubo_driver/server_host", s); //The server_host should be corresponding to the robot controller setup.
    server_host_ = (s=="")? "127.0.0.1" : s;
    std::cout<<"server_host:"<<server_host_<<std::endl;

    /** log in ***/
    int max_link_times = 5;
    int count = 0;
    do {
        count++;
        ret1 = robot_send_service_.robotServiceLogin(server_host_.c_str(), server_port, "aubo", "123456");
    }while(ret1 != aubo_robot_namespace::InterfaceCallSuccCode && count < max_link_times);

    if(ret1 == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        ret2 = robot_receive_service_.robotServiceLogin(server_host_.c_str(), server_port, "aubo", "123456");
        controller_connected_flag_  = true;
        std::cout<<"login success."<<std::endl;
        /** 接口调用: 获取真实臂是否存在 **/
        ret2 = robot_receive_service_.robotServiceGetIsRealRobotExist(real_robot_exist_);
        if(ret2 == aubo_robot_namespace::InterfaceCallSuccCode)
        {
            (real_robot_exist_)? std::cout<<"real robot exist."<<std::endl:std::cout<<"real robot doesn't exist."<<std::endl;
            //power on the robot.
        }
//        ros::param::set("/aubo_driver/robot_connected","1");
        return true;
    }
    else
    {
//        ros::param::set("/aubo_driver/robot_connected","0");
        controller_connected_flag_  = false;
        std::cout<<"login failed."<<std::endl;
        return false;
    }
}

void AuboDriver::run()
{
    ROS_INFO("Start the driver!");

    /** connect to the robot controller **/
    if(connectToRobotController())
    {
        /** Switches to ros-controller **/
        int ret = robot_send_service_.robotServiceEnterTcp2CanbusMode();
        if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
        {
            ROS_INFO("Switches to ros-controller successfully");
            control_option_ = aubo_driver::RosMoveIt;
        }
        else if(ret == aubo_robot_namespace::ErrCode_ResponseReturnError)
        {
            //already connect, disconnect first.
            ret = robot_send_service_.robotServiceLeaveTcp2CanbusMode();
            ret = robot_send_service_.robotServiceEnterTcp2CanbusMode();
            if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
            {
                ROS_INFO("Switches to ros-controller successfully");
                control_option_ = aubo_driver::RosMoveIt;
            }
            else
            {
                control_option_ = aubo_driver::AuboAPI;
                ROS_WARN("Failed to switch to ros-controller, the robot is still controlled by the robot controller!");
            }
        }
        else
        {
            control_option_ = aubo_driver::AuboAPI;
            ROS_WARN("Failed to switch to ros-controller, the robot is still controlled by the robot controller!");
        }

        ret = robot_receive_service_.robotServiceGetCurrentWaypointInfo(rs.wayPoint_);
        if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
        {
            double joints[8];
            for(int i = 0; i < 6; i++)
                joints[i] = rs.wayPoint_.jointpos[i];

            setCurrentPosition(joints);
            setTagrtPosition(joints);
            //send this information to the controller simulator to initialize the position
            std_msgs::Float32MultiArray robot_joints;
            robot_joints.data.resize(axis_number_);
            for(int i = 0; i<axis_number_; i++)
            {
                robot_joints.data[i] = current_joints_[i];
            }
            /** If the driver node launched after the robot_simulator node, this will initialize the joint_positions **/
            joint_target_pub_.publish(robot_joints);
             /** If the driver node launched after the robot_simulator node, this will initialize the joint_positions **/
            ros::param::set("initial_joint_state", joints);
        }
    }

    ros::param::set("/aubo_driver/robot_connected","1");

    //communication Timer between ros node and real robot controller.
    timer_ = nh_.createTimer(ros::Duration(1.0 / TIMER_SPAN_), &AuboDriver::timerCallback, this);
    timer_.start();

    /** get the io states of the robot **/
    mb_publish_thread_ = new std::thread(boost::bind(&AuboDriver::publishIOMsg, this));
}

void AuboDriver::publishIOMsg()
{
    int ret = 0;
    ros::Rate update_rate(50);
    while (ros::ok())
    {
        aubo_msgs::IOState io_msg;
        /** robot control board IO **/
        std::vector<aubo_robot_namespace::RobotIoDesc> status_vector_in;
        std::vector<aubo_robot_namespace::RobotIoDesc> status_vector_out;
        std::vector<aubo_robot_namespace::RobotIoType>  io_type_in;
        std::vector<aubo_robot_namespace::RobotIoType>  io_type_out;
        //user IO
        io_type_in.push_back(aubo_robot_namespace::RobotBoardUserDI);
        io_type_out.push_back(aubo_robot_namespace::RobotBoardUserDO);
        ret = robot_receive_service_.robotServiceGetBoardIOStatus(io_type_in, status_vector_in);
        ret = robot_receive_service_.robotServiceGetBoardIOStatus(io_type_out, status_vector_out);
        /** F1-F6 are reserved. **/
        char num[2];
        for (unsigned int i = 6; i < status_vector_in.size(); i++)
        {
            aubo_msgs::Digital digi;
            num[0] = status_vector_in[i].ioName[5];
            num[1] = status_vector_in[i].ioName[6];
            digi.pin = std::atoi(num);
            //            digi.pin = status_vector_in[i].ioAddr - 36;
            digi.state = status_vector_in[i].ioValue;
            digi.flag = 0;
            io_msg.digital_in_states.push_back(digi);
        }
        for (unsigned int i = 0; i < status_vector_out.size(); i++)
        {
            aubo_msgs::Digital digo;
            num[0] = status_vector_out[i].ioName[5];
            num[1] = status_vector_out[i].ioName[6];
            digo.pin = std::atoi(num);
            int addr = status_vector_out[i].ioAddr;
            //            digo.pin = status_vector_out[i].ioAddr - 32;
            digo.state = status_vector_out[i].ioValue;
            digo.flag = 1;
            io_msg.digital_out_states.push_back(digo);
        }

        status_vector_in.clear();
        status_vector_out.clear();
        io_type_in.clear();
        io_type_out.clear();
        //safety digital IO
        io_type_in.push_back(aubo_robot_namespace::RobotBoardControllerDI);
        io_type_out.push_back(aubo_robot_namespace::RobotBoardControllerDO);
        ret = robot_receive_service_.robotServiceGetBoardIOStatus(io_type_in,status_vector_in);
        ret = robot_receive_service_.robotServiceGetBoardIOStatus(io_type_out,status_vector_out);
        double digitalIn[30];
        for (unsigned int i = 0; i < status_vector_in.size(); i++)
        {
            aubo_msgs::Digital digi;
            digi.pin = status_vector_in[i].ioAddr;
            digi.state = status_vector_in[i].ioValue;
            digi.flag = 0;
            digitalIn[digi.pin] = digi.state;
            io_msg.safety_in_states.push_back(digi);
        }
        if(real_robot_exist_)
        {
            //only works if there is a real robot
            if(digitalIn[0] == 0 || digitalIn[8] == 0)
                emergency_stopped_ = true;
            else
                emergency_stopped_ = false;

            if(digitalIn[1] == 0 || digitalIn[9] == 0)
                protective_stopped_ = true;
            else
                protective_stopped_ = false;
        }

//        for (unsigned int i = 0; i < status_vector_out.size(); i++)
//        {
//            aubo_msgs::Digital digo;

//            digo.pin = status_vector_out[i].ioAddr;
//            digo.state = status_vector_out[i].ioValue;
//            digo.flag = 1;
//            io_msg.safety_out_states.push_back(digo);
//        }
        status_vector_in.clear();
        status_vector_out.clear();
        io_type_in.clear();
        io_type_out.clear();

        io_type_in.push_back(aubo_robot_namespace::RobotBoardUserAI);
        io_type_out.push_back(aubo_robot_namespace::RobotBoardUserAO);
        ret = robot_receive_service_.robotServiceGetBoardIOStatus(io_type_in,status_vector_in);
        ret = robot_receive_service_.robotServiceGetBoardIOStatus(io_type_out,status_vector_out);
        for (unsigned int i = 0; i < status_vector_in.size(); i++)
        {
            aubo_msgs::Analog ana;
            ana.pin = status_vector_in[i].ioAddr;
            ana.state = status_vector_in[i].ioValue;
            io_msg.analog_in_states.push_back(ana);
        }

        for (unsigned int i = 0; i < status_vector_out.size(); i++)
        {
            aubo_msgs::Analog ana;
            ana.pin = status_vector_out[i].ioAddr;
            ana.state = status_vector_out[i].ioValue;
            io_msg.analog_out_states.push_back(ana);
        }

        // robot tool IO
        status_vector_in.clear();
        status_vector_out.clear();
        ret = robot_receive_service_.robotServiceGetAllToolDigitalIOStatus(status_vector_in);
        ret = robot_receive_service_.robotServiceGetAllToolAIStatus(status_vector_out);
        for (unsigned int i = 0; i < status_vector_in.size(); i++)
        {
            aubo_msgs::Digital digo;
            digo.pin = status_vector_in[i].ioAddr;
            digo.state = status_vector_in[i].ioValue;
            digo.flag = (status_vector_in[i].ioType == aubo_robot_namespace::RobotToolDI)? 0 : 1;
            io_msg.tool_io_states.push_back(digo);
        }

        for (unsigned int i = 0; i < status_vector_out.size(); i++)
        {
            aubo_msgs::Analog ana;
            ana.pin = status_vector_out[i].ioAddr;
            ana.state = status_vector_out[i].ioValue;
            io_msg.tool_ai_states.push_back(ana);
        }
        io_msg.stamp = "The current IO states: " + std::to_string(ros::Time::now().toSec()) + "s";
        io_pub_.publish(io_msg);

        //        msg_lock.unlock();
        update_rate.sleep();
        ros::spinOnce();
    }
}

bool AuboDriver::setIO(aubo_msgs::SetIORequest& req, aubo_msgs::SetIOResponse& resp)
{
    resp.success = true;
    if (req.fun == 1)
    {
        robot_send_service_.robotServiceSetBoardIOStatus(aubo_robot_namespace::RobotBoardUserDO,req.pin + 32, req.state);
        ros::Duration(io_flag_delay_).sleep();
    }
    else if (req.fun == 2)
    {
        robot_send_service_.robotServiceSetBoardIOStatus(aubo_robot_namespace::RobotBoardUserAO,req.pin, req.state);
        ros::Duration(io_flag_delay_).sleep();
    }
    else if (req.fun == 3)
    {
        if(req.state == -1)
        {
            robot_send_service_.robotServiceSetToolDigitalIOType((aubo_robot_namespace::ToolDigitalIOAddr)req.pin, aubo_robot_namespace::IO_IN);
            ros::Duration(io_flag_delay_).sleep();
        }
        else
        {
            robot_send_service_.robotServiceSetToolDigitalIOType((aubo_robot_namespace::ToolDigitalIOAddr)req.pin, aubo_robot_namespace::IO_OUT);
            ros::Duration(io_flag_delay_).sleep();
            robot_send_service_.robotServiceSetToolDOStatus((aubo_robot_namespace::ToolDigitalIOAddr)req.pin, (aubo_robot_namespace::IO_STATUS)req.state);
            ros::Duration(io_flag_delay_).sleep();
        }
    }
    else if (req.fun == 4)
    {
        robot_send_service_.robotServiceSetBoardIOStatus(aubo_robot_namespace::RobotToolAO,req.pin, req.state);
        ros::Duration(io_flag_delay_).sleep();
    }
    else if (req.fun == 5)
    {
        //0->0V; 1->12V; 2->24V
        robot_send_service_.robotServiceSetToolPowerVoltageType((aubo_robot_namespace::ToolPowerType) req.state);
    }
    else
    {
        resp.success = false;
    }
    return resp.success;
}

bool AuboDriver::getFK(aubo_msgs::GetFKRequest& req, aubo_msgs::GetFKResponse& resp)
{
    aubo_robot_namespace::wayPoint_S wayPoint;
    double joint[] = {req.joint[0],req.joint[1],req.joint[2],req.joint[3],req.joint[4],req.joint[5]};
    robot_send_service_.robotServiceRobotFk(joint, 6, wayPoint);
    resp.pos.push_back(wayPoint.cartPos.position.x);
    resp.pos.push_back(wayPoint.cartPos.position.y);
    resp.pos.push_back(wayPoint.cartPos.position.z);
    resp.ori.push_back(wayPoint.orientation.w);
    resp.ori.push_back(wayPoint.orientation.x);
    resp.ori.push_back(wayPoint.orientation.y);
    resp.ori.push_back(wayPoint.orientation.z);
}

bool AuboDriver::getIK(aubo_msgs::GetIKRequest& req, aubo_msgs::GetIKResponse& resp)
{
    aubo_robot_namespace::wayPoint_S wayPoint;
    double joint[] = {req.ref_joint[0],req.ref_joint[1],req.ref_joint[2],req.ref_joint[3],req.ref_joint[4],req.ref_joint[5]};
    aubo_robot_namespace::Pos position;
    position.x = req.pos[0]; position.y = req.pos[1]; position.z = req.pos[2];
    aubo_robot_namespace::Ori ori;
    ori.w = req.ori[0]; ori.x = req.ori[1]; ori.y = req.ori[2]; ori.z = req.ori[3];
    robot_send_service_.robotServiceRobotIk(joint, position, ori, wayPoint);
    resp.joint.push_back(wayPoint.jointpos[0]);
    resp.joint.push_back(wayPoint.jointpos[1]);
    resp.joint.push_back(wayPoint.jointpos[2]);
    resp.joint.push_back(wayPoint.jointpos[3]);
    resp.joint.push_back(wayPoint.jointpos[4]);
    resp.joint.push_back(wayPoint.jointpos[5]);
}

}

