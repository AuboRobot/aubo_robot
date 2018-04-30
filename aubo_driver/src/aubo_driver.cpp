#include "aubo_driver/aubo_driver.h"

#include <string>
#include <sys/timeb.h>

namespace aubo_driver {

// robot status parameters
bool AuboDriver::real_robot_exist_ = false;
int AuboDriver::controller_connected_flag_ = 0;
bool AuboDriver::start_move_ = false;
double AuboDriver::last_recieve_point_[ARM_DOF] = {0};
double AuboDriver::current_joints_[ARM_DOF] = {0};
double AuboDriver::target_point_[ARM_DOF] = {0};
int AuboDriver::control_mode_ = aubo_driver::SendTargetGoal;
int AuboDriver::rib_buffer_size_ = 0;
bool AuboDriver::data_recieved_ = false;
int AuboDriver::data_count_ = false;
int AuboDriver::control_option_ = aubo_driver::RosMoveIt;
std::string AuboDriver::joint_name_[ARM_DOF] = {"shoulder_joint","upperArm_joint","foreArm_joint","wrist1_joint","wrist2_joint","wrist3_joint"};

AuboDriver::AuboDriver():buffer_size_(200),io_flag_delay_(0.02)
{
    rs.robot_controller_ = ROBOT_CONTROLLER;
    rib_status_.data.resize(3);
    old_rib_status_[0] = 0; old_rib_status_[1] = 0;old_rib_status_[2] = 0;
    joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 300);
    joint_target_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("real_pose", 50);
    robot_status_pub_ = nh_.advertise<industrial_msgs::RobotStatus>("robot_status", 100);

    io_pub_ = nh_.advertise<aubo_msgs::IOState>("aubo_driver/io_state", 1);

    rib_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("rib_status", 100);
    io_srv_ = nh_.advertiseService("aubo_driver/set_io",&AuboDriver::setIO, this);

    /** update_joint_state **/
    moveit_controller_subs_ = nh_.subscribe("moveItController_cmd", 2000, &AuboDriver::moveItPosCallback,this);
    teach_subs_ = nh_.subscribe("teach_cmd", 10, &AuboDriver::teachCallback,this);
    moveAPI_subs_ = nh_.subscribe("moveAPI_cmd", 10, &AuboDriver::AuboAPICallback,this);

    plan_type_sub_ = nh_.subscribe("plan_type", 1, &AuboDriver::planTypeCallback,this);
}

AuboDriver::~AuboDriver()
{
    /** leave Tcp2CanbusMode**/
    robot_send_service_.robotServiceLeaveTcp2CanbusMode();
    /** log out　**/
    robot_send_service_.robotServiceLogout();
}

void AuboDriver::timerCallback(const ros::TimerEvent& e)
{
    if(controller_connected_flag_)
    {
        /** Query the states of robot joints **/
        int ret = robot_receive_service_.robotServiceGetRobotJointStatus(rs.joint_status_, 6);
        if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
        {
            double joints[] = {rs.joint_status_[0].jointPosJ,rs.joint_status_[1].jointPosJ,rs.joint_status_[2].jointPosJ,rs.joint_status_[3].jointPosJ,rs.joint_status_[4].jointPosJ,rs.joint_status_[5].jointPosJ};
            setCurrentPosition(joints);
        }
        else if(ret == aubo_robot_namespace::ErrCode_SocketDisconnect)
        {
            /** Here we check the connection to satisfy the ROS-I specification **/
            /** Try to connect with the robot again **/
            int max_link_times = 5;
            int count = 0;
            do {
                count++;
                ret = robot_send_service_.robotServiceLogin(server_host_.c_str(), server_port, "aubo", "123456");
            }while(ret != aubo_robot_namespace::InterfaceCallSuccCode && count < max_link_times);

            if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
            {
                controller_connected_flag_ = false;
                ROS_ERROR("Cann't connect to the robot controller!");
            }
            else
            {
                robot_receive_service_.robotServiceLogin(server_host_.c_str(), server_port, "aubo", "123456");
            }
        }

        /** Get the buff size of thr rib **/
        if(ret = robot_receive_service_.robotServiceGetRobotDiagnosisInfo(rs.robot_diagnosis_info_) == 0)
        {
            rib_buffer_size_ = rs.robot_diagnosis_info_.macTargetPosDataSize;
        }
        robot_receive_service_.robotServiceGetRobotCurrentState(rs.state_);
        robot_receive_service_.getErrDescByCode(rs.code_);
    }

    {
        // publish robot_status information to the controller action server.
        robot_status_.mode.val            = (int8)rs.robot_diagnosis_info_.orpeStatus;
        robot_status_.e_stopped.val       = (int8)rs.robot_diagnosis_info_.softEmergency;
        robot_status_.drives_powered.val  = (int8)rs.robot_diagnosis_info_.armPowerStatus;
        robot_status_.motion_possible.val = (int8)rs.state_;
        robot_status_.in_motion.val       = (int8)rs.state_;
//        robot_status_.in_error.val        = (int8)rs.code_;
//        robot_status_.error_code          = (int32)rs.code_;
        robot_status_pub_.publish(robot_status_);
    }

    if(control_mode_ == aubo_driver::SynchronizeWithRealRobot /*|| rs.robot_controller == ROBOT_CONTROLLER*/)
    {
        if(controller_connected_flag_)
        {
            sensor_msgs::JointState joint_state;
            joint_state.header.stamp = ros::Time::now();
            joint_state.name.resize(ARM_DOF);
            joint_state.position.resize(ARM_DOF);
            for(int i = 0; i<ARM_DOF; i++)
            {
                joint_state.name[i] = joint_name_[i];
                joint_state.position[i] = current_joints_[i];
            }
//            joint_states_pub_.publish(joint_state);

            memcpy(last_recieve_point_, current_joints_, sizeof(double) * ARM_DOF);
            memcpy(target_point_, current_joints_, sizeof(double) * ARM_DOF);
        }
        else
        {
            ROS_INFO("There is No Real Robot!!");
        }
    }
    else if(control_mode_ == aubo_driver::SendTargetGoal)
    {
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(ARM_DOF);
        joint_state.position.resize(ARM_DOF);
        for(int i = 0; i<ARM_DOF; i++)
        {
            joint_state.name[i] = joint_name_[i];
            joint_state.position[i] = current_joints_[i];
        }
        joint_states_pub_.publish(joint_state);

        if(control_option_ == aubo_driver::AuboAPI)
        {
            memcpy(target_point_, current_joints_, sizeof(double) * ARM_DOF);
            std_msgs::Float32MultiArray joints;
            joints.data.resize(ARM_DOF);
            for(int i = 0; i<ARM_DOF; i++)
            {
                joints.data[i] = target_point_[i];
            }
            joint_target_pub_.publish(joints);
        }
    }

    //publish the rib_status to the controller simulator
    rib_status_.data[0] = rib_buffer_size_;
    rib_status_.data[1] = control_mode_;
    rib_status_.data[2] = controller_connected_flag_;
    bool pub = false;
    for(int i = 0;i<3;i++)
    {
        if(rib_status_.data[i] != old_rib_status_[i])
        {
            pub = true;
            break;
        }
    }
    if(pub)
    {
        for(int i = 0;i<3;i++)
        {
            old_rib_status_[i] = rib_status_.data[i];
        }
        rib_pub_.publish(rib_status_);
    }
}

bool AuboDriver::roadPointCompare(double *point1, double *point2)
{
    // If there is a enough difference, then it will return true.
    bool ret = false;
    for(int i = 0; i < ARM_DOF;i++)
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
    for(int i = 0; i < ARM_DOF;i++)
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
    for(int i = 0; i < ARM_DOF;i++)
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
        PlanningState ps = buf_queue_.deQueue();
        if(controller_connected_flag_)
        {
            ret = robot_send_service_.robotServiceSetRobotPosData2Canbus(ps.current_joint_pos_);
//            struct timeb tb;
//            ftime(&tb);
//            std::cout<<tb.millitm<<std::endl;
//            std::cout<<ps.current_joint_pos_[0]<<","<<ps.current_joint_pos_[1]<<","<<ps.current_joint_pos_[2]<<","<<ps.current_joint_pos_[3]<<","<<ps.current_joint_pos_[4]<<","<<ps.current_joint_pos_[5]<<std::endl;
        }
        else
        {
            ros::param::set("/rib_buffer_size_","0");
        }
        setTagrtPosition(ps.current_joint_pos_);
    }
    else
    {
        if(start_move_ && !data_recieved_)
        {
            start_move_ = false;
            robot_send_service_.robotServiceLeaveTcp2CanbusMode();
            rs.robot_controller_ = ROBOT_CONTROLLER;
            std::cout<<"leave Tcp2CanbusMode:";
        }
    }
}

void AuboDriver::planTypeCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
    //update the control option and the control mode
    control_option_ = msg->data[0];
    control_mode_ = msg->data[1];
}

void AuboDriver::moveItPosCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    if(control_mode_ == aubo_driver::SendTargetGoal)
    {
        double jointAngle[ARM_DOF];
        for(int i = 0; i < ARM_DOF; i++)
            jointAngle[i] = msg->position[i];
        //    memcpy(ps.current_joint_pos_, jointAngle, sizeof(double) * ARM_DOF);

        if(roadPointCompare(jointAngle,last_recieve_point_))
        {
            data_recieved_ = true;
            data_count_ = 0;

//            std::cout<<"nsec:"<<msg->header.stamp.nsec<<std::endl;
            PlanningState ps;
            memcpy(ps.current_joint_pos_, jointAngle, sizeof(double) * ARM_DOF);
            memcpy(last_recieve_point_, jointAngle, sizeof(double) * ARM_DOF);
            buf_queue_.enQueue(ps);
            if(buf_queue_.getQueueSize() > buffer_size_ && !start_move_)
            {
                //there is a problem, if the size is less than buffer_size_, then the robot will not move!!
                start_move_ = true;
                if(aubo_robot_namespace::InterfaceCallSuccCode != robot_send_service_.robotServiceEnterTcp2CanbusMode())
                {
                    //enter moveit control mode fail, notify the simulator controller to stop the trajectory execution
//                    robotstatus.in_motion.val       = (int8)rs.state;
                    rs.robot_controller_ = ROBOT_CONTROLLER;
                }
                else
                {
//                    robotstatus.in_motion.val       =  5;
                    rs.robot_controller_ = ROS_CONTROLLER;
                }
                std::cout<<"Enter Tcp2CanbusMode."<<std::endl;
            }
        }
    }
}

void AuboDriver::updateControlStatus()
{
    data_count_++;
    if(data_count_ == MAXALLOWEDDELAY)
    {
        data_count_ = 0;
        data_recieved_ = false;
    }
    if(start_move_)
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
//                memcpy(current_joints_, joints, sizeof(double) * ARM_DOF);
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
                memcpy(target_point_, joints, sizeof(double) * ARM_DOF);
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

void AuboDriver::run()
{
//    ros::Rate wait(10);
//    std::string start_signal = "0";
//    while(start_signal == "0")
//    {
//        ros::param::get("/driver_start",start_signal);
//        wait.sleep();
//    }
    ROS_INFO("Start the driver!");
    int ret1 = aubo_robot_namespace::InterfaceCallSuccCode;
    int ret2 = aubo_robot_namespace::InterfaceCallSuccCode;
    std::string s;

    ros::param::get("/aubo_driver/server_host", s); //The server_host should be corresponding to the robot controller setup.
    server_host_ = (s=="")? "192.168.80.5" : s;
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
        aubo_robot_namespace::JointStatus joint_status[ARM_DOF];
        int ret = robot_send_service_.robotServiceGetRobotJointStatus(joint_status, ARM_DOF);
        if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
        {
            double joints[] = {joint_status[0].jointPosJ,joint_status[1].jointPosJ,joint_status[2].jointPosJ,joint_status[3].jointPosJ,joint_status[4].jointPosJ,joint_status[5].jointPosJ};
            setCurrentPosition(joints);
            memcpy(target_point_, current_joints_, sizeof(double) * ARM_DOF);
            //send this information to the controller simulator to initialize the position
            std_msgs::Float32MultiArray robot_joints;
            robot_joints.data.resize(ARM_DOF);
            for(int i = 0; i<ARM_DOF; i++)
            {
                robot_joints.data[i] = target_point_[i];
            }

            joint_target_pub_.publish(robot_joints);
            ros::param::set("/aubo_driver/robot_connected","1");
        }
        ret2 = robot_receive_service_.robotServiceLogin(server_host_.c_str(), server_port, "aubo", "123456");
        controller_connected_flag_  = 1;
        std::cout<<"login success."<<std::endl;
        /** 接口调用: 获取真实臂是否存在 **/
        ret2 = robot_receive_service_.robotServiceGetIsRealRobotExist(real_robot_exist_);
        if(ret2 == aubo_robot_namespace::InterfaceCallSuccCode)
        {
            (real_robot_exist_)? std::cout<<"real robot exist."<<std::endl:std::cout<<"real robot doesn't exist."<<std::endl;
        }

        //communication Timer between ros node and real robot controller.
        timer_ = nh_.createTimer(ros::Duration(1.0 / TIMER_SPAN_), &AuboDriver::timerCallback, this);
        timer_.start();

        /** get the io states of the robot **/
        mb_publish_thread_ = new std::thread(boost::bind(&AuboDriver::publishIOMsg, this));
    }
    else
    {
        ros::param::set("/aubo_driver/robot_connected","0");
        controller_connected_flag_  = 0;
        std::cout<<"login failed."<<std::endl;
    }
}

void AuboDriver::publishIOMsg()
{
    int ret = 0;

    ros::Rate update_rate(100);
    while (ros::ok())
    {
        aubo_msgs::IOState io_msg;
        /** robot control board IO **/
        std::vector<aubo_robot_namespace::RobotIoDesc> status_vector_in;
        std::vector<aubo_robot_namespace::RobotIoDesc> status_vector_out;
        std::vector<aubo_robot_namespace::RobotIoType>  io_type_in;
        std::vector<aubo_robot_namespace::RobotIoType>  io_type_out;
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
}


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


