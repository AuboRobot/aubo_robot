#include "aubo_driver/aubo_driver.h"

#include <string>
#include <sys/timeb.h>

namespace aubo_driver {

// robot status parameters
bool AuboDriver::IsRealRobotExist = false;
int AuboDriver::ControllerConnectedFlag = 0;
bool AuboDriver::startMove = false;
double AuboDriver::lastRecievePoint[ARM_DOF] = {0};
double AuboDriver::currentJoints[ARM_DOF] = {0};
double AuboDriver::targetPoint[ARM_DOF] = {0};
int AuboDriver::controlMode = aubo_driver::SendTargetGoal;
int AuboDriver::ribbuffersize = 0;
bool AuboDriver::dataRecieved = false;
int AuboDriver::dataCount = false;
int AuboDriver::controlOption = aubo_driver::RosMoveIt;
std::string AuboDriver::jointname[ARM_DOF] = {"shoulder_joint","upperArm_joint","foreArm_joint","wrist1_joint","wrist2_joint","wrist3_joint"};

AuboDriver::AuboDriver():BufferSize(200),io_flag_delay_(0.02)
{
    rs.robot_controller = ROBOT_CONTROLLER;
    ribstatus.data.resize(3);
    oldribstatus[0] = 0; oldribstatus[1] = 0;oldribstatus[2] = 0;
    jointstates_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 300);
    jointtarget_pub = nh.advertise<std_msgs::Float32MultiArray>("real_pose", 50);
    robot_status_pub = nh.advertise<industrial_msgs::RobotStatus>("robot_status", 100);

    io_pub = nh.advertise<aubo_msgs::IOState>("aubo_driver/io_state", 1);

    rib_pub = nh.advertise<std_msgs::Int32MultiArray>("rib_status", 100);
    io_srv_ = nh.advertiseService("aubo_driver/set_io",&AuboDriver::setIO, this);

    //    update_joint_state
    moveIt_controller_subs = nh.subscribe("moveItController_cmd", 2000, &AuboDriver::MoveItPosCallback,this);
    teach_subs = nh.subscribe("teach_cmd", 10, &AuboDriver::TeachCallback,this);
    moveAPI_subs = nh.subscribe("moveAPI_cmd", 10, &AuboDriver::AuboAPICallback,this);

    plan_type_sub = nh.subscribe("plan_type", 1, &AuboDriver::plantypeCallback,this);
}

AuboDriver::~AuboDriver()
{
    /** leave Tcp2CanbusMode**/
    robotService.robotServiceLeaveTcp2CanbusMode();
    /** log out　**/
    robotService.robotServiceLogout();

}

void AuboDriver::timerCallback(const ros::TimerEvent& e)
{
    if(ControllerConnectedFlag)
    {
        /** 接口调用: 获取关节状态 **/
        int ret = robotService1.robotServiceGetRobotJointStatus(rs.jointStatus, 6);
        if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
        {
            double joints[] = {rs.jointStatus[0].jointPosJ,rs.jointStatus[1].jointPosJ,rs.jointStatus[2].jointPosJ,rs.jointStatus[3].jointPosJ,rs.jointStatus[4].jointPosJ,rs.jointStatus[5].jointPosJ};
            setCurrentPosition(joints);
        }
//        Get the buff size of thr rib

        if(ret = robotService1.robotServiceGetRobotDiagnosisInfo(rs.robotDiagnosisInfo) == 0)
        {
//                std::cout<<robotDiagnosisInfo.macTargetPosDataSize<<std::endl;
//            std::stringstream sstr2;
//            sstr2<<robotDiagnosisInfo.macTargetPosDataSize;
//            std::string str2;
//            sstr2>>str2;
//            ros::param::set("/RIBBufferSize",str2);
            ribbuffersize = rs.robotDiagnosisInfo.macTargetPosDataSize;
        }

        ret = robotService1.robotServiceGetRobotCurrentState(rs.state);
//        robotService1.getErrDescByCode(rs.code);
    }
    {
        // pub robot_status information to the controller action server.
        robotstatus.mode.val            = (int8)rs.robotDiagnosisInfo.orpeStatus;
        robotstatus.e_stopped.val       = (int8)rs.robotDiagnosisInfo.softEmergency;
        robotstatus.drives_powered.val  = (int8)rs.robotDiagnosisInfo.armPowerStatus;
        robotstatus.motion_possible.val = (int8)rs.state;
        robotstatus.in_motion.val       = (int8)rs.state;
//        robotstatus.in_error.val        = (int8)rs.code;
//        robotstatus.error_code          = (int32)rs.code;
        robot_status_pub.publish(robotstatus);
    }
    if(controlMode == aubo_driver::SynchronizeWithRealRobot /*|| rs.robot_controller == ROBOT_CONTROLLER*/)
    {
        if(ControllerConnectedFlag)
        {
            sensor_msgs::JointState joint_state;
            joint_state.header.stamp = ros::Time::now();
            joint_state.name.resize(ARM_DOF);
            joint_state.position.resize(ARM_DOF);
            for(int i = 0; i<ARM_DOF; i++)
            {
                joint_state.name[i] = jointname[i];
                joint_state.position[i] = currentJoints[i];
            }
//            jointstates_pub.publish(joint_state);

            memcpy(lastRecievePoint, currentJoints, sizeof(double) * ARM_DOF);
            memcpy(targetPoint, currentJoints, sizeof(double) * ARM_DOF);
        }
        else
        {
            ROS_ERROR("There is No Real Robot!!");
        }
    }
    else if(controlMode == aubo_driver::SendTargetGoal)
    {
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(ARM_DOF);
        joint_state.position.resize(ARM_DOF);
        for(int i = 0; i<ARM_DOF; i++)
        {
            joint_state.name[i] = jointname[i];
            joint_state.position[i] = currentJoints[i];
        }
        jointstates_pub.publish(joint_state);

        if(controlOption == aubo_driver::AuboAPI)
        {
            memcpy(targetPoint, currentJoints, sizeof(double) * ARM_DOF);
            std_msgs::Float32MultiArray joints;
            joints.data.resize(ARM_DOF);
            for(int i = 0; i<ARM_DOF; i++)
            {
                joints.data[i] = targetPoint[i];
            }

            jointtarget_pub.publish(joints);
        }
    }

    //publish the rib_status to the controller simulator
    ribstatus.data[0] = ribbuffersize;
    ribstatus.data[1] = controlMode;
    ribstatus.data[2] = ControllerConnectedFlag;
    bool pub = false;
    for(int i = 0;i<3;i++)
    {
        if(ribstatus.data[i] != oldribstatus[i])
        {
            pub = true;
            break;
        }
    }
    if(pub)
    {
        for(int i = 0;i<3;i++)
        {
            oldribstatus[i] = ribstatus.data[i];
        }
        rib_pub.publish(ribstatus);
    }
}

bool AuboDriver::road_point_compare(double *point1, double *point2)
{
    // If there is a enough difference, then it will return true.
    bool ret = false;
    for(int i = 0; i < ARM_DOF;i++)
    {
        if(fabs(point1[i] - point2[i])>=THRESHHOLD)
        {
            ret = true;
            break;
        }
    }
    return ret;
}

double* AuboDriver::getCurrentPosition()
{
    return currentJoints;
}

void AuboDriver::setCurrentPosition(double *target)
{
    for(int i = 0; i < ARM_DOF;i++)
    {
        currentJoints[i] = target[i];
    }
}

double* AuboDriver::getTagrtPosition()
{
    return targetPoint;
}

void AuboDriver::setTagrtPosition(double *target)
{
    for(int i = 0; i < ARM_DOF;i++)
    {
        targetPoint[i] = target[i];
    }
}

bool AuboDriver::setRobotJointsByMoveIt()
{
    int ret = 0;
    // First check if the BufQueue is Empty
    if(!bufQueue.Empty())
    {
        PlanningState ps = bufQueue.DeQueue();
        if(ControllerConnectedFlag)
        {
            ret = robotService.robotServiceSetRobotPosData2Canbus(ps.currentJointPos);
//            struct timeb tb;
//            ftime(&tb);
//            std::cout<<tb.millitm<<std::endl;
//            std::cout<<ps.currentJointPos[0]<<","<<ps.currentJointPos[1]<<","<<ps.currentJointPos[2]<<","<<ps.currentJointPos[3]<<","<<ps.currentJointPos[4]<<","<<ps.currentJointPos[5]<<std::endl;
        }
        else
        {
            ros::param::set("/RIBBufferSize","0");
        }
        setTagrtPosition(ps.currentJointPos);
    }
    else
    {
        if(startMove && !dataRecieved)
        {
            startMove = false;
            robotService.robotServiceLeaveTcp2CanbusMode();
            rs.robot_controller = ROBOT_CONTROLLER;
            std::cout<<"leave Tcp2CanbusMode:";
        }
    }
}

void AuboDriver::plantypeCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
    //update the control option and the control mode
    controlOption = msg->data[0];
    controlMode = msg->data[1];
}

void AuboDriver::MoveItPosCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    if(controlMode == aubo_driver::SendTargetGoal)
    {
        double jointAngle[ARM_DOF];
        jointAngle[0] = msg->position[0];jointAngle[1] = msg->position[1];jointAngle[2] = msg->position[2];
        jointAngle[3] = msg->position[3];jointAngle[4] = msg->position[4];jointAngle[5] = msg->position[5];
        //    memcpy(ps.currentJointPos, jointAngle, sizeof(double) * ARM_DOF);

        if(road_point_compare(jointAngle,lastRecievePoint))
        {
            dataRecieved = true;
            dataCount = 0;

//            std::cout<<"nsec:"<<msg->header.stamp.nsec<<std::endl;
            PlanningState ps;
            memcpy(ps.currentJointPos, jointAngle, sizeof(double) * ARM_DOF);
            memcpy(lastRecievePoint, jointAngle, sizeof(double) * ARM_DOF);
            bufQueue.EnQueue(ps);
            if(bufQueue.GetQueueSize() > BufferSize && !startMove)
            {
                //there is a problem, if the size is less than BufferSize, then the robot will not move!!
                startMove = true;
                if(aubo_robot_namespace::InterfaceCallSuccCode != robotService.robotServiceEnterTcp2CanbusMode())
                {
                    //enter moveit control mode fail, notify the simulator controller to stop the trajectory execution
//                    robotstatus.in_motion.val       = (int8)rs.state;
                    rs.robot_controller = ROBOT_CONTROLLER;
                }
                else
                {
//                    robotstatus.in_motion.val       =  5;
                    rs.robot_controller = ROS_CONTROLLER;
                }
                std::cout<<"Enter Tcp2CanbusMode."<<std::endl;
            }
        }
    }
}

void AuboDriver::updateControlStatus()
{
    dataCount++;
    if(dataCount == MAXALLOWEDDELAY)
    {
        dataCount = 0;
        dataRecieved = false;
    }
    if(startMove)
    {
        setRobotJointsByMoveIt();
    }
}

void AuboDriver::TeachCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    if(controlMode == aubo_driver::Teach)
    {
        double type = msg->data[0];
        double teachMode = msg->data[1];
        if(type == 0)
        {
            //0 for Aubo API;
            if(ControllerConnectedFlag)
            {
                // robotService.robotServiceJointMove(joints, false);
            }
            else
            {
                //update the current joint angle directly
//                memcpy(currentJoints, joints, sizeof(double) * ARM_DOF);
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
    if(controlMode == aubo_driver::SendTargetGoal)
    {
        //ROS_INFO("goal=[%f,%f,%f,%f,%f,%f]",msg->data[0],msg->data[1],msg->data[2],msg->data[3],msg->data[4],msg->data[5]);
        double type = msg->data[0];
        if(type == 0)
        {
            // move to a joint target
            double joints[6] = {msg->data[1],msg->data[2],msg->data[3],msg->data[4],msg->data[5],msg->data[6]};
            if(road_point_compare(joints,targetPoint))
            {
                memcpy(targetPoint, joints, sizeof(double) * ARM_DOF);
                if(ControllerConnectedFlag)
                {
                    int ret = robotService.robotServiceJointMove(joints, true);
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
            // move to a pose target

        }

    }
}

#define server_port 8899
void AuboDriver::run()
{
    ros::Rate wait(10);
    std::string startSignal = "0";
    while(startSignal == "0")
    {
        ros::param::get("/driver_start",startSignal);
        wait.sleep();
    }
    ROS_INFO("Start the driver!");
    int ret1 = aubo_robot_namespace::InterfaceCallSuccCode;
    int ret2 = aubo_robot_namespace::InterfaceCallSuccCode;
    std::string server_host;
    std::string s;

    ros::param::get("aubo_driver/server_port",s);
    server_host = (s=="")?"192.168.80.5":s;
    std::cout<<"server_host:"<<server_host<<std::endl;

    /** log in ***/
    int MaxLinkTimes = 3;
    int count = 0;
    do {
        count++;
        ret1 = robotService.robotServiceLogin(server_host.c_str(), server_port, "aubo", "123456");
    }while(ret1 != aubo_robot_namespace::InterfaceCallSuccCode && count < MaxLinkTimes);

    if(ret1 == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        aubo_robot_namespace::JointStatus jointStatus[6];
        int ret = robotService.robotServiceGetRobotJointStatus(jointStatus, 6);
        if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
        {
            double joints[] = {jointStatus[0].jointPosJ,jointStatus[1].jointPosJ,jointStatus[2].jointPosJ,jointStatus[3].jointPosJ,jointStatus[4].jointPosJ,jointStatus[5].jointPosJ};
            setCurrentPosition(joints);
            memcpy(targetPoint, currentJoints, sizeof(double) * ARM_DOF);
            //send this information to the controller simulator
            std_msgs::Float32MultiArray robotjoints;
            robotjoints.data.resize(ARM_DOF);
            for(int i = 0; i<ARM_DOF; i++)
            {
                robotjoints.data[i] = targetPoint[i];
            }

            jointtarget_pub.publish(robotjoints);
            ros::param::set("/IsRobotConnected","1");
        }
        ret2 = robotService1.robotServiceLogin(server_host.c_str(), server_port, "aubo", "123456");
        ControllerConnectedFlag  = 1;
        std::cout<<"login success."<<std::endl;
        /** 接口调用: 获取真实臂是否存在 **/
        ret2 = robotService1.robotServiceGetIsRealRobotExist(IsRealRobotExist);
        if(ret2 == aubo_robot_namespace::InterfaceCallSuccCode)
        {
            (IsRealRobotExist)? std::cout<<"真实臂存在."<<std::endl:std::cout<<"真实臂不存在."<<std::endl;
        }
    }
    else
    {
        ros::param::set("/IsRobotConnected","0");
        ControllerConnectedFlag  = 0;
        std::cout<<"login failed."<<std::endl;
    }

    //communication Timer between ros node and real robot controller.
    timer = nh.createTimer(ros::Duration(0.020),&AuboDriver::timerCallback,this);
    timer.start();

    //get the io states of the robot
    mb_publish_thread_ = new std::thread(boost::bind(&AuboDriver::publishIOMsg, this));
}

void AuboDriver::publishIOMsg()
{
    int ret = 0;

    ros::Rate update_rate(100);
    while (ros::ok())
    {
        aubo_msgs::IOState io_msg;
        // robot control board IO
        std::vector<aubo_robot_namespace::RobotIoDesc> statusVectorIn;
        std::vector<aubo_robot_namespace::RobotIoDesc> statusVectorOut;
        std::vector<aubo_robot_namespace::RobotIoType>  ioTypeIn;
        std::vector<aubo_robot_namespace::RobotIoType>  ioTypeOut;
        ioTypeIn.push_back(aubo_robot_namespace::RobotBoardUserDI);
        ioTypeOut.push_back(aubo_robot_namespace::RobotBoardUserDO);
        ret = robotService1.robotServiceGetBoardIOStatus(ioTypeIn,statusVectorIn);
        ret = robotService1.robotServiceGetBoardIOStatus(ioTypeOut,statusVectorOut);
        //F1-F6 are reserved.
        char num[2];
        for (unsigned int i = 6; i < statusVectorIn.size(); i++)
        {
            aubo_msgs::Digital digi;
            num[0] = statusVectorIn[i].ioName[5];
            num[1] = statusVectorIn[i].ioName[6];
            digi.pin = std::atoi(num);
//            digi.pin = statusVectorIn[i].ioAddr - 36;
            digi.state = statusVectorIn[i].ioValue;
            digi.flag = 0;
            io_msg.digital_in_states.push_back(digi);
        }
        for (unsigned int i = 0; i < statusVectorOut.size(); i++)
        {
            aubo_msgs::Digital digo;
            num[0] = statusVectorOut[i].ioName[5];
            num[1] = statusVectorOut[i].ioName[6];
            digo.pin = std::atoi(num);
            int addr = statusVectorOut[i].ioAddr;
//            digo.pin = statusVectorOut[i].ioAddr - 32;
            digo.state = statusVectorOut[i].ioValue;
            digo.flag = 1;
            io_msg.digital_out_states.push_back(digo);
        }

        statusVectorIn.clear();
        statusVectorOut.clear();
        ioTypeIn.clear();
        ioTypeOut.clear();
        ioTypeIn.push_back(aubo_robot_namespace::RobotBoardUserAI);
        ioTypeOut.push_back(aubo_robot_namespace::RobotBoardUserAO);
        ret = robotService1.robotServiceGetBoardIOStatus(ioTypeIn,statusVectorIn);
        ret = robotService1.robotServiceGetBoardIOStatus(ioTypeOut,statusVectorOut);
        for (unsigned int i = 0; i < statusVectorIn.size(); i++)
        {
            aubo_msgs::Analog ana;
            ana.pin = statusVectorIn[i].ioAddr;
            ana.state = statusVectorIn[i].ioValue;
            io_msg.analog_in_states.push_back(ana);
        }

        for (unsigned int i = 0; i < statusVectorOut.size(); i++)
        {
            aubo_msgs::Analog ana;
            ana.pin = statusVectorOut[i].ioAddr;
            ana.state = statusVectorOut[i].ioValue;
            io_msg.analog_out_states.push_back(ana);
        }

        // robot tool IO
        statusVectorIn.clear();
        statusVectorOut.clear();
        ret = robotService1.robotServiceGetAllToolDigitalIOStatus(statusVectorIn);
        ret = robotService1.robotServiceGetAllToolAIStatus(statusVectorOut);
        for (unsigned int i = 0; i < statusVectorIn.size(); i++)
        {
            aubo_msgs::Digital digo;
            digo.pin = statusVectorIn[i].ioAddr;
            digo.state = statusVectorIn[i].ioValue;
            digo.flag = (statusVectorIn[i].ioType == aubo_robot_namespace::RobotToolDI)? 0 : 1;
            io_msg.tool_io_states.push_back(digo);
        }

        for (unsigned int i = 0; i < statusVectorOut.size(); i++)
        {
            aubo_msgs::Analog ana;
            ana.pin = statusVectorOut[i].ioAddr;
            ana.state = statusVectorOut[i].ioValue;
            io_msg.tool_ai_states.push_back(ana);
        }
        io_pub.publish(io_msg);

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
        robotService.robotServiceSetBoardIOStatus(aubo_robot_namespace::RobotBoardUserDO,req.pin + 32, req.state);
        ros::Duration(io_flag_delay_).sleep();
    }
    else if (req.fun == 2)
    {
        robotService.robotServiceSetBoardIOStatus(aubo_robot_namespace::RobotBoardUserAO,req.pin, req.state);
        ros::Duration(io_flag_delay_).sleep();
    }
    else if (req.fun == 3)
    {
        if(req.state == -1)
        {
            robotService.robotServiceSetToolDigitalIOType((aubo_robot_namespace::ToolDigitalIOAddr)req.pin, aubo_robot_namespace::IO_IN);
            ros::Duration(io_flag_delay_).sleep();
        }
        else
        {
            robotService.robotServiceSetToolDigitalIOType((aubo_robot_namespace::ToolDigitalIOAddr)req.pin, aubo_robot_namespace::IO_OUT);
            ros::Duration(io_flag_delay_).sleep();
            robotService.robotServiceSetToolDOStatus((aubo_robot_namespace::ToolDigitalIOAddr)req.pin, (aubo_robot_namespace::IO_STATUS)req.state);
            ros::Duration(io_flag_delay_).sleep();
        }
    }
    else if (req.fun == 4)
    {
        robotService.robotServiceSetBoardIOStatus(aubo_robot_namespace::RobotToolAO,req.pin, req.state);
        ros::Duration(io_flag_delay_).sleep();
    }
    else if (req.fun == 5)
    {
        //0->0V; 1->12V; 2->24V
        robotService.robotServiceSetToolPowerVoltageType((aubo_robot_namespace::ToolPowerType) req.state);
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

    AuboDriver aubodriver;
    aubodriver.run();

    ros::AsyncSpinner spinner(6);
    spinner.start();
//    ros::spin();

    ros::Rate loop_rate(500);
    while(ros::ok())
    {
        aubodriver.updateControlStatus();
        loop_rate.sleep();
        ros::spinOnce();
    }
    ROS_WARN("Exiting aubo_driver");
    return(0);
}


