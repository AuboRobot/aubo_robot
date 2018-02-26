/*##############################################################################################################################################
Note:aubo_driver is a sample of TCP/IP interface for AUBO-i5.
There ara some main API for application as follow,please see our_control_api.h for more information.
##############################################################################################################################################
void init_move_profile();                                //初始化move的属性
void set_scurve(int val);                                //设置S曲线是否有效
void set_tcp(double *tcp_pose,int count);                //设置TCP参数
void set_relative_offset(double *offset,int count);      //设置MOVE的偏移量
void set_wait_move_finish(int val);                      //设置MOVE的偏移量   设置是否等待到位信号  即  阻塞与非阻塞
void set_feature(const char *feature_name);              //设置坐标系
void add_waypoint(const double *pos, int count);         //用于MOVE 中增加路点

int  movej(double *pos, int count, double acc, double velc);
int  movel(double *pos, int count, double acc, double velc);
int  movel_to(double x, double y, double z, double acc, double velc);
int  movep(double acc, double velc,double blend_radius,int track_mode);

int  set_payload(double weight, double *cog, int count);  //设置运行时负载

int    is_exist_current_io    ( our_contorl_io_type  io_type, our_contorl_io_mode io_mode,int io_index);   //判断对应IO是否存在
int    set_robot_one_io_status( our_contorl_io_type  io_type, our_contorl_io_mode io_mode,int io_index, double io_value);  //设置指定IO 的状态
double get_robot_one_io_status( our_contorl_io_type  io_type, our_contorl_io_mode io_mode,int io_index);   //获取指定IO 的状态
##############################################################################################################################################*/


#ifndef AUBO_DRIVER_H_
#define AUBO_DRIVER_H_

#include <thread>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <aubo_msgs/SetIO.h>
#include <aubo_msgs/SetPayload.h>
#include <aubo_msgs/SetIORequest.h>
#include <aubo_msgs/SetIOResponse.h>
#include <aubo_msgs/IOState.h>
#include <aubo_msgs/Digital.h>
#include <aubo_msgs/Analog.h>
#include <aubo_msgs/JointPos.h>
#include <industrial_msgs/RobotStatus.h>
#include "aubo_driver/AuboRobotMetaType.h"
#include "aubo_driver/serviceinterface.h"
#include "sensor_msgs/JointState.h"

#define BufferQueueSize 2000
#define THRESHHOLD 0.000001
#define ARM_DOF 6
#define MAXALLOWEDDELAY 20

namespace aubo_driver
{
    struct PlanningState
    {
    //    double currentPosition[aubo_robot_namespace::ARM_DOF];
    //    double currentVelocity[ARM_DOF];
    //    double currentAcceleration[ARM_DOF];
        double currentJointPos[ARM_DOF];
    };
    enum ROBOT_CONTROLLER_MODE
    {
        ROBOT_CONTROLLER=0, //
        ROS_CONTROLLER
    };

    class BufQueue
    {
        public:
            BufQueue()                             //构造函数，置空队列
            {
                front = rear = 0;
            }
            ~BufQueue(){}    //析构函数
            bool Empty()                          //判断队列是否为空
            {
                if(front == rear)
                    return true;
                else
                    return false;
            }

            void EnQueue(PlanningState x)
            {
                if(((rear + 1) % BufferQueueSize) == front)             //判断队列是否已满
                    DeQueue();

                rear = (rear + 1) % BufferQueueSize;             //移动尾指针指向下一个空间
                buf[rear] = x;                        //元素x入队
            }

            PlanningState DeQueue()                    //队头元素出栈
            {
                front = (front + 1) % BufferQueueSize;        //移动队头指针指向下一个空间，即被删元素所在位置
                return buf[front];               //返回被删除的元素的值
            }

            int GetQueueSize()
            {
                return ((rear+BufferQueueSize-front)%BufferQueueSize);
            }
    private:
            PlanningState buf[BufferQueueSize];                  //存放队列的数组
            int front,rear;                      //头指针与尾指针
    };

    class RobotState
    {
    public:
//        RobotState();
//        ~RobotState();
        aubo_robot_namespace::JointStatus jointStatus[6];
        aubo_robot_namespace::RobotDiagnosis robotDiagnosisInfo;
        bool IsRealRobotExist;
        bool isRobotControllerConnected;
        ROBOT_CONTROLLER_MODE robot_controller;
        aubo_robot_namespace::RobotState state;
        aubo_robot_namespace::RobotErrorCode code;
        unsigned char getRobotMode();
        bool isReady();

        void setDisconnected();

        bool getNewDataAvailable();
        void finishedReading();
    };

    class AuboDriver
    {
        public:
            AuboDriver();
            ~AuboDriver();
            bool road_point_compare(double *point1, double *point2);

            double* getCurrentPosition();
            void setCurrentPosition(double *target);
            double* getTagrtPosition();
            void setTagrtPosition(double *target);

            void updateControlStatus();
            void run();
            bool setIO(aubo_msgs::SetIORequest& req, aubo_msgs::SetIOResponse& resp);

        public:
            static bool IsRealRobotExist;
            static bool startMove;
            static int controlOption;
            static double lastRecievePoint[ARM_DOF];
            static std::string jointname[ARM_DOF];

            int BufferSize;
            ServiceInterface robotService;      //send
            ServiceInterface robotService1;     //receive

            RobotState rs;
//            std::thread* mb_publish_thread_;


            BufQueue  bufQueue;
            aubo_msgs::JointPos cur_pos;
            ros::Publisher jointstates_pub;
            ros::Publisher jointtarget_pub;
            ros::Publisher robot_status_pub;
            ros::Subscriber teach_subs;
            ros::Subscriber moveAPI_subs;
            ros::Subscriber moveIt_controller_subs;
            ros::Publisher io_pub;

        private:
            void MoveItPosCallback(const sensor_msgs::JointState::ConstPtr &msg);
            void AuboAPICallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
            void TeachCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
            void chatterCallback1(const std_msgs::Float32MultiArray::ConstPtr &msg);
            void chatterCallback3(const aubo_msgs::IOState::ConstPtr &msg);
            void timerCallback(const ros::TimerEvent& e);
            bool setRobotJointsByMoveIt();
            void plantypeCallback(const std_msgs::Int32MultiArray::ConstPtr &msg);
            void publishIOMsg();

            bool reverse_connected_;

            ros::NodeHandle nh;
            ros::Publisher  rib_pub;
            ros::Subscriber plan_type_sub;
            ros::Timer timer;            
            ros::Timer io_publish_timer;

            ros::ServiceServer io_srv_;
            std::thread* mb_publish_thread_;

            double io_flag_delay_;
            static int ribbuffersize;
            static int controlMode;
            std_msgs::Int32MultiArray ribstatus;
            industrial_msgs::RobotStatus robotstatus;
            int oldribstatus[3];
            static int ControllerConnectedFlag;
            static bool dataRecieved;
            static int dataCount;

            static double currentJoints[ARM_DOF];
            static double targetPoint[ARM_DOF];
    };

    enum ControMode
    {
        Teach = 0,
        SendTargetGoal,
        SynchronizeWithRealRobot
    };

    enum ControlOption
    {
        AuboAPI = 0,
        RosMoveIt
    };
}

#endif /* AUBO_DRIVER_H_ */
