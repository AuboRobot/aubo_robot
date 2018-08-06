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
#include <std_msgs/String.h>
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
#include <control_msgs/FollowJointTrajectoryFeedback.h>

#include "otg/otgnewslib.h"

#define BufferQueueSize 2000
#define ARM_DOF 6
#define MAXALLOWEDDELAY 50
#define server_port 8899
#define BIG_MODULE_RATIO 2 * M_PI / 60.0 / 121
#define SMALL_MODULE_RATIO 2 * M_PI / 60.0 / 101
#define VMAX 3000
#define AMAX 10000
#define JMAX 40000

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

    class BufQueue
    {
        public:
            BufQueue()                             //构造函数，置空队列
            {
                front = rear = 0;
            }
            ~BufQueue(){}    //析构函数
            bool empty()                          //判断队列是否为空
            {
                if(front == rear)
                    return true;
                else
                    return false;
            }

            void enQueue(PlanningState x)
            {
                if(((rear + 1) % BufferQueueSize) == front)             //判断队列是否已满
                    deQueue();

                rear = (rear + 1) % BufferQueueSize;             //移动尾指针指向下一个空间
                buf[rear] = x;                        //元素x入队
            }

            PlanningState deQueue()                    //队头元素出栈
            {
                front = (front + 1) % BufferQueueSize;        //移动队头指针指向下一个空间，即被删元素所在位置
                return buf[front];               //返回被删除的元素的值
            }

            int getQueueSize()
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
        aubo_robot_namespace::JointStatus joint_status_[ARM_DOF];
        aubo_robot_namespace::wayPoint_S wayPoint_;
        aubo_robot_namespace::RobotDiagnosis robot_diagnosis_info_;
        bool IsRealRobotExist;
        bool isRobotControllerConnected;
        ROBOT_CONTROLLER_MODE robot_controller_;
        aubo_robot_namespace::RobotState state_;
        aubo_robot_namespace::RobotErrorCode code_;
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
            bool roadPointCompare(double *point1, double *point2);

            double* getCurrentPosition();
            void setCurrentPosition(double *target);
            double* getTagrtPosition();
            void setTagrtPosition(double *target);

            void updateControlStatus();
            void run();
            bool connectToRobotController();
            bool setIO(aubo_msgs::SetIORequest& req, aubo_msgs::SetIOResponse& resp);

            const int UPDATE_RATE_ = 500;
            const int TIMER_SPAN_ = 50;
            const double THRESHHOLD = 0.000001;

        public:
            static std::string joint_name_[ARM_DOF];

            int buffer_size_;
            ServiceInterface robot_send_service_;      //send
            ServiceInterface robot_receive_service_;     //receive

            RobotState rs;
//            std::thread* mb_publish_thread_;

            BufQueue  buf_queue_;
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
            ros::Subscriber controller_switch_sub_;
            ros::Timer timer_;
            ros::Timer io_publish_timer;

            ros::ServiceServer io_srv_;
            std::thread* mb_publish_thread_;

            double io_flag_delay_;
            std::string server_host_;
            int rib_buffer_size_;
            int control_mode_;
            int collision_class_;
            std_msgs::Int32MultiArray rib_status_;
            industrial_msgs::RobotStatus robot_status_;
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
