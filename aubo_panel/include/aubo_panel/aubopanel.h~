#ifndef AUBO_PANEL_H
#define AUBO_PANEL_H

#include  <ros/ros.h>
#include  <rviz/panel.h>
#include  <sensor_msgs/JointState.h>
#include  <std_msgs/Float32MultiArray.h>
#include  <std_msgs/Int32MultiArray.h>
#include  <aubo_msgs/IOState.h>
#include  <aubo_msgs/GoalPoint.h>
#include  <QWidget>
#include  <QPushButton>
#include  <QTimer>

#define JOINT_LOW_LIMIT  -3.05
#define JOINT_UPPER_LIMIT  3.05

#define ARM_DOF  6

namespace Ui {
    class AuboPanel;
}


namespace aubo_panel
{
    class AuboPanel : public rviz::Panel
    {
        Q_OBJECT

    public:
        explicit AuboPanel(QWidget *parent = 0);
        ~AuboPanel();

    public:
        void initROS();
        int hashPushBtnName(QString name);

    private Q_SLOTS:
        void on_pbn_joint1Left_pressed();

        void on_pbn_XRight_pressed();

        void on_pbn_zero_clicked();
        void on_pbn_classicPos1_clicked();
        void on_pbn_sendGoal_clicked();

        void on_rbx_aubo_clicked();
        void on_rbx_ros_clicked();

        void on_rbx_teach_clicked();
        void on_rbx_goal_clicked();
        void on_rbx_sync_clicked();

        void on_pbn_setIO_clicked();

    protected Q_SLOTS:
        void sendCommand();

    protected:
        QTimer* m_timer;

        // The ROS publisher/subscriber
        ros::Publisher moveAPI_publisher_;
        ros::Publisher teach_publisher_;
        ros::Publisher moveit_publisher_;
        ros::Publisher plantype_publisher_;
        ros::Publisher io_publisher_;
        ros::Publisher plan_publisher_;
        ros::Publisher joint_pub;

        sensor_msgs::JointState jointMsg;
        std_msgs::Float32MultiArray joints;
        aubo_msgs::IOState iostate;
        std_msgs::Float32MultiArray goalPoint;
        std_msgs::Int32MultiArray plantype;

        ros::Subscriber update_jointstate_;
        ros::ServiceServer service;
        void real_pose_Callback(const std_msgs::Float32MultiArray::ConstPtr& msg);
        bool pointCompare(double* point1, double* point2);

        // The ROS node handle.
        ros::NodeHandle nh_;

        float jointVelocities[6];
        float jointPosition[6];
        float lastJointPosition[6];

    private:
        Ui::AuboPanel *ui;
        float m_step;
        float m_speed;

        int m_controMode;
        int m_ControlOption;
    };

    enum ControMode
    {
        Teach = 0,
        SendTargetGoal,
        SynchronizeWithRealRobot
    };

    enum ControOption
    {
        AuboAPI = 0,
        RosMoveIt
    };
}//end of aubo_rviz_plugin

#endif // AUBO_PANEL_H
