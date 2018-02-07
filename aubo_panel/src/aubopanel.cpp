#include "aubo_panel/aubopanel.h"
#include "ui_aubopanel.h"
#include <math.h>

namespace aubo_panel
{

AuboPanel::AuboPanel(QWidget *parent) :
    rviz::Panel(parent),
    ui(new Ui::AuboPanel),
    m_step(0.16),
    m_speed(50.0),
    m_ControlOption(aubo_panel::RosMoveIt)
{
    ui->setupUi(this);

    ui->rbx_Aubo->setChecked(false);
    ui->rbx_Ros->setChecked(true);

    ui->rbx_teach->setChecked(false);
    ui->rbx_goal->setChecked(true);
    ui->rbx_sync->setChecked(false);

    ui->pbn_sendGoal->setEnabled(true);

    ui->cbx_ioType->setMaxCount(4);
    ui->cbx_ioType->addItem("PLC_IO");
    ui->cbx_ioType->addItem("TOOL_IO");
    ui->cbx_ioType->addItem("Board_IO");
    ui->cbx_ioType->addItem("Modbus_IO");

    ui->cbx_ioMode->setMaxCount(4);
    ui->cbx_ioMode->addItem("DO");
    ui->cbx_ioMode->addItem("DI");
    ui->cbx_ioMode->addItem("AO");
    ui->cbx_ioMode->addItem("AI");

    ui->cbx_ioIndex->setMaxCount(8);
    ui->cbx_ioIndex->addItem("1");
    ui->cbx_ioIndex->addItem("2");
    ui->cbx_ioIndex->addItem("3");
    ui->cbx_ioIndex->addItem("4");
    ui->cbx_ioIndex->addItem("5");
    ui->cbx_ioIndex->addItem("6");
    ui->cbx_ioIndex->addItem("7");
    ui->cbx_ioIndex->addItem("8");


    m_controMode = aubo_panel::SendTargetGoal;
    ros::param::set("/ControlMode","1");
    m_timer = new QTimer(this);
    connect(m_timer, SIGNAL(timeout()), this, SLOT(sendCommand()));
    connect(ui->rbx_teach, SIGNAL(clicked()), this, SLOT(on_rbx_teach_clicked()));
    connect(ui->rbx_goal, SIGNAL(clicked()), this, SLOT(on_rbx_goal_clicked()));
    connect(ui->rbx_sync, SIGNAL(clicked()), this, SLOT(on_rbx_sync_clicked));
    connect(ui->pbn_joint1Left, SIGNAL(pressed()), this, SLOT(on_pbn_joint1Left_pressed()));
    connect(ui->pbn_joint1Right, SIGNAL(pressed()), this, SLOT(on_pbn_joint1Left_pressed()));
    connect(ui->pbn_joint2Left, SIGNAL(pressed()), this, SLOT(on_pbn_joint1Left_pressed()));
    connect(ui->pbn_joint2Right, SIGNAL(pressed()), this, SLOT(on_pbn_joint1Left_pressed()));
    connect(ui->pbn_joint3Left, SIGNAL(pressed()), this, SLOT(on_pbn_joint1Left_pressed()));
    connect(ui->pbn_joint3Right, SIGNAL(pressed()), this, SLOT(on_pbn_joint1Left_pressed()));
    connect(ui->pbn_joint4Left, SIGNAL(pressed()), this, SLOT(on_pbn_joint1Left_pressed()));
    connect(ui->pbn_joint4Right, SIGNAL(pressed()), this, SLOT(on_pbn_joint1Left_pressed()));
    connect(ui->pbn_joint5Left, SIGNAL(pressed()), this, SLOT(on_pbn_joint1Left_pressed()));
    connect(ui->pbn_joint5Right, SIGNAL(pressed()), this, SLOT(on_pbn_joint1Left_pressed()));
    connect(ui->pbn_joint6Left, SIGNAL(pressed()), this, SLOT(on_pbn_joint1Left_pressed()));
    connect(ui->pbn_joint6Right, SIGNAL(pressed()), this, SLOT(on_pbn_joint1Left_pressed()));
    connect(ui->rbx_Aubo, SIGNAL(clicked()), this, SLOT(on_rbx_aubo_clicked()));
    connect(ui->rbx_Ros, SIGNAL(clicked()), this, SLOT(on_rbx_ros_clicked()));
    initROS();

    m_timer->start(10);     // Start the timer.
}

AuboPanel::~AuboPanel()
{
    delete ui;
}

void AuboPanel::initROS()
{
    joints.data.resize(6);
    plantype.data.resize(2);
    moveAPI_publisher_ = nh_.advertise<std_msgs::Float32MultiArray>("moveAPI_cmd", 1000);
    teach_publisher_ = nh_.advertise<std_msgs::Float32MultiArray>("teach_cmd", 1000);
    moveit_publisher_ = nh_.advertise<std_msgs::Float32MultiArray>("moveit_target", 1000);
    plantype_publisher_ = nh_.advertise<std_msgs::Int32MultiArray>("plan_type", 1);

//    io_publisher_ = nh_.advertise<aubo_msgs::IOState> ("io_state",1);
    update_jointstate_ = nh_.subscribe("real_pose", 1000, &AuboPanel::real_pose_Callback,this);
    joint_pub = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
}

bool AuboPanel::pointCompare(double* point1, double* point2)
{
  bool ret = true;
  for(int i=0;i<ARM_DOF;i++)
  {
    if(fabs(point1[i]-point2[i])>=0.000001)
    {
       ret = false;
       break;
    }
  }
  return ret;
}

void AuboPanel::real_pose_Callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    lastJointPosition[0] = msg->data[0];
    lastJointPosition[1] = msg->data[1];
    lastJointPosition[2] = msg->data[2];
    lastJointPosition[3] = msg->data[3];
    lastJointPosition[4] = msg->data[4];
    lastJointPosition[5] = msg->data[5];

    if(m_controMode == SynchronizeWithRealRobot)//sync with real robot
    {
        if(0/*!pointCompare(lastJointPosition, joints.data[0])*/)
        {
            joints.data[0] = lastJointPosition[0];
            joints.data[1] = lastJointPosition[1];
            joints.data[2] = lastJointPosition[2];
            joints.data[3] = lastJointPosition[3];
            joints.data[4] = lastJointPosition[4];
            joints.data[5] = lastJointPosition[5];

            ui->lb_joint1->setText(QString::number((msg->data[0]*180/M_PI),'f',6));
            ui->lb_joint2->setText(QString::number((msg->data[1]*180/M_PI),'f',6 ));
            ui->lb_joint3->setText(QString::number((msg->data[2]*180/M_PI),'f',6 ));
            ui->lb_joint4->setText(QString::number((msg->data[3]*180/M_PI),'f',6 ));
            ui->lb_joint5->setText(QString::number((msg->data[4]*180/M_PI),'f',6 ));
            ui->lb_joint6->setText(QString::number((msg->data[5]*180/M_PI),'f',6 ));

            sensor_msgs::JointState joint_state;
            joint_state.header.stamp = ros::Time::now();
            joint_state.name.resize(6);
            joint_state.position.resize(6);
            joint_state.name[0] ="shoulder_joint";
            joint_state.position[0] = joints.data[0];//shoulder;
            joint_state.name[1] ="upperArm_joint";
            joint_state.position[1] = joints.data[1];//upper_shoulder;
            joint_state.name[2] ="foreArm_joint";
            joint_state.position[2] = joints.data[2];//fore_upper;
            joint_state.name[3] ="wrist1_joint";
            joint_state.position[3] = joints.data[3];//wrist1_fore;
            joint_state.name[4] ="wrist2_joint";
            joint_state.position[4] = joints.data[4];//wrist2_wrist1;
            joint_state.name[5] ="wrist3_joint";
            joint_state.position[5] = joints.data[5];//wrist3_wrist2;
            joint_pub.publish(joint_state);
        }
    }
    else if(m_controMode == SendTargetGoal)
    {
//        ui->lb_joint1->setText(QString::number((msg->data[0]*180/M_PI),'f',6));
//        ui->lb_joint2->setText(QString::number((msg->data[1]*180/M_PI),'f',6 ));
//        ui->lb_joint3->setText(QString::number((msg->data[2]*180/M_PI),'f',6 ));
//        ui->lb_joint4->setText(QString::number((msg->data[3]*180/M_PI),'f',6 ));
//        ui->lb_joint5->setText(QString::number((msg->data[4]*180/M_PI),'f',6 ));
//        ui->lb_joint6->setText(QString::number((msg->data[5]*180/M_PI),'f',6 ));
    }
}

int AuboPanel::hashPushBtnName(QString name)
{
    int result = 0;
    if(name == "pbn_joint1Left")
        result = 1;
    else if(name == "pbn_joint1Right")
        result = 2;
    else if(name == "pbn_joint2Left")
        result = 3;
    else if(name == "pbn_joint2Right")
        result = 4;
    else if(name == "pbn_joint3Left")
        result = 5;
    else if(name == "pbn_joint3Right")
        result = 6;
    else if(name == "pbn_joint4Left")
        result = 7;
    else if(name == "pbn_joint4Right")
        result = 8;
    else if(name == "pbn_joint5Left")
        result = 9;
    else if(name == "pbn_joint5Right")
        result = 10;
    else if(name == "pbn_joint6Left")
        result = 11;
    else if(name == "pbn_joint6Right")
        result = 12;
    return result;
}

void AuboPanel::on_pbn_joint1Left_pressed()
{
    QPushButton* btn = qobject_cast<QPushButton*>(sender());
    if(m_controMode == aubo_panel::SynchronizeWithRealRobot)
       return;
    else if(m_controMode == aubo_panel::Teach)
    {
        //publish
        if(m_ControlOption == aubo_panel::AuboAPI)
        {}
        else
        {}
    }
    else
    {
        int index = 0;
        int multiplier = 1;
        switch(hashPushBtnName(btn->objectName()))
        {
            case 1:index = 0;multiplier = -1;break;
            case 2:index = 0;multiplier = 1;break;
            case 3:index = 1;multiplier = -1;break;
            case 4:index = 1;multiplier = 1;break;
            case 5:index = 2;multiplier = -1;break;
            case 6:index = 2;multiplier = 1;break;
            case 7:index = 3;multiplier = -1;break;
            case 8:index = 3;multiplier = 1;break;
            case 9:index = 4;multiplier = -1;break;
            case 10:index = 4;multiplier = 1;break;
            case 11:index = 5;multiplier = -1;break;
            case 12:index = 5;multiplier = 1;break;
        }
        if(multiplier == -1)
        {
            joints.data[index] -= m_step;
            if(joints.data[index] < JOINT_LOW_LIMIT)
            {
                joints.data[index] = JOINT_LOW_LIMIT;
            }
        }
        else
        {
            joints.data[index] += m_step;
            if(joints.data[index] > JOINT_UPPER_LIMIT)
            {
                joints.data[index] = JOINT_UPPER_LIMIT;
            }

        }
        switch(hashPushBtnName(btn->objectName()))
        {
            case 1:
            case 2:ui->lb_joint1->setText(QString::number(joints.data[index]*180.0/M_PI, 'f', 6));break;
            case 3:
            case 4:ui->lb_joint2->setText(QString::number(joints.data[index]*180.0/M_PI, 'f', 6));break;
            case 5:
            case 6:ui->lb_joint3->setText(QString::number(joints.data[index]*180.0/M_PI, 'f', 6));break;
            case 7:
            case 8:ui->lb_joint4->setText(QString::number(joints.data[index]*180.0/M_PI, 'f', 6));break;
            case 9:
            case 10:ui->lb_joint5->setText(QString::number(joints.data[index]*180.0/M_PI, 'f', 6));break;
            case 11:
            case 12:ui->lb_joint6->setText(QString::number(joints.data[index]*180.0/M_PI, 'f', 6));break;
        }
    }
}

void AuboPanel::on_pbn_zero_clicked()
{
    if(m_controMode == aubo_panel::SendTargetGoal)
    {
        goalPoint.data.resize(7);
        goalPoint.data[0] = 0;
        goalPoint.data[1] = 0;
        goalPoint.data[2] = 0;
        goalPoint.data[3] = 0;
        goalPoint.data[4] = 0;
        goalPoint.data[5] = 0;
        goalPoint.data[6] = 0;
        //publish
        if(m_ControlOption == 0)
        {
            moveAPI_publisher_.publish(goalPoint);
        }
        else
        {
            moveit_publisher_.publish(goalPoint);
        }
    }
}

void AuboPanel::on_pbn_classicPos1_clicked()
{
    if(m_controMode == aubo_panel::SendTargetGoal)
    {
        goalPoint.data.resize(7);
        goalPoint.data[0] = 0;
        goalPoint.data[1] = 91*M_PI/180;
        goalPoint.data[2] = 52.3*M_PI/180;
        goalPoint.data[3] = -95*M_PI/180;
        goalPoint.data[4] = 40*M_PI/180;
        goalPoint.data[5] = 92.5*M_PI/180;
        goalPoint.data[6] = 123.8*M_PI/180;
        //publish
        if(m_ControlOption == 0)
        {
            moveAPI_publisher_.publish(goalPoint);
        }
        else
        {
            moveit_publisher_.publish(goalPoint);
        }
    }
}


void AuboPanel::on_pbn_sendGoal_clicked()
{   

    if(m_controMode == aubo_panel::SendTargetGoal)
    {
        if(m_ControlOption == 0)
        {
            goalPoint.data.resize(7);
            goalPoint.data[0] = 0;
            goalPoint.data[1] = joints.data[0];
            goalPoint.data[2] = joints.data[1];
            goalPoint.data[3] = joints.data[2];
            goalPoint.data[4] = joints.data[3];
            goalPoint.data[5] = joints.data[4];
            goalPoint.data[6] = joints.data[5];
            moveAPI_publisher_.publish(goalPoint);
            ROS_INFO("Sent a target goal accepted by AUBO API");
        }
        else
        {
            goalPoint.data.resize(7);
            goalPoint.data[0] = 0;
            goalPoint.data[1] = joints.data[0];
            goalPoint.data[2] = joints.data[1];
            goalPoint.data[3] = joints.data[2];
            goalPoint.data[4] = joints.data[3];
            goalPoint.data[5] = joints.data[4];
            goalPoint.data[6] = joints.data[5];
            moveit_publisher_.publish(goalPoint);
            ROS_INFO("Sent a target goal accepted by MoveIt!");
        }
    }
}

void AuboPanel::on_rbx_aubo_clicked()
{
    ui->rbx_Aubo->setChecked(true);
    ui->rbx_Ros->setChecked(false);
    m_ControlOption = 0;
    plantype.data[0] = m_ControlOption;
    plantype_publisher_.publish(plantype);
    ROS_INFO("ControlOption----0");
}


void AuboPanel::on_rbx_ros_clicked()
{
    ui->rbx_Aubo->setChecked(false);
    ui->rbx_Ros->setChecked(true);
    m_ControlOption = 1;
    plantype.data[0] = m_ControlOption;
    plantype_publisher_.publish(plantype);
    ROS_INFO("ControlOption----1");
}

void AuboPanel::on_rbx_teach_clicked()
{
    ui->rbx_teach->setChecked(true);
    ui->rbx_goal->setChecked(false);
    ui->rbx_sync->setChecked(false);

    ui->pbn_sendGoal->setEnabled(false);
    ui->pbn_zero->setEnabled(false);
    ui->pbn_classicPos->setEnabled(false);

    m_controMode = aubo_panel::Teach;
    plantype.data[1] = m_controMode;
    plantype_publisher_.publish(plantype);
//    ros::param::set("/ControlMode","0");
}

void AuboPanel::on_rbx_goal_clicked()
{
    ui->rbx_teach->setChecked(false);
    ui->rbx_goal->setChecked(true);
    ui->rbx_sync->setChecked(false);

    ui->pbn_sendGoal->setEnabled(true);
    ui->pbn_zero->setEnabled(true);
    ui->pbn_classicPos->setEnabled(true);

    m_controMode = aubo_panel::SendTargetGoal;
    plantype.data[1] = m_controMode;
    plantype_publisher_.publish(plantype);
//    ros::param::set("/ControlMode","1");
}

void AuboPanel::on_rbx_sync_clicked()
{

    ui->rbx_teach->setChecked(false);
    ui->rbx_goal->setChecked(false);
    ui->rbx_sync->setChecked(true);

    ui->pbn_sendGoal->setEnabled(false);
    ui->pbn_zero->setEnabled(false);
    ui->pbn_classicPos->setEnabled(false);

    m_controMode = aubo_panel::SynchronizeWithRealRobot;
    plantype.data[1] = m_controMode;
    plantype_publisher_.publish(plantype);
//    ros::param::set("/ControlMode","2");
}



void AuboPanel::on_pbn_setIO_clicked()
{
//    iostate.type = ui->cbx_ioType->currentIndex()+1;
//    iostate.mode = ui->cbx_ioMode->currentIndex()+1;
//    iostate.index = ui->cbx_ioMode->currentIndex()+1;

//    iostate.state = ui->le_ioState->text().toDouble();

//    if((m_controMode == aubo_panel::SendTargetGoal)&&(m_ControlOption == 1))
//    {
//        io_publisher_.publish(iostate);
//    }
}


void AuboPanel::sendCommand()
{
    if( ros::ok())
    {
        if(m_controMode != aubo_panel::SendTargetGoal)
        {
            //Get service to update the Rviz and Aubo Panel

//            sensor_msgs::JointState joint_state;
//            joint_state.header.stamp = ros::Time::now();
//            joint_state.name.resize(6);
//            joint_state.position.resize(6);
//            joint_state.name[0] ="shoulder_joint";
//            joint_state.position[0] = joints.data[0];//shoulder;
//            joint_state.name[1] ="upperArm_joint";
//            joint_state.position[1] = joints.data[1];//upper_shoulder;
//            joint_state.name[2] ="foreArm_joint";
//            joint_state.position[2] = joints.data[2];//fore_upper;
//            joint_state.name[3] ="wrist1_joint";
//            joint_state.position[3] = joints.data[3];//wrist1_fore;
//            joint_state.name[4] ="wrist2_joint";
//            joint_state.position[4] = joints.data[4];//wrist2_wrist1;
//            joint_state.name[5] ="wrist3_joint";
//            joint_state.position[5] = joints.data[5];//wrist3_wrist2;
//            joint_pub.publish(joint_state);
        }
        else
        {

        }
    }
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aubo_panel::AuboPanel,rviz::Panel)
