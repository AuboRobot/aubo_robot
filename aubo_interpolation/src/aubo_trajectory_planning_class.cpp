#include "aubo_trajectory_planning_class.h"

#define ARM_Number 7


aubo_Trajectory_Planning_Class::aubo_Trajectory_Planning_Class(ros::NodeHandle *NH)
{
  nodehandle = *NH;

  ros::param::get("controller_joint_names",Joint_names_from_ParamServer);

  joint_interpolation_pub_ = nodehandle.advertise<sensor_msgs::JointState>("moveit_controller_cmd", 2000);
  joint_path_sub = nodehandle.subscribe("joint_path_command",100,&aubo_Trajectory_Planning_Class::trajectoryCallBackFromMoveit,this);

  calculation_thread = new std::thread(boost::bind(&aubo_Trajectory_Planning_Class::interpolationTrajectory,this));

  std::cout << "*** Start Receive Moveit's Trajectory Points ***" << std::endl;
}







/**
 * @brief Receive Joint_path_command from Moveit
 *
 *
 *
 *
 *
 */
void aubo_Trajectory_Planning_Class::trajectoryCallBackFromMoveit(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
{
  std::cout << "*** this Node Receive Moveit's Trajectory Points one times ***" << std::endl;

  trajectory_msgs::JointTrajectory m_trajectoryPoint_falseOrder;
  trajectory_msgs::JointTrajectory m_trajectoryPoint_trueOrder;

  //apply memory space
  m_trajectoryPoint_falseOrder.joint_names.resize(ARM_Number);
  m_trajectoryPoint_trueOrder.joint_names.resize(ARM_Number);
  m_trajectoryPoint_falseOrder.points.resize(msg->points.size());
  m_trajectoryPoint_trueOrder.points.resize(msg->points.size());
  for(int i=0; i<msg->points.size(); i++)
  {
    m_trajectoryPoint_falseOrder.points[i].positions.resize(ARM_Number);
    m_trajectoryPoint_falseOrder.points[i].accelerations.resize(ARM_Number);
    m_trajectoryPoint_falseOrder.points[i].velocities.resize(ARM_Number);

    m_trajectoryPoint_trueOrder.points[i].positions.resize(ARM_Number);
    m_trajectoryPoint_trueOrder.points[i].accelerations.resize(ARM_Number);
    m_trajectoryPoint_trueOrder.points[i].velocities.resize(ARM_Number);
  }

  //get joint name from moveit(falseOrder)
  for(int joint_Number=0; joint_Number<ARM_Number; joint_Number++)
  {
    m_trajectoryPoint_falseOrder.joint_names[joint_Number] = msg->joint_names[joint_Number];
  }

  //get joint motion data from moveit(falseOrder)
  for(int Trajectory_PointNumber=0; Trajectory_PointNumber<msg->points.size(); Trajectory_PointNumber++)
  {
    for(int joint_Number=0;joint_Number<ARM_Number;joint_Number++)
    {
      m_trajectoryPoint_falseOrder.points[Trajectory_PointNumber].positions[joint_Number]     = msg->points[Trajectory_PointNumber].positions[joint_Number];
      m_trajectoryPoint_falseOrder.points[Trajectory_PointNumber].accelerations[joint_Number] = msg->points[Trajectory_PointNumber].accelerations[joint_Number];
      m_trajectoryPoint_falseOrder.points[Trajectory_PointNumber].velocities[joint_Number]    = msg->points[Trajectory_PointNumber].velocities[joint_Number];
    }
    m_trajectoryPoint_falseOrder.points[Trajectory_PointNumber].time_from_start = msg->points[Trajectory_PointNumber].time_from_start;
  }

  //order joint data(trueOrder)
  this->Order_trajectoryPoints(&m_trajectoryPoint_falseOrder,m_trajectoryPoint_trueOrder);

  //enqueue
  this->enqueue_trajectoryPlanPoints(&m_trajectoryPoint_trueOrder);
}






/**
 * @brief order trajectory data
 *
 *
 *
 *
 */
void aubo_Trajectory_Planning_Class::Order_trajectoryPoints(trajectory_msgs::JointTrajectory *m_inPoint, trajectory_msgs::JointTrajectory &m_outPoint)
{
  //需要注意６关节和７关节的区别

  int shoulder_joint_position,
      upperArm_joint_position,
      foreArm_joint_position,
      wrist1_joint_position,
      wrist2_joint_position,
      wrist3_joint_position,
      arm_mount_joint_position=0;

  //get joint names now
  for(int joint_number=0; joint_number<ARM_Number; joint_number++)
  {
    m_outPoint.joint_names[joint_number] = Joint_names_from_ParamServer.at(joint_number);
  }

  //find right order
  for(int joint_number=0; joint_number<ARM_Number; joint_number++)
  {
    if(m_inPoint->joint_names[joint_number] == "shoulder_joint")
    {
      shoulder_joint_position = joint_number;
    }
    if(m_inPoint->joint_names[joint_number] == "upperArm_joint")
    {
      upperArm_joint_position = joint_number;
    }
    if(m_inPoint->joint_names[joint_number] == "foreArm_joint")
    {
      foreArm_joint_position = joint_number;
    }
    if(m_inPoint->joint_names[joint_number] == "wrist1_joint")
    {
      wrist1_joint_position = joint_number;
    }
    if(m_inPoint->joint_names[joint_number] == "wrist2_joint")
    {
      wrist2_joint_position = joint_number;
    }
    if(m_inPoint->joint_names[joint_number] == "wrist3_joint")
    {
      wrist3_joint_position = joint_number;
    }
    if(m_inPoint->joint_names[joint_number] == "arm_mount_joint")
    {
      arm_mount_joint_position = joint_number;
    }
  }

  int order[ARM_Number] = {shoulder_joint_position, upperArm_joint_position, foreArm_joint_position,
                           wrist1_joint_position,   wrist2_joint_position,   wrist3_joint_position, arm_mount_joint_position};

  //change data to right order
  for(int Trajectory_PointNumber=0; Trajectory_PointNumber<m_inPoint->points.size(); Trajectory_PointNumber++)
  {
    for(int joint_Number=0;joint_Number<ARM_Number;joint_Number++)
    {
      m_outPoint.points[Trajectory_PointNumber].positions[joint_Number]     = m_inPoint->points[Trajectory_PointNumber].positions[order[joint_Number]];
      m_outPoint.points[Trajectory_PointNumber].accelerations[joint_Number] = m_inPoint->points[Trajectory_PointNumber].accelerations[order[joint_Number]];
      m_outPoint.points[Trajectory_PointNumber].velocities[joint_Number]    = m_inPoint->points[Trajectory_PointNumber].velocities[order[joint_Number]];
    }
    m_outPoint.points[Trajectory_PointNumber].time_from_start = m_inPoint->points[Trajectory_PointNumber].time_from_start;
  }

}






/**
 * @brief enqueue trajectory data
 *
 *
 *
 *
 */
void aubo_Trajectory_Planning_Class::enqueue_trajectoryPlanPoints(trajectory_msgs::JointTrajectory *point)
{
//  for(int i=0;i<point->points.size();i++)
//  {
//    std::cout << point->points[i].positions[0] <<","
//              << point->points[i].positions[1] <<","
//              << point->points[i].positions[2] <<","
//              << point->points[i].positions[3] <<","
//              << point->points[i].positions[4] <<","
//              << point->points[i].positions[5] <<","
//              << point->points[i].positions[6] <<","
//              <<std::endl;
//  }

  trajectory_msgs::JointTrajectoryPoint m_point;
  for(int i=0;i<point->points.size();i++)
  {
    m_point = point->points[i];
    ROS_ERROR("time: %f",point->points[i].time_from_start.toSec());
    std::unique_lock<std::mutex> lock(motion_buffer_mutex);
    motion_buffer_.push(m_point);
    lock.unlock();
  }
}







/**
 * @brief interpolation trajectory data
 *
 *
 *
 */
void aubo_Trajectory_Planning_Class::interpolationTrajectory()
{
  trajectory_msgs::JointTrajectoryPoint last_goal_point, current_goal_point;
  double move_duration, T, T2, T3, T4, T5, tt, ti, t1, t2, t3, t4, t5;
  double a1[ARM_Number], a2[ARM_Number], a3[ARM_Number], a4[ARM_Number], a5[ARM_Number], h[ARM_Number], intermediate_goal_point[ARM_Number];
  double update_duration=0;
  double update_rate_ = 200;

  while(true)
  {

    std::unique_lock<std::mutex> lock(motion_buffer_mutex);
    int buffer_size =  motion_buffer_.size();
    lock.unlock();

    if( buffer_size != 0)
    {
      std::unique_lock<std::mutex> lock(motion_buffer_mutex);
      current_goal_point = motion_buffer_.front();
      motion_buffer_.pop();
      lock.unlock();
//      std::cout << current_goal_point.positions[0] << " : "
//                << current_goal_point.positions[1] << " : "
//                << current_goal_point.positions[2] << " : "
//                << current_goal_point.positions[3] << " : "
//                << current_goal_point.positions[4] << " : "
//                << current_goal_point.positions[5] << " : "
//                << current_goal_point.positions[6] << " : "
//                << std::endl;

     /*************************************************************** interpolation ******************************/
      if(current_goal_point.time_from_start <= last_goal_point.time_from_start)
      {
        move_duration = current_goal_point.time_from_start.toSec();
      }
      else
      {
          T = current_goal_point.time_from_start.toSec() - last_goal_point.time_from_start.toSec();
          memcpy(a1, &last_goal_point.velocities[0], ARM_Number*sizeof(double));
          T2 = T * T;
          T3 = T2 * T;
          T4 = T3 * T;
          T5 = T4 * T;
          for(int i = 0; i < ARM_Number; i++)
          {
              a2[i] = 0.5 * last_goal_point.accelerations[i];
              h[i] = current_goal_point.positions[i] - last_goal_point.positions[i];
              a3[i] =0.5 / T3 * (20*h[i] - (8*current_goal_point.velocities[i] + 12*last_goal_point.velocities[i])*T - (3*last_goal_point.accelerations[i] -current_goal_point.accelerations[i])*T2);
              a4[i] =0.5 / T4 * (-30*h[i] + (14*current_goal_point.velocities[i] + 16*last_goal_point.velocities[i])*T + (3*last_goal_point.accelerations[i] - 2*current_goal_point.accelerations[i])*T2);
              a5[i] =0.5 / T5 * (12*h[i] - 6*(current_goal_point.velocities[i] + last_goal_point.velocities[i])*T + (current_goal_point.accelerations[i] - last_goal_point.accelerations[i])*T2);
          }
          if(update_rate_ > 0)
          {
              update_duration = 1/update_rate_;
              tt = last_goal_point.time_from_start.toSec();
              ti = tt;
              while (tt < current_goal_point.time_from_start.toSec())
              {

                  t1 = tt - ti;
                  t2 = t1 * t1;
                  t3 = t2 * t1;
                  t4 = t3 * t1;
                  t5 = t4 * t1;
                  for(int i = 0; i < ARM_Number; i++)
                      intermediate_goal_point[i] = last_goal_point.positions[i] +a1[i]*t1+a2[i]*t2 +a3[i]*t3 +a4[i]*t4 +a5[i]*t5;
                  tt += update_duration;

//                  std::cout<<"||" << intermediate_goal_point[0] << " : "
//                            << intermediate_goal_point[1] << " : "
//                            << intermediate_goal_point[2] << " : "
//                            << intermediate_goal_point[3] << " : "
//                            << intermediate_goal_point[4] << " : "
//                            << intermediate_goal_point[5] << " : "
//                            << intermediate_goal_point[6] << " : "
//                            << std::endl;
//                 ROS_ERROR("tt:%f    current:%f ",tt,current_goal_point.time_from_start.toSec());

                   this->jointStatePublic(intermediate_goal_point);
              }
              move_duration = current_goal_point.time_from_start.toSec() - tt;
          }
      }
      for(int i = 0; i < ARM_Number; i++)
        intermediate_goal_point[i] = current_goal_point.positions[i];
      this->jointStatePublic(intermediate_goal_point);

      last_goal_point = current_goal_point;
      /*************************************************************** interpolation ******************************/
    }
    else
    {
      ;
    }
    usleep(50*1000);
  }
}





/**
 * @brief public interpolation data to driver
 *
 *
 *
 *
 *
 */
void aubo_Trajectory_Planning_Class::jointStatePublic(double *joint_Position)
{
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(ARM_Number);
    joint_state.position.resize(ARM_Number);

    for(int i = 0; i < ARM_Number; i++)
    {
        joint_state.name[i] = Joint_names_from_ParamServer.at(i);
        joint_state.position[i] = joint_Position[i];
    }

    joint_interpolation_pub_.publish(joint_state);
}
