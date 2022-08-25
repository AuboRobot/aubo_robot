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
#include "aubo_trajectory_planning.h"

#define ARM_Number 7
#define MINIMUM_BUFFER_SIZE_ 250

namespace aubo_trajectory_planning {

aubo_Trajectory_Planning_Class::aubo_Trajectory_Planning_Class(ros::NodeHandle *NH):
                                  driver_buffer_size_(0),log_sdk_status_(false)
{
  nodehandle = *NH;

  ros::param::get("controller_joint_names",Joint_names_from_ParamServer);

  joint_interpolation_pub_ = nodehandle.advertise<trajectory_msgs::JointTrajectoryPoint>("moveItController_cmd", 2000);
  joint_path_sub = nodehandle.subscribe("joint_path_command",100,&aubo_Trajectory_Planning_Class::trajectoryCallBackFromMoveit,this);
  rib_status_sub = nodehandle.subscribe("rib_status", 100, &aubo_Trajectory_Planning_Class::ribStatusCallback, this);

  pthread_mutex_init(&mutex_motion_buffer,NULL);
  pthread_mutex_init(&mutex_driver_buffer,NULL);
  pthread_mutex_init(&mutex_log_sdk_status,NULL);

  std::cout << "*** Start Receive Moveit's Trajectory Points ***" << std::endl;
}





/**
 * @brief check the robot is motioning now
 *
 *
 *
 *
 *
 */
bool aubo_Trajectory_Planning_Class::isMotion()
{
  pthread_mutex_lock(&mutex_motion_buffer);
  bool ret = motion_buffer_.empty();
  pthread_mutex_unlock(&mutex_motion_buffer);

  return !ret;
}







/**
 * @brief clear interpolation array, this operator want the robot stop
 *
 *
 *
 *
 *
 */
void aubo_Trajectory_Planning_Class::stop_interpolation()
{
  pthread_mutex_lock(&mutex_motion_buffer);

  while(motion_buffer_.empty() != true)
  {
    try
    {
      motion_buffer_.pop();
    }
    catch(...)
    {
      ROS_ERROR("Error happened in the aubo_trajectory_planning.cpp stop_interpolation");
      pthread_mutex_unlock(&mutex_motion_buffer);
    }
  }

  pthread_mutex_unlock(&mutex_motion_buffer);
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

  if(!this->isMotion())
  {
    std::cout << "*** The robot not motion now ***" << std::endl;

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
  else
  {
    this->stop_interpolation();

    std::cout << "*** The robot is motioning now ***" << std::endl;
  }

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
  //get joint names now
  for(int joint_number=0; joint_number<ARM_Number; joint_number++)
  {
    m_outPoint.joint_names[joint_number] = Joint_names_from_ParamServer.at(joint_number);
  }


  //change data to right order
  for(int Trajectory_PointNumber=0; Trajectory_PointNumber<m_inPoint->points.size(); Trajectory_PointNumber++)
  {
    for(int joint_Number=0;joint_Number<ARM_Number;joint_Number++)
    {
      m_outPoint.joint_names[joint_Number] = Joint_names_from_ParamServer.at(joint_Number);
      for(int j=0;j<ARM_Number;j++)
      {
        if(m_outPoint.joint_names[joint_Number] == m_inPoint->joint_names[j])
        {
          m_outPoint.points[Trajectory_PointNumber].positions[joint_Number]     = m_inPoint->points[Trajectory_PointNumber].positions[j];
          m_outPoint.points[Trajectory_PointNumber].accelerations[joint_Number] = m_inPoint->points[Trajectory_PointNumber].accelerations[j];
          m_outPoint.points[Trajectory_PointNumber].velocities[joint_Number]    = m_inPoint->points[Trajectory_PointNumber].velocities[j];
          break;
        }
      }
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
  trajectory_msgs::JointTrajectoryPoint m_point;

  for(int i=0;i<point->points.size();i++)
  {
    m_point = point->points[i];

    pthread_mutex_lock(&mutex_motion_buffer);
    motion_buffer_.push(m_point);
    pthread_mutex_unlock(&mutex_motion_buffer);
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

  while(ros::ok())
  {
    pthread_mutex_lock(&mutex_motion_buffer);
    int buffer_size =  motion_buffer_.size();
    pthread_mutex_unlock(&mutex_motion_buffer);

    if( buffer_size != 0)
    {
      pthread_mutex_lock(&mutex_motion_buffer);
      current_goal_point = motion_buffer_.front();
      motion_buffer_.pop();
      pthread_mutex_unlock(&mutex_motion_buffer);

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
              while (tt < current_goal_point.time_from_start.toSec() && ros::ok())
              {

                  t1 = tt - ti;
                  t2 = t1 * t1;
                  t3 = t2 * t1;
                  t4 = t3 * t1;
                  t5 = t4 * t1;
                  for(int i = 0; i < ARM_Number; i++)
                      intermediate_goal_point[i] = last_goal_point.positions[i] +a1[i]*t1+a2[i]*t2 +a3[i]*t3 +a4[i]*t4 +a5[i]*t5;
                  tt += update_duration;
                std::cout<<"||" << intermediate_goal_point[0] << " : "
                         << intermediate_goal_point[1] << " : "
                         << intermediate_goal_point[2] << " : "
                         << intermediate_goal_point[3] << " : "
                         << intermediate_goal_point[4] << " : "
                         << intermediate_goal_point[5] << " : "
                         << intermediate_goal_point[6] << " : "
                         << std::endl;
                   this->checkDriverMemory();
                   this->jointStatePublic(intermediate_goal_point);
              }
              move_duration = current_goal_point.time_from_start.toSec() - tt;
          }
      }
    std::cout<<"|||" << current_goal_point.positions[0] << " : "
             << current_goal_point.positions[1] << " : "
             << current_goal_point.positions[2] << " : "
             << current_goal_point.positions[3] << " : "
             << current_goal_point.positions[4] << " : "
             << current_goal_point.positions[5] << " : "
             << current_goal_point.positions[6] << " : "
             << std::endl;
      for(int i = 0; i < ARM_Number; i++)
        intermediate_goal_point[i] = current_goal_point.positions[i];
        this->checkDriverMemory();
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
 * @brief wait when driver buffer memory more than MINIMUM_BUFFER_SIZE_
 *
 *
 *
 *
 *
 */
void aubo_Trajectory_Planning_Class::checkDriverMemory()
{
   pthread_mutex_lock(&mutex_driver_buffer);
   int m_memoryindriver = driver_buffer_size_;
   pthread_mutex_unlock(&mutex_driver_buffer);

   while(m_memoryindriver > MINIMUM_BUFFER_SIZE_)
   {
     usleep(200*1000);                            //wait 40 point

     ROS_WARN("wait driver buffer memory decrease,its not a error");

     pthread_mutex_lock(&mutex_driver_buffer);
     int m_memoryindriver = driver_buffer_size_;
     pthread_mutex_unlock(&mutex_driver_buffer);
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
    trajectory_msgs::JointTrajectoryPoint joint_state;
    joint_state.time_from_start = ros::Duration(2);
    joint_state.positions.resize(ARM_Number);
    joint_state.accelerations.resize(ARM_Number);
    joint_state.velocities.resize(ARM_Number);

    for(int i = 0; i < ARM_Number; i++)
    {
        joint_state.positions[i] = joint_Position[i];
    }

    joint_interpolation_pub_.publish(joint_state);
}




/**
 * @brief subscrib rib status
 *
 *
 *
 *
 *
 */
void aubo_Trajectory_Planning_Class::ribStatusCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
    try
    {
        if(msg->data[1] == 1)
        {
            ROS_INFO("The robot is controlled by the ros controller!");
        }
        else
        {
            ROS_INFO("The robot is not controlled by the ros controller!");
        }
        pthread_mutex_lock(&mutex_driver_buffer);
        driver_buffer_size_ = msg->data[0];
        pthread_mutex_unlock(&mutex_driver_buffer);

        pthread_mutex_lock(&mutex_log_sdk_status);
        log_sdk_status_ = msg->data[2];
        pthread_mutex_unlock(&mutex_log_sdk_status);
    }
    catch(...)
    {
        ROS_ERROR("Unexpected exception while parsing the aubo driver message in aubo_trajectory_planning::ribStatusCallBack");
    }
}



}





//**************************************************************************** main ***************************



using namespace aubo_trajectory_planning;

void *thread_interpolation(void *arg)
{
  aubo_Trajectory_Planning_Class *object_c = (aubo_trajectory_planning::aubo_Trajectory_Planning_Class *)arg;
  try
  {
      object_c->interpolationTrajectory();
  }
  catch(...)
  {
      delete object_c;
      pthread_exit(NULL);
  }
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "aubo_Trajectory_Planning_Node");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  std::cout << "start run aubo_Trajectory_Planning_Node " <<std::endl;

//*********************************************************************pthread operation
  pthread_t pid1;
  pthread_attr_t attr;
  pthread_attr_init( &attr );
  pthread_attr_setdetachstate( &attr, PTHREAD_CREATE_JOINABLE );

  aubo_Trajectory_Planning_Class *object = new aubo_Trajectory_Planning_Class(&nh);

  int ret = pthread_create(&pid1, &attr, thread_interpolation, object);

  if(ret != 0)
  {
      std::cout << "interpolation pthread_cread failed" << std::endl;
      return 0;
  }

  pthread_join(pid1,NULL);
  pthread_attr_destroy(&attr);
  delete object;
//*********************************************************************pthread operation

  ros::waitForShutdown();
}
