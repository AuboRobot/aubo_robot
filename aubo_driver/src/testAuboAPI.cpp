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
#include "aubo_driver.cpp"

#include <string>
#include <cstdlib>
#include <unistd.h>
#include <math.h>
#include <stdio.h>
#include <sstream>
#include <fstream>

using namespace aubo_driver;

#define MAX_JOINT_ACC 100.0/180.0*M_PI  //unit rad/s^2
#define MAX_JOINT_VEL 50.0/180.0*M_PI   //unit rad/s
#define MAX_END_ACC    4                // unit m/s^2
#define MAX_END_VEL    2                // unit m/s

double zero_poeition[ARM_DOF] = {0};
double initial_poeition[ARM_DOF] = {0.0/180.0*M_PI,  0.0/180.0*M_PI,  90.0/180.0*M_PI, 0.0/180.0*M_PI, 90.0/180.0*M_PI, 0.0/180.0*M_PI};
double postion1[ARM_DOF] = {0.0/180.0*M_PI,  0.0/180.0*M_PI,  90.0/180.0*M_PI, 0.0/180.0*M_PI, 90.0/180.0*M_PI,   0.0/180.0*M_PI};
double postion2[ARM_DOF] = {15.0/180.0*M_PI,  0.0/180.0*M_PI,  90.0/180.0*M_PI, 0.0/180.0*M_PI, 90.0/180.0*M_PI,   0.0/180.0*M_PI};

double postion3[ARM_DOF] = {0.0/180.0*M_PI,  0.0/180.0*M_PI,  45.0/180.0*M_PI, 0.0/180.0*M_PI, 90.0/180.0*M_PI, 0.0/180.0*M_PI};
double postion4[ARM_DOF] = {30.0/180.0*M_PI,  0.0/180.0*M_PI,  90.0/180.0*M_PI, 0.0/180.0*M_PI, 90.0/180.0*M_PI, 0.0/180.0*M_PI};

void testMoveJ(AuboDriver &robot_driver)
{
    /** Initialize move properties ***/
    robot_driver.robot_send_service_.robotServiceInitGlobalMoveProfile();

    /** Set Max joint acc and vel***/
    aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
    aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
    for(int i = 0; i < ARM_DOF; i++)
    {
        jointMaxAcc.jointPara[i] = MAX_JOINT_ACC;
        jointMaxVelc.jointPara[i] = MAX_JOINT_VEL;
    }
    robot_driver.robot_send_service_.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);
    robot_driver.robot_send_service_.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);

    /** Robot move to zero position **/
    int ret = robot_driver.robot_send_service_.robotServiceJointMove(zero_poeition, true);
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        ROS_ERROR("Failed to move to zero postions, error code:%d", ret);


    /** loop for 3 times **/
    for(int i=0; i<3; i++)
    {
        /** set relative offset**/
        aubo_robot_namespace::MoveRelative relativeMoveOnBase;
        relativeMoveOnBase.ena = true;
        relativeMoveOnBase.relativePosition[0] = 0;
        relativeMoveOnBase.relativePosition[1] = 0;
        relativeMoveOnBase.relativePosition[2] = 0.05*(i%4);   //unit:m
        robot_driver.robot_send_service_.robotServiceSetMoveRelativeParam(relativeMoveOnBase);


        /** switch to postion1 by moveJ **/
        robot_driver.robot_send_service_.robotServiceJointMove(postion1, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            ROS_ERROR("Failed to move to zero postion, error code:%d", ret);
            break;
        }

        /** switch to postion1 by moveJ **/
        robot_driver.robot_send_service_.robotServiceJointMove(postion2, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            ROS_ERROR("Failed to move to  postion1, error code:%d", ret);
            break;
        }
    }
}

void testMoveL(AuboDriver &robot_driver)
{
    /** Initialize move properties ***/
    robot_driver.robot_send_service_.robotServiceInitGlobalMoveProfile();

    /** Set Max joint acc and vel***/
    aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
    aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
    for(int i = 0; i < ARM_DOF; i++)
    {
        jointMaxAcc.jointPara[i] = MAX_JOINT_ACC;
        jointMaxVelc.jointPara[i] = MAX_JOINT_VEL;
    }
    robot_driver.robot_send_service_.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);
    robot_driver.robot_send_service_.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);

   /** move to inital position **/
    int ret = robot_driver.robot_send_service_.robotServiceJointMove(initial_poeition, true);
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        ROS_ERROR("Failed to move to initial postion, error code:%d", ret);


    /** Initialize move properties ***/
    robot_driver.robot_send_service_.robotServiceInitGlobalMoveProfile();

    /** Set Max END acc and vel**/
    robot_driver.robot_send_service_.robotServiceSetGlobalMoveEndMaxLineAcc(MAX_END_ACC);
    robot_driver.robot_send_service_.robotServiceSetGlobalMoveEndMaxAngleAcc(MAX_END_ACC);
    robot_driver.robot_send_service_.robotServiceSetGlobalMoveEndMaxLineVelc(MAX_END_VEL);
    robot_driver.robot_send_service_.robotServiceSetGlobalMoveEndMaxAngleVelc(MAX_END_VEL);

     /** loop for 3 times **/
    for(int i=0; i<3; i++)
    {
        ret = robot_driver.robot_send_service_.robotServiceLineMove(postion3, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
            ROS_ERROR("Failed to move to postion3, error code:%d", ret);


        robot_driver.robot_send_service_.robotServiceLineMove(postion4, true);
            ROS_ERROR("Failed to move to postion4, error code:%d", ret);
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "testAuboAPI");

  ros::NodeHandle n;
  AuboDriver robot_driver;
  bool ret = robot_driver.connectToRobotController();

  /** If connect to a real robot, then you need initialize the dynamics parameters　**/
  aubo_robot_namespace::ROBOT_SERVICE_STATE result;
  //tool parameters
  aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
  memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));

  robot_driver.robot_send_service_.rootServiceRobotStartup(toolDynamicsParam/**tool dynamics paramters**/,
                                             6        /*collision class*/,
                                             true     /* Is allowed to read robot pose*/,
                                             true,    /*default */
                                             1000,    /*default */
                                             result); /*initialize*/
  if(ret)
  {
    testMoveJ(robot_driver);
    testMoveL(robot_driver);
  }
  else
      ROS_INFO("Failed to connect to the robot controller");

  return 0;
}
