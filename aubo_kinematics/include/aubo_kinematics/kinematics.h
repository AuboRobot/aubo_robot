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
#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <eigen3/Eigen/Dense>


using Eigen::MatrixXd;
using Eigen::Matrix4d;
using Eigen::VectorXd;
using Eigen::Vector3d;

#define MIN -175    //随机数产生的范围
#define MAX 175
#define MAXIUM_ITERATOR 10

int SIGN(double x);


double antiSinCos(double sA, double cA);

VectorXd RotMatrixtoRPY(const MatrixXd R);

void RotMatrixtoQuat(const MatrixXd R, double quat[]);

MatrixXd RotZ(double t);

MatrixXd RotY(double t);

MatrixXd RotX(double t);

MatrixXd RPYtoRotMatrix(double rpy[]);

void quaternionToOriMatrix(const double quat[], double eerot[]);

void oriandPositiontoTransMatrix(const double eerot[], const double position[], MatrixXd& T);

// input radian
void aubo_forward(MatrixXd& T, const VectorXd q);


// output radian//MatrixXd q_sols(6,8);
int aubo_inverse(MatrixXd& q_sols, const MatrixXd& T);


void transfer(MatrixXd& T, double alpha, double a, double d, double theta);


MatrixXd processIK(const MatrixXd& T);


bool verifyIK(MatrixXd q_sols,const VectorXd q_input);


bool chooseIKonRefJoint(const MatrixXd& q_sols, const VectorXd& q_ref, VectorXd& q_choose);


bool chooseIKonConfiguration(const MatrixXd& q_sols, int choose_mode, VectorXd& q_choose);

bool selectIK(const MatrixXd& q_sols, double AngleLimit[6][2], MatrixXd& q_sols_selected , u_int8_t &num_sols);



bool decisionIK(const MatrixXd& q_sols, int flag, VectorXd& q_choose);


bool decisionIK(const MatrixXd& q_sols, const VectorXd& q_ref, VectorXd& q_choose);

bool GetInverseResult(const MatrixXd& T_target, const VectorXd& q_ref, VectorXd& q_result);






#endif // KINEMATICS_H
