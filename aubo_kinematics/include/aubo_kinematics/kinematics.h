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
