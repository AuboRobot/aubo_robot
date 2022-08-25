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
#include "aubo_kinematics/kinematics.h"
#include <stdarg.h>
#include <sys/time.h>
#define ARM_DOF 6

const double ZERO_THRESH = 1e-4;
const double PI = M_PI;

#ifdef AUBO_I3_PARAMS
const double a2 = 0.266;
const double a3 = 0.2565;
const double d1 = 0.157;
const double d2 = 0.119;
const double d5 = 0.1025;
const double d6 = 0.094;
#endif

#ifdef AUBO_I3s_PARAMS
const double a2 = 0.229;
const double a3 = 0.2015;
const double d1 = 0.1395;
const double d2 = 0.11055;
const double d5 = 0.08955;
const double d6 = 0.09055;
#endif

#ifdef AUBO_I5_PARAMS
const double a2 =  0.408;
const double a3 =  0.376;
const double d1 =  0.122;
const double d2 =  0.1215;
const double d5 =  0.1025;
const double d6 =  0.094;
#endif

#ifdef AUBO_I5s_PARAMS
const double a2 = 0.245;
const double a3 = 0.215;
const double d1 = 0.1215;
const double d2 = 0.1215;
const double d5 = 0.1025;
const double d6 = 0.094;
#endif

#ifdef AUBO_I5l_PARAMS
const double a2 = 0.608;
const double a3 = 0.6395;
const double d1 = 0.1215;
const double d2 = 0.1215;
const double d5 = 0.1025;
const double d6 = 0.094;
#endif

#ifdef AUBO_I7_PARAMS
const double a2 = 0.552;
const double a3 = 0.495;
const double d1 = 0.1632;
const double d2 = 0.178;
const double d5 = 0.1025;
const double d6 = 0.094;
#endif


#ifdef AUBO_I10_PARAMS
const double a2 = 0.647;
const double a3 = 0.6005;
const double d1 = 0.1632;
const double d2 = 0.2013;
const double d5 = 0.1025;
const double d6 = 0.094;
#endif




int SIGN(double x)
{
    return (x > 0) - (x < 0);
}



/**
 * @brief antiSinCos
 * @param sA
 * @param cA
 * @return
 */
double antiSinCos(double sA, double cA)
{
    double eps = 1e-8;
    double angle = 0;
    if((fabs(sA) < eps)&&(fabs(cA) < eps))
    {
        return 0;
    }
    if(fabs(cA) < eps)
        angle = M_PI/2.0*SIGN(sA);
    else if(fabs(sA) < eps)
    {
        if (SIGN(cA) == 1)
            angle = 0;
        else
            angle = M_PI;
    }
    else
    {
        angle = atan2(sA, cA);
    }

    return angle;
}



// simple 3*3MatriXd to RPY
VectorXd RotMatrixtoRPY(const MatrixXd R)
{
    VectorXd rpy(3);
    double qz,qy,qx;
    double a = 1e-5;

    if((fabs(R(0,0)) < 1e-5) && (fabs(R(1,0)) < 1e-5))
    {
        //singularity
        qz = 0;     //yaw
        qy = atan2(-R(2,0), R(0,0));  // pitch
        qx = atan2(-R(1,2), R(1,1));  //roll
    }
    else
    {
        qz = atan2(R(1,0), R(0,0));
        double sz = sin(qz);
        double cz = cos(qz);
        qy = atan2(-R(2,0), cz * R(0,0) + sz * R(1,0));
        qx = atan2(sz * R(0,2) - cz * R(1,2), cz*R(1,1) - sz*R(0,1));
    }
    rpy(0) = qx;
    rpy(1) = qy;
    rpy(2) = qz;

    return rpy;
}



// simple 3*3MatriXd to Quat
void RotMatrixtoQuat(const MatrixXd R, double quat[])
{
    double temp1 = R(0,0) + R(1,1) + R(2,2) + 1;
    if(std::abs(temp1) < 1e-6)
        temp1 = 0.0;
    double w = (1.0/2) * std::sqrt(temp1);
    double x = (R(2,1) - R(1,2))/(4*w);
    double y = (R(0,2) - R(2,0))/(4*w);
    double z = (R(1,0) - R(0,1))/(4*w);

    quat[0] = w;
    quat[1] = x;
    quat[2] = y;
    quat[3] = z;
}



MatrixXd RotZ(double t)
{
    double   ct = cos(t);
    double   st = sin(t);
    MatrixXd rz(3,3);
    rz<< ct,-st,0,
            st,ct,0,
            0,0,1;

    return rz;
}


MatrixXd RotY(double t)
{
    double   ct = cos(t);
    double   st = sin(t);
    MatrixXd ry(3,3);
    ry<< ct,0,st,
            0,1,0,
            -st,0,ct;

    return ry;
}



MatrixXd RotX(double t)
{
    double   ct = cos(t);
    double   st = sin(t);
    MatrixXd rx(3,3);

    rx<< 1,0,0,
            0,ct,-st,
            0,st,ct;

    return rx;
}




/**
 * @brief RPYtoRotMatrix
 * @param rpy: input rpy[] = {qx,qy,qz};
 * @return output R = rotz(qz)*rot(qy)*rot(qx);
 */
MatrixXd RPYtoRotMatrix(double rpy[])
{
    MatrixXd eerot(3, 3);
    eerot = RotZ(rpy[2]) *  RotY(rpy[1]) * RotX(rpy[0]);

    return eerot;
}



void quaternionToOriMatrix(const double quat[], double eerot[])
{
    double qw;
    double qx;
    double qy;
    double qz;
    double n;

    qw = quat[0];
    qx = quat[1];
    qy = quat[2];
    qz = quat[3];
    n = 1.0f/sqrt(qx*qx+qy*qy+qz*qz+qw*qw);
    qw *= n;
    qx *= n;
    qy *= n;
    qz *= n;
    eerot[0] = 1.0f - 2.0f*qy*qy - 2.0f*qz*qz;  eerot[1] = 2.0f*qx*qy - 2.0f*qz*qw;         eerot[2] = 2.0f*qx*qz + 2.0f*qy*qw;
    eerot[3] = 2.0f*qx*qy + 2.0f*qz*qw;         eerot[4] = 1.0f - 2.0f*qx*qx - 2.0f*qz*qz;  eerot[5] = 2.0f*qy*qz - 2.0f*qx*qw;
    eerot[6] = 2.0f*qx*qz - 2.0f*qy*qw;         eerot[7] = 2.0f*qy*qz + 2.0f*qx*qw;         eerot[8] = 1.0f - 2.0f*qx*qx - 2.0f*qy*qy;
}



void oriandPositiontoTransMatrix(const double eerot[], const double position[], MatrixXd& T)
{
    for(int i = 0; i < 3; i++)
    {
        T(i,0) = eerot[3*i + 0];
        T(i,1) = eerot[3*i + 1];
        T(i,2) = eerot[3*i + 2];
        T(i,3) = position[i];
    }
    T.row(3) << 0, 0, 0, 1;
}




// input radian
void aubo_forward(MatrixXd& T, const VectorXd q)
{
    double  q1 = q(0), q2 = q(1), q3 = q(2), q4 = q(3), q5 = q(4), q6 = q(5);
    double  C1 = cos(q1), C2 = cos(q2), C4 = cos(q4), C5 = cos(q5), C6 = cos(q6);
    double  C23 = cos(q2 - q3), C234 = cos(q2 - q3 + q4), C2345 = cos(q2 - q3 + q4 - q5), C2345p = cos(q2 - q3 + q4 + q5);
    double  S1 = sin(q1), S2 = sin(q2), S4 = sin(q4), S5 = sin(q5), S6 = sin(q6);
    double  S23 = sin(q2 - q3), S234 = sin(q2 - q3 + q4);

    T(0,0) = -C6 * S1 * S5 + C1 * (C234 * C5 * C6 - S234 * S6);
    T(0,1) = S1 * S5 * S6 - C1 * (C4 * C6 * S23 + C23 * C6 * S4 + C234 * C5 * S6);
    T(0,2) = C5 * S1 + C1 * C234 * S5;
    T(0,3) = (d2 + C5 * d6) * S1 - C1 * (a2 * S2 + (a3 + C4 * d5) * S23 + C23 * d5 * S4 - C234 * d6 * S5);

    T(1,0) = C234 * C5 * C6 * S1 + C1 * C6 * S5 - S1 * S234 * S6;
    T(1,1) = -C6 * S1 * S234 - (C234 * C5 * S1 + C1 * S5) * S6;
    T(1,2) = -C1 * C5 + C234 * S1 * S5;
    T(1,3) = -C1 * (d2 + C5 * d6) - S1 * (a2 * S2 + (a3 + C4 * d5) * S23 + C23 * d5 * S4 - C234 * d6 * S5);

    T(2,0) = C5 * C6 * S234 + C234 * S6;
    T(2,1) = C234 * C6 - C5 * S234 * S6;
    T(2,2) = S234 * S5;
    T(2,3) = d1 + a2 * C2 + a3 * C23 + d5 * C234 + d6 * C2345/2 - d6 * C2345p / 2;
    T.row(3) << 0, 0, 0, 1;
}



// output radian//MatrixXd q_sols(6,8);
int aubo_inverse(MatrixXd& q_sols, const MatrixXd& T)
{
    bool singularity = false;

    int num_sols = 0;
    double nx = T(0,0); double ox = T(0,1); double ax = T(0,2); double px = T(0,3);
    double ny = T(1,0); double oy = T(1,1); double ay = T(1,2); double py = T(1,3);
    double nz = T(2,0); double oz = T(2,1); double az = T(2,2); double pz = T(2,3);

    //////////////////////// shoulder rotate joint (q1) //////////////////////////////
    VectorXd q1(2);

    double A1 = d6 * ay - py;
    double B1 = d6 * ax - px;
    double R1 = A1 * A1 + B1 * B1 - d2 * d2;


    if(R1 < 0.0)
        return num_sols;
    else
    {
        double R12 = sqrt(R1);
        q1(0) =  antiSinCos(A1, B1) -  antiSinCos(d2, R12);
        q1(1) =  antiSinCos(A1, B1) -  antiSinCos(d2, -R12);
        for(int i = 0; i < 2; i++)
        {
            while(q1(i) > M_PI)
                q1(i) -= 2 * M_PI;
            while(q1(i) < -M_PI)
                q1(i) += 2 * M_PI;
        }
    }

    ////////////////////////////// wrist 2 joint (q5) //////////////////////////////
    MatrixXd q5(2,2);

    for(int i = 0; i < 2; i++)
    {

        double C1 = cos(q1(i)), S1 = sin(q1(i));
        double B5 = -ay * C1 + ax * S1;
        double M5 = (-ny * C1 + nx * S1);
        double N5 = (-oy * C1 + ox * S1);

        double R5 = sqrt(M5 * M5 + N5 * N5);

        q5(i,0) = antiSinCos(R5, B5);
        q5(i,1) = antiSinCos(-R5, B5);
    }

    ////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////// wrist 3 joint (q6) //////////////////////////////
    double q6;
    VectorXd q3(2), q2(2), q4(2);

    for(int i = 0; i < 2; i++)
    {
        for(int j = 0; j < 2; j++)
        {
            // wrist 3 joint (q6) //
            double C1 = cos(q1(i)), S1 = sin(q1(i));
            double S5 = sin(q5(i,j));

            double A6 = (-oy * C1 + ox * S1);
            double B6 = (ny * C1 - nx * S1);

            if(fabs(S5) < ZERO_THRESH) //the condition is only dependent on q1
            {
                singularity = true;
                break;
            }
            else
                q6 = antiSinCos(A6 * S5, B6 * S5);

            /////// joints (q3,q2,q4) //////
            double C6 = cos(q6);
            double S6 = sin(q6);

            double pp1 = C1 * (ax * d6 - px + d5 * ox * C6 + d5 * nx * S6) + S1 * (ay * d6 - py + d5 * oy * C6 + d5 * ny * S6);
            double pp2 = -d1 - az * d6 + pz - d5 * oz * C6 - d5 * nz * S6;
            double B3 = (pp1 * pp1 + pp2 * pp2 - a2 * a2 - a3 * a3) / (2 * a2 * a3);


            if((1 - B3 * B3) < ZERO_THRESH)
            {
                singularity = true;
                continue;
            }
            else
            {
                double Sin3 = sqrt(1 - B3 * B3);
                q3(0) = antiSinCos(Sin3, B3);
                q3(1) = antiSinCos(-Sin3, B3);
            }

            for(int k = 0; k < 2; k++)
            {

                double C3 = cos(q3(k)), S3 = sin(q3(k));
                double A2 = pp1 * (a2 + a3 * C3) + pp2 * (a3 * S3);
                double B2 = pp2 * (a2 + a3 * C3) - pp1 * (a3 * S3);

                q2(k) = antiSinCos(A2, B2);

                double C2 = cos(q2(k)), S2 = sin(q2(k));

                double A4 = -C1 * (ox * C6 + nx * S6) - S1 * (oy * C6 + ny * S6);
                double B4 = oz * C6 + nz * S6;
                double A41 = pp1 - a2 * S2;
                double B41 = pp2 - a2 * C2;

                q4(k) = antiSinCos(A4, B4) - antiSinCos(A41, B41);
                while(q4(k) > M_PI)
                    q4(k) -= 2 * M_PI;
                while(q4(k) < -M_PI)
                    q4(k) += 2 * M_PI;

                q_sols(0,num_sols) = q1(i);    q_sols(1,num_sols) = q2(k);
                q_sols(2,num_sols) = q3(k);    q_sols(3,num_sols) = q4(k);
                q_sols(4,num_sols) = q5(i,j);  q_sols(5,num_sols) = q6;
                num_sols++;
            }
        }
    }

    return num_sols;
}

void transfer(MatrixXd& T, double alpha, double a, double d, double theta)
{
    //modified DH
    T <<    cos(theta),            -sin(theta),           0,           a,
            sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d,
            sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha),  cos(alpha)*d,
            0,                     0,                     0,           1;
}



//input current transformation matrix and last joint angle;
//out matrixxd q_sols;
/**
 * @brief processIK test if all solution can FK to the same pose;
 * @param T
 * @return the true solution ;under test , all real_sols == num_sols;with 1000000 angle test;
 */
MatrixXd processIK(const MatrixXd& T)
{
    MatrixXd q_sols(6,8), q_temp(6,8), q_realsols;
    int num_sols = 0, real_sols = 0;

    num_sols = aubo_inverse(q_sols, T);

    for(int i = 0; i < num_sols; i++)
    {
        MatrixXd TT(4,4);
        VectorXd temp(6);
        double err = 0;

        temp = q_sols.col(i);
        aubo_forward(TT, temp);

        for(int j = 0; j < 4; j++)
        {
            for(int k = 0; k < 4; k++)
                err += fabs(TT(j,k) - T(j,k));
        }
        if(err < 1e-5)
        {
            real_sols++;
            q_temp.col(real_sols - 1) = q_sols.col(i);
        }
    }
    if(real_sols != num_sols)
    {
        std::cout <<"IK ERROR"<<std::endl;
    }

    q_realsols = q_temp.block(0,0,6,real_sols);


    if(real_sols == 0)//if num_sols != 8;
    {
        ;
    }
    return q_realsols;
}




/**
 * @brief verifyIK ;test if can IK to the same joint angle with the input angle;
 * @param q_sols
 * @param q_input
 * @return
 */
bool verifyIK(MatrixXd q_sols,const VectorXd q_input)
{
    //TODO LIST: configuration flag
    bool flag = false;
    double err = 0;
    int nn = q_sols.cols();
    for(int i = 0; i < nn; i++)
    {
        err = (q_input - q_sols.col(i)).norm();
        if(err < ZERO_THRESH)
        {
            flag = true;
        }
    }
    return flag;
}




//choose solution ;input all q_sols,and last q_old;
//out put the nearest q solution;
/**
 * @brief chooseIKonRefJoint , choose mode == 0;
 * @param q_sols
 * @param q_ref
 * @param q_choose
 * @return
 */
bool chooseIKonRefJoint(const MatrixXd& q_sols, const VectorXd& q_ref, VectorXd& q_choose)
{
    int nn = q_sols.cols();
    if(nn == 0)
        return false;

    double sum = (q_ref - q_sols.col(0)).norm();
    double err;
    int index = 0;
    for(int i = 0; i < nn; i++)
    {
        err = (q_ref - q_sols.col(i)).norm();
        if(err < sum)
        {
            index = i;
            sum = err;
        }
    }
    q_choose = q_sols.col(index);

    return true;
}




/**
 * @brief chooseIKonConfiguration choose mode == 0~7;
 * @param q_sols
 * @param choose_mode
 * @param q_choose
 * @return
 */
bool chooseIKonConfiguration(const MatrixXd& q_sols, int choose_mode, VectorXd& q_choose)
{
    int N = q_sols.cols();
    for(int i = 0; i < N; i++)
    {
        int flag = (q_sols(0,i) > 0) + ((q_sols(2,i) > 0) << 1) + ((q_sols(4,i) > 0) << 2);
        if(flag == choose_mode)
        {
            q_choose = q_sols.col(i);
            return true;
        }
    }
    return false;
}





bool selectIK(const MatrixXd& q_sols, double AngleLimit[6][2], MatrixXd& q_sols_selected ,u_int8_t& num_sols)
{
    int N = q_sols.cols();
    if( N == 0) return false;
    u_int8_t num = 0;
    bool valid = true;
    for(int i = 0; i < N; i++)
    {
        valid = true;
        for(int j = 0; j < ARM_DOF; j++)
        {
            if(q_sols(j,i) > AngleLimit[j][1] || q_sols(j,i) < AngleLimit[j][0])
            {
                valid = false;
                break;
            }
        }
        if(valid)
        {
            // to do: storage
            num++;
            q_sols_selected.col(num-1) = q_sols.col(i);
            VectorXd tmp = q_sols.col(i);
            do{
                for(int j = 0; j < ARM_DOF; j++)
                {
                    tmp(j) += 2*PI;
                    if(tmp(j) > AngleLimit[j][1])
                    {
                        valid = false;
                        break;
                    }
                }
                if(valid)
                {
                    num++;
                    q_sols_selected.col(num-1) = tmp;
                }
            }while(valid);

            valid = true;
            tmp = q_sols.col(i);
            do{
                for(int j = 0; j < ARM_DOF; j++)
                {
                    tmp(j) -= 2*PI;
                    if(tmp(j) < AngleLimit[j][0])
                    {
                        valid = false;
                        break;
                    }
                }
                if(valid)
                {
                    num++;
                    q_sols_selected.col(num-1) = tmp;
                }
            }while(valid);
        }
    }

    num_sols = num;

    if(num > 0)
        return true;
    else
        return false;
}





bool decisionIK(const MatrixXd& q_sols, int flag, VectorXd& q_choose)
{    
    bool ret;
    //TODO LIST: configuration flag
    int nn = q_sols.cols();
    if(nn == 0)
        return false;
    ret = chooseIKonConfiguration(q_sols, flag, q_choose);
    return ret;
}




bool decisionIK(const MatrixXd& q_sols, const VectorXd& q_ref, VectorXd& q_choose)
{
    bool ret;
    //TODO LIST: configuration flag
    int nn = q_sols.cols();
    if(nn == 0)
        return false;
    ret = chooseIKonRefJoint(q_sols, q_ref, q_choose);
    return ret;
}


bool GetInverseResult(const MatrixXd& T_target, const VectorXd& q_ref, VectorXd& q_result)
{
    u_int8_t num_sols = 0;

    double maxq = 175.0/180.0*M_PI;
    double AngleLimit[6][2] = {{-maxq,maxq},{-maxq,maxq},{-maxq,maxq},{-maxq,maxq},{-maxq,maxq},{-maxq,maxq}};


    MatrixXd q_sols_all(6,8),
             q_sols_RmZero,
             q_sols_inlimit(6,8),
             q_sol_Valid;

    //inverse and remove zero matrixXd
    num_sols = aubo_inverse(q_sols_all, T_target);

    //remove  zero matrixXd
    q_sols_RmZero = q_sols_all.block(0,0,6,num_sols);

    if(q_sols_RmZero.cols() != 0)
    {
        //remove not in limited data and zero matrixXd
        bool ret2 = selectIK(q_sols_RmZero, AngleLimit, q_sols_inlimit, num_sols);

        //remove  zero matrixXd
        q_sol_Valid = q_sols_inlimit.block(0,0,6,num_sols);

        if((q_sol_Valid.cols() != 0) && (true == ret2))
        {
            bool ret3 = chooseIKonRefJoint(q_sol_Valid, q_ref, q_result);

            if(true == ret3)
            {
                std::cout << " find solution choose  " <<std::endl;
                return true;
            }
            else
            {
                std::cout << " No solution choose  " <<std::endl;
                return false;
            }
        }
        else
        {
            std::cout <<"no valid sols "<<std::endl;
            return false;
        }
    }
    else
    {
        std::cout<< "inverse result num is 0" <<std::endl;
        return false;
    }
}
