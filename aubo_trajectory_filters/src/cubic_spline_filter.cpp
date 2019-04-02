/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017-2019, AUBO Robotics
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

#include "cubic_spline_filter.h"
#include <ros/ros.h>

using namespace aubo_trajectory_filters;

const double DEFAULT_SAMPLE_DURATION = 0.005; //seconds

template<typename T>
CubicSplineFilter<T>::CubicSplineFilter() :
    aubo_trajectory_filters::FilterBase<T>()
{
    ROS_INFO_STREAM("Constructing cubic spline filter");
    sample_duration_ = DEFAULT_SAMPLE_DURATION;
    v0_ = 0;
    vn_ = 0;
    w0_ = 0;
    wn_ = 0;
    this->filter_name_ = "CubicSplineFilter";
    this->filter_type_ = "CubicSplineFilter";
}

template<typename T>
CubicSplineFilter<T>::~CubicSplineFilter()
{
}

template<typename T>
bool CubicSplineFilter<T>::configure()
{
    if (!this->nh_.getParam("sample_duration", sample_duration_))
    {
        ROS_WARN_STREAM( "UniformSampleFilter, params has no attribute sample_duration.");
    }
    ROS_INFO_STREAM("Using a sample_duration value of " << sample_duration_);

    return true;
}

template<typename T>
bool CubicSplineFilter<T>::update(const T& trajectory_in, T& trajectory_out)
{
    bool success = false;
    success = cubicSplinesFilter(trajectory_in, trajectory_out);
    if(success)
        ROS_INFO_STREAM("CubicSplineFilter success: " << " output traj. size: " << trajectory_out.request.trajectory.points.size());
    else
        ROS_WARN_STREAM("CubicSplineFilter failed: ");

    return success;
}


template<typename T>
bool CubicSplineFilter<T>::cubicSplinesFilter(const T& trajectory_in, T& trajectory_out)
{
    ROS_INFO_STREAM("CubicSplineFilter start");
    size_t inputNum = trajectory_in.request.trajectory.points.size();
    if(inputNum < 3)            //at least 3 points to apply the algorithm
        return false;

    trajectory_out = trajectory_in;
    // Clear out the trajectory points

    size_t axisNum = trajectory_in.request.trajectory.points[0].positions.size();
//    double duration_start = trajectory_in.request.trajectory.points[0].time_from_start.toSec();
//    double duration_end = trajectory_in.request.trajectory.points.back().time_from_start.toSec();
    trajectory_out.request.trajectory.points.resize(inputNum+2);
    ROS_INFO_STREAM("Total points."<< inputNum+2);
    for(int i = inputNum; i < inputNum+2; i++)
    {
        trajectory_out.request.trajectory.points[i].positions.resize(axisNum);
        trajectory_out.request.trajectory.points[i].velocities.resize(axisNum);
        trajectory_out.request.trajectory.points[i].accelerations.resize(axisNum);
//        trajectory_out.request.trajectory.points[i].effort.resize(axisNum);
    }
    std::vector<double> inputTime, inputPoints;
    for(int nCount = 0; nCount < axisNum; nCount++)
    {
        inputTime.resize(0);
        inputPoints.resize(0);
        double sigleLength, lastLength = 0, totalLength = 0;
        for(int i = 0; i < inputNum; i++)
        {
            sigleLength = trajectory_in.request.trajectory.points[i].positions[nCount];
            inputPoints.push_back(sigleLength);
//            if(i > 0)
//                totalLength = abs(sigleLength - lastLength);
//            lastLength = sigleLength;
            inputTime.push_back(trajectory_in.request.trajectory.points[i].time_from_start.toSec());   // need to reparamterization if time intertval is not reasonable
        }

        double insertTime1 = inputTime[0]/2 + inputTime[1]/2;
        double insertTime2 = inputTime[inputNum-2]/2 + inputTime[inputNum-1]/2;

        std::vector<double> timeSequence, timeInterval, position, velocity, acceleration, eVector, cVector;
        timeSequence.resize(inputNum+2);
        position.resize(inputNum+2);
        velocity.resize(inputNum+2);
        acceleration.resize(inputNum+2);
        timeInterval.resize(inputNum+1);
        eVector.resize(inputNum);
        cVector.resize(inputNum+2);

        std::vector<std::vector<double> > dMatrix;
        dMatrix.resize(inputNum);

        for(int i = 0; i < inputNum; i++)
        {
            dMatrix[i].resize(inputNum);
            if(i==0)
            {
                timeSequence[0] = inputTime[0];
                timeSequence[1] = insertTime1;
                position[0] = inputPoints[0];
                position[1] = 0;
            }
            else if(i==inputNum-1)
            {
                timeSequence[inputNum] = insertTime2;
                timeSequence[inputNum+1] = inputTime[inputNum-1];
                position[inputNum] = 0;
                position[inputNum+1] = inputPoints[inputNum-1];
            }
            else
            {
                timeSequence[i+1] = inputTime[i];
                position[i+1] = inputPoints[i];
            }
        }

        for(int i = 0; i <inputNum+1; i++)
        {
            timeInterval[i] = timeSequence[i+1] - timeSequence[i];
            if(timeInterval[i] <= 0)
                return false;
        }

        dMatrix[0][0] = 3*timeInterval[0] + 2*timeInterval[1] + timeInterval[0]*timeInterval[0]/timeInterval[1];
        dMatrix[0][1] = timeInterval[1];
        dMatrix[1][0] = timeInterval[1] - timeInterval[0]*timeInterval[0]/timeInterval[1];
        dMatrix[1][1] = 2*(timeInterval[1] + timeInterval[2]);
        dMatrix[1][2] = timeInterval[2];
        eVector[0] = 6*((position[2]-position[0])/timeInterval[1] - v0_*(1+timeInterval[0]/timeInterval[1]) - w0_*timeInterval[0]/2 - w0_*timeInterval[0]*timeInterval[0]/(3*timeInterval[1]));
        eVector[1] = 6*((position[3]-position[2])/timeInterval[2] - (position[2]-position[0])/timeInterval[1] + v0_*timeInterval[0]/timeInterval[1] + w0_*timeInterval[0]*timeInterval[0]/(3*timeInterval[1]));
        for(int i = 2; i <inputNum-2; i++)
        {
            dMatrix[i][i-1] = timeInterval[i];
            dMatrix[i][i] = 2*(timeInterval[i] + timeInterval[i+1]);
            dMatrix[i][i+1] = timeInterval[i+1];
            eVector[i] = 6*((position[i+2] - position[i+1])/timeInterval[i+1] - (position[i+1] - position[i])/ timeInterval[i]);
        }
        dMatrix[inputNum-2][inputNum-3] = timeInterval[inputNum-2];
        dMatrix[inputNum-2][inputNum-2] = 2*(timeInterval[inputNum-2] + timeInterval[inputNum-1]);
        dMatrix[inputNum-2][inputNum-1] = timeInterval[inputNum-1] - timeInterval[inputNum]*timeInterval[inputNum]/timeInterval[inputNum-1];
        dMatrix[inputNum-1][inputNum-2] = timeInterval[inputNum-1];
        dMatrix[inputNum-1][inputNum-1] = 3*timeInterval[inputNum] + 2*timeInterval[inputNum-1] + timeInterval[inputNum]*timeInterval[inputNum]/timeInterval[inputNum-1] ;

        eVector[inputNum-2] = 6*((position[inputNum+1] - position[inputNum-1])/timeInterval[inputNum-1] - (position[inputNum-1] - position[inputNum-2])/timeInterval[inputNum-2] - vn_*timeInterval[inputNum]/timeInterval[inputNum-1] + wn_*timeInterval[inputNum]*timeInterval[inputNum]/(3*timeInterval[inputNum-1]));
        eVector[inputNum-1] = 6*((position[inputNum-1] - position[inputNum+1])/timeInterval[inputNum-1] + vn_ + vn_*timeInterval[inputNum]/timeInterval[inputNum-1] -wn_*timeInterval[inputNum]/2 + wn_*timeInterval[inputNum]*timeInterval[inputNum]/(3*timeInterval[inputNum-1]));

        //inverse of dMatrix
        double mm = 0;
        cVector[0] = dMatrix[0][1] / dMatrix[0][0];
        eVector[0] = eVector[0] /dMatrix[0][0];
        for(int i = 1; i < inputNum; i++)
        {
            mm = 1/(dMatrix[i][i]-dMatrix[i][i-1]*cVector[i-1]);
            cVector[i] = dMatrix[i][i+1]*mm;
            eVector[i] = (eVector[i] - eVector[i-1]*dMatrix[i][i-1])*mm;
        }

        acceleration[inputNum] = eVector[inputNum-1];
        for(int i = inputNum-2; i >= 0; i--)
            acceleration[i+1] = eVector[i] - cVector[i]*acceleration[i+2];
        acceleration[0] = w0_;
        acceleration[inputNum+1] = wn_;

        position[1] = position[0] + timeInterval[0]*v0_ + timeInterval[0]*timeInterval[0]*acceleration[0]/3 + timeInterval[0]*timeInterval[0]*acceleration[1]/6;
        position[inputNum] = position[inputNum+1] - timeInterval[inputNum]*vn_ + timeInterval[inputNum]*timeInterval[inputNum]*acceleration[inputNum+1]/3 + timeInterval[inputNum]*timeInterval[inputNum]*acceleration[inputNum]/6;

        std::vector<double> a0_coef, a1_coef, a2_coef, a3_coef;
        a0_coef.resize(inputNum+1);
        a1_coef.resize(inputNum+1);
        a2_coef.resize(inputNum+1);
        a3_coef.resize(inputNum+1);
        for(int i = 0; i < inputNum+1; i++)
        {
            a0_coef[i] = position[i];
            a1_coef[i] = (position[i+1] - position[i])/timeInterval[i] - acceleration[i]*timeInterval[i]/2 - (acceleration[i+1] - acceleration[i])*timeInterval[i]/6;
            a2_coef[i] = acceleration[i]/2;
            a3_coef[i] = (acceleration[i+1] - acceleration[i])/(6*timeInterval[i]);
        }

        for(int i = 0; i <= inputNum; i++)
            velocity[i+1] = a1_coef[i] + 2*a2_coef[i]*timeInterval[i] + 3*a3_coef[i]*timeInterval[i]*timeInterval[i];
        velocity[0] = v0_;
        velocity[inputNum+1] = vn_;

        for(int i = 0; i < inputNum+2; i++)
        {
            trajectory_out.request.trajectory.points[i].positions[nCount] = position[i];
            trajectory_out.request.trajectory.points[i].velocities[nCount] = velocity[i];
            trajectory_out.request.trajectory.points[i].accelerations[nCount] = acceleration[i];
//            trajectory_out.request.trajectory.points[j].effort[nCount] = 0.0;
            ros::Duration time_from_start_dur(timeSequence[i]);
            trajectory_out.request.trajectory.points[i].time_from_start = time_from_start_dur;
        }
    }
    ROS_INFO_STREAM("CubicSplineFilter finish.");

    return true;
}


// registering planner adapter
CLASS_LOADER_REGISTER_CLASS( aubo_trajectory_filters::CubicSplineFilterAdapter,
                             planning_request_adapter::PlanningRequestAdapter);

/*
 * Old plugin declaration for arm navigation trajectory filters
 PLUGINLIB_DECLARE_CLASS(industrial_trajectory_filters,
 IndustrialNPointFilterJointTrajectoryWithConstraints,
 industrial_trajectory_filters::NPointFilter<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>,
 filters::FilterBase<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>);

 */

