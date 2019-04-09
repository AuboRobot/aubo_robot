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

#ifndef CUBIC_SPLINE_FILTER_H_
#define CUBIC_SPLINE_FILTER_H_

#include "filter_base.h"

namespace aubo_trajectory_filters
{
template<typename T>

  class CubicSplineFilter : public aubo_trajectory_filters::FilterBase<T>
  {
  public:
    /**
     * @brief Default constructor
     */
    CubicSplineFilter();
    /**
     * @brief Default destructor
     */
    ~CubicSplineFilter();

    virtual bool configure();

    bool update(const T& trajectory_in, T& trajectory_out);

    /**
       * @brief Cubic splines with assigned initial and final velocities accelerations
       * @param inputNum -> total number of the points
       * @param inputPoints -> values number of the points
       * @param inputTime -> values number of the time sequence
       * @return
       */
    bool cubicSplinesFilter(const T& trajectory_in, T& trajectory_out);

  private:
    /**
     * @brief uniform sample duration (sec)
     */
    double sample_duration_;
    /**
     * @brief inital velocity(rad/sec)
     */
    double v0_;
    /**
     * @brief final velocity(rad/sec)
     */
    double vn_;
    /**
     * @brief inital acceleration(rad/sec^2)
     */
    double w0_;
    /**
     * @brief final acceleration(rad/sec^2)
     */
    double wn_;
  };

/**
 * @brief Specializing trajectory filter implementation
 */
typedef CubicSplineFilter<MessageAdapter> CubicSplineFilterAdapter;

}

#endif //CUBIC_SPLINE_FILTER_H_
