/*
 * Copyright (c) 2019 Pilz GmbH & Co. KG
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef PRBT_HARDWARE_SUPPORT_BRAKE_TEST_EXECUTOR_H
#define PRBT_HARDWARE_SUPPORT_BRAKE_TEST_EXECUTOR_H

#include <functional>
#include <stdexcept>
#include <string>

#include <ros/ros.h>

#include <pilz_msgs/BrakeTest.h>

#include <prbt_hardware_support/BrakeTest.h>

namespace prbt_hardware_support
{
using DetectRobotMotionFunc = std::function<bool()>;
using ControllerHoldFunc = std::function<void()>;
using TriggerBrakeTestFunc = std::function<BrakeTest::Response()>;
using ControllerUnholdFunc = std::function<void()>;
using BrakeTestResultFunc = std::function<bool(const bool)>;

class BrakeTestExecutorException : public std::runtime_error
{
public:
  BrakeTestExecutorException(const std::string& what_arg) : std::runtime_error(what_arg)
  {
  }
};

/**
 * @brief Triggers execution of brake tests only if the controller is not
 * executing a trajectory.
 *
 */
class BrakeTestExecutor
{
public:
  BrakeTestExecutor(DetectRobotMotionFunc&& detect_robot_motion_func, ControllerHoldFunc&& controller_hold_func,
                    TriggerBrakeTestFunc&& trigger_brake_test_func, ControllerUnholdFunc&& unhold_func,
                    BrakeTestResultFunc&& brake_test_result_fun);

public:
  bool executeBrakeTest(pilz_msgs::BrakeTest::Request& req, pilz_msgs::BrakeTest::Response& response);

private:
  DetectRobotMotionFunc detect_robot_motion_func_;
  ControllerHoldFunc hold_controller_func_;
  TriggerBrakeTestFunc execute_brake_test_func_;
  ControllerUnholdFunc unhold_controller_func_;
  BrakeTestResultFunc brake_test_result_func_;
};

}  // namespace prbt_hardware_support
#endif  // PRBT_HARDWARE_SUPPORT_BRAKE_TEST_EXECUTOR_H
