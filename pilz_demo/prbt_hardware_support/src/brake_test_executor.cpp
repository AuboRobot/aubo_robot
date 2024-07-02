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

#include <pilz_msgs/BrakeTest.h>

#include <prbt_hardware_support/brake_test_executor.h>

namespace prbt_hardware_support
{
BrakeTestExecutor::BrakeTestExecutor(DetectRobotMotionFunc&& detect_robot_motion_func,
                                     ControllerHoldFunc&& controller_hold_func,
                                     TriggerBrakeTestFunc&& trigger_brake_test_func, ControllerUnholdFunc&& unhold_func,
                                     BrakeTestResultFunc&& brake_test_result_func)
  : detect_robot_motion_func_(detect_robot_motion_func)
  , hold_controller_func_(controller_hold_func)
  , execute_brake_test_func_(trigger_brake_test_func)
  , unhold_controller_func_(unhold_func)
  , brake_test_result_func_(brake_test_result_func)
{
  if (!detect_robot_motion_func_)
  {
    throw BrakeTestExecutorException("Function to detect robot motion missing");
  }

  if (!hold_controller_func_)
  {
    throw BrakeTestExecutorException("Function to hold controller missing");
  }

  if (!execute_brake_test_func_)
  {
    throw BrakeTestExecutorException("Function to trigger brake test on robot missing");
  }

  if (!unhold_controller_func_)
  {
    throw BrakeTestExecutorException("Function to unhold controller missing");
  }

  if (!brake_test_result_func_)
  {
    throw BrakeTestExecutorException("Function to send brake test result to FS controller missing");
  }
}

bool BrakeTestExecutor::executeBrakeTest(pilz_msgs::BrakeTest::Request& /*req*/, pilz_msgs::BrakeTest::Response& res)
{
  if (detect_robot_motion_func_())
  {
    res.success = false;
    res.error_msg = "Robot is moving, cannot perform brake test";
    return true;
  }

  ROS_INFO_STREAM("Enter hold for braketest. Do not unhold the controller!");

  hold_controller_func_();

  BrakeTest::Response adapter_result = execute_brake_test_func_();
  ROS_INFO("Brake test result: %i", adapter_result.success);
  if (!adapter_result.success)
  {
    ROS_INFO("Brake test error code: %i", adapter_result.error_code.value);
    if (!adapter_result.error_msg.empty())
    {
      ROS_INFO_STREAM("Brake test error msg: " << adapter_result.error_msg);
    }
  }

  unhold_controller_func_();

  if (!brake_test_result_func_(adapter_result.success))
  {
    res.success = false;
    res.error_msg = "Failed to send brake test result";
  }
  else
  {
    res.success = adapter_result.success;
    res.error_msg = adapter_result.error_msg;
  }

  return true;
}

}  // namespace prbt_hardware_support
