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

#include <boost/optional.hpp>

#include <prbt_hardware_support/operation_mode_setup_executor.h>

namespace prbt_hardware_support
{
OperationModeSetupExecutor::OperationModeSetupExecutor(const MonitorCartesianSpeedFunc& monitor_cartesian_speed_func)
  : monitor_cartesian_speed_func_(monitor_cartesian_speed_func)
{
}

void OperationModeSetupExecutor::updateOperationMode(const pilz_msgs::OperationModes& operation_mode)
{
  ROS_DEBUG("updateOperationMode: %d", operation_mode.value);
  if (operation_mode.time_stamp <= time_stamp_last_op_mode_)
  {
    return;
  }
  time_stamp_last_op_mode_ = operation_mode.time_stamp;

  boost::optional<bool> monitor_cartesian_speed{ boost::none };
  switch (operation_mode.value)
  {
    case pilz_msgs::OperationModes::T1:
      monitor_cartesian_speed = true;
      speed_override_ = 0.1;
      break;
    case pilz_msgs::OperationModes::AUTO:
      monitor_cartesian_speed = false;
      speed_override_ = 1.0;
      break;
    default:
      speed_override_ = 0.0;
      break;
  }

  if (monitor_cartesian_speed && monitor_cartesian_speed_func_)
  {
    monitor_cartesian_speed_func_(monitor_cartesian_speed.get());
  }
}

bool OperationModeSetupExecutor::getSpeedOverride(pilz_msgs::GetSpeedOverride::Request& /*req*/,
                                                  pilz_msgs::GetSpeedOverride::Response& response)
{
  response.speed_override = speed_override_;
  return true;
}

}  // namespace prbt_hardware_support
