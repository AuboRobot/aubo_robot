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

#ifndef OPERATION_MODE_SETUP_EXECUTOR_H
#define OPERATION_MODE_SETUP_EXECUTOR_H

#include <atomic>

#include <ros/ros.h>

#include <pilz_msgs/GetSpeedOverride.h>
#include <pilz_msgs/OperationModes.h>

#include <prbt_hardware_support/monitor_cartesian_speed_func_decl.h>
#include <prbt_hardware_support/get_operation_mode_func_decl.h>

namespace prbt_hardware_support
{
/**
 * @brief Activates speed monitoring and sets the speed override based on
 * the current operation mode.
 */
class OperationModeSetupExecutor
{
public:
  /**
   * @brief Ctor.
   *
   * @param monitor_cartesian_speed_func Function allowing to turn on/off the monitoring of the cartesian speed.
   */
  OperationModeSetupExecutor(const MonitorCartesianSpeedFunc& monitor_cartesian_speed_func);

public:
  /**
   * @brief Function to be called whenever a new operation mode is set.
   */
  void updateOperationMode(const pilz_msgs::OperationModes& operation_mode);

  bool getSpeedOverride(pilz_msgs::GetSpeedOverride::Request& /*req*/, pilz_msgs::GetSpeedOverride::Response& response);

private:
  //! The active speed override
  std::atomic<double> speed_override_{ 0.0 };

  //! Function used to (de-)activate cartesian speed monitoring.
  MonitorCartesianSpeedFunc monitor_cartesian_speed_func_;
  //! Time stamp of the last received operation mode.
  ros::Time time_stamp_last_op_mode_{ ros::Time(0) };
};

}  // namespace prbt_hardware_support

#endif  // OPERATION_MODE_SETUP_EXECUTOR_H
