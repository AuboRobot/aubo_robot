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

#ifndef ADAPTER_OPERATION_MODE_H
#define ADAPTER_OPERATION_MODE_H

#include <mutex>

#include <ros/ros.h>

#include <pilz_msgs/GetOperationMode.h>
#include <pilz_msgs/OperationModes.h>

namespace prbt_hardware_support
{
/**
 * @brief Publishes information on the active operation mode. Also offers a service for querying the operation mode.
 */
class AdapterOperationMode
{
public:
  AdapterOperationMode(ros::NodeHandle& nh);
  virtual ~AdapterOperationMode() = default;

protected:
  /**
   * @brief Stores the operation mode and publishes it, if it has changed.
   */
  void updateOperationMode(const pilz_msgs::OperationModes& mode);

private:
  /**
   * @brief Initializes the operation mode service.
   */
  void initOperationModeService();

  bool getOperationMode(pilz_msgs::GetOperationMode::Request& req, pilz_msgs::GetOperationMode::Response& res);

private:
  //! Store the current operation mode according to OperationModes.msg
  pilz_msgs::OperationModes op_mode_;

  //! Protects read/write of the operation mode
  std::mutex op_mode_mutex_;

  //! The node handle
  ros::NodeHandle& nh_;

  //! Informs about operation mode changes via topic.
  ros::Publisher operation_mode_pub_;

  //! Server serving a service to ask whether a brake test is currently required
  ros::ServiceServer operation_mode_server_;
};

}  // namespace prbt_hardware_support
#endif  // ADAPTER_OPERATION_MODE_H
