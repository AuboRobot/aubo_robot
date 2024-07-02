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

#ifndef MODBUS_ADAPTER_OPERATION_MODE_H
#define MODBUS_ADAPTER_OPERATION_MODE_H

#include <memory>

#include <ros/ros.h>

#include <pilz_msgs/OperationModes.h>

#include <prbt_hardware_support/adapter_operation_mode.h>
#include <prbt_hardware_support/modbus_msg_operation_mode_wrapper.h>
#include <prbt_hardware_support/ModbusMsgInStamped.h>
#include <prbt_hardware_support/modbus_api_spec.h>
#include <prbt_hardware_support/filter_pipeline.h>

namespace prbt_hardware_support
{
/**
 * @brief Listens to the modbus_read topic and offers a service
 * informing about the active operation mode
 */
class ModbusAdapterOperationMode : public AdapterOperationMode
{
public:
  ModbusAdapterOperationMode(ros::NodeHandle& nh, const ModbusApiSpec& api_spec);
  virtual ~ModbusAdapterOperationMode() = default;

private:
  /**
   * @brief Called whenever a new modbus messages arrives.
   *
   * @note Filters like for example the UpdateFilter can restrict
   * the number of incoming messages.
   *
   * @note The operation mode gets unknown in case:
   *    - of a disconnect from the modbus server,
   *    - the modbus message does not contain the expected registers,
   *    - the modbus API version is incorrect.
   */
  void modbusMsgCallback(const ModbusMsgInStampedConstPtr& msg_raw);

private:
  static pilz_msgs::OperationModes createUnknownOperationMode();

private:
  const ModbusApiSpec api_spec_;
  std::unique_ptr<FilterPipeline> filter_pipeline_;
};

}  // namespace prbt_hardware_support
#endif  // MODBUS_ADAPTER_OPERATION_MODE_H
