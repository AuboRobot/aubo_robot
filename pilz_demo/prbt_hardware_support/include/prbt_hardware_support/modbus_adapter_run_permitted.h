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

#ifndef MODBUS_ADAPTER_RUN_PERMITTED_H
#define MODBUS_ADAPTER_RUN_PERMITTED_H

#include <memory>
#include <functional>

#include <ros/ros.h>

#include <prbt_hardware_support/ModbusMsgInStamped.h>
#include <prbt_hardware_support/modbus_api_spec.h>

namespace prbt_hardware_support
{
using UpdateRunPermittedFunc = std::function<void(const bool)>;

/**
 * @brief Listens to the modbus_read topic and reacts to updated RUN_PERMITTED states.
 */
class ModbusAdapterRunPermitted
{
public:
  ModbusAdapterRunPermitted(UpdateRunPermittedFunc&& update_run_permitted_func, const ModbusApiSpec& api_spec);

public:
  /**
   * @brief Called whenever a new modbus messages arrives.
   *
   * @note Filters like for example the UpdateFilter can restrict
   * the number of incoming messages.
   *
   * @note A STOP1 is triggered in case:
   *    - of a disconnect from the modbus server,
   *    - the modbus message does not contain the expected registers,
   *    - the modbus API version is incorrect,
   *    - the RUN_PERMITTED state changes to false.
   */
  void modbusMsgCallback(const ModbusMsgInStampedConstPtr& msg_raw);

private:
  const ModbusApiSpec api_spec_;
  UpdateRunPermittedFunc update_run_permitted_;
};

}  // namespace prbt_hardware_support
#endif  // MODBUS_ADAPTER_RUN_PERMITTED_H
