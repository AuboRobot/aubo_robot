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

#include <prbt_hardware_support/modbus_api_definitions.h>
#include <prbt_hardware_support/modbus_adapter_operation_mode.h>

#include <ros/time.h>

#include <sstream>
#include <functional>

namespace prbt_hardware_support
{
using std::placeholders::_1;

using namespace modbus_api::v3;

ModbusAdapterOperationMode::ModbusAdapterOperationMode(ros::NodeHandle& nh, const ModbusApiSpec& api_spec)
  : AdapterOperationMode(nh)
  , api_spec_(api_spec)
  , filter_pipeline_(new FilterPipeline(nh, std::bind(&ModbusAdapterOperationMode::modbusMsgCallback, this, _1)))
{
}

pilz_msgs::OperationModes ModbusAdapterOperationMode::createUnknownOperationMode()
{
  pilz_msgs::OperationModes op_mode;
  op_mode.time_stamp = ros::Time::now();
  op_mode.value = pilz_msgs::OperationModes::UNKNOWN;
  return op_mode;
}

void ModbusAdapterOperationMode::modbusMsgCallback(const ModbusMsgInStampedConstPtr& msg_raw)
{
  ModbusMsgOperationModeWrapper msg{ msg_raw, api_spec_ };

  if (msg.isDisconnect())
  {
    updateOperationMode(createUnknownOperationMode());
    return;
  }

  try
  {
    msg.checkStructuralIntegrity();
  }
  catch (const prbt_hardware_support::ModbusMsgWrapperException& ex)
  {
    ROS_ERROR_STREAM(ex.what());
    updateOperationMode(createUnknownOperationMode());
    return;
  }

  if (msg.getVersion() != MODBUS_API_VERSION_REQUIRED)
  {
    std::ostringstream os;
    os << "Received Modbus message of unsupported API Version: " << msg.getVersion()
       << ", required Version: " << MODBUS_API_VERSION_REQUIRED;
    os << "\n";
    os << "Can not determine OperationMode from Modbus message.";
    ROS_ERROR_STREAM(os.str());
    updateOperationMode(createUnknownOperationMode());
    return;
  }

  updateOperationMode(msg.getTimeStampedOperationMode());
}

}  // namespace prbt_hardware_support
