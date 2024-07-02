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

#include <sstream>

#include <prbt_hardware_support/modbus_api_definitions.h>
#include <prbt_hardware_support/modbus_adapter_run_permitted.h>
#include <prbt_hardware_support/modbus_msg_run_permitted_wrapper.h>

namespace prbt_hardware_support
{
using namespace modbus_api::v3;

ModbusAdapterRunPermitted::ModbusAdapterRunPermitted(UpdateRunPermittedFunc&& update_run_permitted_func,
                                                     const ModbusApiSpec& api_spec)
  : api_spec_(api_spec), update_run_permitted_(std::move(update_run_permitted_func))
{
}

void ModbusAdapterRunPermitted::modbusMsgCallback(const ModbusMsgInStampedConstPtr& msg_raw)
{
  ModbusMsgRunPermittedWrapper msg(msg_raw, api_spec_);

  if (msg.isDisconnect())
  {
    ROS_ERROR("A disconnect from the modbus server happend.");
    update_run_permitted_(false);
    return;
  }

  try
  {
    msg.checkStructuralIntegrity();
  }
  catch (const ModbusMsgWrapperException& e)
  {
    ROS_ERROR_STREAM(e.what());
    update_run_permitted_(false);
    return;
  }

  if (msg.getVersion() != MODBUS_API_VERSION_REQUIRED)
  {
    std::ostringstream os;
    os << "Received Modbus message of unsupported API Version: " << msg.getVersion()
       << ", required Version: " << MODBUS_API_VERSION_REQUIRED;
    ROS_ERROR_STREAM(os.str());
    update_run_permitted_(false);
    return;
  }

  update_run_permitted_(msg.getRunPermitted());
}

}  // namespace prbt_hardware_support
