/*
 * Copyright (c) 2020 Pilz GmbH & Co. KG
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

#include <prbt_hardware_support/modbus_msg_run_permitted_wrapper.h>

#include <prbt_hardware_support/ModbusMsgInStamped.h>
#include <prbt_hardware_support/modbus_api_spec.h>

using namespace prbt_hardware_support;

ModbusMsgRunPermittedWrapper::ModbusMsgRunPermittedWrapper(const ModbusMsgInStampedConstPtr& modbus_msg_raw,
                                                           const ModbusApiSpec& api_spec)
  : ModbusMsgWrapper(modbus_msg_raw, api_spec)
{
}

void ModbusMsgRunPermittedWrapper::checkStructuralIntegrity() const
{
  ModbusMsgWrapper::checkStructuralIntegrity();

  if (!hasRunPermitted())
  {
    throw ModbusMsgRunPermittedStatusMissing("Received message does not contain a RUN_PERMITTED status.");
  }
}
