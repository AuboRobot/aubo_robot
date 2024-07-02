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

#ifndef WRITE_MODBUS_REGISTER_CALL_H
#define WRITE_MODBUS_REGISTER_CALL_H

#include <ros/ros.h>

#include <prbt_hardware_support/WriteModbusRegister.h>
#include <prbt_hardware_support/register_container.h>

namespace prbt_hardware_support
{
template <class T>
static bool writeModbusRegisterCall(T& modbus_service, const uint16_t& start_idx, const RegCont& values)
{
  WriteModbusRegister srv;
  srv.request.holding_register_block.start_idx = start_idx;
  srv.request.holding_register_block.values = values;

  ROS_DEBUG_STREAM("Calling service: " << modbus_service.getService());
  bool call_success = modbus_service.call(srv);
  if (!call_success)
  {
    ROS_ERROR_STREAM("Service call " << modbus_service.getService() << " failed.");
    return false;
  }

  if (!srv.response.success)
  {
    ROS_ERROR_STREAM("Writing of modbus register failed.");
    return false;
  }

  return true;
}

}  // namespace prbt_hardware_support

#endif  // WRITE_MODBUS_REGISTER_CALL_H
