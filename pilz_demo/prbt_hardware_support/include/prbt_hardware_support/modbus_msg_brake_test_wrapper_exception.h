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

#ifndef MODBUS_MSG_BRAKE_TEST_WRAPPER_EXCEPTION_H
#define MODBUS_MSG_BRAKE_TEST_WRAPPER_EXCEPTION_H

#include <string>
#include <prbt_hardware_support/modbus_msg_wrapper_exception.h>

namespace prbt_hardware_support
{
/**
 * @brief Expection thrown upon construction of ModbusMsgBrakeTestWrapper
 * of the message does not contain the required information.
 */
class ModbusMsgBrakeTestWrapperException : public ModbusMsgWrapperException
{
public:
  ModbusMsgBrakeTestWrapperException(const std::string& what_arg) : ModbusMsgWrapperException(what_arg)
  {
  }
};

}  // namespace prbt_hardware_support

#endif  // MODBUS_MSG_BRAKE_TEST_WRAPPER_EXCEPTION_H
