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

#include <prbt_hardware_support/canopen_braketest_adapter_exception.h>

namespace prbt_hardware_support
{
CANOpenBrakeTestAdapterException::CANOpenBrakeTestAdapterException(const std::string& what_arg,
                                                                   const int8_t error_value)
  : std::runtime_error(what_arg), error_value_(error_value)
{
}

}  // namespace prbt_hardware_support
