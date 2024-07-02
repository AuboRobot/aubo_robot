/*
 * Copyright (c) 2018 Pilz GmbH & Co. KG
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

#ifndef MODBUS_API_DEFINITIONS_H
#define MODBUS_API_DEFINITIONS_H

#include <string>

namespace prbt_hardware_support
{
namespace modbus_api
{
namespace v3
{
static constexpr uint16_t MODBUS_API_VERSION_REQUIRED{ 3 };

static constexpr uint16_t MODBUS_BRAKE_TEST_PASSED{ 2 };
static constexpr uint16_t MODBUS_BRAKE_TEST_NOT_PASSED{ 1 };
static constexpr uint16_t MODBUS_BRAKE_TEST_PERFORMED{ 2 };
static constexpr uint16_t MODBUS_BRAKE_TEST_NOT_PERFORMED{ 1 };

static constexpr uint16_t MODBUS_OPERATION_MODE_NONE{ 0 };
static constexpr uint16_t MODBUS_OPERATION_MODE_T1{ 1 };
static constexpr uint16_t MODBUS_OPERATION_MODE_T2{ 2 };
static constexpr uint16_t MODBUS_OPERATION_MODE_AUTO{ 3 };

static constexpr uint16_t MODBUS_RUN_PERMITTED_TRUE{ 1 };
static constexpr uint16_t MODBUS_RUN_PERMITTED_FALSE{ 0 };
}  // namespace v3
}  // namespace modbus_api

}  // namespace prbt_hardware_support
#endif  // MODBUS_API_DEFINITIONS_H
