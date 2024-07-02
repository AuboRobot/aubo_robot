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

#ifndef PRBT_HARDWARE_SUPPORT_COMMON_H
#define PRBT_HARDWARE_SUPPORT_COMMON_H

#include <string>

namespace prbt_hardware_support
{
// Topic names
static const std::string TOPIC_MODBUS_READ = "/pilz_modbus_client_node/modbus_read";
static const std::string SERVICE_MODBUS_WRITE = "/pilz_modbus_client_node/modbus_write";

}  // namespace prbt_hardware_support
#endif  // PRBT_HARDWARE_SUPPORT_COMMON_H
