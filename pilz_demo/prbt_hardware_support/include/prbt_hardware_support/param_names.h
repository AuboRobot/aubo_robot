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

#ifndef PRBT_HARDWARE_SUPPORT_PARAM_NAMES_H
#define PRBT_HARDWARE_SUPPORT_PARAM_NAMES_H

#include <string>

namespace prbt_hardware_support
{
static const std::string PARAM_MODBUS_SERVER_IP_STR{ "modbus_server_ip" };
static const std::string PARAM_MODBUS_SERVER_PORT_STR{ "modbus_server_port" };
static const std::string PARAM_MODBUS_RESPONSE_TIMEOUT_STR{ "modbus_response_timeout" };
static const std::string PARAM_MODBUS_READ_TOPIC_NAME_STR{ "modbus_read_topic_name" };
static const std::string PARAM_MODBUS_WRITE_SERVICE_NAME_STR{ "modbus_write_service_name" };
static const std::string PARAM_INDEX_OF_FIRST_REGISTER_TO_READ_STR{ "index_of_first_register_to_read" };
static const std::string PARAM_NUM_REGISTERS_TO_READ_STR{ "num_registers_to_read" };
static const std::string PARAM_MODBUS_CONNECTION_RETRIES{ "modbus_connection_retries" };
static const std::string PARAM_MODBUS_CONNECTION_RETRY_TIMEOUT{ "modbus_connection_retry_timeout" };

}  // namespace prbt_hardware_support

#endif  // PARAM_NAMES_H
