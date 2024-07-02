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

#ifndef MODBUS_SOCKET_CONNECTION_CHECK_H
#define MODBUS_SOCKET_CONNECTION_CHECK_H

namespace prbt_hardware_support
{
/**
 * @brief Test the ip connection by connecting to the modbus server
 *
 * @param ip of the modbus server
 * @param port of the modbus server
 * @return true if the connection to the server succeeded
 * @return false if the connection to the server failed
 */
bool checkIPConnection(const char* ip, const unsigned int& port);

}  // namespace prbt_hardware_support

#endif  // MODBUS_SOCKET_CONNECTION_CHECK_H
