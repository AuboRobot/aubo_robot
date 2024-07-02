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

#ifndef PILZ_MODBUS_EXCEPTIONS_H
#define PILZ_MODBUS_EXCEPTIONS_H

#include <stdexcept>

/**
 * @brief Expection thrown by prbt_hardware_support::LibModbusClient::readHoldingRegister
 * if a disconnect from the server occured.
 *
 */
class ModbusExceptionDisconnect : public std::runtime_error
{
public:
  ModbusExceptionDisconnect(const std::string& what_arg) : std::runtime_error(what_arg){};
};

#endif  // PILZ_MODBUS_EXCEPTIONS_H
