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

#ifndef MODBUS_CLIENT_H
#define MODBUS_CLIENT_H

#include <vector>
#include <cstdint>

#include "register_container.h"

namespace prbt_hardware_support
{
class ModbusClient
{
public:
  // LCOV_EXCL_START
  virtual ~ModbusClient() = default;
  // LCOV_EXCL_STOP

public:
  /**
   * @brief Initialize the modbus client by connecting to the server
   *
   * @param ip of the server
   * @param port to use
   * @return true if the connection to the server succeeded
   * @return false if the connection to the server failed
   */
  virtual bool init(const char* ip, unsigned int port) = 0;

  /**
   * @brief Set the response timeout
   *
   * Interval to wait for a response from the server if within this timespan no answer is received readHoldingRegisters
   * will throw a ModbusExceptionDisconnect
   *
   * @param timeout_ms in ms
   */
  virtual void setResponseTimeoutInMs(unsigned long timeout_ms) = 0;

  /**
   * @brief Get the response timeout
   *
   * @return response timeout
   */
  virtual unsigned long getResponseTimeoutInMs() = 0;

  /**
   * @brief Read the holding registers
   *
   * @param addr starting address to read from
   * @param nb number of registers to read
   * @throw ModbusExceptionDisconnect if a disconnect from the server happens
   * @returns the registers content.
   */
  virtual RegCont readHoldingRegister(int addr, int nb) = 0;

  virtual RegCont writeReadHoldingRegister(const int write_addr, const RegCont& write_reg, const int read_addr,
                                           const int read_nb) = 0;
};

}  // namespace prbt_hardware_support

#endif  // MODBUS_CLIENT_H
