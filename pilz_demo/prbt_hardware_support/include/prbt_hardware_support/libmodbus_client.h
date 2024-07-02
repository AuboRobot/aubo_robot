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

#ifndef LIBMODBUS_CLIENT_H
#define LIBMODBUS_CLIENT_H

#include <modbus/modbus.h>

#include <vector>

#include <prbt_hardware_support/modbus_client.h>

namespace prbt_hardware_support
{
/**
 * @brief Wrapper around libmodbus, see https://libmodbus.org/
 *
 * Used by PilzModbusClient to access a modbus server
 */
class LibModbusClient : public ModbusClient
{
public:
  //! @brief See base class.
  virtual ~LibModbusClient() override;

  //! @brief See base class.
  bool init(const char* ip, unsigned int port) override;

  //! @brief See base class.
  void setResponseTimeoutInMs(unsigned long timeout_ms) override;

  //! @brief See base class.
  unsigned long getResponseTimeoutInMs() override;

  //! @brief See base class.
  RegCont readHoldingRegister(int addr, int nb) override;

  //! @brief See base class.
  RegCont writeReadHoldingRegister(const int write_addr, const RegCont& write_reg, const int read_addr,
                                   const int read_nb) override;

  /**
   * @brief Close connection with server
   */
  void close();

private:
  modbus_t* modbus_connection_{ nullptr };
};

}  // namespace prbt_hardware_support

#endif  // LIBMODBUS_CLIENT_H
