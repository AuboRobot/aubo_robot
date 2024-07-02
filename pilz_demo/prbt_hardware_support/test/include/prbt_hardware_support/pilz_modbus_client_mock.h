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

#ifndef PRBT_HARDWARE_SUPPORT_PILZ_MODBUS_SERVER_MOCK_H
#define PRBT_HARDWARE_SUPPORT_PILZ_MODBUS_SERVER_MOCK_H

#include <gmock/gmock.h>

#include <prbt_hardware_support/modbus_client.h>

/**
 * @brief Mock used in the unittest of the PilzModbusClient
 */

class PilzModbusClientMock : public prbt_hardware_support::ModbusClient
{
public:
  MOCK_METHOD2(init, bool(const char* ip, unsigned int port));
  MOCK_METHOD0(connect, void());
  MOCK_METHOD1(setResponseTimeoutInMs, void(unsigned long timeout_ms));
  MOCK_METHOD2(readHoldingRegister, std::vector<uint16_t>(int addr, int nb));
  MOCK_METHOD4(writeReadHoldingRegister,
               std::vector<uint16_t>(const int write_addr, const std::vector<uint16_t>& write_reg, const int read_addr,
                                     const int read_nb));
  MOCK_METHOD0(getResponseTimeoutInMs, unsigned long());
};

#endif  // PRBT_HARDWARE_SUPPORT_PILZ_MODBUS_SERVER_MOCK_H
