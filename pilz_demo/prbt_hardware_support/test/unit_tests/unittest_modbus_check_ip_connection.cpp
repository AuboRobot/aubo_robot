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

#include <gtest/gtest.h>

#include <numeric>
#include <modbus/modbus.h>

#include <prbt_hardware_support/libmodbus_client.h>
#include <prbt_hardware_support/modbus_check_ip_connection.h>
#include <prbt_hardware_support/pilz_modbus_server_mock.h>
#include <prbt_hardware_support/pilz_modbus_exceptions.h>
#include <prbt_hardware_support/client_tests_common.h>

namespace modbus_socket_connection_check_test
{
using namespace prbt_hardware_support;

// Each testcase should have its own port in order to avoid conflicts between them
constexpr unsigned int START_PORT{ 20600 };
constexpr unsigned int END_PORT{ 20700 };
constexpr unsigned int DEFAULT_REGISTER_SIZE{ 514 };
constexpr unsigned int DEFAULT_WRITE_IDX{ 512 };
constexpr unsigned int WRONG_PORT = 4711;

class ModbusConnectionCheckTestsuite : public testing::Test
{
public:
  static void SetUpTestCase();  // NOLINT
  void SetUp() override;
  void TearDown() override;

protected:
  void shutdownModbusServer(PilzModbusServerMock* server, LibModbusClient& client);

protected:
  static unsigned int testPort();

protected:
  LibModbusClient client_;
  std::unique_ptr<PilzModbusServerMock> server_{ new PilzModbusServerMock(DEFAULT_REGISTER_SIZE) };

protected:
  static unsigned int ACTIVE_PORT_IDX_;
  static std::vector<unsigned int> PORTS_FOR_TEST_;
};

unsigned int ModbusConnectionCheckTestsuite::ACTIVE_PORT_IDX_ = 0;
std::vector<unsigned int> ModbusConnectionCheckTestsuite::PORTS_FOR_TEST_ =
    std::vector<unsigned int>(END_PORT - START_PORT);

void ModbusConnectionCheckTestsuite::SetUp()
{
  EXPECT_FALSE(checkIPConnection(LOCALHOST, testPort())) << "No Server";
  server_->startAsync(LOCALHOST, testPort());

  // Needed to make sure server is actually present. (Not optimal,
  // needs adaption inside the server mock)
  EXPECT_TRUE(client_.init(LOCALHOST, testPort())) << "Server not present";
}

void ModbusConnectionCheckTestsuite::TearDown()
{
  // Use next port on next test
  ACTIVE_PORT_IDX_++;
  shutdownModbusServer(server_.get(), client_);
  client_.close();
}

unsigned int ModbusConnectionCheckTestsuite::testPort()
{
  return PORTS_FOR_TEST_.at(ACTIVE_PORT_IDX_ % PORTS_FOR_TEST_.size());
}

void ModbusConnectionCheckTestsuite::SetUpTestCase()  // NOLINT
{
  std::iota(PORTS_FOR_TEST_.begin(), PORTS_FOR_TEST_.end(), START_PORT);
}

void ModbusConnectionCheckTestsuite::shutdownModbusServer(PilzModbusServerMock* server, LibModbusClient& client)
{
  server->setTerminateFlag();
  RegCont reg_to_write_by_client{ 1 };
  try
  {
    client.writeReadHoldingRegister(DEFAULT_REGISTER_SIZE, reg_to_write_by_client, DEFAULT_WRITE_IDX,
                                    DEFAULT_REGISTER_SIZE - DEFAULT_WRITE_IDX);
  }
  catch (const ModbusExceptionDisconnect& /* ex */)
  {
    // Tolerated exception
  }

  server->terminate();
}

TEST_F(ModbusConnectionCheckTestsuite, testReactionToCorrectConnection)
{
  EXPECT_TRUE(checkIPConnection(LOCALHOST, testPort()))
      << "Unexpected reaction to proper connection (ip address and port number correct).";
}

TEST_F(ModbusConnectionCheckTestsuite, testReactionToIncorrectPort)
{
  EXPECT_FALSE(checkIPConnection(LOCALHOST, WRONG_PORT)) << "Unexpected reaction to incorrect port number.";
}

TEST_F(ModbusConnectionCheckTestsuite, testReactionToIncorrectIP)
{
  EXPECT_FALSE(checkIPConnection("192.192.192.192", testPort())) << "Unexpected reaction to incorrect ip address.";
}

TEST_F(ModbusConnectionCheckTestsuite, testReactionToIncorrectPortAndIncorrectIP)
{
  ASSERT_NE(WRONG_PORT, testPort());
  EXPECT_FALSE(checkIPConnection("192.192.192.192", WRONG_PORT))
      << "Unexpected reaction to incorrect port number and ip address.";
}

}  // namespace modbus_socket_connection_check_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
