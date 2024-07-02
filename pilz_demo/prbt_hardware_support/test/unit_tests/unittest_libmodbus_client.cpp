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

#include <gtest/gtest.h>

#include <thread>
#include <chrono>
#include <vector>
#include <numeric>

#include <modbus/modbus.h>

#include <prbt_hardware_support/libmodbus_client.h>
#include <prbt_hardware_support/pilz_modbus_server_mock.h>
#include <prbt_hardware_support/pilz_modbus_exceptions.h>

#include <prbt_hardware_support/client_tests_common.h>

namespace pilz_modbus_client_test
{
using namespace prbt_hardware_support;

// Each testcase should have its own port in order to avoid conflicts between them
constexpr unsigned int START_PORT{ 20500 };
constexpr unsigned int END_PORT{ 20600 };
static unsigned int ACTIVE_PORT_IDX{ 0 };
static std::vector<unsigned int> PORTS_FOR_TEST(END_PORT - START_PORT);

constexpr unsigned int DEFAULT_REGISTER_SIZE{ 514 };
constexpr unsigned int DEFAULT_WRITE_IDX{ 512 };
constexpr unsigned int DEFAULT_READ_IDX{ 77 };

class LibModbusClientTest : public testing::Test
{
public:
  static void SetUpTestCase();  // NOLINT
  void TearDown() override;
  unsigned int testPort();

protected:
  void shutdownModbusServer(PilzModbusServerMock* server, LibModbusClient& client);
};

void LibModbusClientTest::TearDown()
{
  // Use next port on next test
  ACTIVE_PORT_IDX++;
}

unsigned int LibModbusClientTest::testPort()
{
  return PORTS_FOR_TEST.at(ACTIVE_PORT_IDX % PORTS_FOR_TEST.size());
}

void LibModbusClientTest::SetUpTestCase()  // NOLINT
{
  std::iota(PORTS_FOR_TEST.begin(), PORTS_FOR_TEST.end(), START_PORT);
}

void LibModbusClientTest::shutdownModbusServer(PilzModbusServerMock* server, LibModbusClient& client)
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

/**
 * @brief Test increases function coverage by ensuring that all Dtor variants
 * are called.
 */
TEST_F(LibModbusClientTest, testModbusClientDtor)
{
  ModbusClient* client = new LibModbusClient();
  delete client;
}

/**
 * @brief Test construction via new
 *
 * This test is used to at least once call the default destructor used for destructing objects allocated on the heap.
 * (If this test is missing function coverage will drop)
 */
TEST_F(LibModbusClientTest, testNewInit)
{
  std::shared_ptr<LibModbusClient> client(new LibModbusClient());
}

/**
 * @brief Test successfull init
 */
TEST_F(LibModbusClientTest, testInitialization)
{
  LibModbusClient client;
  std::shared_ptr<PilzModbusServerMock> server(new PilzModbusServerMock(DEFAULT_REGISTER_SIZE));
  server->startAsync(LOCALHOST, testPort());

  EXPECT_TRUE(client.init(LOCALHOST, testPort()));

  shutdownModbusServer(server.get(), client);
  client.close();
}

/**
 * @brief Test unsuccessfull init if no server is present
 */
TEST_F(LibModbusClientTest, testFailingInitIfNoServer)
{
  LibModbusClient client;
  EXPECT_FALSE(client.init(LOCALHOST, testPort()));
}

/**
 * @brief Tests that holding registers set on the server are correctly read by the client
 */
TEST_F(LibModbusClientTest, testReadRegisters)
{
  LibModbusClient client;
  std::shared_ptr<PilzModbusServerMock> server(new PilzModbusServerMock(DEFAULT_REGISTER_SIZE));
  server->startAsync(LOCALHOST, testPort());

  server->setHoldingRegister(RegCont{ 1, 2 }, DEFAULT_WRITE_IDX);

  EXPECT_TRUE(client.init(LOCALHOST, testPort()));

  RegCont res = client.readHoldingRegister(DEFAULT_WRITE_IDX, 2);
  RegCont res_expected{ 1, 2 };
  EXPECT_EQ(res_expected, res);

  shutdownModbusServer(server.get(), client);
  client.close();
}

/**
 * @brief Tests that holding registers are correctly written by client.
 */
TEST_F(LibModbusClientTest, testWritingRegisters)
{
  LibModbusClient client;
  std::shared_ptr<PilzModbusServerMock> server(new PilzModbusServerMock(DEFAULT_REGISTER_SIZE));
  server->startAsync(LOCALHOST, testPort());

  const RegCont write_reg{ 1, 2 };
  server->setHoldingRegister(write_reg, DEFAULT_WRITE_IDX);

  EXPECT_TRUE(client.init(LOCALHOST, testPort()));

  RegCont reg_to_write_by_client{ 8, 3, 7 };

  client.writeReadHoldingRegister(DEFAULT_READ_IDX, reg_to_write_by_client, DEFAULT_WRITE_IDX,
                                  static_cast<int>(write_reg.size()));

  RegCont res = client.readHoldingRegister(DEFAULT_WRITE_IDX, static_cast<int>(write_reg.size()));
  RegCont res_expected{ 1, 2 };
  EXPECT_EQ(res_expected, res);

  RegCont actual_hold_reg{ server->readHoldingRegister(DEFAULT_READ_IDX, reg_to_write_by_client.size()) };
  for (RegCont::size_type i = 0; i < reg_to_write_by_client.size(); ++i)
  {
    EXPECT_EQ(reg_to_write_by_client.at(i), actual_hold_reg.at(i));
  }

  shutdownModbusServer(server.get(), client);
  client.close();
}

/**
 * @brief Tests that exception is thrown if the user tries
 * to call the write function with a negative number of write of registers
 * to read.
 */
TEST_F(LibModbusClientTest, testNegativeNumberOfRegistersToRead)
{
  LibModbusClient client;
  std::shared_ptr<PilzModbusServerMock> server(new PilzModbusServerMock(DEFAULT_REGISTER_SIZE));
  server->startAsync(LOCALHOST, testPort());

  server->setHoldingRegister(RegCont{ 1, 2 }, DEFAULT_WRITE_IDX);

  EXPECT_TRUE(client.init(LOCALHOST, testPort()));

  const int negative_read_nb{ -2 };
  RegCont reg_to_write_by_client{ 8, 3, 7 };
  EXPECT_THROW(
      client.writeReadHoldingRegister(DEFAULT_READ_IDX, reg_to_write_by_client, DEFAULT_WRITE_IDX, negative_read_nb),
      std::invalid_argument);

  shutdownModbusServer(server.get(), client);
  client.close();
}

/**
 * @brief Tests that exception is thrown if the user tries
 * to call the write function with a register containing too many elements.
 */
TEST_F(LibModbusClientTest, testOutOfRangeRegisterSize)
{
  LibModbusClient client;
  std::shared_ptr<PilzModbusServerMock> server(new PilzModbusServerMock(DEFAULT_REGISTER_SIZE));
  server->startAsync(LOCALHOST, testPort());

  const RegCont write_reg{ 1, 2 };
  server->setHoldingRegister(write_reg, DEFAULT_WRITE_IDX);

  EXPECT_TRUE(client.init(LOCALHOST, testPort()));

  RegCont reg_to_write_by_client(static_cast<unsigned int>(std::numeric_limits<int>::max()) + 1u, 0);
  EXPECT_THROW(client.writeReadHoldingRegister(DEFAULT_READ_IDX, reg_to_write_by_client, DEFAULT_WRITE_IDX,
                                               static_cast<int>(write_reg.size())),
               std::invalid_argument);

  shutdownModbusServer(server.get(), client);
  client.close();
}

/**
 * @brief Tests that exception is thrown if modbus connections fails before
 * call to write & read function.
 */
TEST_F(LibModbusClientTest, testDisconnectBeforeReadWriteOp)
{
  LibModbusClient client;

  EXPECT_THROW(client.writeReadHoldingRegister(DEFAULT_READ_IDX, RegCont{ 8, 3, 7 }, DEFAULT_WRITE_IDX, 0),
               ModbusExceptionDisconnect);

  client.close();
}

/**
 * @brief Tests that exception is thrown if modbus connections fails during
 * write & read operation.
 */
TEST_F(LibModbusClientTest, testDisconnectDuringReadWriteOp)
{
  LibModbusClient client;
  std::shared_ptr<PilzModbusServerMock> server(new PilzModbusServerMock(DEFAULT_REGISTER_SIZE));
  server->startAsync(LOCALHOST, testPort());

  const RegCont write_reg{ 1, 2 };
  server->setHoldingRegister(write_reg, DEFAULT_WRITE_IDX);

  EXPECT_TRUE(client.init(LOCALHOST, testPort()));
  shutdownModbusServer(server.get(), client);

  RegCont reg_to_write_by_client{ 8, 3, 7 };
  EXPECT_THROW(client.writeReadHoldingRegister(DEFAULT_READ_IDX, reg_to_write_by_client, DEFAULT_WRITE_IDX,
                                               static_cast<int>(write_reg.size())),
               ModbusExceptionDisconnect);

  client.close();
}

/**
 * @brief Tests that after a failed init (e.g. no available modbus server) reading throws a exception
 */
TEST_F(LibModbusClientTest, testReadRegistersNoInit)
{
  LibModbusClient client;
  EXPECT_FALSE(client.init(LOCALHOST, testPort()));

  EXPECT_THROW(client.readHoldingRegister(DEFAULT_WRITE_IDX, 2), ModbusExceptionDisconnect);
}

/**
 * @brief Tests that after a failed init (e.g. after disconnect from server) reading throws a exception
 */
TEST_F(LibModbusClientTest, testReadRegistersTerminatedServer)
{
  LibModbusClient client;
  std::shared_ptr<PilzModbusServerMock> server(new PilzModbusServerMock(DEFAULT_REGISTER_SIZE));
  server->startAsync(LOCALHOST, testPort());
  server->setHoldingRegister(RegCont{ 1, 2 }, DEFAULT_WRITE_IDX);

  EXPECT_TRUE(client.init(LOCALHOST, testPort()));
  shutdownModbusServer(server.get(), client);

  EXPECT_THROW(client.readHoldingRegister(DEFAULT_WRITE_IDX, 2), ModbusExceptionDisconnect);
  client.close();
}

/**
 * @brief Tests that setting reponse timeout on the client will return the same timeout on getResponseTimeoutInMs
 */
TEST_F(LibModbusClientTest, setResponseTimeout)
{
  LibModbusClient client;
  std::shared_ptr<PilzModbusServerMock> server(new PilzModbusServerMock(514));
  server->startAsync(LOCALHOST, testPort());

  unsigned long timeout_ms = 3;

  EXPECT_TRUE(client.init(LOCALHOST, testPort()));
  client.setResponseTimeoutInMs(timeout_ms);
  EXPECT_EQ(timeout_ms, client.getResponseTimeoutInMs());

  shutdownModbusServer(server.get(), client);
  client.close();
}

}  // namespace pilz_modbus_client_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
