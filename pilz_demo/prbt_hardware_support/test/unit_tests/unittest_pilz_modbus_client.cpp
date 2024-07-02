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

#include <chrono>
#include <memory>
#include <mutex>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

#include <ros/ros.h>
#include <modbus/modbus.h>

#include <prbt_hardware_support/modbus_topic_definitions.h>
#include <prbt_hardware_support/ModbusMsgInStamped.h>
#include <prbt_hardware_support/pilz_modbus_client_mock.h>
#include <prbt_hardware_support/pilz_modbus_client.h>
#include <prbt_hardware_support/pilz_modbus_exceptions.h>
#include <prbt_hardware_support/pilz_modbus_client_exception.h>

#include <prbt_hardware_support/client_tests_common.h>

#include <pilz_testutils/async_test.h>

namespace pilz_modbus_client_test
{
using ::testing::_;
using ::testing::AnyNumber;
using ::testing::AtLeast;
using ::testing::InSequence;
using ::testing::Invoke;
using ::testing::InvokeWithoutArgs;
using ::testing::Return;
using ::testing::Throw;
using namespace prbt_hardware_support;

static constexpr unsigned int DEFAULT_MODBUS_PORT_TEST{ 502 };
static constexpr unsigned int REGISTER_FIRST_IDX_TEST{ 512 };
static constexpr unsigned int REGISTER_SIZE_TEST{ 2 };
static constexpr unsigned int RESPONSE_TIMEOUT{ 10 };

static constexpr double WAIT_FOR_START_TIMEOUT_S{ 3.0 };
static constexpr double WAIT_SLEEPTIME_S{ 0.1 };
static constexpr double WAIT_FOR_STOP_TIMEOUT_S{ 3.0 };
static constexpr double WAIT_FOR_SERVICE_TIMEOUT_S{ 3.0 };

/**
 * @brief Helper class to simplify the threading of the PilzModbusClient.
 */
class PilzModbusClientExecutor
{
public:
  PilzModbusClientExecutor(PilzModbusClient* client);

public:
  void start();
  void stop();

private:
  PilzModbusClient* client_;
  std::thread client_thread_;
};

PilzModbusClientExecutor::PilzModbusClientExecutor(PilzModbusClient* client) : client_(client)
{
}

void PilzModbusClientExecutor::start()
{
  client_thread_ = std::thread(&PilzModbusClient::run, client_);

  ros::Time start_waiting = ros::Time::now();
  while (!client_->isRunning())
  {
    ros::Duration(WAIT_SLEEPTIME_S).sleep();
    if (ros::Time::now() > start_waiting + ros::Duration(WAIT_FOR_START_TIMEOUT_S))
    {
      break;
    }
  }
}

void PilzModbusClientExecutor::stop()
{
  client_->terminate();
  ros::Time start_waiting = ros::Time::now();
  while (client_->isRunning())
  {
    ros::Duration(WAIT_SLEEPTIME_S).sleep();
    if (ros::Time::now() > start_waiting + ros::Duration(WAIT_FOR_STOP_TIMEOUT_S))
    {
      break;
    }
  }
  client_thread_.join();
}

/**
 * @brief Test if PilzModbusClient correctly publishes ROS-Modbus messages.
 *
 * It subscribes to the topic of the test object with a mocked callback function.
 * The actual client needed by the test object is replaced by a mock object.
 */
class PilzModbusClientTests : public testing::Test, public testing::AsyncTest
{
public:
  void SetUp() override;

protected:
  ros::Subscriber subscriber_;
  ros::AsyncSpinner spinner_{ 2 };
  ros::NodeHandle nh_;

  MOCK_METHOD1(modbus_read_cb, void(const ModbusMsgInStampedConstPtr& msg));
};

void PilzModbusClientTests::SetUp()
{
  subscriber_ = nh_.subscribe<ModbusMsgInStamped>(prbt_hardware_support::TOPIC_MODBUS_READ, 1,
                                                  &PilzModbusClientTests::modbus_read_cb, this);
  spinner_.start();
}

MATCHER_P(IsSuccessfullRead, vec, "")
{
  return arg->holding_registers.data == vec;
}
MATCHER(IsDisconnect, "")
{
  return arg->disconnect.data;
}

/**
 * @brief Test that PilzModbusClient initializes its client properly
 */
TEST_F(PilzModbusClientTests, testInitialization)
{
  std::unique_ptr<PilzModbusClientMock> mock(new PilzModbusClientMock());

  EXPECT_CALL(*mock, init(_, _)).Times(1).WillOnce(Return(true));

  std::vector<unsigned short> registers(REGISTER_SIZE_TEST);
  std::iota(registers.begin(), registers.end(), REGISTER_FIRST_IDX_TEST);

  PilzModbusClient client(nh_, registers, std::move(mock), RESPONSE_TIMEOUT, prbt_hardware_support::TOPIC_MODBUS_READ,
                          prbt_hardware_support::SERVICE_MODBUS_WRITE);

  EXPECT_TRUE(client.init(LOCALHOST, DEFAULT_MODBUS_PORT_TEST));
}

/**
 * @brief Test that initialization is repeated if it does not work at first try
 */
TEST_F(PilzModbusClientTests, testInitializationWithRetry)
{
  std::unique_ptr<PilzModbusClientMock> mock(new PilzModbusClientMock());

  EXPECT_CALL(*mock, init(_, _)).Times(2).WillOnce(Return(false)).WillOnce(Return(true));

  std::vector<unsigned short> registers(REGISTER_SIZE_TEST);
  std::iota(registers.begin(), registers.end(), REGISTER_FIRST_IDX_TEST);

  PilzModbusClient client(nh_, registers, std::move(mock), RESPONSE_TIMEOUT, prbt_hardware_support::TOPIC_MODBUS_READ,
                          prbt_hardware_support::SERVICE_MODBUS_WRITE);

  EXPECT_TRUE(client.init(LOCALHOST, DEFAULT_MODBUS_PORT_TEST, REGISTER_SIZE_TEST, ros::Duration(0.1)));
}

/**
 * @brief Test that double initialization fails on seconds try
 */
TEST_F(PilzModbusClientTests, doubleInitialization)
{
  std::unique_ptr<PilzModbusClientMock> mock(new PilzModbusClientMock());

  EXPECT_CALL(*mock, init(_, _)).Times(1).WillOnce(Return(true));

  std::vector<unsigned short> registers(REGISTER_SIZE_TEST);
  std::iota(registers.begin(), registers.end(), REGISTER_FIRST_IDX_TEST);

  PilzModbusClient client(nh_, registers, std::move(mock), RESPONSE_TIMEOUT, prbt_hardware_support::TOPIC_MODBUS_READ,
                          prbt_hardware_support::SERVICE_MODBUS_WRITE);

  EXPECT_TRUE(client.init(LOCALHOST, DEFAULT_MODBUS_PORT_TEST));
  EXPECT_FALSE(client.init(LOCALHOST, DEFAULT_MODBUS_PORT_TEST));
}

/**
 * @brief Test the initialization of the modbus read client if init fails.
 */
TEST_F(PilzModbusClientTests, failingInitialization)
{
  std::unique_ptr<PilzModbusClientMock> mock(new PilzModbusClientMock());

  EXPECT_CALL(*mock, init(_, _)).Times(1).WillOnce(Return(false));

  std::vector<unsigned short> registers(REGISTER_SIZE_TEST);
  std::iota(registers.begin(), registers.end(), REGISTER_FIRST_IDX_TEST);

  PilzModbusClient client(nh_, registers, std::move(mock), RESPONSE_TIMEOUT, prbt_hardware_support::TOPIC_MODBUS_READ,
                          prbt_hardware_support::SERVICE_MODBUS_WRITE);

  EXPECT_FALSE(client.init(LOCALHOST, DEFAULT_MODBUS_PORT_TEST));
}

/**
 * @brief Test that repeated failed initialization yields in a failed init for the caller
 */
TEST_F(PilzModbusClientTests, failingInitializationWithRetry)
{
  std::unique_ptr<PilzModbusClientMock> mock(new PilzModbusClientMock());

  EXPECT_CALL(*mock, init(_, _)).Times(2).WillOnce(Return(false)).WillOnce(Return(false));

  std::vector<unsigned short> registers(REGISTER_SIZE_TEST);
  std::iota(registers.begin(), registers.end(), REGISTER_FIRST_IDX_TEST);

  PilzModbusClient client(nh_, registers, std::move(mock), RESPONSE_TIMEOUT, prbt_hardware_support::TOPIC_MODBUS_READ,
                          prbt_hardware_support::SERVICE_MODBUS_WRITE);

  EXPECT_FALSE(client.init(LOCALHOST, DEFAULT_MODBUS_PORT_TEST, REGISTER_SIZE_TEST, ros::Duration(0.1)));
}

/**
 * @brief Test that PilzModbusClient properly reads from the registers and reacts to disconnects
 */
TEST_F(PilzModbusClientTests, properReadingAndDisconnect)
{
  std::unique_ptr<PilzModbusClientMock> mock(new PilzModbusClientMock());

  {
    InSequence s;
    EXPECT_CALL(*mock, init(_, _)).Times(1).WillOnce(Return(true));
    EXPECT_CALL(*mock, readHoldingRegister(_, _))
        .WillOnce(Return(std::vector<uint16_t>{ 1, 2 }))
        .WillOnce(Return(std::vector<uint16_t>{ 1, 2 }))
        .WillOnce(Return(std::vector<uint16_t>{ 3, 4 }))
        .WillOnce(Throw(ModbusExceptionDisconnect("disconnect_message")));
  }
  {
    InSequence s;
    EXPECT_CALL(*this, modbus_read_cb(IsSuccessfullRead(std::vector<uint16_t>{ 1, 2 }))).Times(2);
    EXPECT_CALL(*this, modbus_read_cb(IsSuccessfullRead(std::vector<uint16_t>{ 3, 4 }))).Times(1);
    EXPECT_CALL(*this, modbus_read_cb(IsDisconnect())).Times(1).WillOnce(ACTION_OPEN_BARRIER_VOID("disconnected"));
  }

  std::vector<unsigned short> registers(REGISTER_SIZE_TEST);
  std::iota(registers.begin(), registers.end(), REGISTER_FIRST_IDX_TEST);

  PilzModbusClient client(nh_, registers, std::move(mock), RESPONSE_TIMEOUT, prbt_hardware_support::TOPIC_MODBUS_READ,
                          prbt_hardware_support::SERVICE_MODBUS_WRITE);

  EXPECT_TRUE(client.init(LOCALHOST, DEFAULT_MODBUS_PORT_TEST));
  EXPECT_NO_THROW(client.run());
  BARRIER("disconnected");
}

/**
 * @brief Try to run the modbus read client without a foregoing call to init()
 */
TEST_F(PilzModbusClientTests, runningWithoutInit)
{
  std::unique_ptr<PilzModbusClientMock> mock(new PilzModbusClientMock());

  EXPECT_CALL(*mock, init(_, _)).Times(0);
  EXPECT_CALL(*mock, readHoldingRegister(_, _)).Times(0);
  EXPECT_CALL(*this, modbus_read_cb(_)).Times(0);

  std::vector<unsigned short> registers(REGISTER_SIZE_TEST);
  std::iota(registers.begin(), registers.end(), REGISTER_FIRST_IDX_TEST);

  PilzModbusClient client(nh_, registers, std::move(mock), RESPONSE_TIMEOUT, prbt_hardware_support::TOPIC_MODBUS_READ,
                          prbt_hardware_support::SERVICE_MODBUS_WRITE);

  EXPECT_THROW(client.run(), PilzModbusClientException);
}

/**
 * @brief Test that a running modbus read client can be stopped with the terminate method
 */
TEST_F(PilzModbusClientTests, terminateRunningClient)
{
  std::unique_ptr<PilzModbusClientMock> mock(new PilzModbusClientMock());

  EXPECT_CALL(*mock, init(_, _)).Times(1).WillOnce(Return(true));
  ON_CALL(*mock, readHoldingRegister(_, _)).WillByDefault(Return(std::vector<uint16_t>{ 3, 4 }));
  ON_CALL(*this, modbus_read_cb(IsSuccessfullRead(std::vector<uint16_t>{ 3, 4 }))).WillByDefault(Return());

  std::vector<unsigned short> registers(REGISTER_SIZE_TEST);
  std::iota(registers.begin(), registers.end(), REGISTER_FIRST_IDX_TEST);

  auto client = std::make_shared<PilzModbusClient>(nh_, registers, std::move(mock), RESPONSE_TIMEOUT,
                                                   prbt_hardware_support::TOPIC_MODBUS_READ,
                                                   prbt_hardware_support::SERVICE_MODBUS_WRITE);

  EXPECT_TRUE(client->init(LOCALHOST, DEFAULT_MODBUS_PORT_TEST));

  PilzModbusClientExecutor executor(client.get());
  executor.start();
  EXPECT_TRUE(client->isRunning());
  executor.stop();
  EXPECT_FALSE(client->isRunning());
}

void callbackDummy(const ModbusMsgInStampedConstPtr& /*msg*/)
{
}

/**
 * @brief Tests that topic name can be changed.
 */
TEST_F(PilzModbusClientTests, testTopicNameChange)
{
  std::unique_ptr<PilzModbusClientMock> mock(new PilzModbusClientMock());

  const std::string topic_name{ "test_topic_name" };

  std::vector<unsigned short> registers(REGISTER_SIZE_TEST);
  std::iota(registers.begin(), registers.end(), REGISTER_FIRST_IDX_TEST);

  auto client = std::make_shared<PilzModbusClient>(nh_, registers, std::move(mock), RESPONSE_TIMEOUT, topic_name,
                                                   prbt_hardware_support::SERVICE_MODBUS_WRITE);
  // Wait for a moment to ensure the topic is "up and running"
  ros::Duration(WAIT_SLEEPTIME_S).sleep();
  ros::Subscriber sub = nh_.subscribe<ModbusMsgInStamped>(topic_name, 1, &callbackDummy);
  ASSERT_EQ(1u, sub.getNumPublishers());
}

/**
 * @brief Tests that response timeout can be changed.
 */
TEST_F(PilzModbusClientTests, testSettingOfTimeOut)
{
  std::unique_ptr<PilzModbusClientMock> mock(new PilzModbusClientMock());

  const unsigned int response_timeout{ RESPONSE_TIMEOUT + 4 };

  EXPECT_CALL(*mock, init(_, _)).Times(1).WillOnce(Return(true));

  EXPECT_CALL(*mock, setResponseTimeoutInMs(response_timeout)).Times(1);

  std::vector<unsigned short> registers(REGISTER_SIZE_TEST);
  std::iota(registers.begin(), registers.end(), REGISTER_FIRST_IDX_TEST);

  auto client = std::make_shared<PilzModbusClient>(nh_, registers, std::move(mock), response_timeout,
                                                   prbt_hardware_support::TOPIC_MODBUS_READ,
                                                   prbt_hardware_support::SERVICE_MODBUS_WRITE);

  EXPECT_TRUE(client->init(LOCALHOST, DEFAULT_MODBUS_PORT_TEST));
}

/**
 * @brief Strictly increases the holding register each time the
 * "readHoldingRegister" method is called.
 */
class HoldingRegisterIncreaser
{
public:
  HoldingRegisterIncreaser(unsigned int register_index = 0u) : register_index_(register_index)
  {
  }

public:
  RegCont readHoldingRegister(int /*addr*/, int nb)
  {
    RegCont vec(static_cast<RegCont::size_type>(nb), 0u);
    vec.at(register_index_) = curr_count_;
    ++curr_count_;
    return vec;
  }

  void reset()
  {
    curr_count_ = 0;
  }

private:
  const unsigned int register_index_;
  uint16_t curr_count_{ 0 };
};

class RegisterBuffer
{
public:
  void add(const ModbusMsgInStampedConstPtr& msg)
  {
    std::unique_lock<std::mutex> lk(m_);
    buffer_ = msg->holding_registers.data.at(0);
  }

  uint16_t get()
  {
    std::unique_lock<std::mutex> lk(m_);
    return buffer_;
  }

private:
  std::mutex m_;
  uint16_t buffer_;
};

/**
 * @brief Tests that the frequency with which the registers are read,
 * can be changed.
 *
 * This test does NOT precisly check (tolerance ca. 10%) if the exact
 * frequency could be set. It just checks that the frequency was
 * changed.
 *
 *
 * Test Sequence:
 * - 1. Various frequencies are checked. Measure the number of register accesses within ~1 seconds.
 *
 * Expected Results:
 * - 1. The measured frequence is within a 10% tolerance
 */
TEST_F(PilzModbusClientTests, testSettingReadFrequency)
{
  std::vector<double> checked_frequencies{ 20.0, 40.0, 60.0 };

  for (const auto& expected_read_frequency : checked_frequencies)
  {
    std::unique_ptr<PilzModbusClientMock> mock(new PilzModbusClientMock());

    EXPECT_CALL(*mock, init(_, _)).Times(1).WillOnce(Return(true));

    HoldingRegisterIncreaser fake_holding_register;
    ON_CALL(*mock, readHoldingRegister(_, _))
        .WillByDefault(Invoke(&fake_holding_register, &HoldingRegisterIncreaser::readHoldingRegister));

    RegisterBuffer buffer;
    ON_CALL(*this, modbus_read_cb(_)).WillByDefault(Invoke(&buffer, &RegisterBuffer::add));

    std::vector<unsigned short> registers(REGISTER_SIZE_TEST);
    std::iota(registers.begin(), registers.end(), REGISTER_FIRST_IDX_TEST);

    auto client = std::make_shared<PilzModbusClient>(
        nh_, registers, std::move(mock), RESPONSE_TIMEOUT, prbt_hardware_support::TOPIC_MODBUS_READ,
        prbt_hardware_support::SERVICE_MODBUS_WRITE, expected_read_frequency);

    EXPECT_TRUE(client->init(LOCALHOST, DEFAULT_MODBUS_PORT_TEST));

    PilzModbusClientExecutor executor(client.get());
    auto time_start = ros::Time::now();
    executor.start();

    ASSERT_TRUE(client->isRunning());

    // Time for measuring
    ros::Duration(1).sleep();

    auto final_value = buffer.get();
    auto time_stop = ros::Time::now();
    executor.stop();

    // Checks
    auto actual_frequency = (final_value - 1) / (time_stop - time_start).toSec();
    EXPECT_NEAR(expected_read_frequency, actual_frequency, 0.1 * expected_read_frequency);

    EXPECT_FALSE(client->isRunning());
  }
}

/**
 * @brief Tests that modbus client can properly write and read from
 * holding register (with one call).
 */
TEST_F(PilzModbusClientTests, testWritingOfHoldingRegister)
{
  std::unique_ptr<PilzModbusClientMock> mock(new PilzModbusClientMock());

  EXPECT_CALL(*mock, init(_, _)).Times(1).WillOnce(Return(true));

  ON_CALL(*mock, readHoldingRegister(_, _)).WillByDefault(Return(std::vector<uint16_t>{ 1, 7 }));

  // Data to be expected
  RegCont expected_write_reg{ 1, 5, 4 };
  WriteModbusRegister reg_write_srv;
  reg_write_srv.request.holding_register_block.start_idx = 3;
  reg_write_srv.request.holding_register_block.values = expected_write_reg;

  RegCont expected_read_reg{ 14, 17 };

  EXPECT_CALL(*mock, writeReadHoldingRegister(static_cast<int>(reg_write_srv.request.holding_register_block.start_idx),
                                              expected_write_reg, REGISTER_FIRST_IDX_TEST,
                                              static_cast<int>(expected_read_reg.size())))
      .Times(1)
      .WillOnce(testing::DoAll(ACTION_OPEN_BARRIER_VOID("writeHoldingRegister"), Return(expected_read_reg)));

  EXPECT_CALL(*this, modbus_read_cb(_)).Times(AnyNumber());

  EXPECT_CALL(*this, modbus_read_cb(IsSuccessfullRead(expected_read_reg))).Times(AtLeast(1));

  std::vector<unsigned short> registers(expected_read_reg.size());
  std::iota(registers.begin(), registers.end(), REGISTER_FIRST_IDX_TEST);

  auto modbus_client = std::make_shared<PilzModbusClient>(nh_, registers, std::move(mock), RESPONSE_TIMEOUT,
                                                          prbt_hardware_support::TOPIC_MODBUS_READ,
                                                          prbt_hardware_support::SERVICE_MODBUS_WRITE);

  EXPECT_TRUE(modbus_client->init(LOCALHOST, DEFAULT_MODBUS_PORT_TEST)) << "Initialization of modbus client failed";

  ros::ServiceClient writer_client =
      nh_.serviceClient<WriteModbusRegister>(prbt_hardware_support::SERVICE_MODBUS_WRITE);
  ASSERT_TRUE(writer_client.waitForExistence(ros::Duration(WAIT_FOR_SERVICE_TIMEOUT_S))) << "Modbus write service was "
                                                                                            "not advertised in time";

  PilzModbusClientExecutor executor(modbus_client.get());
  executor.start();
  EXPECT_TRUE(modbus_client->isRunning()) << "Modbus client not running";

  EXPECT_TRUE(writer_client.call(reg_write_srv)) << "Modbus write service failed";
  EXPECT_TRUE(reg_write_srv.response.success) << "Modbus write service could not write modbus register";

  BARRIER("writeHoldingRegister");

  executor.stop();
  EXPECT_FALSE(modbus_client->isRunning());
}

/**
 * @brief Tests the split_into_blocks method
 */
TEST_F(PilzModbusClientTests, testSplitIntoBlocksFcn)
{
  // lists with same elements will throw exception
  std::vector<unsigned short> in_throw = { 1, 2, 1 };
  ASSERT_THROW(PilzModbusClient::splitIntoBlocks(in_throw), PilzModbusClientException);

  // properly splitting into blocks
  std::vector<unsigned short> in_split = { 1, 2, 4, 5 };
  std::vector<std::vector<unsigned short>> out_split = PilzModbusClient::splitIntoBlocks(in_split);
  std::vector<std::vector<unsigned short>> expected_out_split = { { 1, 2 }, { 4, 5 } };
  EXPECT_EQ(out_split, expected_out_split);

  // not splitting already consecutive blocks
  std::vector<unsigned short> in_nosplit = { 1, 2, 3 };
  std::vector<std::vector<unsigned short>> out_nosplit = PilzModbusClient::splitIntoBlocks(in_nosplit);
  std::vector<std::vector<unsigned short>> expected_out_nosplit = { { 1, 2, 3 } };
  EXPECT_EQ(out_nosplit, expected_out_nosplit);
}
}  // namespace pilz_modbus_client_test

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "pilz_modbus_client_test");
  ros::NodeHandle nh;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
