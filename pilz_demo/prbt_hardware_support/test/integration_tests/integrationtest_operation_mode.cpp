/*
 * Copyright (c) 2019 Pilz GmbH & Co. KG
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
#include <gmock/gmock.h>

#include <memory>
#include <string>

#include <ros/ros.h>
#include <modbus/modbus.h>

#include <pilz_msgs/OperationModes.h>

#include <pilz_testutils/async_test.h>

#include <prbt_hardware_support/pilz_modbus_server_mock.h>
#include <prbt_hardware_support/ros_test_helper.h>
#include <prbt_hardware_support/modbus_api_definitions.h>
#include <prbt_hardware_support/modbus_api_spec.h>

namespace prbt_hardware_support
{
using namespace modbus_api::v3;

static const std::string TOPIC_OPERATION_MODE{ "/prbt/operation_mode" };
static constexpr int OPERATION_MODE_QUEUE_SIZE{ 1 };

static const std::string OPERATION_MODE_CALLBACK_EVENT{ "operation_mode_callback_event" };

/**
 * @brief Redirects callbacks of a ros::Subscriber to a mock method.
 */
class OperationModeSubscriberMock
{
public:
  /**
   * @brief Actual subscription takes place here
   */
  void initialize();

  MOCK_METHOD1(callback, void(const pilz_msgs::OperationModesConstPtr& msg));

protected:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
};

void OperationModeSubscriberMock::initialize()
{
  subscriber_ =
      nh_.subscribe(TOPIC_OPERATION_MODE, OPERATION_MODE_QUEUE_SIZE, &OperationModeSubscriberMock::callback, this);
}

/**
 * @brief OperationModeIntegrationTest checks if the chain
 * ModbusServerMock -> ModbusReadClient -> ModbusAdapterOperationMode functions properly
 */
class OperationModeIntegrationTest : public testing::Test, public testing::AsyncTest
{
protected:
  using OperationModes = pilz_msgs::OperationModes;

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_{ "~" };
};

MATCHER_P(IsExpectedOperationMode, exp_mode, "unexpected operation mode")
{
  return arg->value == exp_mode;
}

/**
 * @tests{Get_OperationMode_mechanism,
 *  Test that the expected result is obtained on the operation mode topic.
 * }
 *
 * @note Due to the asynchronicity of the test each step of the sequence passed successful
 *       allows the next step to be taken. See testing::AsyncTest for details.
 *
 * Data send via:
 *  ModbusServerMock -> ModbusReadClient -> ModbusAdapterOperationMode
 *
 * Test Sequence:
 *    1. Start Modbus-server in separate thread. Make sure that the nodes are up and subscribe to operation mode topic.
 *    2. Send message with T1 in operation mode register and with the correct API version.
 *    3. Send message with 0 (unknown) in operation mode register and with the correct API version.
 *    4. Send message with T2 in operation mode register and with the correct API version.
 *    5. Send message with AUTO in operation mode register and with the correct API version.
 *    6. Send message with 99 (unknown) in operation mode register and with the correct API version.
 *    7. Terminate ModbusServerMock.
 *
 * Expected Results:
 *    1. A message with unknown operation mode is obtained
 *    2. A message with T1 operation mode is obtained
 *    3. A message with unknown operation mode is obtained
 *    4. A message with T2 operation mode is obtained
 *    5. A message with AUTO operation mode is obtained
 *    6. A message with unknown operation mode is obtained
 *    7. -
 */
TEST_F(OperationModeIntegrationTest, testOperationModeRequestAnnouncement)
{
  /**********
   * Setup *
   **********/
  std::string ip;
  int port;
  ASSERT_TRUE(nh_priv_.getParam("modbus_server_ip", ip));
  ASSERT_TRUE(nh_priv_.getParam("modbus_server_port", port));

  ModbusApiSpec api_spec{ nh_ };

  unsigned int modbus_register_size{ api_spec.getMaxRegisterDefinition() + 1U };

  /**********
   * Step 1 *
   **********/
  prbt_hardware_support::PilzModbusServerMock modbus_server(modbus_register_size);

  std::thread modbus_server_thread(&initalizeAndRun<prbt_hardware_support::PilzModbusServerMock>,
                                   std::ref(modbus_server), ip.c_str(), static_cast<unsigned int>(port));

  waitForNode("/pilz_modbus_client_node");
  waitForNode("/modbus_adapter_operation_mode_node");

  using ::testing::StrictMock;
  StrictMock<OperationModeSubscriberMock> subscriber;

  EXPECT_CALL(subscriber, callback(IsExpectedOperationMode(OperationModes::UNKNOWN)))
      .WillOnce(ACTION_OPEN_BARRIER_VOID(OPERATION_MODE_CALLBACK_EVENT));

  subscriber.initialize();

  BARRIER(OPERATION_MODE_CALLBACK_EVENT);

  /**********
   * Step 2 *
   **********/
  ASSERT_TRUE(api_spec.hasRegisterDefinition(modbus_api_spec::VERSION));
  unsigned int version_register = api_spec.getRegisterDefinition(modbus_api_spec::VERSION);

  modbus_server.setHoldingRegister({ { version_register, MODBUS_API_VERSION_REQUIRED } });

  EXPECT_CALL(subscriber, callback(IsExpectedOperationMode(OperationModes::T1)))
      .WillOnce(ACTION_OPEN_BARRIER_VOID(OPERATION_MODE_CALLBACK_EVENT));

  ASSERT_TRUE(api_spec.hasRegisterDefinition(modbus_api_spec::OPERATION_MODE));
  unsigned int op_mode_register = api_spec.getRegisterDefinition(modbus_api_spec::OPERATION_MODE);

  modbus_server.setHoldingRegister({ { op_mode_register, 1 } });

  BARRIER(OPERATION_MODE_CALLBACK_EVENT);

  /**********
   * Step 3 *
   **********/
  EXPECT_CALL(subscriber, callback(IsExpectedOperationMode(OperationModes::UNKNOWN)))
      .WillOnce(ACTION_OPEN_BARRIER_VOID(OPERATION_MODE_CALLBACK_EVENT));

  modbus_server.setHoldingRegister({ { op_mode_register, 0 } });

  BARRIER(OPERATION_MODE_CALLBACK_EVENT);

  /**********
   * Step 4 *
   **********/
  EXPECT_CALL(subscriber, callback(IsExpectedOperationMode(OperationModes::T2)))
      .WillOnce(ACTION_OPEN_BARRIER_VOID(OPERATION_MODE_CALLBACK_EVENT));

  modbus_server.setHoldingRegister({ { op_mode_register, 2 } });

  BARRIER(OPERATION_MODE_CALLBACK_EVENT);

  /**********
   * Step 5 *
   **********/
  EXPECT_CALL(subscriber, callback(IsExpectedOperationMode(OperationModes::AUTO)))
      .WillOnce(ACTION_OPEN_BARRIER_VOID(OPERATION_MODE_CALLBACK_EVENT));

  modbus_server.setHoldingRegister({ { op_mode_register, 3 } });

  BARRIER(OPERATION_MODE_CALLBACK_EVENT);

  /**********
   * Step 6 *
   **********/
  EXPECT_CALL(subscriber, callback(IsExpectedOperationMode(OperationModes::UNKNOWN)))
      .WillOnce(ACTION_OPEN_BARRIER_VOID(OPERATION_MODE_CALLBACK_EVENT));

  modbus_server.setHoldingRegister({ { op_mode_register, 99 } });

  BARRIER(OPERATION_MODE_CALLBACK_EVENT);

  /**********
   * Step 7 *
   **********/
  modbus_server.terminate();
  modbus_server_thread.join();
}

}  // namespace prbt_hardware_support

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "integrationtest_operation_mode");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner{ 1 };
  spinner.start();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
