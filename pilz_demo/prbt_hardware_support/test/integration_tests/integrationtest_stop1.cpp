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
#include <gmock/gmock.h>

#include <thread>
#include <memory>
#include <chrono>
#include <vector>
#include <functional>
#include <condition_variable>
#include <atomic>
#include <string>

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include <modbus/modbus.h>

#include <prbt_hardware_support/modbus_topic_definitions.h>
#include <prbt_hardware_support/modbus_api_definitions.h>
#include <prbt_hardware_support/ModbusMsgInStamped.h>
#include <prbt_hardware_support/pilz_modbus_server_mock.h>
#include <prbt_hardware_support/pilz_modbus_client.h>
#include <prbt_hardware_support/pilz_manipulator_mock.h>
#include <prbt_hardware_support/modbus_api_spec.h>

#include <prbt_hardware_support/ros_test_helper.h>

#include <pilz_testutils/async_test.h>

namespace prbt_hardware_support
{
using ::testing::_;
using ::testing::AtLeast;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Invoke;
using ::testing::InvokeWithoutArgs;
using ::testing::Return;
using ::testing::SetArgReferee;

using namespace modbus_api::v3;

static const std::string RECOVER_SERVICE_NAME{ "recover" };
static const std::string HALT_SERVICE_NAME{ "halt" };
static const std::string HOLD_SERVICE_NAME{ "hold" };
static const std::string UNHOLD_SERVICE_NAME{ "unhold" };

/**
 * @brief Stop1IntegrationTest checks if the chain
 * ModbusServerMock -> ModbusReadClient -> RunPermittedModbusAdapter -> ManipulatorMock functions properly
 *
 * @note the test is derived from testing::AsyncTest which allows the asynchronous processes to re-sync
 *
 */
class Stop1IntegrationTest : public testing::Test, public testing::AsyncTest
{
public:
  void SetUp() override;
  void TearDown() override;

public:
  bool serviceCallStub(const std::string& barrier_name, const std_srvs::Trigger::Response& res_exp,
                       std_srvs::Trigger::Request& /*req*/, std_srvs::Trigger::Response& res)
  {
    this->triggerClearEvent(barrier_name);
    res = res_exp;
    return true;
  }

protected:
  // Serves both the controller (/hold + /unhold) and the driver (/halt + /recover) services
  ManipulatorMock manipulator;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_{ "~" };
};

void Stop1IntegrationTest::SetUp()
{
  manipulator.advertiseServices(nh_, HOLD_SERVICE_NAME, UNHOLD_SERVICE_NAME, HALT_SERVICE_NAME, RECOVER_SERVICE_NAME);
}

void Stop1IntegrationTest::TearDown()
{
}

/**
 * @brief Test that correct service calls occurs based on RUN_PERMITTED state.
 *
 * @tests{execution_of_stop1,
 *  Test that drives are recovered and controller is unhold
 *  after RUN_PERMITTED switch: false->true.
 * }
 *
 * @tests{Stop1_Trigger,
 *  Test that Stop 1 is triggered if RUN_PERMITTED value changes to false.
 * }
 *
 *
 * @note Due to the asynchronicity of the test each step of the sequence passed successful
 *       allows the next step to be taken. See testing::AsyncTest for details.
 *
 *
 * Data send via:
 * ModbusServerMock -> ModbusReadClient -> RunPermittedModbusAdapter -> ManipulatorMock connection
 *
 * Test Sequence:
 *    1. Start Modbus-server in seperate thread. Make sure that the nodes are up.
 *       Send a RUN_PERMITTED clear message with the correct API version.
 *    2. Send a RUN_PERMITTED active message with the correct API version.
 *    3. Send a RUN_PERMITTED clear message with the correct API version.
 *    4. Terminate Modbus-server to cause a disconnect.
 *
 * Expected Results:
 *    1. The manipulator mock should receive a call to /recover after that a call to /unhold.
 *       No other calls should happen.
 *    2. The manipulator mock should receive a call to /hold after that a call to /halt.
 *       No other calls should happen.
 *    3. The manipulator mock should receive a call to /recover after that a call to /unhold.
 *       No other calls should happen.
 *    4. The manipulator mock should receive a call to /hold after that a call to /halt.
 *       No other calls should happen.
 */
TEST_F(Stop1IntegrationTest, testServiceCallbacks)
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
  prbt_hardware_support::PilzModbusServerMock modbus_server(static_cast<unsigned int>(modbus_register_size));

  std::thread modbus_server_thread(&initalizeAndRun<prbt_hardware_support::PilzModbusServerMock>,
                                   std::ref(modbus_server), ip.c_str(), static_cast<unsigned int>(port));

  waitForNode("/pilz_modbus_client_node");
  waitForNode("/modbus_adapter_run_permitted_node");
  waitForNode("/stop1_executor_node");

  using std::placeholders::_1;
  using std::placeholders::_2;

  std_srvs::Trigger::Response res_exp;
  res_exp.success = true;

  // We expect:
  {
    InSequence dummy;

    // Call from RUN_PERMITTED clear
    EXPECT_CALL(manipulator, holdCb(_, _)).Times(0);
    EXPECT_CALL(manipulator, haltCb(_, _)).Times(0);
    EXPECT_CALL(manipulator, recoverCb(_, _)).Times(1).WillOnce(DoAll(SetArgReferee<1>(res_exp), Return(true)));
    EXPECT_CALL(manipulator, unholdCb(_, _))
        .Times(1)
        .WillOnce(Invoke(std::bind(&Stop1IntegrationTest::serviceCallStub, this, "unhold_callback", res_exp, _1, _2)));
    // Expected came true -> go on
  }

  // This should trigger the expected reaction
  ASSERT_TRUE(api_spec.hasRegisterDefinition(modbus_api_spec::VERSION));
  unsigned int version_register = api_spec.getRegisterDefinition(modbus_api_spec::VERSION);

  ASSERT_TRUE(api_spec.hasRegisterDefinition(modbus_api_spec::RUN_PERMITTED));
  unsigned int run_permitted_register = api_spec.getRegisterDefinition(modbus_api_spec::RUN_PERMITTED);

  modbus_server.setHoldingRegister(
      { { version_register, MODBUS_API_VERSION_REQUIRED }, { run_permitted_register, MODBUS_RUN_PERMITTED_TRUE } });

  /**********
   * Step 2 *
   **********/
  BARRIER("unhold_callback");

  {
    InSequence dummy;

    // Call from RUN_PERMITTED active
    EXPECT_CALL(manipulator, unholdCb(_, _)).Times(0);
    EXPECT_CALL(manipulator, recoverCb(_, _)).Times(0);
    EXPECT_CALL(manipulator, holdCb(_, _)).Times(1).WillOnce(DoAll(SetArgReferee<1>(res_exp), Return(true)));
    EXPECT_CALL(manipulator, haltCb(_, _))
        .Times(1)
        .WillOnce(Invoke(std::bind(&Stop1IntegrationTest::serviceCallStub, this, "halt_callback", res_exp, _1, _2)));
  }

  modbus_server.setHoldingRegister({ { run_permitted_register, MODBUS_RUN_PERMITTED_FALSE } });

  /**********
   * Step 3 *
   **********/
  BARRIER("halt_callback");

  {
    InSequence dummy;

    // Call from RUN_PERMITTED clear
    EXPECT_CALL(manipulator, holdCb(_, _)).Times(0);
    EXPECT_CALL(manipulator, haltCb(_, _)).Times(0);
    EXPECT_CALL(manipulator, recoverCb(_, _)).Times(1).WillOnce(DoAll(SetArgReferee<1>(res_exp), Return(true)));
    EXPECT_CALL(manipulator, unholdCb(_, _))
        .Times(1)
        .WillOnce(Invoke(std::bind(&Stop1IntegrationTest::serviceCallStub, this, "unhold_callback", res_exp, _1, _2)));
  }

  modbus_server.setHoldingRegister({ { run_permitted_register, MODBUS_RUN_PERMITTED_TRUE } });

  /**********
   * Step 4 *
   **********/
  BARRIER("unhold_callback");
  {
    InSequence dummy;

    // Call from Disconnect
    EXPECT_CALL(manipulator, holdCb(_, _)).Times(1).WillOnce(DoAll(SetArgReferee<1>(res_exp), Return(true)));
    EXPECT_CALL(manipulator, unholdCb(_, _)).Times(0);
    EXPECT_CALL(manipulator, recoverCb(_, _)).Times(0);
    EXPECT_CALL(manipulator, haltCb(_, _))
        .Times(1)
        .WillOnce(Invoke(std::bind(&Stop1IntegrationTest::serviceCallStub, this, "halt_callback", res_exp, _1, _2)));
  }

  modbus_server.terminate();
  modbus_server_thread.join();

  /**********
   * Step 5 *
   **********/
  BARRIER("halt_callback");  // Needed for proper async finish
}

}  // namespace prbt_hardware_support

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "integrationtest_stop1");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner{ 1 };
  spinner.start();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
