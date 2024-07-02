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

#include <ros/ros.h>

#include <pilz_msgs/BrakeTest.h>

#include <pilz_testutils/joint_state_publisher_mock.h>

#include <prbt_hardware_support/WriteModbusRegister.h>
#include <prbt_hardware_support/canopen_chain_node_mock.h>
#include <prbt_hardware_support/modbus_api_spec.h>
#include <prbt_hardware_support/pilz_manipulator_mock.h>
#include <prbt_hardware_support/pilz_modbus_server_mock.h>
#include <prbt_hardware_support/ros_test_helper.h>

static const std::string EXECUTE_BRAKE_TEST_SERVICE_NAME{ "/prbt/execute_braketest" };
static const std::string MODBUS_WRITE_SERVICE_NAME{ "/pilz_modbus_client_node/modbus_write" };
static const std::string CONTROLLER_HOLD_MODE_SERVICE_NAME{ "/prbt/manipulator_joint_trajectory_controller/hold" };
static const std::string CONTROLLER_UNHOLD_MODE_SERVICE_NAME{ "/prbt/manipulator_joint_trajectory_controller/unhold" };

static constexpr double WAIT_FOR_BRAKE_TEST_SERVICE_TIMEOUT_S{ 5.0 };
static constexpr uint16_t MODBUS_BRAKE_TEST_PREPARE_VALUE{ 1 };
static constexpr uint16_t MODBUS_BRAKE_TEST_EXPECTED_VALUE{ 2 };

namespace prbt_hardware_support
{
bool brakeTestRegisterSetOnServer(PilzModbusServerMock& server, unsigned short register_perf,
                                  unsigned short register_res, uint16_t expectation, float sleep_per_try_s,
                                  unsigned short retries)
{
  RegCont content_perf, content_res;
  for (int i = 0; i <= retries; i++)
  {
    ROS_INFO("Retry %d, waited for %.1fs so far", i, sleep_per_try_s * static_cast<float>(i));
    content_perf = server.readHoldingRegister(register_perf, 1);
    content_res = server.readHoldingRegister(register_res, 1);
    if (content_perf[0] == expectation && content_res[0] == expectation)
      return true;  // expected result
    ros::Duration(sleep_per_try_s).sleep();
  }
  return false;
}

/**
 * @tests{Execute_BrakeTest_mechanism,
 *  Test the BrakeTest service node.
 * }
 *
 * Test Sequence:
 *  1. Preparing modbus mock
 *  2. Initialize CANOpen mock, JointStatesPublisher mock and manipulator mock
 *  3. Wait for BrakeTest service
 *  4. Make the service call
 *  5. Check contents of relevant modbus registers
 *  6. Shutdown of modbus mock
 *
 * Expected Results:
 *  1. -
 *  2. -
 *  3. Service is available
 *  4. Service call is successful
 *  5. Brake test success was set in modbus
 *  6. -
 */
TEST(IntegrationtestExecuteBrakeTest, testBrakeTestService)
{
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  /**********
   * Step 1 *
   **********/
  std::string ip;
  int port;
  ASSERT_TRUE(nh_private.getParam("modbus_server_ip", ip));
  ASSERT_TRUE(nh_private.getParam("modbus_server_port", port));

  ModbusApiSpec write_api_spec{ nh, "write_api_spec/" };
  unsigned int modbus_register_size{ write_api_spec.getMaxRegisterDefinition() + 1U };
  prbt_hardware_support::PilzModbusServerMock modbus_server(modbus_register_size);
  std::thread modbus_server_thread(&initalizeAndRun<prbt_hardware_support::PilzModbusServerMock>,
                                   std::ref(modbus_server), ip.c_str(), static_cast<unsigned int>(port));
  ASSERT_TRUE(write_api_spec.hasRegisterDefinition(modbus_api_spec::BRAKETEST_PERFORMED));
  unsigned short register_perf = write_api_spec.getRegisterDefinition(modbus_api_spec::BRAKETEST_PERFORMED);
  ASSERT_TRUE(write_api_spec.hasRegisterDefinition(modbus_api_spec::BRAKETEST_RESULT));
  unsigned short register_res = write_api_spec.getRegisterDefinition(modbus_api_spec::BRAKETEST_RESULT);

  modbus_server.setHoldingRegister({ { register_perf, MODBUS_BRAKE_TEST_PREPARE_VALUE } });
  modbus_server.setHoldingRegister({ { register_res, MODBUS_BRAKE_TEST_PREPARE_VALUE } });

  /**********
   * Step 2 *
   **********/
  CANOpenChainNodeMock canopen_mock;

  pilz_testutils::JointStatePublisherMock joint_states_pub;
  joint_states_pub.startPublishingAsync();

  ManipulatorMock manipulator;
  manipulator.advertiseHoldService(nh, CONTROLLER_HOLD_MODE_SERVICE_NAME);
  manipulator.advertiseUnholdService(nh, CONTROLLER_UNHOLD_MODE_SERVICE_NAME);

  /**********
   * Step 3 *
   **********/
  ros::ServiceClient brake_test_srv_client = nh.serviceClient<pilz_msgs::BrakeTest>(EXECUTE_BRAKE_TEST_SERVICE_NAME);
  EXPECT_TRUE(brake_test_srv_client.waitForExistence(ros::Duration(WAIT_FOR_BRAKE_TEST_SERVICE_TIMEOUT_S)));

  /**********
   * Step 4 *
   **********/
  pilz_msgs::BrakeTest srv;
  EXPECT_TRUE(brake_test_srv_client.call(srv));

  /**********
   * Step 5 *
   **********/
  EXPECT_TRUE(brakeTestRegisterSetOnServer(modbus_server, register_perf, register_res, MODBUS_BRAKE_TEST_EXPECTED_VALUE,
                                           1, 10));

  /**********
   * Step 6 *
   **********/
  joint_states_pub.stopPublishing();
  modbus_server.terminate();
  modbus_server_thread.join();
}

}  // namespace prbt_hardware_support

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "integrationtest_trigger_brake_test");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner{ 1 };
  spinner.start();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
