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

#include <functional>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include <pilz_utils/wait_for_service.h>

#include <prbt_hardware_support/brake_test_executor.h>
#include <prbt_hardware_support/SendBrakeTestResult.h>
#include <prbt_hardware_support/BrakeTest.h>
#include <prbt_hardware_support/brake_test_utils.h>
#include <prbt_hardware_support/brake_test_executor_node_service_calls.h>

static const std::string EXECUTE_BRAKETEST_SERVICE_NAME{ "/prbt/execute_braketest" };
static const std::string BRAKETEST_ADAPTER_SERVICE_NAME{ "/prbt/braketest_adapter_node/trigger_braketest" };

static const std::string CONTROLLER_HOLD_MODE_SERVICE_NAME{ "/prbt/manipulator_joint_trajectory_controller/hold" };
static const std::string CONTROLLER_UNHOLD_MODE_SERVICE_NAME{ "/prbt/manipulator_joint_trajectory_controller/unhold" };
static const std::string BRAKE_TEST_RESULT_SERVICE_NAME{ "/prbt/send_brake_test_result" };

using namespace prbt_hardware_support;

/**
 * @brief Provides service to execute a braketest
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "brake_test_executor");
  ros::NodeHandle nh{ "~" };

  pilz_utils::waitForService(CONTROLLER_HOLD_MODE_SERVICE_NAME);
  ros::ServiceClient hold_client = nh.serviceClient<std_srvs::Trigger>(CONTROLLER_HOLD_MODE_SERVICE_NAME);

  pilz_utils::waitForService(BRAKETEST_ADAPTER_SERVICE_NAME);
  ros::ServiceClient brake_test_execute_client = nh.serviceClient<BrakeTest>(BRAKETEST_ADAPTER_SERVICE_NAME);

  pilz_utils::waitForService(CONTROLLER_UNHOLD_MODE_SERVICE_NAME);
  ros::ServiceClient unhold_client = nh.serviceClient<std_srvs::Trigger>(CONTROLLER_UNHOLD_MODE_SERVICE_NAME);

  pilz_utils::waitForService(BRAKE_TEST_RESULT_SERVICE_NAME);
  ros::ServiceClient brake_test_result_client = nh.serviceClient<SendBrakeTestResult>(BRAKE_TEST_RESULT_SERVICE_NAME);

  using std::placeholders::_1;
  prbt_hardware_support::BrakeTestExecutor brake_test_executor(
      std::bind(BrakeTestUtils::detectRobotMotion, DEFAULT_ROBOT_MOTION_TIMEOUT_S),
      std::bind(triggerServiceCall<ros::ServiceClient>, hold_client),
      std::bind(executeBrakeTestCall<ros::ServiceClient>, brake_test_execute_client),
      std::bind(triggerServiceCall<ros::ServiceClient>, unhold_client),
      std::bind(sendBrakeTestResultCall<ros::ServiceClient>, brake_test_result_client, _1));

  ros::ServiceServer brake_test_srv =
      nh.advertiseService(EXECUTE_BRAKETEST_SERVICE_NAME, &BrakeTestExecutor::executeBrakeTest, &brake_test_executor);

  ros::spin();

  return EXIT_FAILURE;
}
