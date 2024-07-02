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

#include <utility>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <ros/ros.h>

#include <std_srvs/SetBool.h>

#include <pilz_msgs/GetSpeedOverride.h>

#include <pilz_testutils/async_test.h>

#include <pilz_msgs/OperationModes.h>
#include <prbt_hardware_support/ros_test_helper.h>

namespace prbt_hardware_support
{
static const std::string OPERATON_MODE_SETUP_EXECUTOR_NODE_NAME{ "/operation_mode_setup_executor_node" };
static const std::string MONITOR_CARTESIAN_SPEED_SERVICE{ "manipulator_joint_trajectory_controller/"
                                                          "monitor_cartesian_speed" };
static const std::string OPERATION_MODE_TOPIC{ "operation_mode" };
static const std::string GET_SPEED_OVERRIDE_SERVICE{ "get_speed_override" };
static const std::string MONITOR_CARTESIAN_SPEED_CALLBACK_EVENT{ "monitor_cartesian_speed_callback_event" };

static constexpr uint32_t DEFAULT_QUEUE_SIZE{ 10 };

static constexpr double EXPECTED_SPEED_OVERRIDE_AUTO{ 1.0 };
static constexpr double EXPECTED_SPEED_OVERRIDE_T1{ 0.1 };

struct TestData
{
  constexpr TestData(const int8_t op_mode_, const bool expected_request_data_, const double expected_speed_override_)
    : op_mode(op_mode_)
    , expected_request_data(expected_request_data_)
    , expected_speed_override(expected_speed_override_)
  {
  }

  const int8_t op_mode;

  const bool expected_request_data;
  const double expected_speed_override;
};

static constexpr TestData OP_MODE_T1_TEST_DATA{ pilz_msgs::OperationModes::T1, true, EXPECTED_SPEED_OVERRIDE_T1 };
static constexpr TestData OP_MODE_AUTO_TEST_DATA{ pilz_msgs::OperationModes::AUTO, false,
                                                  EXPECTED_SPEED_OVERRIDE_AUTO };

//! @brief Provides the monitor_cartesian_speed service.
class ControllerMock
{
public:
  void advertiseService();
  MOCK_METHOD2(monitor_cartesian_speed_callback_, bool(std_srvs::SetBoolRequest&, std_srvs::SetBoolResponse&));

private:
  ros::NodeHandle nh_;
  ros::ServiceServer monitor_cartesian_speed_srv_;
};

void ControllerMock::advertiseService()
{
  monitor_cartesian_speed_srv_ =
      nh_.advertiseService(MONITOR_CARTESIAN_SPEED_SERVICE, &ControllerMock::monitor_cartesian_speed_callback_, this);
}

/**
 * @brief Test the operation_mode_setup_executor_node
 *
 * Complement the unittest of the operation_mode_setup_executor by covering the respective node.
 */
class OperationModeSetupTest : public testing::Test,
                               public testing::AsyncTest,
                               public testing::WithParamInterface<TestData>
{
public:
  void SetUp() override;

protected:
  using OperationModes = pilz_msgs::OperationModes;
  using GetSpeedOverride = pilz_msgs::GetSpeedOverride;

protected:
  testing::AssertionResult isSpeedOverrideEqualTo(const double& expected_speed_override);

protected:
  ros::NodeHandle nh_;
  ControllerMock controller_mock_;
  ros::Publisher operation_mode_pub_;
  ros::ServiceClient get_speed_override_client_;
};

void OperationModeSetupTest::SetUp()
{
  controller_mock_.advertiseService();
  operation_mode_pub_ = nh_.advertise<OperationModes>(OPERATION_MODE_TOPIC, DEFAULT_QUEUE_SIZE, true);

  ASSERT_TRUE(waitForNode(OPERATON_MODE_SETUP_EXECUTOR_NODE_NAME));
  ASSERT_TRUE(ros::service::waitForService(GET_SPEED_OVERRIDE_SERVICE));
  ASSERT_TRUE(waitForSubscriber(operation_mode_pub_));
}

testing::AssertionResult OperationModeSetupTest::isSpeedOverrideEqualTo(const double& expected_speed_override)
{
  if (!get_speed_override_client_.isValid())
  {
    get_speed_override_client_ = nh_.serviceClient<GetSpeedOverride>(GET_SPEED_OVERRIDE_SERVICE);
  }

  GetSpeedOverride get_speed_override;
  if (!get_speed_override_client_.call(get_speed_override))
  {
    return testing::AssertionFailure() << "Service call failed: " << get_speed_override_client_.getService();
  }
  if (get_speed_override.response.speed_override != expected_speed_override)
  {
    return testing::AssertionFailure() << "Speed overrides do not match. Expected: " << expected_speed_override
                                       << ", actual: " << get_speed_override.response.speed_override;
  }
  return testing::AssertionSuccess();
}

using testing::_;
using testing::DoAll;
using testing::Field;
using testing::Return;
using testing::SetArgReferee;

/**
 * @brief Tests that the  Cartesian speed monitoring is activated or de-activated depending on the operation mode.
 */
TEST_P(OperationModeSetupTest, testSpeedMonitoringActivation)
{
  const auto test_data{ GetParam() };

  std_srvs::SetBoolResponse response;
  response.success = true;
  EXPECT_CALL(controller_mock_, monitor_cartesian_speed_callback_(
                                    Field(&std_srvs::SetBoolRequest::data, test_data.expected_request_data), _))
      .WillOnce(DoAll(SetArgReferee<1>(response), ACTION_OPEN_BARRIER(MONITOR_CARTESIAN_SPEED_CALLBACK_EVENT)));

  OperationModes msg;
  msg.value = test_data.op_mode;
  msg.time_stamp = ros::Time::now();
  operation_mode_pub_.publish(msg);

  BARRIER(MONITOR_CARTESIAN_SPEED_CALLBACK_EVENT);

  EXPECT_TRUE(isSpeedOverrideEqualTo(test_data.expected_speed_override));
}

INSTANTIATE_TEST_SUITE_P(TestActivationOfSpeedMonitoring, OperationModeSetupTest,
                         testing::Values(OP_MODE_AUTO_TEST_DATA, OP_MODE_T1_TEST_DATA));

}  // namespace prbt_hardware_support

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "integrationtest_operation_mode_setup");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner{ 1 };
  spinner.start();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
