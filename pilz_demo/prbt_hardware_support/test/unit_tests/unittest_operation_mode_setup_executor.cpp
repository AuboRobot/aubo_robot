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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <ros/time.h>

#include <std_srvs/SetBool.h>

#include <pilz_msgs/GetSpeedOverride.h>
#include <pilz_msgs/OperationModes.h>

#include <prbt_hardware_support/operation_mode_setup_executor.h>
#include <prbt_hardware_support/operation_mode_setup_executor_node_service_calls.h>

namespace operation_mode_setup_executor_tests
{
using namespace prbt_hardware_support;

using ::testing::_;
using ::testing::DoAll;
using ::testing::Le;
using ::testing::Return;
using ::testing::SetArgReferee;

using pilz_msgs::OperationModes;

class OperationModeSetupExecutorTest : public ::testing::Test
{
public:
  void SetUp() override;
  void TearDown() override;

  MOCK_METHOD1(monitorCartesianSpeed, bool(const bool));

public:
  std::unique_ptr<OperationModeSetupExecutor> executor_;
};

void OperationModeSetupExecutorTest::SetUp()
{
  ros::Time::init();

  OperationModes op_mode;
  op_mode.time_stamp = ros::Time::now();
  op_mode.value = OperationModes::UNKNOWN;

  ON_CALL(*this, monitorCartesianSpeed(_)).WillByDefault(Return(true));

  executor_ = std::unique_ptr<OperationModeSetupExecutor>(new OperationModeSetupExecutor(
      std::bind(&OperationModeSetupExecutorTest::monitorCartesianSpeed, this, std::placeholders::_1)));
}

void OperationModeSetupExecutorTest::TearDown()
{
  ros::Time::shutdown();
}

/**
 * @tests{speed_monitoring_per_operation_mode,
 * Tests that the cartesian speed is monitored depending on the current operation mode.
 * }
 *
 * Test Sequence:
 *  1. Call updateOperationMode() with operation mode T1 and current time stamp.
 *  2. Call updateOperationMode() with operation mode AUTO and current time stamp.
 *
 * Expected Results:
 *  1. monitorCartesianSpeed() is called with arg=true.
 *  2. monitorCartesianSpeed() is called with arg=false.
 */
TEST_F(OperationModeSetupExecutorTest, testUpdateOperationMode)
{
  /**********
   * Step 1 *
   **********/
  OperationModes op_mode;
  op_mode.time_stamp = ros::Time::now();
  op_mode.value = OperationModes::T1;

  EXPECT_CALL(*this, monitorCartesianSpeed(true)).WillOnce(Return(true));

  executor_->updateOperationMode(op_mode);

  /**********
   * Step 2 *
   **********/
  OperationModes op_mode2;
  op_mode2.time_stamp = ros::Time::now();
  op_mode2.value = OperationModes::AUTO;

  EXPECT_CALL(*this, monitorCartesianSpeed(false)).WillOnce(Return(true));

  executor_->updateOperationMode(op_mode2);
}

/**
 * @tests{speed_monitoring_per_operation_mode,
 * Test updateOperationMode() with no change in operation mode.
 * }
 *
 * Test Sequence:
 *  1. Call updateOperationMode() with operation mode T1 and current time stamp.
 *  2. Call updateOperationMode() with operation mode T1 and current time stamp.
 *
 * Expected Results:
 *  1. monitorCartesianSpeed() is called with arg=true.
 *  2. monitorCartesianSpeed() can be called with arg=true.
 */
TEST_F(OperationModeSetupExecutorTest, testUpdateOperationModeSameMode)
{
  /**********
   * Step 1 *
   **********/
  OperationModes op_mode;
  op_mode.time_stamp = ros::Time::now();
  op_mode.value = OperationModes::T1;

  EXPECT_CALL(*this, monitorCartesianSpeed(true)).WillOnce(Return(true));

  executor_->updateOperationMode(op_mode);

  /**********
   * Step 2 *
   **********/
  OperationModes op_mode2;
  op_mode2.time_stamp = ros::Time::now();
  op_mode2.value = op_mode.value;

  EXPECT_CALL(*this, monitorCartesianSpeed(true)).WillRepeatedly(Return(true));

  executor_->updateOperationMode(op_mode2);
}

/**
 * @tests{speed_monitoring_per_operation_mode,
 * Test updateOperationMode() with no change in time.
 * }
 *
 * Test Sequence:
 *  1. Call updateOperationMode() with operation mode T1 and current time stamp.
 *  2. Call updateOperationMode() with operation mode AUTO and previous time stamp.
 *
 * Expected Results:
 *  1. monitorCartesianSpeed() is called with arg=true.
 *  2. monitorCartesianSpeed() is not called.
 */
TEST_F(OperationModeSetupExecutorTest, testUpdateOperationModeSameTime)
{
  /**********
   * Step 1 *
   **********/
  OperationModes op_mode;
  op_mode.time_stamp = ros::Time::now();
  op_mode.value = OperationModes::T1;

  EXPECT_CALL(*this, monitorCartesianSpeed(true)).WillOnce(Return(true));

  executor_->updateOperationMode(op_mode);

  /**********
   * Step 2 *
   **********/
  OperationModes op_mode2;
  op_mode2.time_stamp = op_mode.time_stamp;
  op_mode2.value = OperationModes::AUTO;

  EXPECT_CALL(*this, monitorCartesianSpeed(_)).Times(0);

  executor_->updateOperationMode(op_mode2);
}

/**
 * @tests{speed_monitoring_per_operation_mode,
 * Test updateOperationMode() with unknown operation mode.
 * }
 *
 * Test Sequence:
 *  1. Call updateOperationMode() with operation mode UNKNOWN and current time stamp.
 *
 * Expected Results:
 *  1. monitorCartesianSpeed() is not called.
 */
TEST_F(OperationModeSetupExecutorTest, testUpdateOperationModeUnknownMode)
{
  /**********
   * Step 1 *
   **********/
  OperationModes op_mode;
  op_mode.time_stamp = ros::Time::now();
  op_mode.value = OperationModes::UNKNOWN;

  EXPECT_CALL(*this, monitorCartesianSpeed(_)).Times(0);

  executor_->updateOperationMode(op_mode);
}

class MonitorCartesianSpeedServiceMock
{
public:
  MOCK_METHOD1(call, bool(std_srvs::SetBool& srv));
  MOCK_METHOD0(getService, std::string());
};

MATCHER_P(IsSpeedMonitoringSettingCorrect, active_flag, "")
{
  return arg.request.data == active_flag;
}

/**
 * @brief Tests the correct behavior in case the MonitorCartesianSpeed service
 * succeeds.
 */
TEST_F(OperationModeSetupExecutorTest, testMonitorCartesianSpeedSrvSuccess)
{
  const bool exp_active_flag{ false };
  std_srvs::SetBool exp_srv;
  exp_srv.request.data = exp_active_flag;

  MonitorCartesianSpeedServiceMock mock;
  EXPECT_CALL(mock, getService()).WillRepeatedly(Return("TestServiceName"));
  EXPECT_CALL(mock, call(IsSpeedMonitoringSettingCorrect(exp_active_flag))).Times(1).WillOnce(Return(true));

  EXPECT_TRUE(monitorCartesianSpeedSrv<MonitorCartesianSpeedServiceMock>(mock, exp_active_flag));
}

/**
 * @brief Tests the correct behavior in case the MonitorCartesianSpeed service fails.
 */
TEST_F(OperationModeSetupExecutorTest, testMonitorCartesianSpeedSrvFailure)
{
  MonitorCartesianSpeedServiceMock mock;
  EXPECT_CALL(mock, getService()).WillRepeatedly(Return("TestServiceName"));
  EXPECT_CALL(mock, call(_)).Times(1).WillOnce(Return(false));

  const bool exp_active_flag{ false };
  EXPECT_FALSE(monitorCartesianSpeedSrv<MonitorCartesianSpeedServiceMock>(mock, exp_active_flag));
}

class OperationModeSetupExecutorTestSpeedOverride
  : public OperationModeSetupExecutorTest,
    public ::testing::WithParamInterface<std::pair<pilz_msgs::OperationModes::_value_type, double>>
{
  OperationModes::_value_type getMode()
  {
    return GetParam().first;
  }

  double getSpeedOverride()
  {
    return GetParam().second;
  }
};

// Usage: If you don't care about uninteresting calls the the MOCK_METHOD but want to suppress the warning
typedef ::testing::NiceMock<OperationModeSetupExecutorTestSpeedOverride>
    OperationModeSetupExecutorTestSpeedOverrideNice;

/**
 * @tests{speed_override_per_operation_mode,
 * Tests that speed override is set according to current operation mode.
 * }
 *
 */
TEST_P(OperationModeSetupExecutorTestSpeedOverrideNice, testSpeedOverride)
{
  /**********
   * Step 1 *
   **********/
  OperationModes op_mode;
  op_mode.time_stamp = ros::Time::now();
  op_mode.value = GetParam().first;

  executor_->updateOperationMode(op_mode);

  auto req = pilz_msgs::GetSpeedOverrideRequest();
  auto res = pilz_msgs::GetSpeedOverrideResponse();
  executor_->getSpeedOverride(req, res);
  EXPECT_EQ(res.speed_override, GetParam().second);
}

INSTANTIATE_TEST_SUITE_P(SpeedOverrideModeTests, OperationModeSetupExecutorTestSpeedOverrideNice,
                         ::testing::Values(std::pair<OperationModes::_value_type, double>(OperationModes::UNKNOWN, 0.0),
                                           std::pair<OperationModes::_value_type, double>(OperationModes::T1, 0.1),
                                           std::pair<OperationModes::_value_type, double>(OperationModes::AUTO, 1.0)));

}  // namespace operation_mode_setup_executor_tests

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
