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

#include <string>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <ros/ros.h>

#include <std_srvs/SetBool.h>

#include <pilz_testutils/async_test.h>

namespace disable_speed_monitoring_tests
{
static const std::string SPEED_MONITOR_SERVICE_NAME{ "/prbt/manipulator_joint_trajectory_controller/"
                                                     "monitor_cartesian_speed" };

static const std::string SPEED_MONITOR_CALLBACK_EVENT{ "speed_monitor_callback" };

class SpeedMonitorServiceMock
{
public:
  MOCK_METHOD2(callback, bool(std_srvs::SetBool::Request&, std_srvs::SetBool::Response&));
};

class DisableSpeedMonitoringTest : public testing::Test, public testing::AsyncTest
{
};

/**
 * @tests{speed_monitoring_without_operation_mode_support,
 *  Tests that speed monitoring gets deactivated via service call from the disable_speed_monitoring node
 *  once the service is advertised.
 * }
 */
TEST_F(DisableSpeedMonitoringTest, testDisableSpeedMonitoring)
{
  using testing::_;
  using testing::Field;
  using testing::Return;
  using testing::SetArgReferee;

  ros::NodeHandle nh;
  SpeedMonitorServiceMock service_mock;

  std_srvs::SetBoolResponse response;
  response.success = true;

  ON_CALL(service_mock, callback(_, _)).WillByDefault(DoAll(SetArgReferee<1>(response), Return(true)));

  EXPECT_CALL(service_mock, callback(Field(&std_srvs::SetBoolRequest::data, false), _))
      .WillOnce(DoAll(SetArgReferee<1>(response), ACTION_OPEN_BARRIER(SPEED_MONITOR_CALLBACK_EVENT)));

  ros::ServiceServer speed_monitor_srv =
      nh.advertiseService(SPEED_MONITOR_SERVICE_NAME, &SpeedMonitorServiceMock::callback, &service_mock);

  BARRIER(SPEED_MONITOR_CALLBACK_EVENT);
}

}  // namespace disable_speed_monitoring_tests

int main(int argc, char** argv)
{
  ros::init(argc, argv, "unittest_disable_speed_monitoring");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner{ 1 };
  spinner.start();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
