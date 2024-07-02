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

#include <string>

#include <ros/ros.h>

#include <std_srvs/SetBool.h>

#include <pilz_msgs/GetOperationMode.h>

#include <pilz_utils/get_param.h>
#include <pilz_utils/wait_for_service.h>

#include <prbt_hardware_support/operation_mode_setup_executor.h>
#include <prbt_hardware_support/monitor_cartesian_speed_func_decl.h>
#include <prbt_hardware_support/get_operation_mode_func_decl.h>
#include <prbt_hardware_support/operation_mode_setup_executor_node_service_calls.h>

static const std::string MONITOR_CARTESIAN_SPEED_SERVICE{ "manipulator_joint_trajectory_controller/"
                                                          "monitor_cartesian_speed" };
static const std::string OPERATION_MODE_TOPIC{ "operation_mode" };
static const std::string GET_SPEED_OVERRIDE_SERVICE{ "get_speed_override" };

static constexpr uint32_t DEFAULT_QUEUE_SIZE{ 10 };

using namespace prbt_hardware_support;

/**
 * @brief Read necessary parameters, start and initialize the
 * prbt_hardware_support::OperationModeSetupExecutor
 */
// LCOV_EXCL_START
int main(int argc, char** argv)
{
  ros::init(argc, argv, "operation_mode_setup_executor");
  ros::NodeHandle nh;

  using std::placeholders::_1;
  pilz_utils::waitForService(MONITOR_CARTESIAN_SPEED_SERVICE);
  ros::ServiceClient monitor_cartesian_speed_srv = nh.serviceClient<std_srvs::SetBool>(MONITOR_CARTESIAN_SPEED_SERVICE);
  MonitorCartesianSpeedFunc monitor_cartesian_speed_func =
      std::bind(monitorCartesianSpeedSrv<ros::ServiceClient>, monitor_cartesian_speed_srv, _1);

  OperationModeSetupExecutor op_mode_executor(monitor_cartesian_speed_func);

  ros::Subscriber operation_mode_sub = nh.subscribe(
      OPERATION_MODE_TOPIC, DEFAULT_QUEUE_SIZE, &OperationModeSetupExecutor::updateOperationMode, &op_mode_executor);

  ros::ServiceServer speed_override_srv =
      nh.advertiseService(GET_SPEED_OVERRIDE_SERVICE, &OperationModeSetupExecutor::getSpeedOverride, &op_mode_executor);
  ros::spin();

  return EXIT_FAILURE;
}
// LCOV_EXCL_STOP
