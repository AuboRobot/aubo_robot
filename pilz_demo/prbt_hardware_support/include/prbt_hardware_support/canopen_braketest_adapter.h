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

#ifndef PRBT_HARDWARE_SUPPORT_CANOPEN_BRAKETEST_ADAPTER_H
#define PRBT_HARDWARE_SUPPORT_CANOPEN_BRAKETEST_ADAPTER_H

#include <ros/ros.h>

#include <string>
#include <utility>

#include <canopen_chain_node/GetObject.h>
#include <canopen_chain_node/SetObject.h>

#include <prbt_hardware_support/BrakeTest.h>

namespace prbt_hardware_support
{
/**
 * @brief Executes the brake test for all joints. A brake test is triggered
 * via service call.
 */
class CANOpenBrakeTestAdapter
{
public:
  CANOpenBrakeTestAdapter(ros::NodeHandle& nh);

private:
  using BrakeTestStatus = std::pair<int8_t, std::string>;

private:
  bool triggerBrakeTests(BrakeTest::Request& req, BrakeTest::Response& response);
  void triggerBrakeTestForNode(const std::string& node_name);
  BrakeTestStatus getBrakeTestStatusForNode(const std::string& node_name);
  ros::Duration getBrakeTestDuration(const std::string& node_name);
  void checkBrakeTestResultForNode(const std::string& node_name);
  std::vector<std::string> getNodeNames();
  ros::Duration getMaximumBrakeTestDuration(const std::vector<std::string>& node_names);

private:
  ros::NodeHandle nh_;
  //! Service which triggers brake tests for all joints.
  ros::ServiceServer brake_test_srv_;

  ros::ServiceClient canopen_srv_get_client_;
  ros::ServiceClient canopen_srv_set_client_;
};

}  // namespace prbt_hardware_support
#endif  // PRBT_HARDWARE_SUPPORT_CANOPEN_BRAKETEST_ADAPTER_H
