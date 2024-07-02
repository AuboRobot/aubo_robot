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

#ifndef BRAKE_TEST_EXECUTOR_NODE_SERVICE_CALLS_H
#define BRAKE_TEST_EXECUTOR_NODE_SERVICE_CALLS_H

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include <prbt_hardware_support/BrakeTest.h>
#include <prbt_hardware_support/SendBrakeTestResult.h>

namespace prbt_hardware_support
{
template <class T>
static void triggerServiceCall(T& client)
{
  ROS_DEBUG_STREAM("Calling service: " << client.getService() << ")");
  std_srvs::Trigger srv;
  if (!client.call(srv))
  {
    ROS_WARN_STREAM("Calling service " << client.getService() << " failed.");
    return;
  }

  if (!srv.response.success)
  {
    ROS_WARN_STREAM("Execution of service " << client.getService()
                                            << " failed with error message\n:" << srv.response.message);
  }
}

template <class T>
static BrakeTest::Response executeBrakeTestCall(T& client)
{
  ROS_DEBUG_STREAM("Calling service: " << client.getService() << ")");
  BrakeTest srv;
  if (!client.call(srv))
  {
    ROS_WARN_STREAM("Calling service " << client.getService() << " failed.");
    BrakeTest::Response res;
    res.success = false;
    res.error_msg = "Failed to trigger brake test via service " + client.getService();
    res.error_code.value = BrakeTestErrorCodes::TRIGGER_BRAKETEST_SERVICE_FAILURE;
    return res;
  }
  return srv.response;
}

template <class T>
static bool sendBrakeTestResultCall(T& client, const bool brake_test_result)
{
  ROS_DEBUG_STREAM("Calling service: " << client.getService());
  SendBrakeTestResult srv;
  srv.request.result = brake_test_result;
  return client.call(srv);
}

}  // namespace prbt_hardware_support

#endif  // BRAKE_TEST_EXECUTOR_NODE_SERVICE_CALLS_H
