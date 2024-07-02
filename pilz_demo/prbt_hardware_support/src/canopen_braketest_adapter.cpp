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

#include <algorithm>
#include <functional>
#include <sstream>

#include <XmlRpcValue.h>
#include <XmlRpcException.h>

#include <canopen_chain_node/GetObject.h>
#include <canopen_chain_node/SetObject.h>

#include <prbt_hardware_support/BrakeTest.h>
#include <prbt_hardware_support/BrakeTestErrorCodes.h>
#include <prbt_hardware_support/canopen_braketest_adapter.h>
#include <prbt_hardware_support/canopen_braketest_adapter_exception.h>

namespace prbt_hardware_support
{
static const std::string TRIGGER_BRAKETEST_SERVICE_NAME{ "/trigger_braketest" };
static const std::string CANOPEN_GETOBJECT_SERVICE_NAME{ "/prbt/driver/get_object" };
static const std::string CANOPEN_SETOBJECT_SERVICE_NAME{ "/prbt/driver/set_object" };
static const std::string CANOPEN_NODES_PARAMETER_NAME{ "/prbt/driver/nodes" };
static const std::string NODE_BRAKETEST_ENABLED_PARAMETER{ "braketest_required" };
static constexpr double WAIT_FOR_SERVICE_TIMEOUT_S{ 5.0 };

static const std::string GET_BRAKETEST_DURATION_OBJECT{ "2060sub1" };
static const std::string SET_START_BRAKETEST_OBJECT{ "2060sub2" };
static const std::string GET_BRAKETEST_STATUS_OBJECT{ "2060sub3" };

CANOpenBrakeTestAdapter::CANOpenBrakeTestAdapter(ros::NodeHandle& nh) : nh_(nh)
{
  brake_test_srv_ = nh_.advertiseService(nh_.getNamespace() + TRIGGER_BRAKETEST_SERVICE_NAME,
                                         &CANOpenBrakeTestAdapter::triggerBrakeTests, this);

  canopen_srv_get_client_ = nh_.serviceClient<canopen_chain_node::GetObject>(CANOPEN_GETOBJECT_SERVICE_NAME);
  canopen_srv_set_client_ = nh_.serviceClient<canopen_chain_node::SetObject>(CANOPEN_SETOBJECT_SERVICE_NAME);

  if (!canopen_srv_get_client_.waitForExistence(ros::Duration(WAIT_FOR_SERVICE_TIMEOUT_S)))
  {
    throw CANOpenBrakeTestAdapterException("Service " + canopen_srv_get_client_.getService() + " not available.");
  }
  if (!canopen_srv_set_client_.waitForExistence(ros::Duration(WAIT_FOR_SERVICE_TIMEOUT_S)))
  {
    throw CANOpenBrakeTestAdapterException("Service " + canopen_srv_set_client_.getService() + " not available.");
  }
}

ros::Duration CANOpenBrakeTestAdapter::getBrakeTestDuration(const std::string& node_name)
{
  canopen_chain_node::GetObject srv;
  srv.request.node = node_name;
  srv.request.object = GET_BRAKETEST_DURATION_OBJECT;
  srv.request.cached = false;

  ROS_INFO_STREAM("Call \"brake test duration\" service for \"" << node_name << "\"");
  if (!canopen_srv_get_client_.call(srv))
  {
    throw CANOpenBrakeTestAdapterException("CANopen service to request brake test duration failed",
                                           BrakeTestErrorCodes::GET_DURATION_FAILURE);
  }

  if (!srv.response.success)
  {
    throw CANOpenBrakeTestAdapterException(srv.response.message, BrakeTestErrorCodes::GET_DURATION_FAILURE);
  }

  ROS_INFO_STREAM("Brake test duration for node \"" << node_name << "\" is: " << srv.response.value << "ms");
  return ros::Duration(std::stoi(srv.response.value) / 1000, 0);
}

ros::Duration CANOpenBrakeTestAdapter::getMaximumBrakeTestDuration(const std::vector<std::string>& node_names)
{
  std::vector<ros::Duration> durations;
  std::transform(node_names.begin(), node_names.end(), std::back_inserter(durations),
                 std::bind(&CANOpenBrakeTestAdapter::getBrakeTestDuration, this, std::placeholders::_1));
  return *std::max_element(durations.begin(), durations.end());
}

void CANOpenBrakeTestAdapter::triggerBrakeTestForNode(const std::string& node_name)
{
  canopen_chain_node::SetObject srv;
  srv.request.node = node_name;
  srv.request.object = SET_START_BRAKETEST_OBJECT;
  srv.request.value = "1";  // Demand brake test
  srv.request.cached = false;

  ROS_INFO_STREAM("Call \"trigger brake test\" service for \"" << node_name << "\"");
  if (!canopen_srv_set_client_.call(srv))
  {
    throw CANOpenBrakeTestAdapterException("CANopen service for brake test execution failed",
                                           BrakeTestErrorCodes::START_BRAKE_TEST_FAILURE);
  }

  if (!srv.response.success)
  {
    throw CANOpenBrakeTestAdapterException(srv.response.message, BrakeTestErrorCodes::START_BRAKE_TEST_FAILURE);
  }
}

CANOpenBrakeTestAdapter::BrakeTestStatus
CANOpenBrakeTestAdapter::getBrakeTestStatusForNode(const std::string& node_name)
{
  canopen_chain_node::GetObject srv;
  srv.request.node = node_name;
  srv.request.object = GET_BRAKETEST_STATUS_OBJECT;
  srv.request.cached = false;

  ROS_INFO_STREAM("Call \"get status brake test\" service for \"" << node_name << "\"");
  if (!canopen_srv_get_client_.call(srv))
  {
    throw CANOpenBrakeTestAdapterException("CANopen service to request brake test status failed",
                                           BrakeTestErrorCodes::GET_STATUS_FAILURE);
  }

  if (!srv.response.success)
  {
    throw CANOpenBrakeTestAdapterException("Reading of CANopen to determine brake test status failed",
                                           BrakeTestErrorCodes::GET_STATUS_FAILURE);
  }

  BrakeTestStatus status;
  status.first = static_cast<int8_t>(srv.response.value.data()[0]);
  status.second = srv.response.message;
  return status;
}

void CANOpenBrakeTestAdapter::checkBrakeTestResultForNode(const std::string& node_name)
{
  BrakeTestStatus status{ getBrakeTestStatusForNode(node_name) };
  if (status.first != BrakeTestErrorCodes::STATUS_SUCCESS)
  {
    ROS_ERROR("Brake test for %s failed (Status: %d)", node_name.c_str(), status.first);
    throw CANOpenBrakeTestAdapterException(status.second, status.first);
  }
}

std::vector<std::string> CANOpenBrakeTestAdapter::getNodeNames()
{
  XmlRpc::XmlRpcValue rpc;
  if (!nh_.getParam(CANOPEN_NODES_PARAMETER_NAME, rpc))
  {
    throw CANOpenBrakeTestAdapterException("Could not read node names", BrakeTestErrorCodes::GET_NODE_NAMES_FAILURE);
  }

  std::vector<std::string> node_names;
  for (auto& rpci : rpc)
  {
    auto node_name = rpci.first.c_str();

    try
    {
      auto braketest =
          rpci.second.hasMember(NODE_BRAKETEST_ENABLED_PARAMETER) && rpci.second[NODE_BRAKETEST_ENABLED_PARAMETER];

      if (braketest)
      {
        node_names.push_back(node_name);
        ROS_INFO_STREAM("Braketest required for: " << node_name);
      }
      else
      {
        ROS_INFO_STREAM("No Braketest required for: " << node_name);
      }
    }
    catch (const XmlRpc::XmlRpcException& e)
    {
      ROS_ERROR_STREAM("Wrong type on parameter " << NODE_BRAKETEST_ENABLED_PARAMETER << e.getMessage());

      throw CANOpenBrakeTestAdapterException("Could not read parameter " + NODE_BRAKETEST_ENABLED_PARAMETER + " of " +
                                                 node_name,
                                             BrakeTestErrorCodes::GET_NODE_NAMES_FAILURE);
    }
  }
  return node_names;
}

bool CANOpenBrakeTestAdapter::triggerBrakeTests(BrakeTest::Request& /*req*/, BrakeTest::Response& response)
{
  try
  {
    auto node_names = getNodeNames();

    // should be executed prior to brake tests
    auto max_duration = getMaximumBrakeTestDuration(node_names);

    for (const auto& node_name : node_names)
    {
      ROS_INFO_STREAM("Perform brake test for node \"" << node_name << "\"...");
      triggerBrakeTestForNode(node_name);
    }
    max_duration.sleep();

    std::for_each(node_names.begin(), node_names.end(),
                  std::bind(&CANOpenBrakeTestAdapter::checkBrakeTestResultForNode, this, std::placeholders::_1));
  }
  catch (const CANOpenBrakeTestAdapterException& ex)
  {
    ROS_ERROR_STREAM("Brake test failed: " << ex.what());
    response.success = false;
    response.error_code.value = ex.getErrorValue();
    response.error_msg = ex.what();
    return true;
  }

  response.success = true;
  return true;
}
}  // namespace prbt_hardware_support
