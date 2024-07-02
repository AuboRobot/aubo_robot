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

#include <functional>

#include <ros/ros.h>

#include <pilz_utils/wait_for_service.h>

#include <prbt_hardware_support/modbus_api_spec.h>
#include <prbt_hardware_support/modbus_adapter_run_permitted.h>
#include <prbt_hardware_support/filter_pipeline.h>
#include <prbt_hardware_support/modbus_api_spec.h>
#include <std_srvs/SetBool.h>

using namespace prbt_hardware_support;

static const std::string RUN_PERMITTED_SERVICE_NAME{ "run_permitted" };

// LCOV_EXCL_START
static void sendRunPermittedUpdate(ros::ServiceClient& run_permitted_service, const bool run_permitted)
{
  std_srvs::SetBool srv;
  srv.request.data = run_permitted;
  if (!run_permitted_service.call(srv))
  {
    ROS_ERROR_STREAM("RUN_PERMITTED service call failed");
  }

  if (!srv.response.success)
  {
    ROS_ERROR_STREAM(srv.response.message);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "modbus_adapter_run_permitted");
  ros::NodeHandle nh;

  using std::placeholders::_1;

  ModbusApiSpec api_spec{ nh };
  pilz_utils::waitForService(RUN_PERMITTED_SERVICE_NAME);
  ros::ServiceClient run_permitted_service = nh.serviceClient<std_srvs::SetBool>(RUN_PERMITTED_SERVICE_NAME);
  ModbusAdapterRunPermitted adapter_run_permitted(std::bind(sendRunPermittedUpdate, run_permitted_service, _1),
                                                  api_spec);
  FilterPipeline filter_pipeline(nh,
                                 std::bind(&ModbusAdapterRunPermitted::modbusMsgCallback, &adapter_run_permitted, _1));

  ros::spin();

  return EXIT_FAILURE;
}
// LCOV_EXCL_STOP
