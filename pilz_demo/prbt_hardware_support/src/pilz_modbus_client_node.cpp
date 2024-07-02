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

#include <algorithm>
#include <numeric>
#include <stdlib.h>

#include <ros/ros.h>

#include <pilz_utils/get_param.h>
#include <prbt_hardware_support/libmodbus_client.h>
#include <prbt_hardware_support/pilz_modbus_client.h>
#include <prbt_hardware_support/param_names.h>
#include <prbt_hardware_support/pilz_modbus_client_exception.h>
#include <prbt_hardware_support/modbus_topic_definitions.h>
#include <prbt_hardware_support/modbus_api_spec.h>

static constexpr int32_t MODBUS_CONNECTION_RETRIES_DEFAULT{ -1 };
static constexpr double MODBUS_CONNECTION_RETRY_TIMEOUT_S_DEFAULT{ 1.0 };
static constexpr int MODBUS_RESPONSE_TIMEOUT_MS{ 20 };

using namespace prbt_hardware_support;

/**
 * @brief Read requested parameters, start and initialize the prbt_hardware_support::PilzModbusClient
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "modbus_client_node");

  ros::NodeHandle pnh{ "~" };
  ros::NodeHandle nh;

  // LCOV_EXCL_START Simple parameter reading not analyzed

  std::string ip;
  int port;
  std::vector<unsigned short> registers_to_read;

  try
  {
    ip = pilz_utils::getParam<std::string>(pnh, PARAM_MODBUS_SERVER_IP_STR);
    port = pilz_utils::getParam<int>(pnh, PARAM_MODBUS_SERVER_PORT_STR);

    bool has_register_range_parameters =
        pnh.hasParam(PARAM_NUM_REGISTERS_TO_READ_STR) && pnh.hasParam(PARAM_INDEX_OF_FIRST_REGISTER_TO_READ_STR);
    if (has_register_range_parameters)
    {
      int num_registers_to_read = pilz_utils::getParam<int>(pnh, PARAM_NUM_REGISTERS_TO_READ_STR);
      int index_of_first_register = pilz_utils::getParam<int>(pnh, PARAM_INDEX_OF_FIRST_REGISTER_TO_READ_STR);
      registers_to_read = std::vector<unsigned short>(static_cast<unsigned long>(num_registers_to_read));
      std::iota(registers_to_read.begin(), registers_to_read.end(), index_of_first_register);
    }
    else
    {
      ROS_INFO_STREAM("Parameters for register range are not set. Will try to determine range from api spec...");
      ModbusApiSpec api_spec(nh);
      api_spec.getAllDefinedRegisters(registers_to_read);
      ROS_DEBUG("registers_to_read.size() %zu", registers_to_read.size());
    }
  }
  catch (const std::runtime_error& ex)
  {
    ROS_ERROR_STREAM(ex.what());
    return EXIT_FAILURE;
  }

  int32_t modbus_connection_retries{ MODBUS_CONNECTION_RETRIES_DEFAULT };
  pnh.param<int32_t>(PARAM_MODBUS_CONNECTION_RETRIES, modbus_connection_retries, MODBUS_CONNECTION_RETRIES_DEFAULT);

  double modbus_connection_retry_timeout_s{ MODBUS_CONNECTION_RETRY_TIMEOUT_S_DEFAULT };
  pnh.param<double>(PARAM_MODBUS_CONNECTION_RETRY_TIMEOUT, modbus_connection_retry_timeout_s,
                    MODBUS_CONNECTION_RETRY_TIMEOUT_S_DEFAULT);

  int response_timeout_ms;
  pnh.param<int>(PARAM_MODBUS_RESPONSE_TIMEOUT_STR, response_timeout_ms, MODBUS_RESPONSE_TIMEOUT_MS);

  std::string modbus_read_topic_name;
  nh.param<std::string>(PARAM_MODBUS_READ_TOPIC_NAME_STR, modbus_read_topic_name, TOPIC_MODBUS_READ);

  std::string modbus_write_service_name;
  nh.param<std::string>(PARAM_MODBUS_WRITE_SERVICE_NAME_STR, modbus_write_service_name, SERVICE_MODBUS_WRITE);

  // LCOV_EXCL_STOP

  prbt_hardware_support::PilzModbusClient modbus_client(
      pnh, registers_to_read, std::unique_ptr<LibModbusClient>(new LibModbusClient()),
      static_cast<unsigned int>(response_timeout_ms), modbus_read_topic_name, modbus_write_service_name);

  ROS_DEBUG_STREAM("Modbus client IP: " << ip << " | Port: " << port);
  std::ostringstream oss;
  if (!registers_to_read.empty())
  {
    std::copy(registers_to_read.begin(), registers_to_read.end() - 1, std::ostream_iterator<unsigned short>(oss, ","));
    oss << registers_to_read.back();
  }
  ROS_DEBUG_STREAM("Registers to read: " << oss.str());
  ROS_DEBUG_STREAM("Modbus response timeout: " << response_timeout_ms);
  ROS_DEBUG_STREAM("Modbus read topic: \"" << modbus_read_topic_name << "\"");
  ROS_DEBUG_STREAM("Modbus write service: \"" << modbus_write_service_name << "\"");

  bool res = modbus_client.init(ip.c_str(), static_cast<unsigned int>(port), modbus_connection_retries,
                                ros::Duration(modbus_connection_retry_timeout_s));

  ROS_DEBUG_STREAM("Connection with modbus server " << ip << ":" << port << " established");

  // LCOV_EXCL_START inside this main ignored, tested multiple times in the unittest
  if (!res)
  {
    ROS_ERROR_STREAM("Could not establish modbus connection with: "
                     << ip << ":" << port
                     << ". Make sure that your cables are connected properly and that "
                        "you have set the correct ip address and port.");

    return EXIT_FAILURE;
  }

  try
  {
    modbus_client.run();
  }
  catch (PilzModbusClientException& e)
  {
    ROS_ERROR_STREAM(e.what());
    return EXIT_FAILURE;
  }
  // LCOV_EXCL_STOP

  // If the client stop we don't want to kill this node since
  // with required=true this would kill all nodes and no STOP1 would be performed in case of a disconnect.
  ros::spin();

  return EXIT_SUCCESS;
}
