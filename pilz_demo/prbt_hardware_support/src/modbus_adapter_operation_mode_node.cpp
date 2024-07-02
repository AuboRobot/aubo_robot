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

#include <ros/ros.h>

#include <prbt_hardware_support/modbus_adapter_operation_mode.h>
#include <prbt_hardware_support/modbus_api_spec.h>

/**
 * @brief Starts a modbus operation mode adapter and runs it until a failure occurs.
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "modbus_adapter_operation_mode");
  ros::NodeHandle pnh{ "~" };
  ros::NodeHandle nh{};

  prbt_hardware_support::ModbusApiSpec api_spec(nh);

  prbt_hardware_support::ModbusAdapterOperationMode adapter_operation_mode(pnh, api_spec);

  ros::spin();

  return EXIT_FAILURE;
}
