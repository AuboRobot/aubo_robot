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
#include <prbt_hardware_support/canopen_braketest_adapter.h>

/**
 * @brief Provides service to execute a braketest
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "canopen_braketest_adapter_node");
  ros::NodeHandle nh{ "~" };

  prbt_hardware_support::CANOpenBrakeTestAdapter brake_test_adapter(nh);

  ros::spin();

  return EXIT_FAILURE;
}
