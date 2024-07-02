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

#include <atomic>

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <pilz_msgs/GetSpeedOverride.h>
#include <prbt_hardware_support/FakeSpeedOverrideConfig.h>

std::atomic<double> SPEED_OVERRIDE{ 1.0 };

void dynamic_set_speed_override(prbt_hardware_support::FakeSpeedOverrideConfig& config, uint32_t level)
{
  ROS_INFO("Reconfigure Request: %f", config.speed_override);
  SPEED_OVERRIDE = config.speed_override;
}

bool getSpeedOverride(pilz_msgs::GetSpeedOverride::Request& /*req*/, pilz_msgs::GetSpeedOverride::Response& res)
{
  res.speed_override = SPEED_OVERRIDE;
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fake_speed_override");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("/prbt/get_speed_override", getSpeedOverride);

  dynamic_reconfigure::Server<prbt_hardware_support::FakeSpeedOverrideConfig> server;
  dynamic_reconfigure::Server<prbt_hardware_support::FakeSpeedOverrideConfig>::CallbackType f;

  f = boost::bind(&dynamic_set_speed_override, _1, _2);
  server.setCallback(f);

  ros::spin();
  return 0;
}