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

#ifndef OPERATION_MODE_SETUP_EXECUTOR_NODE_SERVICE_CALLS_H
#define OPERATION_MODE_SETUP_EXECUTOR_NODE_SERVICE_CALLS_H

#include <ros/ros.h>

#include <std_srvs/SetBool.h>

namespace prbt_hardware_support
{
template <class T>
static bool monitorCartesianSpeedSrv(T& srv_client, const bool active)
{
  std_srvs::SetBool srv_msg;
  srv_msg.request.data = active;
  bool call_success = srv_client.call(srv_msg);
  if (!call_success)
  {
    ROS_ERROR_STREAM("No success calling service: " << srv_client.getService());
  }
  return call_success;
}

}  // namespace prbt_hardware_support

#endif  // OPERATION_MODE_SETUP_EXECUTOR_NODE_SERVICE_CALLS_H
