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

#ifndef PRBT_HARDWARE_SUPPORT_CANOPEN_CHAIN_NODE_MOCK_H
#define PRBT_HARDWARE_SUPPORT_CANOPEN_CHAIN_NODE_MOCK_H

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <ros/ros.h>
#include <canopen_chain_node/GetObject.h>
#include <canopen_chain_node/SetObject.h>

namespace prbt_hardware_support
{
class CANOpenChainNodeMock
{
public:
  /**
   * @brief Advertise get_object and set_object services for CANOpen objects and call setDefaultActions().
   */
  CANOpenChainNodeMock();

  /**
   * @brief Set default actions on all expected service calls.
   *
   * Does not set any expectations.
   */
  void setDefaultActions();

  /**
   * @brief Set expectations on all mock methods, that can be fullfilled in any case.
   *
   * Allows any number of calls on get_obj() and set_obj.
   */
  void expectAnything();

  /**
   * @brief Un-advertise services.
   */
  void shutdown();

  /**
   * @brief Un-advertise the set service.
   */

  void shutdownSetService();
  /**
   * @brief Un-advertise the get service.
   */
  void shutdownGetService();

public:
  MOCK_METHOD2(get_obj, bool(canopen_chain_node::GetObjectRequest&, canopen_chain_node::GetObjectResponse&));
  MOCK_METHOD2(set_obj, bool(canopen_chain_node::SetObjectRequest&, canopen_chain_node::SetObjectResponse&));

private:
  ros::NodeHandle nh_;
  ros::ServiceServer get_obj_serv_;
  ros::ServiceServer set_obj_serv_;
};

}  // namespace prbt_hardware_support

#endif
