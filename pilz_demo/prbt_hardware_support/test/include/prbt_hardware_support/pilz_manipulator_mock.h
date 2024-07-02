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

#ifndef PRBT_HARDWARE_SUPPORT_PILZ_MANIPULATOR_MOCK_H
#define PRBT_HARDWARE_SUPPORT_PILZ_MANIPULATOR_MOCK_H

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <std_srvs/Trigger.h>

/**
 * @brief Mocks the ROS Api of the manipulator relevant for stopping and holding.
 *
 * prbt_hardware_support::PilzRunPermittedModbusAdapterNode relies on 4 advertised services for its functionality.
 * For easy testing these services are advertised by this Gmock supported Mock class.
 *
 */
class ManipulatorMock
{
public:
  void advertiseHoldService(ros::NodeHandle nh, std::string hold_service_name);
  void advertiseUnholdService(ros::NodeHandle nh, std::string unhold_service_name);
  void advertiseHaltService(ros::NodeHandle nh, std::string halt_service_name);
  void advertiseRecoverService(ros::NodeHandle nh, std::string recover_service_name);

  void advertiseServices(ros::NodeHandle nh, std::string hold_service_name, std::string unhold_service_name,
                         std::string halt_service_name, std::string recover_service_name);

  MOCK_METHOD2(holdCb, bool(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp));
  MOCK_METHOD2(unholdCb, bool(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp));
  MOCK_METHOD2(haltCb, bool(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp));
  MOCK_METHOD2(recoverCb, bool(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp));

private:
  bool holdCb_internal(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);
  bool unholdCb_internal(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);
  bool haltCb_internal(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);
  bool recoverCb_internal(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);

  ros::ServiceServer hold_srv_;
  ros::ServiceServer unhold_srv_;
  ros::ServiceServer halt_srv_;
  ros::ServiceServer recover_srv_;
};

void ManipulatorMock::advertiseHoldService(ros::NodeHandle nh, std::string hold_service_name)
{
  hold_srv_ = nh.advertiseService(hold_service_name, &ManipulatorMock::holdCb_internal, this);
  ROS_DEBUG_NAMED("ManipulatorMock", "Advertised %s", hold_srv_.getService().c_str());
}

void ManipulatorMock::advertiseUnholdService(ros::NodeHandle nh, std::string unhold_service_name)
{
  unhold_srv_ = nh.advertiseService(unhold_service_name, &ManipulatorMock::unholdCb_internal, this);
  ROS_DEBUG_NAMED("ManipulatorMock", "Advertised %s", unhold_srv_.getService().c_str());
}

void ManipulatorMock::advertiseHaltService(ros::NodeHandle nh, std::string halt_service_name)
{
  halt_srv_ = nh.advertiseService(halt_service_name, &ManipulatorMock::haltCb_internal, this);
  ROS_DEBUG_NAMED("ManipulatorMock", "Advertised %s", halt_srv_.getService().c_str());
}

void ManipulatorMock::advertiseRecoverService(ros::NodeHandle nh, std::string recover_service_name)
{
  recover_srv_ = nh.advertiseService(recover_service_name, &ManipulatorMock::recoverCb_internal, this);
  ROS_DEBUG_NAMED("ManipulatorMock", "Advertised %s", recover_srv_.getService().c_str());
}

bool ManipulatorMock::holdCb_internal(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
  ROS_DEBUG_NAMED("ManipulatorMock", "Call to %s", hold_srv_.getService().c_str());
  return holdCb(req, resp);
}

bool ManipulatorMock::unholdCb_internal(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
  ROS_DEBUG_NAMED("ManipulatorMock", "Call to %s", unhold_srv_.getService().c_str());
  return unholdCb(req, resp);
}

bool ManipulatorMock::haltCb_internal(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
  ROS_DEBUG_NAMED("ManipulatorMock", "Call to %s", halt_srv_.getService().c_str());
  return haltCb(req, resp);
}

bool ManipulatorMock::recoverCb_internal(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
  ROS_DEBUG_NAMED("ManipulatorMock", "Call to %s", recover_srv_.getService().c_str());
  return recoverCb(req, resp);
}

void ManipulatorMock::advertiseServices(ros::NodeHandle nh, std::string hold_service_name,
                                        std::string unhold_service_name, std::string halt_service_name,
                                        std::string recover_service_name)
{
  advertiseHoldService(nh, hold_service_name);
  advertiseUnholdService(nh, unhold_service_name);
  advertiseHaltService(nh, halt_service_name);
  advertiseRecoverService(nh, recover_service_name);
}

#endif  // PRBT_HARDWARE_SUPPORT_PILZ_MANIPULATOR_MOCK_H
