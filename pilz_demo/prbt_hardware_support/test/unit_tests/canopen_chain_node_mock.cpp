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

#include <gmock/gmock.h>

#include <ros/ros.h>

#include <prbt_hardware_support/canopen_chain_node_mock.h>

namespace prbt_hardware_support
{
static const std::string CANOPEN_GETOBJECT_SERVICE_NAME{ "/prbt/driver/get_object" };
static const std::string CANOPEN_SETOBJECT_SERVICE_NAME{ "/prbt/driver/set_object" };

static const std::string BRAKE_TEST_DURATION_OBJECT_INDEX{ "2060sub1" };
static const std::string START_BRAKE_TEST_OBJECT_INDEX{ "2060sub2" };
static const std::string BRAKE_TEST_STATUS_OBJECT_INDEX{ "2060sub3" };

using canopen_chain_node::GetObjectRequest;
using canopen_chain_node::GetObjectResponse;
using canopen_chain_node::SetObjectRequest;
using canopen_chain_node::SetObjectResponse;

CANOpenChainNodeMock::CANOpenChainNodeMock()
{
  get_obj_serv_ = nh_.advertiseService(CANOPEN_GETOBJECT_SERVICE_NAME, &CANOpenChainNodeMock::get_obj, this);
  set_obj_serv_ = nh_.advertiseService(CANOPEN_SETOBJECT_SERVICE_NAME, &CANOpenChainNodeMock::set_obj, this);

  setDefaultActions();
}

void CANOpenChainNodeMock::setDefaultActions()
{
  using ::testing::_;
  using ::testing::DoAll;
  using ::testing::Field;
  using ::testing::Return;
  using ::testing::SetArgReferee;

  GetObjectResponse duration_resp;
  duration_resp.success = true;
  duration_resp.value = "1";

  SetObjectResponse start_resp;
  start_resp.success = true;

  GetObjectResponse status_resp;
  status_resp.success = true;
  status_resp.value = "\x02";

  // Set response for service calls getting the brake_test_duration object
  ON_CALL(*this, get_obj(Field(&GetObjectRequest::object, BRAKE_TEST_DURATION_OBJECT_INDEX), _))
      .WillByDefault(DoAll(SetArgReferee<1>(duration_resp), Return(true)));

  // Set response for service calls setting the start_brake_test object
  ON_CALL(*this, set_obj(Field(&SetObjectRequest::object, START_BRAKE_TEST_OBJECT_INDEX), _))
      .WillByDefault(DoAll(SetArgReferee<1>(start_resp), Return(true)));

  // Set response for service calls getting the brake_test_status object
  ON_CALL(*this, get_obj(Field(&GetObjectRequest::object, BRAKE_TEST_STATUS_OBJECT_INDEX), _))
      .WillByDefault(DoAll(SetArgReferee<1>(status_resp), Return(true)));
}

void CANOpenChainNodeMock::expectAnything()
{
  using ::testing::_;
  using ::testing::AnyNumber;
  using ::testing::Field;

  EXPECT_CALL(*this, get_obj(_, _)).Times(AnyNumber());
  EXPECT_CALL(*this, set_obj(_, _)).Times(AnyNumber());

  // Gripper should not be called
  EXPECT_CALL(*this, get_obj(Field(&GetObjectRequest::node, "gripper_joint"), _)).Times(0);

  EXPECT_CALL(*this, set_obj(Field(&SetObjectRequest::node, "gripper_joint"), _)).Times(0);
}

void CANOpenChainNodeMock::shutdown()
{
  get_obj_serv_.shutdown();
  set_obj_serv_.shutdown();
}

void CANOpenChainNodeMock::shutdownGetService()
{
  get_obj_serv_.shutdown();
}

void CANOpenChainNodeMock::shutdownSetService()
{
  set_obj_serv_.shutdown();
}

}  // namespace prbt_hardware_support
