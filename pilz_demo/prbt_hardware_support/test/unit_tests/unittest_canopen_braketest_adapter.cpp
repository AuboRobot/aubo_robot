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

#include <functional>
#include <memory>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <ros/ros.h>
#include <canopen_chain_node/GetObject.h>
#include <canopen_chain_node/SetObject.h>

#include <prbt_hardware_support/canopen_braketest_adapter.h>
#include <prbt_hardware_support/canopen_braketest_adapter_exception.h>
#include <prbt_hardware_support/BrakeTest.h>
#include <prbt_hardware_support/canopen_chain_node_mock.h>

namespace canopen_braketest_adapter_test
{
using namespace prbt_hardware_support;
using namespace testing;

using canopen_chain_node::GetObjectRequest;
using canopen_chain_node::GetObjectResponse;
using canopen_chain_node::SetObjectRequest;
using canopen_chain_node::SetObjectResponse;

static const std::string BRAKE_TEST_SERVICE_NAME{ "/prbt/braketest_adapter_node/trigger_braketest" };

static const std::string BRAKE_TEST_DURATION_OBJECT_INDEX{ "2060sub1" };
static const std::string START_BRAKE_TEST_OBJECT_INDEX{ "2060sub2" };
static const std::string BRAKE_TEST_STATUS_OBJECT_INDEX{ "2060sub3" };

static const std::string NODE_NAMES_PARAMETER_NAME{ "/prbt/driver/nodes" };
static const std::string BRAKETEST_REQUIRED_NAME{ "braketest_required" };
static const std::string NODE_NAMES_PREFIX{ "prbt_joint_" };
static const std::vector<size_t> NODE_TEST_SET{ { 0, 2, 5 } };

#define DEFAULT_SETUP                                                                                                  \
  CANOpenChainNodeMock canopen_chain_node;                                                                             \
  ros::NodeHandle nh_adapter("/prbt/braketest_adapter_node");                                                          \
  prbt_hardware_support::CANOpenBrakeTestAdapter canopen_braketest_adapter(nh_adapter);                                \
  ros::ServiceClient brake_test_srv_client = nh_.serviceClient<BrakeTest>(BRAKE_TEST_SERVICE_NAME);                    \
                                                                                                                       \
  ASSERT_TRUE(brake_test_srv_client.exists()) << "Brake test service not available.";

class CanOpenBraketestAdapterTest : public Test
{
protected:
  ros::NodeHandle nh_;
};

/**
 * @tests{Execute_BrakeTest_mechanism,
 * Test increases function coverage by ensuring that all Dtor variants
 * are called.
 * }
 */
TEST_F(CanOpenBraketestAdapterTest, testCANOpenBrakeTestAdapterExceptionDtor)
{
  {
    std::shared_ptr<CANOpenBrakeTestAdapterException> ex{ new CANOpenBrakeTestAdapterException("Test msg") };
  }
}

/**
 * @tests{Execute_BrakeTest_mechanism,
 *  Test execution of brake tests without the CAN get service.
 * }
 *
 * Test Sequence:
 *  1. Shutdown CAN get service mock.
 *  2. Call brake test service.
 *
 * Expected Results:
 *  1. -
 *  2. Brake test service responds with correct error case.
 */
TEST_F(CanOpenBraketestAdapterTest, testBrakeTestServiceWithoutCANGetService)
{
  CANOpenChainNodeMock canopen_chain_node;
  canopen_chain_node.shutdownGetService();

  EXPECT_THROW(prbt_hardware_support::CANOpenBrakeTestAdapter canopen_braketest_adapter(nh_),
               CANOpenBrakeTestAdapterException);
}

/**
 * @tests{Execute_BrakeTest_mechanism,
 *  Test execution of brake tests without the CAN set service.
 * }
 *
 * Test Sequence:
 *  1. Shutdown CAN set service mock.
 *  2. Call brake test service.
 *
 * Expected Results:
 *  1. -
 *  2. Brake test service responds with correct error case.
 */
TEST_F(CanOpenBraketestAdapterTest, testBrakeTestServiceWithoutCANSetService)
{
  CANOpenChainNodeMock canopen_chain_node;
  canopen_chain_node.shutdownSetService();

  EXPECT_THROW(prbt_hardware_support::CANOpenBrakeTestAdapter canopen_braketest_adapter(nh_),
               CANOpenBrakeTestAdapterException);
}

/**
 * @tests{Execute_BrakeTest_mechanism,
 *  Test execution of brake tests without node names on the parameter server.
 * }
 *
 * Test Sequence:
 *  1. Delete node names parameter.
 *  2. Call brake test service.
 *  3. Restore node names parameter.
 *
 * Expected Results:
 *  1. -
 *  2. Brake test service responds with correct error case.
 *  3. -
 */
TEST_F(CanOpenBraketestAdapterTest, testBrakeTestServiceWithoutNodeParameters)
{
  DEFAULT_SETUP

  /**********
   * Step 1 *
   **********/
  XmlRpc::XmlRpcValue rpc;
  ASSERT_TRUE(nh_.getParam(NODE_NAMES_PARAMETER_NAME, rpc));
  nh_.deleteParam(NODE_NAMES_PARAMETER_NAME);

  /**********
   * Step 3 *
   **********/
  BrakeTest srv;
  EXPECT_TRUE(brake_test_srv_client.call(srv)) << "Failed to call brake test service.";
  EXPECT_FALSE(srv.response.success);
  EXPECT_EQ(BrakeTestErrorCodes::GET_NODE_NAMES_FAILURE, srv.response.error_code.value);

  nh_.setParam(NODE_NAMES_PARAMETER_NAME, rpc);
}

/**
 * @tests{Execute_BrakeTest_mechanism,
 *  Test execution of brake tests with an error type in the node
 *  braketest_required parameter.
 * }
 *
 * Test Sequence:
 *  1. Modify the parameter by setting a int (bool is expected)
 *  2. Call brake test service.
 *  3. Restore node names parameter.
 *
 * Expected Results:
 *  1. -
 *  2. Brake test service responds with correct error case.
 *  3. -
 */
TEST_F(CanOpenBraketestAdapterTest, testBrakeTestServiceWithNodeParametersTypeError)
{
  DEFAULT_SETUP

  /**********
   * Step 1 *
   **********/
  XmlRpc::XmlRpcValue rpc;
  ASSERT_TRUE(nh_.getParam(NODE_NAMES_PARAMETER_NAME, rpc));
  auto param_modified_name = NODE_NAMES_PARAMETER_NAME + "/" + NODE_NAMES_PREFIX + "1/" + BRAKETEST_REQUIRED_NAME;
  nh_.setParam(param_modified_name, 99);

  /**********
   * Step 3 *
   **********/
  BrakeTest srv;
  EXPECT_TRUE(brake_test_srv_client.call(srv)) << "Failed to call brake test service.";
  EXPECT_FALSE(srv.response.success);
  EXPECT_EQ(BrakeTestErrorCodes::GET_NODE_NAMES_FAILURE, srv.response.error_code.value);

  nh_.setParam(NODE_NAMES_PARAMETER_NAME, rpc);
}

/**
 * @tests{Execute_BrakeTest_mechanism,
 *  Test execution of brake tests when the service call getting
 *  the brake_test_duration object fails.
 * }
 *
 * This is repeated for the first and the last node and one in between.
 *
 * Test Sequence:
 *  1. Set expectations on CANOpen mock object. Let service call getting the brake_test_duration object fail for the
 *     selected node.
 *  2. Call brake test service.
 *  3. Verify and clear expectations.
 *
 * Expected Results:
 *  1. -
 *  2. The brake test service responds with correct error case.
 *  3. There is:
 *     - at least one read request on the brake_test_duration object for the selected node,
 *     - no write request for the selected node.
 */
TEST_F(CanOpenBraketestAdapterTest, testGetBrakeTestDurationServiceCallFailure)
{
  DEFAULT_SETUP

  for (size_t node_index : NODE_TEST_SET)
  {
    std::string node{ NODE_NAMES_PREFIX + std::to_string(node_index + 1) };

    /**********
     * Step 1 *
     **********/
    // Avoid unmatched expectations
    canopen_chain_node.expectAnything();

    EXPECT_CALL(canopen_chain_node, get_obj(AllOf(Field(&GetObjectRequest::node, node),
                                                  Field(&GetObjectRequest::object, BRAKE_TEST_DURATION_OBJECT_INDEX)),
                                            _))
        .Times(AtLeast(1))
        .WillRepeatedly(Return(false));

    EXPECT_CALL(canopen_chain_node, set_obj(AllOf(Field(&SetObjectRequest::node, node),
                                                  Field(&SetObjectRequest::object, START_BRAKE_TEST_OBJECT_INDEX)),
                                            _))
        .Times(0);

    /**********
     * Step 2 *
     **********/
    BrakeTest srv;
    EXPECT_TRUE(brake_test_srv_client.call(srv)) << "Failed to call brake test service.";
    EXPECT_FALSE(srv.response.success);
    EXPECT_EQ(BrakeTestErrorCodes::GET_DURATION_FAILURE, srv.response.error_code.value);

    /**********
     * Step 3 *
     **********/
    ASSERT_TRUE(Mock::VerifyAndClearExpectations(&canopen_chain_node));
  }
}

/**
 * @tests{Execute_BrakeTest_mechanism,
 *  Test execution of brake tests when the service call getting
 *  the brake_test_duration object responds an error.
 * }
 *
 * This is repeated for the first and the last node and one in between.
 *
 * Test Sequence:
 *  1. Set expectations on CANOpen mock object. Let service call getting the brake_test_duration object respond an error
 *     for the selected node.
 *  2. Call brake test service.
 *  3. Verify and clear expectations.
 *
 * Expected Results:
 *  1. -
 *  2. The brake test service responds with correct error case.
 *  3. There is:
 *     - at least one read request on the brake_test_duration object for the selected node,
 *     - no write request for the selected node.
 */
TEST_F(CanOpenBraketestAdapterTest, testGetBrakeTestDurationServiceResponseFailure)
{
  DEFAULT_SETUP

  GetObjectResponse duration_resp;
  duration_resp.success = false;

  for (size_t node_index : NODE_TEST_SET)
  {
    std::string node{ NODE_NAMES_PREFIX + std::to_string(node_index + 1) };

    /**********
     * Step 1 *
     **********/
    // Avoid unmatched expectations
    canopen_chain_node.expectAnything();

    EXPECT_CALL(canopen_chain_node, get_obj(AllOf(Field(&GetObjectRequest::node, node),
                                                  Field(&GetObjectRequest::object, BRAKE_TEST_DURATION_OBJECT_INDEX)),
                                            _))
        .Times(AtLeast(1))
        .WillRepeatedly(DoAll(SetArgReferee<1>(duration_resp), Return(true)));

    EXPECT_CALL(canopen_chain_node, set_obj(AllOf(Field(&SetObjectRequest::node, node),
                                                  Field(&SetObjectRequest::object, START_BRAKE_TEST_OBJECT_INDEX)),
                                            _))
        .Times(0);

    /**********
     * Step 2 *
     **********/
    BrakeTest srv;
    EXPECT_TRUE(brake_test_srv_client.call(srv)) << "Failed to call brake test service.";
    EXPECT_FALSE(srv.response.success);
    EXPECT_EQ(BrakeTestErrorCodes::GET_DURATION_FAILURE, srv.response.error_code.value);

    /**********
     * Step 3 *
     **********/
    ASSERT_TRUE(Mock::VerifyAndClearExpectations(&canopen_chain_node));
  }
}

/**
 * @tests{Execute_BrakeTest_mechanism,
 *  Test successful execution of brake tests.
 * }
 *
 * This is repeated for the first and the last node and one in between.
 *
 * Test Sequence:
 *  1. Set expectations on CANOpen mock object.
 *  2. Call brake test service.
 *  3. Verify and clear expectations.
 *
 * Expected Results:
 *  1. -
 *  2. The brake test service responds with success.
 *  3. There is:
 *     - at least one read request on brake_test_duration object for the selected node,
 *     - exactly one write request for the selected node.
 */
TEST_F(CanOpenBraketestAdapterTest, testStartBrakeTestServiceCallSuccess)
{
  DEFAULT_SETUP

  /**********
   * Step 1 *
   **********/
  canopen_chain_node.setDefaultActions();

  /**********
   * Step 2 *
   **********/
  BrakeTest srv;
  EXPECT_TRUE(brake_test_srv_client.call(srv)) << "Failed to call brake test service.";
  EXPECT_TRUE(srv.response.success);
}

/**
 * @tests{Execute_BrakeTest_mechanism,
 *  Test execution of brake tests when the service call setting
 *  the start_brake_test object fails.
 * }
 *
 * This is repeated for the first and the last node and one in between.
 *
 * Test Sequence:
 *  1. Set expectations on CANOpen mock object. Let the service call getting the start_brake_test object fail for the
 *     selected node.
 *  2. Call brake test service.
 *  3. Verify and clear expectations.
 *
 * Expected Results:
 *  1. -
 *  2. The brake test service responds with correct error case.
 *  3. There is:
 *     - at least one read request on brake_test_duration object for the selected node,
 *     - exactly one write request for the selected node.
 */
TEST_F(CanOpenBraketestAdapterTest, testStartBrakeTestServiceCallFailure)
{
  DEFAULT_SETUP

  for (size_t node_index : NODE_TEST_SET)
  {
    std::string node{ NODE_NAMES_PREFIX + std::to_string(node_index + 1) };

    /**********
     * Step 1 *
     **********/
    // Avoid unmatched expectations
    canopen_chain_node.expectAnything();

    EXPECT_CALL(canopen_chain_node, get_obj(AllOf(Field(&GetObjectRequest::node, node),
                                                  Field(&GetObjectRequest::object, BRAKE_TEST_DURATION_OBJECT_INDEX)),
                                            _))
        .Times(AtLeast(1));

    EXPECT_CALL(canopen_chain_node, set_obj(AllOf(Field(&SetObjectRequest::node, node),
                                                  Field(&SetObjectRequest::object, START_BRAKE_TEST_OBJECT_INDEX)),
                                            _))
        .WillOnce(Return(false));

    /**********
     * Step 2 *
     **********/
    BrakeTest srv;
    EXPECT_TRUE(brake_test_srv_client.call(srv)) << "Failed to call brake test service.";
    EXPECT_FALSE(srv.response.success);
    EXPECT_EQ(BrakeTestErrorCodes::START_BRAKE_TEST_FAILURE, srv.response.error_code.value);

    /**********
     * Step 3 *
     **********/
    Mock::VerifyAndClearExpectations(&canopen_chain_node);
  }
}

/**
 * @tests{Execute_BrakeTest_mechanism,
 *  Test execution of brake tests when the service call setting
 *  the start_brake_test object responds an error.
 * }
 *
 * This is repeated for the first and the last node and one in between.
 *
 * Test Sequence:
 *  1. Set expectations on CANOpen mock object. Let the service call getting the start_brake_test object respond an
 * error for the selected node.
 *  2. Call brake test service.
 *  3. Verify and clear expectations.
 *
 * Expected Results:
 *  1. -
 *  2. The brake test service responds with correct error case.
 *  3. There is:
 *     - at least one read request on brake_test_duration object for the selected node,
 *     - exactly one write request for the selected node.
 */
TEST_F(CanOpenBraketestAdapterTest, testStartBrakeTestServiceResponseFailure)
{
  DEFAULT_SETUP

  SetObjectResponse start_resp;
  start_resp.success = false;

  for (size_t node_index : NODE_TEST_SET)
  {
    std::string node{ NODE_NAMES_PREFIX + std::to_string(node_index + 1) };

    /**********
     * Step 1 *
     **********/
    // Avoid unmatched expectations
    canopen_chain_node.expectAnything();

    EXPECT_CALL(canopen_chain_node, get_obj(AllOf(Field(&GetObjectRequest::node, node),
                                                  Field(&GetObjectRequest::object, BRAKE_TEST_DURATION_OBJECT_INDEX)),
                                            _))
        .Times(AtLeast(1));

    EXPECT_CALL(canopen_chain_node, set_obj(AllOf(Field(&SetObjectRequest::node, node),
                                                  Field(&SetObjectRequest::object, START_BRAKE_TEST_OBJECT_INDEX)),
                                            _))
        .WillOnce(DoAll(SetArgReferee<1>(start_resp), Return(true)));

    /**********
     * Step 2 *
     **********/
    BrakeTest srv;
    EXPECT_TRUE(brake_test_srv_client.call(srv)) << "Failed to call brake test service.";
    EXPECT_FALSE(srv.response.success);
    EXPECT_EQ(BrakeTestErrorCodes::START_BRAKE_TEST_FAILURE, srv.response.error_code.value);

    /**********
     * Step 3 *
     **********/
    Mock::VerifyAndClearExpectations(&canopen_chain_node);
  }
}

/**
 * @tests{Execute_BrakeTest_mechanism,
 *  Test execution of brake tests when the service call getting
 *  the brake_test_status object fails.
 * }
 *
 * This is repeated for the first and the last node and one in between.
 *
 * Test Sequence:
 *  1. Set expectations on CANOpen mock object. Let all services respond success, but let the service call getting the
 *     brake_test_status object fail for a selected node.
 *  2. Call brake test service.
 *  3. Verify and clear expectations.
 *
 * Expected Results:
 *  1. -
 *  2. The brake test service responds with correct error case.
 *  3. There is at least one read request on brake_test_status object for the selected node.
 */
TEST_F(CanOpenBraketestAdapterTest, testBrakeTestStatusServiceCallFailure)
{
  DEFAULT_SETUP

  for (size_t node_index : NODE_TEST_SET)
  {
    std::string node{ NODE_NAMES_PREFIX + std::to_string(node_index + 1) };

    /**********
     * Step 1 *
     **********/
    // Avoid unmatched expectations
    canopen_chain_node.expectAnything();

    EXPECT_CALL(canopen_chain_node, get_obj(AllOf(Field(&GetObjectRequest::node, node),
                                                  Field(&GetObjectRequest::object, BRAKE_TEST_STATUS_OBJECT_INDEX)),
                                            _))
        .Times(AtLeast(1))
        .WillRepeatedly(Return(false));

    /**********
     * Step 2 *
     **********/
    BrakeTest srv;
    EXPECT_TRUE(brake_test_srv_client.call(srv)) << "Failed to call brake test service.";
    EXPECT_FALSE(srv.response.success);
    EXPECT_EQ(BrakeTestErrorCodes::GET_STATUS_FAILURE, srv.response.error_code.value);

    /**********
     * Step 3 *
     **********/
    Mock::VerifyAndClearExpectations(&canopen_chain_node);
  }
}

/**
 * @tests{Execute_BrakeTest_mechanism,
 *  Test execution of brake tests when the service call getting
 *  the brake_test_status object responds an error.
 * }
 *
 * This is repeated for the first and the last node and one in between.
 *
 * Test Sequence:
 *  1. Set expectations on CANOpen mock object. Let the service call getting the brake_test_status object respond
 *     an error for the selected node.
 *  2. Call brake test service.
 *  3. Verify and clear expectations.
 *
 * Expected Results:
 *  1. -
 *  2. The brake test service responds with correct error case.
 *  3. There is at least one read request on brake_test_status object for the selected node.
 */
TEST_F(CanOpenBraketestAdapterTest, testBrakeTestStatusServiceResponseFailure)
{
  DEFAULT_SETUP

  GetObjectResponse status_resp;
  status_resp.success = false;

  for (size_t node_index : NODE_TEST_SET)
  {
    std::string node{ NODE_NAMES_PREFIX + std::to_string(node_index + 1) };

    /**********
     * Step 1 *
     **********/
    // Avoid unmatched expectations
    canopen_chain_node.expectAnything();

    EXPECT_CALL(canopen_chain_node, get_obj(AllOf(Field(&GetObjectRequest::node, node),
                                                  Field(&GetObjectRequest::object, BRAKE_TEST_STATUS_OBJECT_INDEX)),
                                            _))
        .Times(AtLeast(1))
        .WillRepeatedly(DoAll(SetArgReferee<1>(status_resp), Return(true)));

    /**********
     * Step 2 *
     **********/
    BrakeTest srv;
    EXPECT_TRUE(brake_test_srv_client.call(srv)) << "Failed to call brake test service.";
    EXPECT_FALSE(srv.response.success);
    EXPECT_EQ(BrakeTestErrorCodes::GET_STATUS_FAILURE, srv.response.error_code.value);

    /**********
     * Step 3 *
     **********/
    Mock::VerifyAndClearExpectations(&canopen_chain_node);
  }
}

/**
 * @tests{Execute_BrakeTest_mechanism,
 *  Test execution of brake tests when the brake_test_status is unknown.
 * }
 *
 * This is repeated for the first and the last node and one in between.
 *
 * Test Sequence:
 *  1. Set expectations on CANOpen mock object. Let the service call getting the brake_test_status object respond
 *     status unknown for the selected node.
 *  2. Call brake test service.
 *  3. Verify and clear expectations.
 *
 * Expected Results:
 *  1. -
 *  2. The brake test service responds with correct error case.
 *  3. There is at least one read request on brake_test_status object for the selected node.
 */
TEST_F(CanOpenBraketestAdapterTest, testBrakeTestStatusUnknown)
{
  DEFAULT_SETUP

  GetObjectResponse status_resp;
  status_resp.success = true;
  status_resp.value = "\0";

  for (size_t node_index : NODE_TEST_SET)
  {
    std::string node{ NODE_NAMES_PREFIX + std::to_string(node_index + 1) };

    /**********
     * Step 1 *
     **********/
    // Avoid unmatched expectations
    canopen_chain_node.expectAnything();

    EXPECT_CALL(canopen_chain_node, get_obj(AllOf(Field(&GetObjectRequest::node, node),
                                                  Field(&GetObjectRequest::object, BRAKE_TEST_STATUS_OBJECT_INDEX)),
                                            _))
        .Times(AtLeast(1))
        .WillRepeatedly(DoAll(SetArgReferee<1>(status_resp), Return(true)));

    /**********
     * Step 2 *
     **********/
    BrakeTest srv;
    EXPECT_TRUE(brake_test_srv_client.call(srv)) << "Failed to call brake test service.";
    EXPECT_FALSE(srv.response.success);
    EXPECT_EQ(BrakeTestErrorCodes::STATUS_UNKNOWN, srv.response.error_code.value);

    /**********
     * Step 3 *
     **********/
    Mock::VerifyAndClearExpectations(&canopen_chain_node);
  }
}

/**
 * @tests{Execute_BrakeTest_mechanism,
 *  Test execution of brake tests when the brake_test_status is performed.
 * }
 *
 * This is repeated for the first and the last node and one in between.
 *
 * Test Sequence:
 *  1. Set expectations on CANOpen mock object. Let the service call getting the brake_test_status object respond
 *     status performed for the selected node.
 *  2. Call brake test service.
 *  3. Verify and clear expectations.
 *
 * Expected Results:
 *  1. -
 *  2. The brake test service responds with correct error case.
 *  3. There is at least one read request on brake_test_status object for the selected node.
 */
TEST_F(CanOpenBraketestAdapterTest, testBrakeTestStatusPerformed)
{
  DEFAULT_SETUP

  GetObjectResponse status_resp;
  status_resp.success = true;
  status_resp.value = "\x01";

  for (size_t node_index : NODE_TEST_SET)
  {
    std::string node{ NODE_NAMES_PREFIX + std::to_string(node_index + 1) };

    /**********
     * Step 1 *
     **********/
    // Avoid unmatched expectations
    canopen_chain_node.expectAnything();

    EXPECT_CALL(canopen_chain_node, get_obj(AllOf(Field(&GetObjectRequest::node, node),
                                                  Field(&GetObjectRequest::object, BRAKE_TEST_STATUS_OBJECT_INDEX)),
                                            _))
        .Times(AtLeast(1))
        .WillRepeatedly(DoAll(SetArgReferee<1>(status_resp), Return(true)));

    /**********
     * Step 2 *
     **********/
    BrakeTest srv;
    EXPECT_TRUE(brake_test_srv_client.call(srv)) << "Failed to call brake test service.";
    EXPECT_FALSE(srv.response.success);
    EXPECT_EQ(BrakeTestErrorCodes::STATUS_PERFORMING, srv.response.error_code.value);

    /**********
     * Step 3 *
     **********/
    Mock::VerifyAndClearExpectations(&canopen_chain_node);
  }
}

/**
 * @tests{Execute_BrakeTest_mechanism,
 *  Test execution of brake tests when the brake_test_status is not successful.
 * }
 *
 * This is repeated for the first and the last node and one in between.
 *
 * Test Sequence:
 *  1. Set expectations on CANOpen mock object. Let the service call getting the brake_test_status object respond
 *     status not successful for the selected node.
 *  2. Call brake test service.
 *  3. Verify and clear expectations.
 *
 * Expected Results:
 *  1. -
 *  2. The brake test service responds with correct error case.
 *  3. There is at least one read request on brake_test_status object for the selected node.
 */
TEST_F(CanOpenBraketestAdapterTest, testBrakeTestStatusNotSuccessful)
{
  DEFAULT_SETUP

  GetObjectResponse status_resp;
  status_resp.success = true;
  status_resp.value = "\x03";

  for (size_t node_index : NODE_TEST_SET)
  {
    std::string node{ NODE_NAMES_PREFIX + std::to_string(node_index + 1) };

    /**********
     * Step 1 *
     **********/
    // Avoid unmatched expectations
    canopen_chain_node.expectAnything();

    EXPECT_CALL(canopen_chain_node, get_obj(AllOf(Field(&GetObjectRequest::node, node),
                                                  Field(&GetObjectRequest::object, BRAKE_TEST_STATUS_OBJECT_INDEX)),
                                            _))
        .Times(AtLeast(1))
        .WillRepeatedly(DoAll(SetArgReferee<1>(status_resp), Return(true)));

    /**********
     * Step 2 *
     **********/
    BrakeTest srv;
    EXPECT_TRUE(brake_test_srv_client.call(srv)) << "Failed to call brake test service.";
    EXPECT_FALSE(srv.response.success);
    EXPECT_EQ(BrakeTestErrorCodes::STATUS_NO_SUCCESS, srv.response.error_code.value);

    /**********
     * Step 3 *
     **********/
    Mock::VerifyAndClearExpectations(&canopen_chain_node);
  }
}

/**
 * @tests{Execute_BrakeTest_mechanism,
 *  Test execution of brake tests when the brake_test_status is
 *  not actively controlled.
 * }
 *
 * This is repeated for the first and the last node and one in between.
 *
 * Test Sequence:
 *  1. Set expectations on CANOpen mock object. Let the service call getting the brake_test_status object respond
 *     status not actively controlled for the selected node.
 *  2. Call brake test service.
 *  3. Verify and clear expectations.
 *
 * Expected Results:
 *  1. -
 *  2. The brake test service responds with correct error case.
 *  3. There is at least one read request on brake_test_status object for the selected node.
 */
TEST_F(CanOpenBraketestAdapterTest, testBrakeTestStatusNotActivelyControlled)
{
  DEFAULT_SETUP

  GetObjectResponse status_resp;
  status_resp.success = true;
  status_resp.value = "\x04";

  for (size_t node_index : NODE_TEST_SET)
  {
    std::string node{ NODE_NAMES_PREFIX + std::to_string(node_index + 1) };

    /**********
     * Step 1 *
     **********/
    // Avoid unmatched expectations
    canopen_chain_node.expectAnything();

    EXPECT_CALL(canopen_chain_node, get_obj(AllOf(Field(&GetObjectRequest::node, node),
                                                  Field(&GetObjectRequest::object, BRAKE_TEST_STATUS_OBJECT_INDEX)),
                                            _))
        .Times(AtLeast(1))
        .WillRepeatedly(DoAll(SetArgReferee<1>(status_resp), Return(true)));

    /**********
     * Step 2 *
     **********/
    BrakeTest srv;
    EXPECT_TRUE(brake_test_srv_client.call(srv)) << "Failed to call brake test service.";
    EXPECT_FALSE(srv.response.success);
    EXPECT_EQ(BrakeTestErrorCodes::STATUS_NO_CONTROL, srv.response.error_code.value);

    /**********
     * Step 3 *
     **********/
    Mock::VerifyAndClearExpectations(&canopen_chain_node);
  }
}

}  // namespace canopen_braketest_adapter_test

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "unittest_brake_test_executor");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner{ 2 };
  spinner.start();

  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
