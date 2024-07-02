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

#include <std_srvs/Trigger.h>

#include <pilz_msgs/BrakeTest.h>

#include <prbt_hardware_support/brake_test_executor.h>
#include <prbt_hardware_support/BrakeTestErrorCodes.h>
#include <prbt_hardware_support/brake_test_executor_node_service_calls.h>

namespace brake_test_executor_test
{
using namespace prbt_hardware_support;
using namespace testing;

using std::placeholders::_1;

class SystemMock
{
public:
  MOCK_METHOD0(detectMotion, bool());
  MOCK_METHOD0(holdController, void());
  MOCK_METHOD0(executeBrakeTest, BrakeTest::Response());
  MOCK_METHOD0(unholdController, void());
  MOCK_METHOD1(sendBrakeTestResult, bool(const bool brake_test_result));
};

/**
 * @tests{Execute_BrakeTest_mechanism,
 * Test execution of brake tests while robot is not moving.
 * }
 *
 * Test Sequence:
 *  - Set expectations and action on service calls + Call brake test service
 *
 * Expected Results:
 *  - Brake tests are executed successfully. In strict order:
 *     - Hold mode is triggered
 *     - Brake test execution is triggered
 *     - Unhold is triggered
 */
TEST(BrakeTestExecutorTest, testBrakeTestTriggeringRobotNotMoving)
{
  SystemMock mock;
  BrakeTestExecutor brake_test_executor(
      std::bind(&SystemMock::detectMotion, &mock), std::bind(&SystemMock::holdController, &mock),
      std::bind(&SystemMock::executeBrakeTest, &mock), std::bind(&SystemMock::unholdController, &mock),
      std::bind(&SystemMock::sendBrakeTestResult, &mock, _1));

  {
    InSequence dummy;

    EXPECT_CALL(mock, detectMotion()).Times(1).WillOnce(Return(false));
    EXPECT_CALL(mock, holdController()).Times(1);
    EXPECT_CALL(mock, executeBrakeTest()).Times(1).WillOnce(testing::Invoke([]() {
      BrakeTest::Response res;
      res.success = true;
      return res;
    }));
    EXPECT_CALL(mock, unholdController()).Times(1);
    EXPECT_CALL(mock, sendBrakeTestResult(_)).Times(1).WillOnce(Return(true));
  }

  pilz_msgs::BrakeTest brake_test_srv;
  EXPECT_TRUE(brake_test_executor.executeBrakeTest(brake_test_srv.request, brake_test_srv.response)) << "Failed to "
                                                                                                        "call brake "
                                                                                                        "test service.";
  EXPECT_TRUE(brake_test_srv.response.success)
      << "Brake tests failed unexpectedly. Message: " << brake_test_srv.response.error_msg;
}

/**
 * @tests{Execute_BrakeTest_mechanism,
 *  Test execution of brake tests while robot is moving.
 * }
 *
 * Test Sequence:
 *  1.  Set expectations and action on service calls + Call brake test service.
 *
 * Expected Results:
 *  1.  Brake tests cannot be triggered.
 *      Respective error message is returned. The hold service is not called.
 */
TEST(BrakeTestExecutorTest, testBrakeTestServiceWithRobotMotion)
{
  SystemMock mock;
  BrakeTestExecutor brake_test_executor(
      std::bind(&SystemMock::detectMotion, &mock), std::bind(&SystemMock::holdController, &mock),
      std::bind(&SystemMock::executeBrakeTest, &mock), std::bind(&SystemMock::unholdController, &mock),
      std::bind(&SystemMock::sendBrakeTestResult, &mock, _1));

  EXPECT_CALL(mock, detectMotion()).Times(1).WillOnce(Return(true));
  EXPECT_CALL(mock, holdController()).Times(0);
  EXPECT_CALL(mock, executeBrakeTest()).Times(0);
  EXPECT_CALL(mock, unholdController()).Times(0);
  EXPECT_CALL(mock, sendBrakeTestResult(_)).Times(0);

  pilz_msgs::BrakeTest brake_test_srv;
  EXPECT_TRUE(brake_test_executor.executeBrakeTest(brake_test_srv.request, brake_test_srv.response));
  EXPECT_FALSE(brake_test_srv.response.success) << "Brake tests was unexpectedly successful";
}

/**
 * @tests{Execute_BrakeTest_mechanism,
 * Test behaviour if the triggering of the braketest fails.
 * }
 *
 * Test Sequence:
 *  1.  Set expectations and action on the service call
 *      (Return false as response on call.) + Call brake test service.
 *
 * Expected Results:
 *  1. Error is returned upon service call. In strict order:
 *     - Hold mode is triggered
 *     - Brake test execution is triggered
 *     - Unhold is triggered
 */
TEST(BrakeTestExecutorTest, testBrakeTestServiceTriggerFails)
{
  SystemMock mock;
  BrakeTestExecutor brake_test_executor(
      std::bind(&SystemMock::detectMotion, &mock), std::bind(&SystemMock::holdController, &mock),
      std::bind(&SystemMock::executeBrakeTest, &mock), std::bind(&SystemMock::unholdController, &mock),
      std::bind(&SystemMock::sendBrakeTestResult, &mock, _1));

  {
    InSequence dummy;

    EXPECT_CALL(mock, detectMotion()).Times(1).WillOnce(Return(false));
    EXPECT_CALL(mock, holdController()).Times(1);
    EXPECT_CALL(mock, executeBrakeTest()).Times(1).WillOnce(testing::Invoke([]() {
      BrakeTest::Response res;
      res.success = false;
      res.error_msg = "Test error message";
      res.error_code.value = BrakeTestErrorCodes::FAILURE;
      return res;
    }));
    EXPECT_CALL(mock, unholdController()).Times(1);
    EXPECT_CALL(mock, sendBrakeTestResult(_)).Times(1).WillOnce(Return(true));
  }

  pilz_msgs::BrakeTest brake_test_srv;
  EXPECT_TRUE(brake_test_executor.executeBrakeTest(brake_test_srv.request, brake_test_srv.response));
  EXPECT_FALSE(brake_test_srv.response.success);
}

/**
 * @tests{Execute_BrakeTest_mechanism,
 * Test behaviour if the sending of the brake test result fails.
 * }
 *
 * Test Sequence:
 *  1.  Set expectations + Call brake test service.
 *
 * Expected Results:
 *  1.  Function returns false and error code matches expected error code.
 */
TEST(BrakeTestExecutorTest, testBrakeTestResultServiceFails)
{
  SystemMock mock;
  BrakeTestExecutor brake_test_executor(
      std::bind(&SystemMock::detectMotion, &mock), std::bind(&SystemMock::holdController, &mock),
      std::bind(&SystemMock::executeBrakeTest, &mock), std::bind(&SystemMock::unholdController, &mock),
      std::bind(&SystemMock::sendBrakeTestResult, &mock, _1));

  {
    InSequence dummy;

    EXPECT_CALL(mock, detectMotion()).Times(1).WillOnce(Return(false));
    EXPECT_CALL(mock, holdController()).Times(1);
    EXPECT_CALL(mock, executeBrakeTest()).Times(1).WillOnce(testing::Invoke([]() {
      BrakeTest::Response res;
      res.success = true;
      return res;
    }));
    EXPECT_CALL(mock, unholdController()).Times(1);
    EXPECT_CALL(mock, sendBrakeTestResult(_)).Times(1).WillOnce(Return(false));
  }

  pilz_msgs::BrakeTest brake_test_srv;
  EXPECT_TRUE(brake_test_executor.executeBrakeTest(brake_test_srv.request, brake_test_srv.response));
  EXPECT_FALSE(brake_test_srv.response.success);
}

/**
 * @brief Checks that exception is thrown if function to hold controller
 * is missing.
 */
TEST(BrakeTestExecutorTest, testMissingHoldFunc)
{
  SystemMock mock;
  EXPECT_THROW(BrakeTestExecutor(std::bind(&SystemMock::detectMotion, &mock), nullptr,
                                 std::bind(&SystemMock::executeBrakeTest, &mock),
                                 std::bind(&SystemMock::unholdController, &mock),
                                 std::bind(&SystemMock::sendBrakeTestResult, &mock, _1)),
               BrakeTestExecutorException);
}

/**
 * @brief Checks that exception is thrown if function to unhold controller
 * is missing.
 */
TEST(BrakeTestExecutorTest, testMissingUnholdFunc)
{
  SystemMock mock;
  EXPECT_THROW(BrakeTestExecutor(std::bind(&SystemMock::detectMotion, &mock),
                                 std::bind(&SystemMock::holdController, &mock),
                                 std::bind(&SystemMock::executeBrakeTest, &mock), nullptr,
                                 std::bind(&SystemMock::sendBrakeTestResult, &mock, _1)),
               BrakeTestExecutorException);
}

/**
 * @brief Checks that exception is thrown if function to detect robot motion
 * is missing.
 */
TEST(BrakeTestExecutorTest, testMissingDetectMotionFunc)
{
  SystemMock mock;
  EXPECT_THROW(BrakeTestExecutor(nullptr, std::bind(&SystemMock::holdController, &mock),
                                 std::bind(&SystemMock::executeBrakeTest, &mock),
                                 std::bind(&SystemMock::unholdController, &mock),
                                 std::bind(&SystemMock::sendBrakeTestResult, &mock, _1)),
               BrakeTestExecutorException);
}

/**
 * @brief Checks that exception is thrown if function to execute brake test
 * on robot is missing.
 */
TEST(BrakeTestExecutorTest, testMissingExecuteBrakeTestFunc)
{
  SystemMock mock;
  EXPECT_THROW(BrakeTestExecutor(std::bind(&SystemMock::detectMotion, &mock),
                                 std::bind(&SystemMock::holdController, &mock), nullptr,
                                 std::bind(&SystemMock::unholdController, &mock),
                                 std::bind(&SystemMock::sendBrakeTestResult, &mock, _1)),
               BrakeTestExecutorException);
}

/**
 * @brief Checks that exception is thrown if function to send brake test
 * result is missing.
 */
TEST(BrakeTestExecutorTest, testMissingSendBrakeTestResultFunc)
{
  SystemMock mock;
  EXPECT_THROW(BrakeTestExecutor(std::bind(&SystemMock::detectMotion, &mock),
                                 std::bind(&SystemMock::holdController, &mock),
                                 std::bind(&SystemMock::executeBrakeTest, &mock),
                                 std::bind(&SystemMock::unholdController, &mock), nullptr),
               BrakeTestExecutorException);
}

/**
 * @brief Test increases function coverage by ensuring that all Dtor variants
 * are called.
 */
TEST(BrakeTestExecutorTest, testDtorBrakeTestExecutorException)
{
  std::unique_ptr<BrakeTestExecutorException> ex{ new BrakeTestExecutorException("TestException") };
}

class TriggerServiceMock
{
public:
  MOCK_METHOD1(call, bool(std_srvs::Trigger& srv));
  MOCK_METHOD0(getService, std::string());
};

/**
 * @brief Tests case that trigger service fails.
 */
TEST(BrakeTestExecutorTest, testTriggerServiceCallFailure)
{
  TriggerServiceMock mock;
  EXPECT_CALL(mock, getService()).WillRepeatedly(Return("TestServiceName"));
  EXPECT_CALL(mock, call(_)).Times(1).WillOnce(Return(false));

  triggerServiceCall<TriggerServiceMock>(mock);
}

/**
 * @brief Tests what happens in case service is response is false (Tests
 * mainly exists to complete code coverage).
 */
TEST(BrakeTestExecutorTest, testTriggerServiceCallResponseFalse)
{
  TriggerServiceMock mock;
  EXPECT_CALL(mock, getService()).WillRepeatedly(Return("TestServiceName"));
  std_srvs::Trigger exp_srv;
  exp_srv.response.success = false;
  EXPECT_CALL(mock, call(_)).Times(1).WillOnce(DoAll(SetArgReferee<0>(exp_srv), Return(true)));

  triggerServiceCall<TriggerServiceMock>(mock);
}

class BrakeTestServiceMock
{
public:
  MOCK_METHOD1(call, bool(BrakeTest& srv));
  MOCK_METHOD0(getService, std::string());
};

/**
 * @brief Tests that:
 *    - false is returned if BrakeTest service fails,
 *    - correct error code is returned if BrakeTest service fails.
 */
TEST(BrakeTestExecutorTest, testExecuteBrakeTestCallFailure)
{
  BrakeTestServiceMock mock;
  EXPECT_CALL(mock, getService()).WillRepeatedly(Return("TestServiceName"));
  EXPECT_CALL(mock, call(_)).Times(1).WillOnce(Return(false));

  BrakeTest::Response res{ executeBrakeTestCall<BrakeTestServiceMock>(mock) };
  EXPECT_FALSE(res.success);
  EXPECT_EQ(res.error_code.value, BrakeTestErrorCodes::TRIGGER_BRAKETEST_SERVICE_FAILURE);
}

class SendBrakeTestResltServiceMock
{
public:
  MOCK_METHOD1(call, bool(SendBrakeTestResult& srv));
  MOCK_METHOD0(getService, std::string());
};

/**
 * @brief Tests that false is returned if SendBrakeTestResult service fails.
 */
TEST(BrakeTestExecutorTest, testSendBrakeTestResultCallFailure)
{
  SendBrakeTestResltServiceMock mock;
  EXPECT_CALL(mock, getService()).WillRepeatedly(Return("TestServiceName"));
  EXPECT_CALL(mock, call(_)).Times(1).WillOnce(Return(false));

  EXPECT_FALSE(sendBrakeTestResultCall<SendBrakeTestResltServiceMock>(mock, true));
}

MATCHER(IsRequestResultFalse, "")
{
  return !arg.request.result;
}

/**
 * @brief Tests that SendBrakeTestResult service is called with correct value.
 */
TEST(BrakeTestExecutorTest, testSendBrakeTestResultCallSuccess)
{
  SendBrakeTestResltServiceMock mock;
  EXPECT_CALL(mock, getService()).WillRepeatedly(Return("TestServiceName"));
  EXPECT_CALL(mock, call(IsRequestResultFalse())).Times(1).WillOnce(Return(true));

  EXPECT_TRUE(sendBrakeTestResultCall<SendBrakeTestResltServiceMock>(mock, false));
}

}  // namespace brake_test_executor_test

int main(int argc, char* argv[])
{
  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
