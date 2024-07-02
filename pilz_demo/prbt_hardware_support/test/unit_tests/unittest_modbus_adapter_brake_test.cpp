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

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <functional>
#include <memory>

#include <pilz_msgs/IsBrakeTestRequired.h>
#include <pilz_msgs/IsBrakeTestRequiredResult.h>

#include <prbt_hardware_support/modbus_api_definitions.h>
#include <prbt_hardware_support/modbus_adapter_brake_test.h>
#include <prbt_hardware_support/modbus_msg_in_builder.h>
#include <prbt_hardware_support/register_container.h>
#include <prbt_hardware_support/modbus_msg_brake_test_wrapper.h>
#include <prbt_hardware_support/modbus_msg_brake_test_wrapper_exception.h>
#include <prbt_hardware_support/modbus_adapter_brake_test_exception.h>
#include <prbt_hardware_support/register_container.h>
#include <prbt_hardware_support/write_modbus_register_call.h>

#include <pilz_testutils/async_test.h>

namespace prbt_hardware_support
{
using namespace modbus_api::v3;

static const ModbusApiSpec TEST_API_SPEC{ { modbus_api_spec::VERSION, 969 },
                                          { modbus_api_spec::BRAKETEST_REQUEST, 973 } };

static const ModbusApiSpec TEST_API_WRITE_SPEC{ { modbus_api_spec::BRAKETEST_PERFORMED, 77 },
                                                { modbus_api_spec::BRAKETEST_RESULT, 78 } };

using std::placeholders::_1;
using std::placeholders::_2;

using ::testing::_;
using ::testing::DoAll;
using ::testing::Return;
using ::testing::SetArgReferee;

class ModbusMock
{
public:
  MOCK_METHOD2(modbsWriteRegisterFunc, bool(const uint16_t&, const RegCont&));
};

static ModbusMsgInStampedPtr createDefaultBrakeTestModbusMsg(
    const uint16_t brake_test_required_value, const unsigned int modbus_api_version = MODBUS_API_VERSION_REQUIRED,
    const uint32_t brake_test_required_index = TEST_API_SPEC.getRegisterDefinition(modbus_api_spec::BRAKETEST_REQUEST))
{
  const uint32_t first_index_to_read{ TEST_API_SPEC.getRegisterDefinition(modbus_api_spec::VERSION) };
  const uint32_t last_index_to_read{ brake_test_required_index };
  static int msg_time_counter{ 1 };
  RegCont tab_reg(last_index_to_read - first_index_to_read + 1);
  tab_reg[0] = static_cast<uint16_t>(modbus_api_version);
  tab_reg[last_index_to_read - first_index_to_read] = brake_test_required_value;
  ModbusMsgInStampedPtr msg{ ModbusMsgInBuilder::createDefaultModbusMsgIn(first_index_to_read, tab_reg) };
  msg->header.stamp = ros::Time(msg_time_counter++);
  return msg;
}

/**
 * @brief Test increases function coverage by ensuring that all Dtor variants
 * are called.
 */
TEST(ModbusAdapterBrakeTestTest, testModbusMsgBrakeTestWrapperExceptionDtor)
{
  std::shared_ptr<ModbusMsgBrakeTestWrapperException> msg_wrapper{ new ModbusMsgBrakeTestWrapperException("Test msg") };
}

/**
 * @brief Test increases function coverage by ensuring that all Dtor variants
 * are called.
 */
TEST(ModbusAdapterBrakeTestTest, testModbusMsgBrakeTestWrapperDtor)
{
  {
    std::shared_ptr<ModbusMsgBrakeTestWrapper> msg_wrapper{ new ModbusMsgBrakeTestWrapper(
        createDefaultBrakeTestModbusMsg(REGISTER_VALUE_BRAKETEST_REQUIRED), TEST_API_SPEC) };
  }
}

/**
 * @tests{Is_BrakeTest_required_mechanism,
 *  Tests the value returned if no modbus messages are
 * }
 *
 * Test Sequence:
 *  1. Call service without sending a message via modbus before
 *
 * Expected Results:
 *  1. The service returns true, and the it is unknown if a braketest has to be performed
 */
TEST(ModbusAdapterBrakeTestTest, testNoMessageReceived)
{
  ModbusMock mock;
  ModbusAdapterBrakeTest brake_test_adapter(std::bind(&ModbusMock::modbsWriteRegisterFunc, &mock, _1, _2),
                                            TEST_API_SPEC, TEST_API_WRITE_SPEC);

  pilz_msgs::IsBrakeTestRequired srv;
  EXPECT_TRUE(brake_test_adapter.isBrakeTestRequired(srv.request, srv.response));
  EXPECT_EQ(pilz_msgs::IsBrakeTestRequiredResult::UNKNOWN, srv.response.result.value);
}

/**
 * @tests{Is_BrakeTest_required_mechanism,
 *  Tests the handling of an incoming modbus message informing about
 *  a required brake test.
 * }
 *
 * Test Sequence:
 *  1. Publish modbus message informing about a required brake test.
 *
 * Expected Results:
 *  1. The service returns true
 */
TEST(ModbusAdapterBrakeTestTest, testBrakeTestRequired)
{
  ModbusMock mock;
  ModbusAdapterBrakeTest brake_test_adapter(std::bind(&ModbusMock::modbsWriteRegisterFunc, &mock, _1, _2),
                                            TEST_API_SPEC, TEST_API_WRITE_SPEC);

  brake_test_adapter.modbusMsgCallback(createDefaultBrakeTestModbusMsg(REGISTER_VALUE_BRAKETEST_REQUIRED));

  pilz_msgs::IsBrakeTestRequired srv;
  EXPECT_TRUE(brake_test_adapter.isBrakeTestRequired(srv.request, srv.response));
  EXPECT_EQ(pilz_msgs::IsBrakeTestRequiredResult::REQUIRED, srv.response.result.value);
}

/**
 * @tests{Is_BrakeTest_required_mechanism,
 *  Tests the handling of an incoming modbus message informing about
 *  a brake test not being required.
 * }
 *
 * Test Sequence:
 *  1. Publish modbus message informing about a brake test not being required.
 *
 * Expected Results:
 *  1. The service returns false
 */
TEST(ModbusAdapterBrakeTestTest, testBrakeTestNotRequired)
{
  ModbusMock mock;
  ModbusAdapterBrakeTest brake_test_adapter(std::bind(&ModbusMock::modbsWriteRegisterFunc, &mock, _1, _2),
                                            TEST_API_SPEC, TEST_API_WRITE_SPEC);

  brake_test_adapter.modbusMsgCallback(createDefaultBrakeTestModbusMsg(REGISTER_VALUE_BRAKETEST_NOT_REQUIRED));

  pilz_msgs::IsBrakeTestRequired srv;
  EXPECT_TRUE(brake_test_adapter.isBrakeTestRequired(srv.request, srv.response));
  EXPECT_EQ(pilz_msgs::IsBrakeTestRequiredResult::NOT_REQUIRED, srv.response.result.value);
}

/**
 * @tests{Is_BrakeTest_required_mechanism,
 *  Tests the handling of an incoming modbus message informing
 *  about a disconnect.
 * }
 *
 * Test Sequence:
 *  1. Publish modbus message informing about a disconnect.
 *
 * Expected Results:
 *  1. The state is unknown
 */
TEST(ModbusAdapterBrakeTestTest, testDisconnect)
{
  constexpr uint32_t offset{ 0 };
  const RegCont holding_register;
  ModbusMsgInStampedPtr msg{ ModbusMsgInBuilder::createDefaultModbusMsgIn(offset, holding_register) };
  msg->disconnect.data = true;

  ModbusMock mock;
  ModbusAdapterBrakeTest brake_test_adapter(std::bind(&ModbusMock::modbsWriteRegisterFunc, &mock, _1, _2),
                                            TEST_API_SPEC, TEST_API_WRITE_SPEC);

  brake_test_adapter.modbusMsgCallback(msg);

  pilz_msgs::IsBrakeTestRequired srv;
  EXPECT_TRUE(brake_test_adapter.isBrakeTestRequired(srv.request, srv.response));
  EXPECT_EQ(pilz_msgs::IsBrakeTestRequiredResult::UNKNOWN, srv.response.result.value);
}

/**
 * @tests{Is_BrakeTest_required_mechanism,
 *  Tests the handling of an incoming modbus message with
 *  incorrect api version.
 * }
 *
 * Test Sequence:
 *  1. Publish modbus message with incorrect api version.
 *
 * Expected Results:
 *  1. The state is unknown
 */
TEST(ModbusAdapterBrakeTestTest, testModbusIncorrectApiVersion)
{
  ModbusMock mock;
  ModbusAdapterBrakeTest brake_test_adapter(std::bind(&ModbusMock::modbsWriteRegisterFunc, &mock, _1, _2),
                                            TEST_API_SPEC, TEST_API_WRITE_SPEC);

  brake_test_adapter.modbusMsgCallback(createDefaultBrakeTestModbusMsg(REGISTER_VALUE_BRAKETEST_REQUIRED, 0));

  pilz_msgs::IsBrakeTestRequired srv;
  EXPECT_TRUE(brake_test_adapter.isBrakeTestRequired(srv.request, srv.response));
  EXPECT_EQ(pilz_msgs::IsBrakeTestRequiredResult::UNKNOWN, srv.response.result.value);
}

/**
 * @tests{Is_BrakeTest_required_mechanism,
 *  Tests the handling of an incoming modbus message without api version.
 * }
 *
 * Test Sequence:
 *  1. Publish modbus message without api version.
 *
 * Expected Results:
 *  1. The state is unknown
 */
TEST(ModbusAdapterBrakeTestTest, testModbusWithoutApiVersion)
{
  auto msg{ createDefaultBrakeTestModbusMsg(REGISTER_VALUE_BRAKETEST_REQUIRED,
                                            TEST_API_SPEC.getRegisterDefinition(modbus_api_spec::VERSION),
                                            TEST_API_SPEC.getRegisterDefinition(modbus_api_spec::BRAKETEST_REQUEST)) };
  msg->holding_registers.data.clear();

  ModbusMock mock;
  ModbusAdapterBrakeTest brake_test_adapter(std::bind(&ModbusMock::modbsWriteRegisterFunc, &mock, _1, _2),
                                            TEST_API_SPEC, TEST_API_WRITE_SPEC);

  brake_test_adapter.modbusMsgCallback(msg);

  pilz_msgs::IsBrakeTestRequired srv;
  EXPECT_TRUE(brake_test_adapter.isBrakeTestRequired(srv.request, srv.response));
  EXPECT_EQ(pilz_msgs::IsBrakeTestRequiredResult::UNKNOWN, srv.response.result.value);
}

/**
 * @tests{Is_BrakeTest_required_mechanism,
 *  Tests the handling of an incoming modbus message without
 *  a brake test status.
 * }
 *
 * Test Sequence:
 *  1. Publish modbus message without a brake test status.
 *
 * Expected Results:
 *  1. The state is unknown
 */
TEST(ModbusAdapterBrakeTestTest, testBrakeTestRequiredRegisterMissing)
{
  ModbusMock mock;
  ModbusAdapterBrakeTest brake_test_adapter(std::bind(&ModbusMock::modbsWriteRegisterFunc, &mock, _1, _2),
                                            TEST_API_SPEC, TEST_API_WRITE_SPEC);

  brake_test_adapter.modbusMsgCallback(createDefaultBrakeTestModbusMsg(
      REGISTER_VALUE_BRAKETEST_REQUIRED, TEST_API_SPEC.getRegisterDefinition(modbus_api_spec::VERSION),
      TEST_API_SPEC.getRegisterDefinition(modbus_api_spec::BRAKETEST_REQUEST) - 1));

  pilz_msgs::IsBrakeTestRequired srv;
  EXPECT_TRUE(brake_test_adapter.isBrakeTestRequired(srv.request, srv.response));
  EXPECT_EQ(pilz_msgs::IsBrakeTestRequiredResult::UNKNOWN, srv.response.result.value);
}

/**
 * @tests{Is_BrakeTest_required_mechanism,
 *  Tests the handling of an incoming modbus message with a undefined brake test status register value.
 * }
 *
 * Test Sequence:
 *  1. Publish modbus message with a undefine brake test status register value.
 *
 * Expected Results:
 *  1. The state is unknown
 */
TEST(ModbusAdapterBrakeTestTest, testBrakeTestRequiredRegisterUndefinedValue)
{
  ModbusMock mock;
  ModbusAdapterBrakeTest brake_test_adapter(std::bind(&ModbusMock::modbsWriteRegisterFunc, &mock, _1, _2),
                                            TEST_API_SPEC, TEST_API_WRITE_SPEC);

  brake_test_adapter.modbusMsgCallback(createDefaultBrakeTestModbusMsg(555 /* some arbitrary value */));

  pilz_msgs::IsBrakeTestRequired srv;
  EXPECT_TRUE(brake_test_adapter.isBrakeTestRequired(srv.request, srv.response));
  EXPECT_EQ(pilz_msgs::IsBrakeTestRequiredResult::UNKNOWN, srv.response.result.value);
}

/**
 * @brief Test increases function coverage by ensuring that all Dtor variants
 * are called.
 */
TEST(ModbusAdapterBrakeTestTest, testModbusApiSpecExceptionDtor)
{
  std::shared_ptr<ModbusAdapterBrakeTestException> ex{ new ModbusAdapterBrakeTestException("Test msg") };
}

/**
 * @brief Test execution of brake tests when there is a problem
 * in the api definition.
 *
 * Test Sequence:
 *  1. Execute a brake test with missing definition for BRAKETEST_PERFORMED
 *  2. Execute a brake test with missing definition for BRAKETEST_RESULT
 *  3. Execute a brake test with both values defined 1 apart
 *  4. Execute a brake test with both values defined 2 apart
 *
 * Expected Results:
 *  1. A BrakeTestExecutorException is thown
 *  2. A BrakeTestExecutorException is thown
 *  3. No Exception is thrown
 *  4. A BrakeTestExecutorException is thown
 */
TEST(ModbusAdapterBrakeTestTest, testBrakeTestTriggeringWrongApiDef)
{
  /**********
   * Step 1 *
   **********/
  {
    ModbusApiSpec api_write_spec{ { modbus_api_spec::BRAKETEST_RESULT, 78 } };
    ModbusMock mock;
    ASSERT_THROW(ModbusAdapterBrakeTest bte_no_perf(std::bind(&ModbusMock::modbsWriteRegisterFunc, &mock, _1, _2),
                                                    TEST_API_SPEC, api_write_spec),
                 ModbusAdapterBrakeTestException);
  }

  /**********
   * Step 2 *
   **********/
  {
    ModbusApiSpec api_write_spec{ { modbus_api_spec::BRAKETEST_PERFORMED, 77 } };
    ModbusMock mock;
    ASSERT_THROW(ModbusAdapterBrakeTest bte_no_res(std::bind(&ModbusMock::modbsWriteRegisterFunc, &mock, _1, _2),
                                                   TEST_API_SPEC, api_write_spec),
                 ModbusAdapterBrakeTestException);
  }

  /**********
   * Step 3 *
   **********/
  {
    ModbusApiSpec api_write_spec{ { modbus_api_spec::BRAKETEST_PERFORMED, 100 },
                                  { modbus_api_spec::BRAKETEST_RESULT, 99 } };
    ModbusMock mock;
    ASSERT_NO_THROW(ModbusAdapterBrakeTest bte_one_apart(std::bind(&ModbusMock::modbsWriteRegisterFunc, &mock, _1, _2),
                                                         TEST_API_SPEC, api_write_spec));
  }

  /**********
   * Step 4 *
   **********/
  {
    ModbusApiSpec api_write_spec{ { modbus_api_spec::BRAKETEST_PERFORMED, 100 },
                                  { modbus_api_spec::BRAKETEST_RESULT, 98 } };
    ModbusMock mock;
    ASSERT_THROW(ModbusAdapterBrakeTest bte_two_apart(std::bind(&ModbusMock::modbsWriteRegisterFunc, &mock, _1, _2),
                                                      TEST_API_SPEC, api_write_spec),
                 ModbusAdapterBrakeTestException);
  }
}

/**
 * @brief Tests that missing modbus register write functions leads to
 * response.success==false.
 */
TEST(ModbusAdapterBrakeTestTest, testMissingModbusWriteFunc)
{
  ModbusMock mock;
  ModbusAdapterBrakeTest brake_test_adapter(nullptr, TEST_API_SPEC, TEST_API_WRITE_SPEC);

  SendBrakeTestResult srv;
  EXPECT_TRUE(brake_test_adapter.sendBrakeTestResult(srv.request, srv.response));
  EXPECT_FALSE(srv.response.success);
}

/**
 * @brief Tests that failing modbus register write functions leads to
 * response.success==false.
 */
TEST(ModbusAdapterBrakeTestTest, testFailingModbusWriteFunc)
{
  ModbusMock mock;
  EXPECT_CALL(mock, modbsWriteRegisterFunc(_, _)).Times(1).WillOnce(Return(false));
  ModbusAdapterBrakeTest brake_test_adapter(std::bind(&ModbusMock::modbsWriteRegisterFunc, &mock, _1, _2),
                                            TEST_API_SPEC, TEST_API_WRITE_SPEC);

  SendBrakeTestResult srv;
  EXPECT_TRUE(brake_test_adapter.sendBrakeTestResult(srv.request, srv.response));
  EXPECT_FALSE(srv.response.success);
}

/**
 * @brief Tests that failing modbus register write functions leads to
 * response.success==false.
 */
TEST(ModbusAdapterBrakeTestTest, testSecondTimeFailingModbusWriteFunc)
{
  ModbusMock mock;
  EXPECT_CALL(mock, modbsWriteRegisterFunc(_, _)).Times(2).WillOnce(Return(true)).WillOnce(Return(false));
  ModbusAdapterBrakeTest brake_test_adapter(std::bind(&ModbusMock::modbsWriteRegisterFunc, &mock, _1, _2),
                                            TEST_API_SPEC, TEST_API_WRITE_SPEC);

  SendBrakeTestResult srv;
  EXPECT_TRUE(brake_test_adapter.sendBrakeTestResult(srv.request, srv.response));
  EXPECT_FALSE(srv.response.success);
}

class ServiceMock
{
public:
  MOCK_METHOD1(call, bool(WriteModbusRegister& srv));
  MOCK_METHOD0(getService, std::string());
};

/**
 * @brief Tests that a service.response=false leads to return value false.
 */
TEST(ModbusAdapterBrakeTestTest, testWriteModbusRegisterCallResponseFalse)
{
  WriteModbusRegister res_exp;
  res_exp.response.success = false;

  ServiceMock mock;
  EXPECT_CALL(mock, getService()).WillRepeatedly(Return("TestServiceName"));
  EXPECT_CALL(mock, call(_)).Times(1).WillOnce(DoAll(SetArgReferee<0>(res_exp), Return(true)));

  EXPECT_FALSE(writeModbusRegisterCall<ServiceMock>(mock, 0, {}));
}

/**
 * @brief Tests that a service call failure leads to return value false.
 */
TEST(ModbusAdapterBrakeTestTest, testWriteModbusRegisterCallFailure)
{
  WriteModbusRegister res_exp;
  res_exp.response.success = false;

  ServiceMock mock;
  EXPECT_CALL(mock, getService()).WillRepeatedly(Return("TestServiceName"));
  EXPECT_CALL(mock, call(_)).Times(1).WillOnce(Return(false));

  EXPECT_FALSE(writeModbusRegisterCall<ServiceMock>(mock, 0, {}));
}

}  // namespace prbt_hardware_support

int main(int argc, char* argv[])
{
  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
