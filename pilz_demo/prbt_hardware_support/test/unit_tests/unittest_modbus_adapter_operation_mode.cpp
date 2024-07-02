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

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <algorithm>
#include <memory>

#include <pilz_msgs/OperationModes.h>

#include <pilz_testutils/async_test.h>

#include <prbt_hardware_support/modbus_api_definitions.h>
#include <prbt_hardware_support/modbus_adapter_operation_mode.h>
#include <prbt_hardware_support/modbus_topic_definitions.h>
#include <prbt_hardware_support/register_container.h>
#include <prbt_hardware_support/modbus_msg_in_builder.h>

namespace prbt_hardware_support
{
using namespace modbus_api::v3;

static constexpr int DEFAULT_QUEUE_SIZE_MODBUS{ 1 };
static const std::string SERVICE_NAME_OPERATION_MODE = "/prbt/get_operation_mode";
static const std::string TOPIC_OPERATION_MODE{ "/prbt/operation_mode" };
static constexpr int OPERATION_MODE_QUEUE_SIZE{ 1 };

static const std::string OPERATION_MODE_CALLBACK_EVENT{ "operation_mode_callback_event" };

static const ModbusApiSpec TEST_API_SPEC{ { modbus_api_spec::VERSION, 1 }, { modbus_api_spec::OPERATION_MODE, 11 } };

static const std::vector<uint16_t> OPERATION_MODES{ 1, 2, 3 };

/**
 * @brief Redirects callbacks of a ros::Subscriber to a mock method.
 */
class OperationModeSubscriberMock
{
public:
  /**
   * @brief Actual subscription takes place here
   */
  void initialize();

  MOCK_METHOD1(callback, void(const pilz_msgs::OperationModesConstPtr& msg));

protected:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
};

void OperationModeSubscriberMock::initialize()
{
  subscriber_ =
      nh_.subscribe(TOPIC_OPERATION_MODE, OPERATION_MODE_QUEUE_SIZE, &OperationModeSubscriberMock::callback, this);
}

using ::testing::StrictMock;

/**
 * @brief Test fixture for unit-tests of the ModbusAdapterOperationMode.
 *
 * Publish messages on the modbus topic and check on the operation_mode topic
 * if the expectations are met.
 */
class ModbusAdapterOperationModeTest : public testing::Test, public testing::AsyncTest
{
public:
  ModbusAdapterOperationModeTest();
  ~ModbusAdapterOperationModeTest() override;

protected:
  using GetOperationMode = pilz_msgs::GetOperationMode;
  using OperationModes = pilz_msgs::OperationModes;

protected:
  ros::AsyncSpinner spinner_{ 2 };
  ros::NodeHandle nh_;
  std::shared_ptr<ModbusAdapterOperationMode> adapter_operation_mode_;
  ros::Publisher modbus_topic_pub_;
  ros::ServiceClient operation_mode_client_;
  StrictMock<OperationModeSubscriberMock> subscriber_;
};

ModbusAdapterOperationModeTest::ModbusAdapterOperationModeTest()
{
  // Initialize for ROS time if not already initialized
  if (!ros::Time::isValid())
  {
    ros::Time::init();
  }

  adapter_operation_mode_.reset(new ModbusAdapterOperationMode(nh_, TEST_API_SPEC));
  modbus_topic_pub_ = nh_.advertise<ModbusMsgInStamped>(TOPIC_MODBUS_READ, DEFAULT_QUEUE_SIZE_MODBUS);
  operation_mode_client_ = nh_.serviceClient<GetOperationMode>(SERVICE_NAME_OPERATION_MODE);

  spinner_.start();
}

ModbusAdapterOperationModeTest::~ModbusAdapterOperationModeTest()
{
  // Before the destructors of the class members are called, we have
  // to ensure that all topic and service calls done by the AsyncSpinner
  // threads are finished. Otherwise, we sporadically will see threading
  // exceptions like:
  // "boost::mutex::~mutex(): Assertion `!res' failed".
  spinner_.stop();
}

/**
 * @brief Test increases function coverage by ensuring that all Dtor variants
 * are called.
 */
TEST_F(ModbusAdapterOperationModeTest, testAdapterOperationModeDtor)
{
  std::shared_ptr<AdapterOperationMode> adapter_op_mode(new AdapterOperationMode(nh_));
}

/**
 * @brief Test increases function coverage by ensuring that all Dtor variants
 * are called.
 */
TEST_F(ModbusAdapterOperationModeTest, testModbusMsgOperationModeWrapperExceptionDtor)
{
  std::shared_ptr<ModbusMsgOperationModeWrapperException> ex(new ModbusMsgOperationModeWrapperException("Test "
                                                                                                        "message"));
}

/**
 * @brief Test increases function coverage by ensuring that all Dtor variants
 * are called.
 */
TEST_F(ModbusAdapterOperationModeTest, testModbusMsgOperationModeWrapperDtor)
{
  ModbusMsgInBuilder builder(TEST_API_SPEC);
  builder.setApiVersion(MODBUS_API_VERSION_REQUIRED).setOperationMode(OperationModes::T1);
  std::shared_ptr<ModbusMsgOperationModeWrapper> wrapper(
      new ModbusMsgOperationModeWrapper(builder.build(ros::Time::now()), TEST_API_SPEC));
}

MATCHER_P(IsExpectedOperationMode, exp_mode, "unexpected operation mode")
{
  return arg->value == exp_mode;
}

/**
 * @brief Tests that initial operation mode is UNKNOWN.
 *
 * Waits for callback and calls service.
 */
TEST_F(ModbusAdapterOperationModeTest, testInitialOperationMode)
{
  EXPECT_CALL(subscriber_, callback(IsExpectedOperationMode(OperationModes::UNKNOWN)))
      .WillOnce(ACTION_OPEN_BARRIER_VOID(OPERATION_MODE_CALLBACK_EVENT));

  subscriber_.initialize();

  BARRIER(OPERATION_MODE_CALLBACK_EVENT);

  GetOperationMode srv;
  ASSERT_TRUE(operation_mode_client_.call(srv));
  EXPECT_EQ(OperationModes::UNKNOWN, srv.response.mode.value);
}

/**
 * @tests{Get_OperationMode_mechanism,
 *  Tests that operation mode UNKNOWN is returned if the
 *  operation mode register is missing in the modbus message.
 * }
 *
 * Test Sequence:
 *    1. Subscribe to operation modes topic.
 *    2. Send modbus message containing operation mode T1.
 *    3. Send modbus message which does not contain an operation mode.
 *
 * Expected Results:
 *    1. Operation mode UNKNOWN is published.
 *    2. Operation mode T1 is published.
 *    3. Operation mode UNKNOWN is published.
 */
TEST_F(ModbusAdapterOperationModeTest, testMissingOperationModeRegister)
{
  /**********
   * Step 1 *
   **********/
  ROS_DEBUG("+++  Step 1 +++");
  EXPECT_CALL(subscriber_, callback(IsExpectedOperationMode(OperationModes::UNKNOWN)))
      .WillOnce(ACTION_OPEN_BARRIER_VOID(OPERATION_MODE_CALLBACK_EVENT));

  subscriber_.initialize();

  BARRIER(OPERATION_MODE_CALLBACK_EVENT);

  /**********
   * Step 2 *
   **********/
  EXPECT_CALL(subscriber_, callback(IsExpectedOperationMode(OperationModes::T1)))
      .WillOnce(ACTION_OPEN_BARRIER_VOID(OPERATION_MODE_CALLBACK_EVENT));

  ModbusMsgInBuilder builder(TEST_API_SPEC);
  builder.setApiVersion(MODBUS_API_VERSION_REQUIRED);

  ROS_DEBUG("+++  Step 2 +++");
  modbus_topic_pub_.publish(builder.setOperationMode(OperationModes::T1).build(ros::Time::now()));

  BARRIER(OPERATION_MODE_CALLBACK_EVENT);

  /**********
   * Step 3 *
   **********/
  EXPECT_CALL(subscriber_, callback(IsExpectedOperationMode(OperationModes::UNKNOWN)))
      .WillOnce(ACTION_OPEN_BARRIER_VOID(OPERATION_MODE_CALLBACK_EVENT));

  ModbusMsgInStampedPtr msg{ builder.setOperationMode(OperationModes::T1).build(ros::Time::now()) };

  ROS_DEBUG("+++  Step 3 +++");
  // Remove operation mode from modbus message
  ASSERT_GT(TEST_API_SPEC.getRegisterDefinition(modbus_api_spec::OPERATION_MODE),
            TEST_API_SPEC.getRegisterDefinition(modbus_api_spec::VERSION))
      << "For the test to work correctly, the operation mode register has to be stored in the last register.";
  msg->holding_registers.data.erase(--msg->holding_registers.data.end());
  const uint32_t new_offset = TEST_API_SPEC.getRegisterDefinition(modbus_api_spec::VERSION);
  msg->holding_registers.layout.data_offset = new_offset;
  ModbusMsgInBuilder::setDefaultLayout(&(msg->holding_registers.layout), new_offset,
                                       static_cast<uint32_t>(msg->holding_registers.data.size()));

  modbus_topic_pub_.publish(msg);

  BARRIER(OPERATION_MODE_CALLBACK_EVENT);
}

/**
 * @tests{Get_OperationMode_mechanism,
 *  Tests the handling of an incoming modbus message informing
 *  about changing operation mode.
 * }
 *
 * Test Sequence:
 *  1. Subscribe to operation modes topic.
 *  2. Publish modbus message informing about changing operation mode. Repeat for all possible operation modes.
 *
 * Expected Results:
 *  1. Operation mode UNKNOWN is published.
 *  2. All operation modes are published in the order from above.
 *     GetOperationMode service returns expected operation mode.
 */
TEST_F(ModbusAdapterOperationModeTest, testOperationModeChange)
{
  /**********
   * Step 1 *
   **********/
  ROS_DEBUG("+++  Step 1 +++");
  EXPECT_CALL(subscriber_, callback(IsExpectedOperationMode(OperationModes::UNKNOWN)))
      .WillOnce(ACTION_OPEN_BARRIER_VOID(OPERATION_MODE_CALLBACK_EVENT));

  subscriber_.initialize();

  BARRIER(OPERATION_MODE_CALLBACK_EVENT);

  /**********
   * Step 2 *
   **********/
  ModbusMsgInBuilder builder(TEST_API_SPEC);
  builder.setApiVersion(MODBUS_API_VERSION_REQUIRED);
  for (const auto& mode : OPERATION_MODES)
  {
    EXPECT_CALL(subscriber_, callback(IsExpectedOperationMode(mode)))
        .WillOnce(ACTION_OPEN_BARRIER_VOID(OPERATION_MODE_CALLBACK_EVENT));

    modbus_topic_pub_.publish(builder.setOperationMode(mode).build(ros::Time::now()));

    BARRIER(OPERATION_MODE_CALLBACK_EVENT);

    GetOperationMode srv;
    ASSERT_TRUE(operation_mode_client_.call(srv));
    EXPECT_EQ(mode, srv.response.mode.value);
  }
}

/**
 * @tests{Get_OperationMode_mechanism,
 *  Tests the handling of an incoming modbus message informing
 *  about a disconnect.
 * }
 *
 * Test Sequence:
 *  1. Subscribe to operation modes topic.
 *  2. Publish modbus message informing about operation mode T1.
 *  3. Publish modbus message informing about disconnect.
 *
 * Expected Results:
 *  1. Operation mode UNKNOWN is published.
 *  2. Operation mode T1 is published.
 *  3. Operation mode UNKNOWN is published.
 */
TEST_F(ModbusAdapterOperationModeTest, testDisconnect)
{
  /**********
   * Step 1 *
   **********/
  EXPECT_CALL(subscriber_, callback(IsExpectedOperationMode(OperationModes::UNKNOWN)))
      .WillOnce(ACTION_OPEN_BARRIER_VOID(OPERATION_MODE_CALLBACK_EVENT));

  subscriber_.initialize();

  BARRIER(OPERATION_MODE_CALLBACK_EVENT);

  /**********
   * Step 2 *
   **********/
  EXPECT_CALL(subscriber_, callback(IsExpectedOperationMode(OperationModes::T1)))
      .WillOnce(ACTION_OPEN_BARRIER_VOID(OPERATION_MODE_CALLBACK_EVENT));

  ModbusMsgInBuilder builder(TEST_API_SPEC);
  builder.setApiVersion(MODBUS_API_VERSION_REQUIRED).setOperationMode(OperationModes::T1);

  modbus_topic_pub_.publish(builder.build(ros::Time::now()));

  BARRIER(OPERATION_MODE_CALLBACK_EVENT);

  /**********
   * Step 3 *
   **********/
  EXPECT_CALL(subscriber_, callback(IsExpectedOperationMode(OperationModes::UNKNOWN)))
      .WillOnce(ACTION_OPEN_BARRIER_VOID(OPERATION_MODE_CALLBACK_EVENT));

  uint32_t offset{ 0 };
  RegCont holding_register;
  ModbusMsgInStampedPtr msg{ ModbusMsgInBuilder::createDefaultModbusMsgIn(offset, holding_register) };
  msg->disconnect.data = true;
  modbus_topic_pub_.publish(msg);

  BARRIER(OPERATION_MODE_CALLBACK_EVENT);
}

/**
 * @tests{Get_OperationMode_mechanism,
 *  Tests the handling of an incoming modbus message with
 *  unexpected operation mode.
 * }
 *
 * Test Sequence:
 *  1. Subscribe to operation modes topic.
 *  2. Publish modbus message with operation mode T1.
 *  3. Publish modbus message with an unexpected operation mode.
 *
 * Expected Results:
 *  1. Operation mode UNKNOWN is published.
 *  2. Operation mode T1 is published.
 *  3. Operation mode UNKNOWN is published.
 */
TEST_F(ModbusAdapterOperationModeTest, testModbusUnexpectedOperationMode)
{
  /**********
   * Step 1 *
   **********/
  EXPECT_CALL(subscriber_, callback(IsExpectedOperationMode(OperationModes::UNKNOWN)))
      .WillOnce(ACTION_OPEN_BARRIER_VOID(OPERATION_MODE_CALLBACK_EVENT));

  subscriber_.initialize();

  BARRIER(OPERATION_MODE_CALLBACK_EVENT);

  /**********
   * Step 2 *
   **********/
  EXPECT_CALL(subscriber_, callback(IsExpectedOperationMode(OperationModes::T1)))
      .WillOnce(ACTION_OPEN_BARRIER_VOID(OPERATION_MODE_CALLBACK_EVENT));

  ModbusMsgInBuilder builder(TEST_API_SPEC);
  builder.setApiVersion(MODBUS_API_VERSION_REQUIRED).setOperationMode(OperationModes::T1);

  modbus_topic_pub_.publish(builder.build(ros::Time::now()));

  BARRIER(OPERATION_MODE_CALLBACK_EVENT);

  /**********
   * Step 3 *
   **********/
  EXPECT_CALL(subscriber_, callback(IsExpectedOperationMode(OperationModes::UNKNOWN)))
      .WillOnce(ACTION_OPEN_BARRIER_VOID(OPERATION_MODE_CALLBACK_EVENT));

  builder.setApiVersion(MODBUS_API_VERSION_REQUIRED).setOperationMode(1234 /* stupid value */);
  modbus_topic_pub_.publish(builder.build(ros::Time::now()));

  BARRIER(OPERATION_MODE_CALLBACK_EVENT);
}

/**
 * @tests{Get_OperationMode_mechanism,
 *  Tests the handling of an incoming modbus message with incorrect api version.
 * }
 *
 * Test Sequence:
 *  1. Publish modbus message with operation mode T1 and correct version.
 *  2. Publish modbus message with operation mode T2 and incorrect version.
 *
 * Expected Results:
 *  1. The version is T1
 *  2. The version is Unknown
 */
TEST_F(ModbusAdapterOperationModeTest, testModbusIncorrectApiVersion)
{
  /**********
   * Step 0 *
   **********/
  EXPECT_CALL(subscriber_, callback(IsExpectedOperationMode(OperationModes::UNKNOWN)))
      .WillOnce(ACTION_OPEN_BARRIER_VOID(OPERATION_MODE_CALLBACK_EVENT));

  subscriber_.initialize();

  BARRIER(OPERATION_MODE_CALLBACK_EVENT);

  /**********
   * Step 1 *
   **********/
  EXPECT_CALL(subscriber_, callback(IsExpectedOperationMode(OperationModes::T1)))
      .WillOnce(ACTION_OPEN_BARRIER_VOID(OPERATION_MODE_CALLBACK_EVENT));

  ModbusMsgInBuilder builder(TEST_API_SPEC);
  builder.setApiVersion(MODBUS_API_VERSION_REQUIRED).setOperationMode(OperationModes::T1);

  modbus_topic_pub_.publish(builder.build(ros::Time::now()));

  BARRIER(OPERATION_MODE_CALLBACK_EVENT);

  /**********
   * Step 2 *
   **********/
  EXPECT_CALL(subscriber_, callback(IsExpectedOperationMode(OperationModes::UNKNOWN)))
      .WillOnce(ACTION_OPEN_BARRIER_VOID(OPERATION_MODE_CALLBACK_EVENT));

  builder.setApiVersion(0 /* wrong version */).setOperationMode(OperationModes::T2);
  modbus_topic_pub_.publish(builder.build(ros::Time::now()));

  BARRIER(OPERATION_MODE_CALLBACK_EVENT);
}

}  // namespace prbt_hardware_support

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "unittest_modbus_adapter_operation_mode");
  ros::NodeHandle nh;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
