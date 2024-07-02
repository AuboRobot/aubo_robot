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

#include <string>
#include <vector>
#include <stdexcept>
#include <memory>
#include <functional>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <prbt_hardware_support/modbus_api_spec.h>
#include <prbt_hardware_support/param_names.h>
#include <prbt_hardware_support/ModbusMsgInStamped.h>
#include <prbt_hardware_support/modbus_msg_run_permitted_wrapper.h>
#include <prbt_hardware_support/modbus_msg_in_builder.h>
#include <prbt_hardware_support/modbus_adapter_run_permitted.h>
#include <prbt_hardware_support/register_container.h>

namespace prbt_hardware_support
{
static constexpr uint16_t RUN_PERMITTED_INVALID_VALUE{ 2 };

static const ModbusApiSpec TEST_API_SPEC{ { modbus_api_spec::VERSION, 513 }, { modbus_api_spec::RUN_PERMITTED, 512 } };

using namespace prbt_hardware_support;
using namespace modbus_api::v3;

using std::placeholders::_1;

class ModbusAdapterRunPermittedTest : public ::testing::Test
{
public:
  ModbusMsgInStampedPtr createDefaultRunPermittedModbusMsg(uint16_t run_permitted_clear);

public:
  MOCK_METHOD1(sendRunPermittedUpdate, void(const bool run_permitted));
};

ModbusMsgInStampedPtr ModbusAdapterRunPermittedTest::createDefaultRunPermittedModbusMsg(uint16_t run_permitted)
{
  static int msg_time_counter{ 1 };
  RegCont tab_reg(TEST_API_SPEC.size());
  tab_reg[0] = run_permitted;
  tab_reg[1] = MODBUS_API_VERSION_REQUIRED;
  ModbusMsgInStampedPtr msg{ ModbusMsgInBuilder::createDefaultModbusMsgIn(TEST_API_SPEC.getMinRegisterDefinition(),
                                                                          tab_reg) };
  msg->header.stamp = ros::Time(msg_time_counter++);
  return msg;
}

/**
 * @brief Test increases function coverage by ensuring that all Dtor variants
 * are called.
 */
TEST_F(ModbusAdapterRunPermittedTest, testModbusMsgWrapperExceptionDtor)
{
  std::shared_ptr<ModbusMsgWrapperException> es{ new ModbusMsgWrapperException("Test msg") };
}

/**
 * @brief Test increases function coverage by ensuring that all Dtor variants
 * are called.
 */
TEST_F(ModbusAdapterRunPermittedTest, testModbusMsgRunPermittedWrapperDtor)
{
  ModbusMsgInStampedConstPtr msg_const_ptr{ createDefaultRunPermittedModbusMsg(MODBUS_RUN_PERMITTED_TRUE) };
  std::shared_ptr<ModbusMsgRunPermittedWrapper> ex{ new ModbusMsgRunPermittedWrapper(msg_const_ptr, TEST_API_SPEC) };
}

/**
 * @brief Tests that a modbus message giving RUN_PERMITTED true leads to RUN_PERMITTED true update.
 */
TEST_F(ModbusAdapterRunPermittedTest, testRunPermittedClearMsg)
{
  EXPECT_CALL(*this, sendRunPermittedUpdate(true)).Times(1);

  ModbusAdapterRunPermitted run_permitted_adapter{ ModbusAdapterRunPermitted(
      std::bind(&ModbusAdapterRunPermittedTest::sendRunPermittedUpdate, this, _1), TEST_API_SPEC) };
  run_permitted_adapter.modbusMsgCallback(createDefaultRunPermittedModbusMsg(MODBUS_RUN_PERMITTED_TRUE));
}

/**
 * @brief Tests that a modbus message giving RUN_PERMITTED false leads to RUN_PERMITTED false update.
 */
TEST_F(ModbusAdapterRunPermittedTest, testRunPermittedActiveMsg)
{
  EXPECT_CALL(*this, sendRunPermittedUpdate(false)).Times(1);

  ModbusAdapterRunPermitted run_permitted_adapter{ ModbusAdapterRunPermitted(
      std::bind(&ModbusAdapterRunPermittedTest::sendRunPermittedUpdate, this, _1), TEST_API_SPEC) };
  run_permitted_adapter.modbusMsgCallback(createDefaultRunPermittedModbusMsg(MODBUS_RUN_PERMITTED_FALSE));
}

/**
 * @brief Tests that a modbus message giving an invalid RUN_PERMITTED status leads to RUN_PERMITTED false update.
 */
TEST_F(ModbusAdapterRunPermittedTest, testRunPermittedInvalidMsg)
{
  EXPECT_CALL(*this, sendRunPermittedUpdate(false)).Times(1);

  ModbusAdapterRunPermitted run_permitted_adapter{ ModbusAdapterRunPermitted(
      std::bind(&ModbusAdapterRunPermittedTest::sendRunPermittedUpdate, this, _1), TEST_API_SPEC) };
  run_permitted_adapter.modbusMsgCallback(createDefaultRunPermittedModbusMsg(RUN_PERMITTED_INVALID_VALUE));
}

/**
 * @tests{Stop1_Trigger,
 *  Tests that a modbus message indicating a disconnect from modbus leads
 *  to RUN_PERMITTED false update even if the recieved modbus message content would give
 *  RUN_PERMITTED true.
 * }
 */
TEST_F(ModbusAdapterRunPermittedTest, testDisconnectNoRunPermittedMsg)
{
  EXPECT_CALL(*this, sendRunPermittedUpdate(false)).Times(1);

  ModbusAdapterRunPermitted run_permitted_adapter{ ModbusAdapterRunPermitted(
      std::bind(&ModbusAdapterRunPermittedTest::sendRunPermittedUpdate, this, _1), TEST_API_SPEC) };

  ModbusMsgInStampedPtr msg = createDefaultRunPermittedModbusMsg(MODBUS_RUN_PERMITTED_TRUE);
  msg->disconnect.data = true;
  run_permitted_adapter.modbusMsgCallback(msg);
}

/**
 * @tests{Stop1_Trigger,
 *  Tests that a modbus message indicating a disconnect from modbus leads
 *  to RUN_PERMITTED false update if the received the modbus message itself
 *  would also give RUN_PERMITTED true.
 * }
 *
 */
TEST_F(ModbusAdapterRunPermittedTest, testDisconnectWithRunPermittedMsg)
{
  EXPECT_CALL(*this, sendRunPermittedUpdate(false)).Times(1);

  ModbusAdapterRunPermitted run_permitted_adapter{ ModbusAdapterRunPermitted(
      std::bind(&ModbusAdapterRunPermittedTest::sendRunPermittedUpdate, this, _1), TEST_API_SPEC) };

  ModbusMsgInStampedPtr msg = createDefaultRunPermittedModbusMsg(MODBUS_RUN_PERMITTED_FALSE);
  msg->disconnect.data = true;
  run_permitted_adapter.modbusMsgCallback(msg);
}

/**
 * @tests{Stop1_Trigger,
 *  Tests that a modbus message indicating a disconnect from modbus with no
 *  other data defined in the modbus message leads to RUN_PERMITTED false update
 * }
 */
TEST_F(ModbusAdapterRunPermittedTest, testDisconnectPure)
{
  EXPECT_CALL(*this, sendRunPermittedUpdate(false)).Times(1);

  ModbusAdapterRunPermitted run_permitted_adapter{ ModbusAdapterRunPermitted(
      std::bind(&ModbusAdapterRunPermittedTest::sendRunPermittedUpdate, this, _1), TEST_API_SPEC) };

  ModbusMsgInStampedPtr msg(new ModbusMsgInStamped());
  ros::Time::init();
  msg->header.stamp = ros::Time::now();
  msg->disconnect.data = true;
  run_permitted_adapter.modbusMsgCallback(msg);
}

/**
 * @tests{Stop1_Trigger,
 *  Tests that RUN_PERMITTED false update is sent if no version is defined in modbus
 *  message.
 * }
 */
TEST_F(ModbusAdapterRunPermittedTest, testNoVersion)
{
  EXPECT_CALL(*this, sendRunPermittedUpdate(false)).Times(1);

  ModbusAdapterRunPermitted run_permitted_adapter{ ModbusAdapterRunPermitted(
      std::bind(&ModbusAdapterRunPermittedTest::sendRunPermittedUpdate, this, _1), TEST_API_SPEC) };

  ModbusMsgInStampedPtr msg = createDefaultRunPermittedModbusMsg(MODBUS_RUN_PERMITTED_FALSE);
  msg->holding_registers.data.pop_back();
  run_permitted_adapter.modbusMsgCallback(msg);
}

/**
 * @tests{Stop1_Trigger,
 *  Tests that RUN_PERMITTED false update is sent if the modbus message contains
 *  a incorrect version number.
 * }
 */
TEST_F(ModbusAdapterRunPermittedTest, testWrongVersion)
{
  EXPECT_CALL(*this, sendRunPermittedUpdate(false)).Times(1);

  ModbusAdapterRunPermitted run_permitted_adapter{ ModbusAdapterRunPermitted(
      std::bind(&ModbusAdapterRunPermittedTest::sendRunPermittedUpdate, this, _1), TEST_API_SPEC) };

  ModbusMsgInStampedPtr msg = createDefaultRunPermittedModbusMsg(MODBUS_RUN_PERMITTED_FALSE);
  msg->holding_registers.data[1] = 0;
  run_permitted_adapter.modbusMsgCallback(msg);
}

/**
 * @tests{Stop1_Trigger,
 *  Tests that RUN_PERMITTED false update is sent if the modbus message contains
 *  version 1.
 * }
 *
 * @note Version 1 had mistake in specification on the hardware therefore
 * not supported at all.
 */
TEST_F(ModbusAdapterRunPermittedTest, testVersion1)
{
  EXPECT_CALL(*this, sendRunPermittedUpdate(false)).Times(1);

  ModbusAdapterRunPermitted run_permitted_adapter{ ModbusAdapterRunPermitted(
      std::bind(&ModbusAdapterRunPermittedTest::sendRunPermittedUpdate, this, _1), TEST_API_SPEC) };

  ModbusMsgInStampedPtr msg = createDefaultRunPermittedModbusMsg(MODBUS_RUN_PERMITTED_FALSE);
  msg->holding_registers.data[1] = 1;
  run_permitted_adapter.modbusMsgCallback(msg);
}

/**
 * @tests{Stop1_Trigger,
 *  Tests that RUN_PERMITTED false update is sent if the modbus message contains
 *  version 2.
 * }
 *
 * @note Version 2 is outdated and not used in the field therefore
 * not supported at all.
 */
TEST_F(ModbusAdapterRunPermittedTest, testVersion2)
{
  EXPECT_CALL(*this, sendRunPermittedUpdate(false)).Times(1);

  ModbusAdapterRunPermitted run_permitted_adapter{ ModbusAdapterRunPermitted(
      std::bind(&ModbusAdapterRunPermittedTest::sendRunPermittedUpdate, this, _1), TEST_API_SPEC) };

  ModbusMsgInStampedPtr msg = createDefaultRunPermittedModbusMsg(MODBUS_RUN_PERMITTED_FALSE);
  msg->holding_registers.data[1] = 2;
  run_permitted_adapter.modbusMsgCallback(msg);
}

/**
 * @tests{Stop1_Trigger,
 *  Test that RUN_PERMITTED false update is sent if the modbus message does not
 *  define/contain the RUN_PERMITTED flag.
 * }
 */
TEST_F(ModbusAdapterRunPermittedTest, testNoRunPermitted)
{
  EXPECT_CALL(*this, sendRunPermittedUpdate(false)).Times(1);

  ModbusAdapterRunPermitted run_permitted_adapter{ ModbusAdapterRunPermitted(
      std::bind(&ModbusAdapterRunPermittedTest::sendRunPermittedUpdate, this, _1), TEST_API_SPEC) };

  ModbusMsgInStampedPtr msg = createDefaultRunPermittedModbusMsg(MODBUS_RUN_PERMITTED_FALSE);
  msg->holding_registers.data.erase(msg->holding_registers.data.begin());
  msg->holding_registers.layout.data_offset = TEST_API_SPEC.getRegisterDefinition(modbus_api_spec::VERSION);
  run_permitted_adapter.modbusMsgCallback(msg);
}

/**
 * @brief Check construction of the exception (essentially for full function coverage)
 */
TEST_F(ModbusAdapterRunPermittedTest, ModbusMsgExceptionCTOR)
{
  std::shared_ptr<ModbusMsgRunPermittedStatusMissing> exception_ptr{ new ModbusMsgRunPermittedStatusMissing("test") };
}

}  // namespace prbt_hardware_support

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
