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

#include <memory>
#include <vector>

#include <prbt_hardware_support/modbus_api_spec.h>
#include <prbt_hardware_support/modbus_msg_wrapper.h>
#include <prbt_hardware_support/modbus_msg_in_builder.h>
#include <prbt_hardware_support/register_container.h>

using namespace prbt_hardware_support;

namespace modbus_msg_wrapper
{
static const ModbusApiSpec TEST_API_SPEC{ { modbus_api_spec::VERSION, 0 } };

/**
 * @brief Test increases function coverage by ensuring that all Dtor variants
 * are called.
 */
TEST(ModbusMsgWrapperTest, testModbusMsgWrapperDtor)
{
  uint32_t offset{ 0 };
  const uint16_t modbus_api_version{ 1 };
  RegCont holding_register{ modbus_api_version };
  ModbusMsgInStampedPtr msg{ ModbusMsgInBuilder::createDefaultModbusMsgIn(offset, holding_register) };

  ModbusMsgInStampedConstPtr msg_const_ptr{ msg };
  std::shared_ptr<ModbusMsgWrapper> ex(new ModbusMsgWrapper(msg_const_ptr, TEST_API_SPEC));
}

}  // namespace modbus_msg_wrapper

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
