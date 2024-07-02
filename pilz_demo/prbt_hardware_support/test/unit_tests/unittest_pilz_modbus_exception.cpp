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
#include <memory>
#include <string>

#include <prbt_hardware_support/pilz_modbus_exceptions.h>

namespace pilz_modbus_exception_test
{
/**
 * @brief Test message setting
 */
TEST(PilzModbusExceptionTests, testMsgWhat)
{
  const std::string test_string{ "TEST_STRING" };
  std::shared_ptr<ModbusExceptionDisconnect> exception(new ModbusExceptionDisconnect(test_string));
  EXPECT_EQ(test_string, exception->what());
}

}  // namespace pilz_modbus_exception_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
