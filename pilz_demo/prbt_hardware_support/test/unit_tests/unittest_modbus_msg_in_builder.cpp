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

#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include <prbt_hardware_support/modbus_msg_in_builder.h>

namespace pilz_modbus_msg_in_utils_test
{
using namespace prbt_hardware_support;

/**
 * @brief Tests that exception is thrown if the register container size
 * exceeds the size allowed by the std_msgs::MultiArrayLayout class.
 */
TEST(ModbusMsgInBuilderTest, testOutOfRangeRegister)
{
  std::unique_ptr<std_msgs::MultiArrayLayout> layout{ new std_msgs::MultiArrayLayout() };
  const RegCont::size_type n{
    static_cast<RegCont::size_type>(std::numeric_limits<std_msgs::MultiArrayDimension::_size_type>::max()) + 1
  };
  EXPECT_THROW(ModbusMsgInBuilder::setDefaultLayout(layout.get(), 1, n), std::invalid_argument);
}

}  // namespace pilz_modbus_msg_in_utils_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
