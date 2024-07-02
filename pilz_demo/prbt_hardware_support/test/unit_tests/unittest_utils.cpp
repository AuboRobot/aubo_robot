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

#include <prbt_hardware_support/utils.h>
#include <boost/core/demangle.hpp>

using namespace prbt_hardware_support;

namespace test_namespace
{
class TestClass
{
};
}  // namespace test_namespace

/**
 * @brief Test the function turning a given instance into a string of its classname
 */
TEST(ClassNameTest, testGettingClassName)
{
  test_namespace::TestClass test_instance;
  auto full_name = boost::core::demangle(typeid(test_instance).name());
  EXPECT_EQ("TestClass", className(full_name));
}

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
