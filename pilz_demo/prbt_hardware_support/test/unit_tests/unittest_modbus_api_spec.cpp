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
#include <string>
#include <memory>

#include <prbt_hardware_support/modbus_api_spec.h>

using namespace prbt_hardware_support;
using namespace prbt_hardware_support::modbus_api_spec;
using namespace ::testing;

class NodeHandleMock
{
public:
  MOCK_CONST_METHOD0(getNamespace, std::string(void));
  MOCK_CONST_METHOD2(getParam, bool(const std::string& key, XmlRpc::XmlRpcValue& v));
};

/**
 * @brief Test increases function coverage by ensuring that all Dtor variants
 * are called.
 */
TEST(ModbusApiSpecTest, testModbusApiSpecExceptionDtor)
{
  std::shared_ptr<ModbusApiSpecException> ex{ new ModbusApiSpecException("Test msg") };
}

TEST(ModbusApiSpecTest, ConstructionViaInitilizerListRead)
{
  ModbusApiSpec api_spec{ { "test", 123 } };
  ASSERT_TRUE(api_spec.hasRegisterDefinition("test"));
  EXPECT_EQ(123u, api_spec.getRegisterDefinition("test"));

  EXPECT_EQ(123u, api_spec.getMinRegisterDefinition());
  EXPECT_EQ(123u, api_spec.getMaxRegisterDefinition());
}

TEST(ModbusApiSpecTest, ConstructionViaInitilizerListReadNonExistent)
{
  ModbusApiSpec api_spec{};
  EXPECT_THROW(api_spec.getRegisterDefinition("test"), ModbusApiSpecException);
  EXPECT_THROW(api_spec.getMinRegisterDefinition(), ModbusApiSpecException);
  EXPECT_THROW(api_spec.getMaxRegisterDefinition(), ModbusApiSpecException);
}

TEST(ModbusApiSpecTest, ConstructionViaInitilizerListOverwrite)
{
  ModbusApiSpec api_spec{ { "test", 123 }, { "dummy", 234 } };

  EXPECT_EQ(123u, api_spec.getMinRegisterDefinition());
  EXPECT_EQ(234u, api_spec.getMaxRegisterDefinition());

  api_spec.setRegisterDefinition("test", 555);
  ASSERT_TRUE(api_spec.hasRegisterDefinition("test"));
  EXPECT_EQ(555u, api_spec.getRegisterDefinition("test"));

  EXPECT_EQ(234u, api_spec.getMinRegisterDefinition());
  EXPECT_EQ(555u, api_spec.getMaxRegisterDefinition());
}

TEST(ModbusApiSpecTest, NodeHandleConstructionSimpleRead)
{
  XmlRpc::XmlRpcValue rpc_value;
  rpc_value["A"] = 123;

  NodeHandleMock nh;
  EXPECT_CALL(nh, getParam("read_api_spec/", _)).WillOnce(DoAll(SetArgReferee<1>(rpc_value), Return(true)));

  ModbusApiSpecTemplated<NodeHandleMock> api_spec{ nh };

  ASSERT_TRUE(api_spec.hasRegisterDefinition("A"));
  EXPECT_EQ(123u, api_spec.getRegisterDefinition("A"));
}

TEST(ModbusApiSpecTest, NodeHandleConstructionMissingApiSpec)
{
  XmlRpc::XmlRpcValue rpc_value;
  rpc_value["A"] = 123;

  NodeHandleMock nh;
  EXPECT_CALL(nh, getParam("read_api_spec/", _)).Times(1).WillOnce(Return(false));

  // Just for the error message in the exception
  EXPECT_CALL(nh, getNamespace()).Times(1).WillOnce(Return("testnamespace"));

  EXPECT_THROW(ModbusApiSpecTemplated<NodeHandleMock>{ nh }, ModbusApiSpecException);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
