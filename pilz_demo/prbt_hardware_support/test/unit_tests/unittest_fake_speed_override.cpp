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

#include <ros/ros.h>

#include <gtest/gtest.h>

#include <pilz_msgs/GetSpeedOverride.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <prbt_hardware_support/FakeSpeedOverrideConfig.h>

namespace prbt_hardware_support
{
static constexpr double EXPECTED_DEFAULT_SPEED_OVERRIDE{ 1.0 };

class FakeSpeedOverrideTest : public testing::Test
{
protected:
  void SetUp() override;

  bool setSpeedOverrideParameter(double speed_override);

  ros::ServiceClient client;
  ros::ServiceClient dynamic_parameter_client;
  pilz_msgs::GetSpeedOverride srv;

  FakeSpeedOverrideConfig conf;
};

void FakeSpeedOverrideTest::SetUp()
{
  ros::NodeHandle n;
  client = n.serviceClient<pilz_msgs::GetSpeedOverride>("/prbt/get_speed_override");
  dynamic_parameter_client = n.serviceClient<dynamic_reconfigure::Reconfigure>("/fake_speed_override_node/"
                                                                               "set_parameters");
}

bool FakeSpeedOverrideTest::setSpeedOverrideParameter(double speed_override)
{
  dynamic_reconfigure::Reconfigure rec;
  dynamic_reconfigure::DoubleParameter speed_override_param;
  speed_override_param.name = "speed_override";
  speed_override_param.value = speed_override;

  rec.request.config.doubles.push_back(speed_override_param);

  return dynamic_parameter_client.call(rec);
}

/**
 * @tests{speed_override_for_simulated_robot,
 *  Tests the default value of the fake speed override.
 * }
 * @tests{speed_override_without_operation_mode_support,
 *  Tests the default value of the fake speed override.
 * }
 */
TEST_F(FakeSpeedOverrideTest, defaultSpeedOverride)
{
  ASSERT_TRUE(client.call(srv));
  EXPECT_EQ(srv.response.speed_override, EXPECTED_DEFAULT_SPEED_OVERRIDE);
}

TEST_F(FakeSpeedOverrideTest, testSettingSpeedOverride)
{
  ASSERT_TRUE(client.call(srv));
  EXPECT_EQ(srv.response.speed_override, conf.__getDefault__().speed_override);

  const auto test_value = 0.5;
  EXPECT_TRUE(setSpeedOverrideParameter(test_value));
  EXPECT_TRUE(client.call(srv));
  EXPECT_EQ(srv.response.speed_override, test_value);
}

TEST_F(FakeSpeedOverrideTest, testSettingSpeedOverrideToLow)
{
  EXPECT_TRUE(setSpeedOverrideParameter(conf.__getMin__().speed_override - 0.1));

  ASSERT_TRUE(client.call(srv));
  EXPECT_EQ(srv.response.speed_override, conf.__getMin__().speed_override);
}

TEST_F(FakeSpeedOverrideTest, testSettingSpeedOverrideToHigh)
{
  EXPECT_TRUE(setSpeedOverrideParameter(conf.__getMax__().speed_override + 0.1));

  ASSERT_TRUE(client.call(srv));
  EXPECT_EQ(srv.response.speed_override, conf.__getMax__().speed_override);
}

}  // namespace prbt_hardware_support

int main(int argc, char** argv)
{
  ros::init(argc, argv, "unittest_fake_speed_override");
  ros::NodeHandle nh;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
