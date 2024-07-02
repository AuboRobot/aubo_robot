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
#include <memory>
#include <boost/bind.hpp>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <ros/ros.h>

#include <prbt_hardware_support/update_filter.h>
#include <prbt_hardware_support/ModbusMsgInStamped.h>
#include <message_filters/pass_through.h>
#include <message_filters/subscriber.h>

namespace mf = message_filters;

template class message_filters::UpdateFilter<prbt_hardware_support::ModbusMsgInStamped>;

namespace update_filter_test
{
using ::testing::_;
using ::testing::AtLeast;
using ::testing::Return;
using ::testing::Throw;
using namespace prbt_hardware_support;

class CallbackReceiver
{
public:
  MOCK_METHOD1(modbusInMsgCallback, void(const ModbusMsgInStampedConstPtr& msg));
};

class TestPublisher : public ros::Publisher
{
public:
  TestPublisher(const Publisher& rhs) : ros::Publisher::Publisher(rhs)
  {
  }

  template <typename M>
  void publishAndSpin(const boost::shared_ptr<M>& message) const
  {
    this->publish(message);
    ros::spinOnce();
  }
};

/**
 * @brief UpdateFilterTest tests the UpdateFilter class
 */
class UpdateFilterTest : public testing::Test
{
public:
  void SetUp() override;
  void TearDown() override;
  std::shared_ptr<CallbackReceiver> callback_receiver_;
};

void UpdateFilterTest::SetUp()
{
  this->callback_receiver_.reset(new CallbackReceiver());
}

void UpdateFilterTest::TearDown()
{
  this->callback_receiver_ = nullptr;
}

/**
 * @brief Tests that the filter properly only passes message to the callback that have a newer timestamp
 */
TEST_F(UpdateFilterTest, testFilteringThroughSubscriber)
{
  std::string test_topic_name = "/test_topic";

  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<ModbusMsgInStamped>(test_topic_name, 1);
  TestPublisher test_pub(pub);
  message_filters::Subscriber<ModbusMsgInStamped> modbus_sub(nh, test_topic_name, 1);

  mf::UpdateFilter<ModbusMsgInStamped> update_filter(modbus_sub);

  // Register to modbus callback
  update_filter.registerCallback(boost::bind(&CallbackReceiver::modbusInMsgCallback, this->callback_receiver_, _1));

  // 3 Messages will be send, only two are new
  EXPECT_CALL(*(this->callback_receiver_.get()), modbusInMsgCallback(_)).Times(3);

  ModbusMsgInStampedPtr msg(new ModbusMsgInStamped());

  msg->header.stamp = ros::Time(1);
  test_pub.publishAndSpin(msg);  // Should pass - CALL 1
  test_pub.publishAndSpin(msg);

  msg->header.stamp = ros::Time(2);
  test_pub.publishAndSpin(msg);  // Should pass - CALL 2
  test_pub.publishAndSpin(msg);
  test_pub.publishAndSpin(msg);

  msg->header.stamp = ros::Time(3);
  test_pub.publishAndSpin(msg);  // Should pass - CALL 3
}

}  // namespace update_filter_test

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "unittest_update_filter");
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();  // Needed on message construction
  return RUN_ALL_TESTS();
}
