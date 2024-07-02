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

#include <atomic>
#include <algorithm>
#include <mutex>
#include <thread>
#include <memory>

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <pilz_testutils/joint_state_publisher_mock.h>
#include <pilz_utils/wait_for_message.h>

#include <prbt_hardware_support/brake_test_utils.h>
#include <prbt_hardware_support/brake_test_utils_exception.h>

namespace brake_test_utils_test
{
using namespace prbt_hardware_support;
using sensor_msgs::JointState;
using sensor_msgs::JointStateConstPtr;
using sensor_msgs::JointStatePtr;

static const std::string JOINT_STATES_TOPIC_NAME{ "/prbt/joint_states" };

/**
 * @brief Checks for identical names and positions in joint state messages.
 */
static ::testing::AssertionResult compareJointStateMessages(const JointStateConstPtr& msg1,
                                                            const JointStateConstPtr& msg2)
{
  if (msg1->name.size() != msg2->name.size())
  {
    return ::testing::AssertionFailure() << "Joint numbers in joint state messages do not match.";
  }
  if (!std::equal(msg1->name.begin(), msg1->name.end(), msg2->name.begin(),
                  [](const std::string& name1, const std::string& name2) { return name1 == name2; }))
  {
    return ::testing::AssertionFailure() << "Joint names in joint state messages do not match.";
  }
  if (msg1->position.size() != msg2->position.size())
  {
    return ::testing::AssertionFailure() << "Joint numbers in joint state messages do not match.";
  }
  if (!BrakeTestUtils::compareJointStatePositions(msg1, msg2))
  {
    return ::testing::AssertionFailure() << "Joint positions in joint state messages do not match.";
  }

  return ::testing::AssertionSuccess();
}

/**
 * @brief Test increases function coverage by ensuring that all Dtor variants
 * are called.
 */
TEST(BrakeTestUtilsTest, testExceptionDtor)
{
  {
    std::shared_ptr<BrakeTestUtilsException> ex{ new BrakeTestUtilsException("Test msg") };
  }

  {
    std::shared_ptr<GetCurrentJointStatesException> ex{ new GetCurrentJointStatesException("Test msg") };
  }
}

/**
 * Test the compareJointStatePositions utility function
 *
 * Test Sequence:
 *  1. Compare two equal joint states
 *  2. Compare two joint states with a difference in all joints slightly below the tolerance
 *  3. Compare two joint_states with a difference in one joint slightly above the tolerance,
 *     repeat the test for all joints.
 *
 * Expected Results:
 *  1. Returns true
 *  2. Returns true
 *  3. Always returns false
 */
TEST(BrakeTestUtilsTest, testCompareJointStatePositions)
{
  /**********
   * Step 1 *
   **********/
  JointStatePtr msg1{ boost::make_shared<JointState>() };
  msg1->name = { "joint1", "joint2" };
  msg1->position = { 0.1, 0.11 };

  JointStatePtr msg2{ boost::make_shared<JointState>() };
  msg2->name = { "joint1", "joint2" };
  msg2->position = { 0.1, 0.11 };

  EXPECT_TRUE(BrakeTestUtils::compareJointStatePositions(msg1, msg2));

  /**********
   * Step 2 *
   **********/
  double tolerance = 0.0001;
  for (double& pos : msg2->position)
  {
    pos += 0.9 * tolerance;
  }

  EXPECT_TRUE(BrakeTestUtils::compareJointStatePositions(msg1, msg2, tolerance));

  /**********
   * Step 3 *
   **********/
  for (size_t i = 0; i < msg2->position.size(); ++i)
  {
    for (size_t j = 0; j < msg2->position.size(); ++j)
    {
      msg2->position[j] = msg1->position[j];
    }
    msg2->position[i] += 1.1 * tolerance;

    EXPECT_FALSE(BrakeTestUtils::compareJointStatePositions(msg1, msg2, tolerance));
  }
}

/**
 * Test the getCurrentJointStates utility function
 *
 * Test Sequence:
 *  1. Call getCurrentJointStates() without publishing to the joint_states topic
 *  2. Call getCurrentJointStates() and publish on the joint_states topic asynchronously
 *
 * Expected Results:
 *  1. Exception is thrown
 *  2. Published message is obtained
 */
TEST(BrakeTestUtilsTest, testGetCurrentJointStates)
{
  /**********
   * Step 1 *
   **********/
  EXPECT_THROW(BrakeTestUtils::getCurrentJointStates(), GetCurrentJointStatesException);

  /**********
   * Step 2 *
   **********/
  pilz_testutils::JointStatePublisherMock joint_states_pub;
  const double joint1_start_position{ 0.92 };
  joint_states_pub.startPublishingAsync(joint1_start_position);

  pilz_utils::waitForMessage<JointState>(JOINT_STATES_TOPIC_NAME);

  auto expected_msg = joint_states_pub.getNextMessage();

  try
  {
    auto msg = BrakeTestUtils::getCurrentJointStates();
    EXPECT_TRUE(compareJointStateMessages(msg, expected_msg));
  }
  catch (const std::exception& e)
  {
    ADD_FAILURE() << e.what();
  }

  joint_states_pub.stopPublishing();
}

/**
 * Test the detectRobotMotion utility function
 *
 * Test Sequence:
 *  1. Publish fixed joint states and call detectRobotMotion()
 *  2. Publish changing joint states and call detectRobotMotion()
 *
 * Expected Results:
 *  1. Returns false.
 *  2. Returns true.
 */
TEST(BrakeTestUtilsTest, testDetectRobotMotion)
{
  /**********
   * Step 1 *
   **********/
  pilz_testutils::JointStatePublisherMock joint_states_pub;
  joint_states_pub.startPublishingAsync();

  pilz_utils::waitForMessage<JointState>(JOINT_STATES_TOPIC_NAME);

  EXPECT_FALSE(BrakeTestUtils::detectRobotMotion());

  /**********
   * Step 2 *
   **********/
  joint_states_pub.setJoint1Velocity(0.1);

  EXPECT_TRUE(BrakeTestUtils::detectRobotMotion());

  joint_states_pub.stopPublishing();
}

}  // namespace brake_test_utils_test

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "unittest_brake_test_utils");
  ros::NodeHandle nh;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
