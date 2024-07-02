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

#ifndef PRBT_HARDWARE_SUPPORT_BRAKE_TEST_UTILS_H
#define PRBT_HARDWARE_SUPPORT_BRAKE_TEST_UTILS_H

#include <math.h>
#include <algorithm>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <prbt_hardware_support/brake_test_utils_exception.h>

static const std::string TOPIC_NAME{ "/prbt/joint_states" };
static constexpr double JOINT_STATES_COMPARISON_FREQUENCY{ 10.0 };
static constexpr double JOINT_STATES_TOPIC_TIMEOUT{ 1.0 };
static constexpr double DEFAULT_JOINT_STATES_COMPARISON_TOLERANCE{ 0.001 };
static constexpr double DEFAULT_ROBOT_MOTION_TIMEOUT_S{ 1.0 };

namespace prbt_hardware_support
{
class BrakeTestUtils
{
public:
  /**
   * @brief return true if a robot motion was detected, false otherwise.
   *
   * This function periodically checks if the robot has moved by comparing joint positions from the joint_states topic
   * until the timeout is reached.
   */
  static bool detectRobotMotion(double timeout_s = DEFAULT_ROBOT_MOTION_TIMEOUT_S);

  /**
   * @brief wait for a single message on the joint_states topic and return it.
   *
   * @throws GetCurrentJointStatesException
   */
  inline static sensor_msgs::JointStateConstPtr getCurrentJointStates();

  /**
   * @brief return true if the joint state positions are equal up to a given tolerance, false otherwise.
   */
  inline static bool compareJointStatePositions(const sensor_msgs::JointStateConstPtr& msg1,
                                                const sensor_msgs::JointStateConstPtr& msg2,
                                                const double tol = DEFAULT_JOINT_STATES_COMPARISON_TOLERANCE);
};

bool BrakeTestUtils::detectRobotMotion(double timeout_s)
{
  auto msg_start{ getCurrentJointStates() };

  ros::Time start = ros::Time::now();
  ros::Rate r(JOINT_STATES_COMPARISON_FREQUENCY);
  while ((ros::Time::now() - start).toSec() < timeout_s && ros::ok())
  {
    auto msg_current{ getCurrentJointStates() };
    if (!compareJointStatePositions(msg_start, msg_current))
    {
      return true;
    }
    r.sleep();
  }

  return false;
}

sensor_msgs::JointStateConstPtr BrakeTestUtils::getCurrentJointStates()
{
  auto msg{ ros::topic::waitForMessage<sensor_msgs::JointState>(TOPIC_NAME,
                                                                ros::Duration(JOINT_STATES_TOPIC_TIMEOUT)) };
  if (msg == nullptr)
  {
    throw GetCurrentJointStatesException("Could not obtain message from joint_states topic.");
  }
  return msg;
}

bool BrakeTestUtils::compareJointStatePositions(const sensor_msgs::JointStateConstPtr& msg1,
                                                const sensor_msgs::JointStateConstPtr& msg2, const double tol)
{
  return std::equal(msg1->position.begin(), msg1->position.end(), msg2->position.begin(),
                    [tol](double pos1, double pos2) { return (fabs(pos1 - pos2) < tol); });
}

}  // namespace prbt_hardware_support

#endif  // PRBT_HARDWARE_SUPPORT_BRAKE_TEST_UTILS_H
