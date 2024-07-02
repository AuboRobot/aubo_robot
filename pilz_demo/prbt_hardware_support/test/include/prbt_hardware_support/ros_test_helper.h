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

#ifndef PRBT_HARDWARE_SUPPORT_ROS_TEST_HELPER_H
#define PRBT_HARDWARE_SUPPORT_ROS_TEST_HELPER_H

#include <string>
#include <algorithm>

#include <ros/ros.h>
#include <gtest/gtest.h>

namespace prbt_hardware_support
{
/**
 * @brief Checks if a node is up and running.
 * @param node_name
 * @return True if the node was found, false otherwise.
 */
inline bool checkForNode(std::string node_name)
{
  std::vector<std::string> node_names;
  return (ros::master::getNodes(node_names) &&
          std::find(node_names.begin(), node_names.end(), node_name) == node_names.end());
}

/**
 * @brief Blocks until a node defined by node_name comes up.
 * @param node_name The name of the node to wait for.
 * @param loop_frequency Frequency at which the system is checked for the node.
 */
inline ::testing::AssertionResult waitForNode(const std::string node_name, const double loop_frequency = 10.0,
                                              const ros::Duration timeout = ros::Duration(10.0))
{
  ROS_INFO_STREAM("Waiting for Node " << node_name);
  std::vector<std::string> node_names;
  const auto timeout_time = ros::Time::now() + timeout;

  while (ros::master::getNodes(node_names) &&
         std::find(node_names.begin(), node_names.end(), node_name) == node_names.end())
  {
    if (ros::Time::now() > timeout_time)
    {
      return ::testing::AssertionFailure() << "Timeout while waiting for node: \"" << node_name << "\"";
    }
    ros::Rate(loop_frequency).sleep();
  }
  return ::testing::AssertionSuccess() << "Found note \"" << node_name << "\"";
}

/**
 * @brief Blocks until the specified number of subscribers is registered on a topic.
 * @param publisher The publisher of the topic.
 * @param subsciber_count Number of subscribers to wait for.
 * @param loop_frequency Frequency at which topic is checked for subscribers.
 */
inline ::testing::AssertionResult waitForSubscriber(const ros::Publisher& publisher,
                                                    const unsigned int subscriber_count = 1,
                                                    const double loop_frequency = 10.0,
                                                    const ros::Duration timeout = ros::Duration(10.0))
{
  ROS_INFO_STREAM("Waiting for " << subscriber_count << " subscriber to topic \"" << publisher.getTopic() << "\"");
  ros::Rate rate(loop_frequency);
  const auto timeout_time = ros::Time::now() + timeout;
  while (publisher.getNumSubscribers() < subscriber_count && ros::ok())
  {
    if (ros::Time::now() > timeout_time)
    {
      return ::testing::AssertionFailure() << "Timeout while waiting for " << subscriber_count
                                           << " subscriber to topic \"" << publisher.getTopic() << "\"";
    }
    rate.sleep();
  }
  return ::testing::AssertionSuccess() << "Registered " << subscriber_count << " subscriber to topic \""
                                       << publisher.getTopic() << "\"";
}

/**
 * @brief Blocks until a node defined by node_name can no longer be found.
 * @param node_name
 * @param timeout
 * @param loop_frequency Frequency at which the system is checked for the node.
 */
inline ::testing::AssertionResult waitForNodeShutdown(std::string node_name, double timeout = 1.0,
                                                      double loop_frequency = 10.0)
{
  std::vector<std::string> node_names;
  auto start_time = ros::Time::now();
  while (ros::master::getNodes(node_names) &&
         std::find(node_names.begin(), node_names.end(), node_name) != node_names.end())
  {
    node_names.clear();
    if (ros::Time::now() > start_time + ros::Duration(timeout))
    {
      return ::testing::AssertionFailure() << "Timed out waiting for shutdown of Node " << node_name;
    }
    ros::Rate(loop_frequency).sleep();
  }
  return ::testing::AssertionSuccess();
}

template <class T>
static void initalizeAndRun(T& obj, const char* ip, unsigned int port)
{
  if (!obj.init(ip, port))
  {
    ROS_ERROR("Initialization failed.");
    return;
  }
  ROS_INFO_STREAM("Starting Server on " << ip << ":" << port);

  obj.run();
}

}  // namespace prbt_hardware_support

#endif  // PRBT_HARDWARE_SUPPORT_ROS_TEST_HELPER_H
