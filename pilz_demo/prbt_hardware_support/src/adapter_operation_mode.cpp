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

#include <prbt_hardware_support/adapter_operation_mode.h>

namespace prbt_hardware_support
{
static const std::string TOPIC_OPERATION_MODE = "/prbt/operation_mode";
static const std::string SERVICE_NAME_GET_OPERATION_MODE = "/prbt/get_operation_mode";

static constexpr int DEFAULT_QUEUE_SIZE{ 10 };

AdapterOperationMode::AdapterOperationMode(ros::NodeHandle& nh) : nh_(nh)
{
  op_mode_.time_stamp = ros::Time::now();
  op_mode_.value = pilz_msgs::OperationModes::UNKNOWN;

  operation_mode_pub_ = nh_.advertise<pilz_msgs::OperationModes>(TOPIC_OPERATION_MODE, DEFAULT_QUEUE_SIZE,
                                                                 true);  // latched publisher
  // publish initial operation mode before first switch
  operation_mode_pub_.publish(op_mode_);

  initOperationModeService();
}

void AdapterOperationMode::initOperationModeService()
{
  operation_mode_server_ =
      nh_.advertiseService(SERVICE_NAME_GET_OPERATION_MODE, &AdapterOperationMode::getOperationMode, this);
}

void AdapterOperationMode::updateOperationMode(const pilz_msgs::OperationModes& new_op_mode)
{
  std::unique_lock<std::mutex> lock(op_mode_mutex_);
  const int8_t last_op_mode_value{ op_mode_.value };
  op_mode_ = new_op_mode;
  lock.unlock();

  if (new_op_mode.value != last_op_mode_value)
  {
    ROS_INFO_STREAM("Operation Mode switch: " << static_cast<int>(last_op_mode_value) << " -> "
                                              << static_cast<int>(new_op_mode.value));
    operation_mode_pub_.publish(new_op_mode);
  }
}

bool AdapterOperationMode::getOperationMode(pilz_msgs::GetOperationMode::Request& /*req*/,
                                            pilz_msgs::GetOperationMode::Response& res)
{
  std::lock_guard<std::mutex> lock(op_mode_mutex_);
  res.mode = op_mode_;
  return true;
}

}  // namespace prbt_hardware_support
