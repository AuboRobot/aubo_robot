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

#ifndef FILTER_PIPELINE_H
#define FILTER_PIPELINE_H

#include <memory>
#include <stdexcept>

#include <functional>

#include <message_filters/subscriber.h>

#include <prbt_hardware_support/modbus_api_spec.h>
#include <prbt_hardware_support/ModbusMsgInStamped.h>
#include <prbt_hardware_support/update_filter.h>
#include <prbt_hardware_support/modbus_topic_definitions.h>

namespace prbt_hardware_support
{
/**
 * @brief An abstraction of a series of filters which ensures
 * that only Modbus messages with different timestamps pass the pipeline.
 */
class FilterPipeline
{
public:
  using TCallbackFunc = std::function<void(const ModbusMsgInStampedConstPtr&)>;

  FilterPipeline(ros::NodeHandle& nh, TCallbackFunc callback_func);

private:
  //! Subscribes to TOPIC_MODBUS_READ and redirects received messages
  //! to the update-filter.
  std::shared_ptr<message_filters::Subscriber<ModbusMsgInStamped> > modbus_read_sub_;

  //! Filters consecutive messages with the same timestamp.
  //! Passed messages are redirected to the callback_func.
  std::shared_ptr<message_filters::UpdateFilter<ModbusMsgInStamped> > update_filter_;
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
inline FilterPipeline::FilterPipeline(ros::NodeHandle& nh, TCallbackFunc callback_func)
{
  if (!callback_func)
  {
    throw std::invalid_argument("Argument \"callback_func\" must not be empty");
  }
  modbus_read_sub_ = std::make_shared<message_filters::Subscriber<ModbusMsgInStamped> >(nh, TOPIC_MODBUS_READ, 1);
  update_filter_ = std::make_shared<message_filters::UpdateFilter<ModbusMsgInStamped> >(*modbus_read_sub_);
  update_filter_->registerCallback(callback_func);
}

}  // namespace prbt_hardware_support

#endif  // FILTER_PIPELINE_H
