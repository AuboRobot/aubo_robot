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

#ifndef UPDATE_FILTER_H
#define UPDATE_FILTER_H

#include <message_filters/simple_filter.h>

#include <vector>

namespace message_filters
{
/**
 * @brief Filters consecutive messages with the same timestamp. Only the first message passes, all consecutive
 * are dropped. It is templated on the message type to be filtered.
 *
 * \code
 *   message_filters::Subscriber sub<MsgStamped>(nh, TOPIC_NAME, 1);
 *   message_filters::UpdateFilter filter<MsgStamped>(&sub));
 *
 *   // Register all callback receiving filtered output
 *   filter.registerCallback(&filteredCallback);
 * \endcode
 */
template <typename M>
class UpdateFilter : public SimpleFilter<M>
{
public:
  typedef boost::shared_ptr<M const> MConstPtr;
  typedef ros::MessageEvent<M const> EventType;

  /**
   * @brief Construct the filter and connect to the output of another filter
   */
  template <typename F>
  UpdateFilter(F& f)
  {
    connectInput(f);
  }

  /**
   * @brief Connect to the output of another filter
   */
  template <class F>
  void connectInput(F& f)
  {
    incoming_connection_.disconnect();
    incoming_connection_ =
        f.registerCallback(typename UpdateFilter<M>::EventCallback(boost::bind(&UpdateFilter::cb, this, _1)));
  }

private:
  void cb(const EventType& evt)
  {
    if (evt.getMessage()->header.stamp == last_message_time_)
    {
      return;
    }

    last_message_time_ = evt.getMessage()->header.stamp;

    this->signalMessage(evt);
  }

  Connection incoming_connection_;

  ros::Time last_message_time_;
};

}  // namespace message_filters

#endif  // UPDATE_FILTER_H
