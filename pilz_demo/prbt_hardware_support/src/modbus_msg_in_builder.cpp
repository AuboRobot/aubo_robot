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

#include <prbt_hardware_support/modbus_msg_in_builder.h>

#include <stdexcept>

#include <std_msgs/MultiArrayDimension.h>

namespace prbt_hardware_support
{
ModbusMsgInBuilder::ModbusMsgInBuilder(const ModbusApiSpec& api_spec) : api_spec_(api_spec)
{
}

ModbusMsgInStampedPtr ModbusMsgInBuilder::build(const ros::Time& time) const
{
  // Get the minimum and maximum register
  uint32_t first_index_to_read{ register_values_.cbegin()->first };
  uint32_t last_index_to_read{ register_values_.crbegin()->first };

  // Setup vector
  RegCont tab_reg(last_index_to_read - first_index_to_read + 1);
  for (auto reg : register_values_)
  {
    tab_reg[reg.first - first_index_to_read] = reg.second;
  }

  ModbusMsgInStampedPtr msg{ createDefaultModbusMsgIn(first_index_to_read, tab_reg) };
  msg->header.stamp = time;
  return msg;
}

void ModbusMsgInBuilder::setDefaultLayout(std_msgs::MultiArrayLayout* layout,
                                          const std_msgs::MultiArrayLayout::_data_offset_type& offset,
                                          const RegCont::size_type& size)
{
  if (size > std::numeric_limits<std_msgs::MultiArrayDimension::_size_type>::max())
  {
    throw std::invalid_argument("Argument \"size\" must not exceed max value of type "
                                "\"std_msgs::MultiArrayDimension::_size_type\"");
  }

  layout->data_offset = offset;
  layout->dim.push_back(std_msgs::MultiArrayDimension());
  layout->dim.back().size = static_cast<std_msgs::MultiArrayDimension::_size_type>(size);
  layout->dim.back().stride = 1;
  layout->dim.back().label = "Data in holding register";
}

ModbusMsgInStampedPtr ModbusMsgInBuilder::createDefaultModbusMsgIn(
    const std_msgs::MultiArrayLayout::_data_offset_type& offset, const RegCont& holding_register)
{
  ModbusMsgInStampedPtr msg{ new ModbusMsgInStamped() };
  setDefaultLayout(&(msg->holding_registers.layout), offset, holding_register.size());
  msg->holding_registers.data = holding_register;
  msg->disconnect.data = false;

  return msg;
}

}  // namespace prbt_hardware_support
