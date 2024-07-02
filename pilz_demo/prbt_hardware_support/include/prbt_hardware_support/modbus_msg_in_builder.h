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

#ifndef MODBUS_MSG_IN_BUILDER_H
#define MODBUS_MSG_IN_BUILDER_H

#include <map>

#include <std_msgs/MultiArrayLayout.h>

#include <prbt_hardware_support/ModbusMsgInStamped.h>
#include <prbt_hardware_support/modbus_api_spec.h>
#include <prbt_hardware_support/register_container.h>

namespace prbt_hardware_support
{
/**
 * @brief Help on construction for ModbusMsgIn Messages.
 *
 * Mainly intended for usage in tests.
 *
 */
class ModbusMsgInBuilder
{
public:
  ModbusMsgInBuilder(const ModbusApiSpec& api_spec);

public:
  void setRegister(const unsigned int register_n, const uint16_t value);

  ModbusMsgInBuilder& setRunPermitted(const uint16_t run_permitted);
  ModbusMsgInBuilder& setOperationMode(const uint16_t mode);
  ModbusMsgInBuilder& setApiVersion(const uint16_t version);

  ModbusMsgInStampedPtr build(const ros::Time& time) const;

public:
  /**
   * @brief Creates a standard ModbusMsgIn which contains default values for
   * all essential elements of the message.
   */
  static ModbusMsgInStampedPtr createDefaultModbusMsgIn(const std_msgs::MultiArrayLayout::_data_offset_type& offset,
                                                        const RegCont& holding_register);

  static void setDefaultLayout(std_msgs::MultiArrayLayout* layout,
                               const std_msgs::MultiArrayLayout::_data_offset_type& offset,
                               const RegCont::size_type& size);

private:
  const ModbusApiSpec api_spec_;
  std::map<unsigned int, uint16_t> register_values_;
};

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
inline ModbusMsgInBuilder& ModbusMsgInBuilder::setRunPermitted(const uint16_t run_permitted)
{
  setRegister(api_spec_.getRegisterDefinition(modbus_api_spec::RUN_PERMITTED), run_permitted);
  return *this;
}

inline ModbusMsgInBuilder& ModbusMsgInBuilder::setOperationMode(const uint16_t mode)
{
  setRegister(api_spec_.getRegisterDefinition(modbus_api_spec::OPERATION_MODE), mode);
  return *this;
}

inline ModbusMsgInBuilder& ModbusMsgInBuilder::setApiVersion(const uint16_t version)
{
  setRegister(api_spec_.getRegisterDefinition(modbus_api_spec::VERSION), version);
  return *this;
}

inline void ModbusMsgInBuilder::setRegister(const unsigned int register_n, const uint16_t value)
{
  register_values_[register_n] = value;
}

}  // namespace prbt_hardware_support

#endif  // MODBUS_MSG_IN_BUILDER_H
