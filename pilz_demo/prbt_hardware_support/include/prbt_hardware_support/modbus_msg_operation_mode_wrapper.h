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
#ifndef MODBUS_MSG_OPERATION_MODE_WRAPPER_H
#define MODBUS_MSG_OPERATION_MODE_WRAPPER_H

#include <pilz_msgs/OperationModes.h>

#include <prbt_hardware_support/ModbusMsgInStamped.h>
#include <prbt_hardware_support/modbus_api_definitions.h>
#include <prbt_hardware_support/modbus_api_spec.h>
#include <prbt_hardware_support/modbus_msg_wrapper.h>
#include <prbt_hardware_support/modbus_msg_operation_mode_wrapper_exception.h>

namespace prbt_hardware_support
{
using namespace modbus_api::v3;

/**
 * @brief Wrapper class to add semantic to a raw ModbusMsgInStamped
 *
 * Allows to easy access to the content behind a raw modbus message
 * which is assumed to contain data about the operation mode.
 */
class ModbusMsgOperationModeWrapper : public ModbusMsgWrapper
{
public:
  ModbusMsgOperationModeWrapper(const ModbusMsgInStampedConstPtr& modbus_msg_raw, const ModbusApiSpec& api_spec);

  /**
   * @brief Calls ModbusMsgWrapper::checkStructuralIntegrity().
   *
   * @throw ModbusMsgOperationModeWrapperException if operation mode
   * register is missing.
   */
  virtual void checkStructuralIntegrity() const override;

  /**
   * @brief Get the operation mode field from the Modbus message.
   *
   * @return The current operation mode according to OperationModes.msg
   */
  int8_t getOperationMode() const;

  /**
   * @returns the operation mode and the corresponding time stamp.
   */
  pilz_msgs::OperationModes getTimeStampedOperationMode() const;

private:
  /**
   * @brief Check if the message contains a operation mode definition.
   *
   * @return true if a operation mode is defined, false otherwise.
   */
  bool hasOperationMode() const;
};

inline ModbusMsgOperationModeWrapper::ModbusMsgOperationModeWrapper(const ModbusMsgInStampedConstPtr& modbus_msg_raw,
                                                                    const ModbusApiSpec& api_spec)
  : ModbusMsgWrapper(modbus_msg_raw, api_spec)
{
}

inline bool ModbusMsgOperationModeWrapper::hasOperationMode() const
{
  return hasRegister(getApiSpec().getRegisterDefinition(modbus_api_spec::OPERATION_MODE));
}

inline int8_t ModbusMsgOperationModeWrapper::getOperationMode() const
{
  switch (getRegister(getApiSpec().getRegisterDefinition(modbus_api_spec::OPERATION_MODE)))
  {
    case MODBUS_OPERATION_MODE_NONE:
      return pilz_msgs::OperationModes::UNKNOWN;
    case MODBUS_OPERATION_MODE_T1:
      return pilz_msgs::OperationModes::T1;
    case MODBUS_OPERATION_MODE_T2:
      return pilz_msgs::OperationModes::T2;
    case MODBUS_OPERATION_MODE_AUTO:
      return pilz_msgs::OperationModes::AUTO;
    default:
      return pilz_msgs::OperationModes::UNKNOWN;
  }
}

inline pilz_msgs::OperationModes ModbusMsgOperationModeWrapper::getTimeStampedOperationMode() const
{
  pilz_msgs::OperationModes op_mode;
  op_mode.time_stamp = getTimeStamp();
  op_mode.value = getOperationMode();
  return op_mode;
}

inline void ModbusMsgOperationModeWrapper::checkStructuralIntegrity() const
{
  ModbusMsgWrapper::checkStructuralIntegrity();

  if (!hasOperationMode())
  {
    throw ModbusMsgOperationModeWrapperException("Received message does not contain information about the operation "
                                                 "mode.");
  }
}

}  // namespace prbt_hardware_support

#endif  // MODBUS_MSG_OPERATION_MODE_WRAPPER_H
