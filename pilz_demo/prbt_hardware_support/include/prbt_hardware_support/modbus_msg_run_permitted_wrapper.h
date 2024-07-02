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

#ifndef MODBUS_MSG_RUN_PERMITTED_WRAPPER_H
#define MODBUS_MSG_RUN_PERMITTED_WRAPPER_H

#include <prbt_hardware_support/ModbusMsgInStamped.h>
#include <prbt_hardware_support/modbus_api_definitions.h>
#include <prbt_hardware_support/modbus_msg_run_permitted_wrapper_exception.h>
#include <prbt_hardware_support/modbus_msg_wrapper.h>

namespace prbt_hardware_support
{
using namespace modbus_api::v3;

/**
 * @brief Wrapper class to add semantic to a raw ModbusMsgInStamped
 *
 * Allows to easy access to the content behind a raw modbus message which is assumed to contain
 * data about RUN_PERMITTED clearance.
 */
class ModbusMsgRunPermittedWrapper : public ModbusMsgWrapper
{
public:
  ModbusMsgRunPermittedWrapper(const ModbusMsgInStampedConstPtr& modbus_msg_raw, const ModbusApiSpec& api_spec);

  /**
   * @brief Calls ModbusMsgWrapper::checkStructuralIntegrity().
   *
   * @throw ModbusMsgRunPermittedStatusMissing if RUN_PERMITTED register is missing.
   */
  virtual void checkStructuralIntegrity() const override;

  /**
   * @brief Get the RUN_PERMITTED from the Modbus message
   *
   * @return true if the RUN_PERMITTED is active (manipulator should stop)
   * @return false if the RUN_PERMITTED is clear (manipulator can move)
   */
  bool getRunPermitted() const;

private:
  /**
   * @brief Check if the message contains a RUN_PERMITTED definition
   *
   * @return true if a RUN_PERMITTED is defined
   * @return false if there is no RUN_PERMITTED defined in the message
   */
  bool hasRunPermitted() const;
};

inline bool ModbusMsgRunPermittedWrapper::hasRunPermitted() const
{
  return hasRegister(getApiSpec().getRegisterDefinition(modbus_api_spec::RUN_PERMITTED));
}

inline bool ModbusMsgRunPermittedWrapper::getRunPermitted() const
{
  switch (getRegister(getApiSpec().getRegisterDefinition(modbus_api_spec::RUN_PERMITTED)))
  {
    case MODBUS_RUN_PERMITTED_TRUE:
      return true;
    case MODBUS_RUN_PERMITTED_FALSE:
      return false;
    default:
      return false;
  }
}

}  // namespace prbt_hardware_support

#endif  // MODBUS_MSG_RUN_PERMITTED_WRAPPER_H
