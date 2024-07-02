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
#include <prbt_hardware_support/modbus_adapter_brake_test.h>

#include <sstream>
#include <algorithm>

#include <prbt_hardware_support/modbus_api_definitions.h>
#include <prbt_hardware_support/modbus_msg_brake_test_wrapper.h>
#include <prbt_hardware_support/modbus_adapter_brake_test_exception.h>

namespace prbt_hardware_support
{
using namespace modbus_api::v3;

ModbusAdapterBrakeTest::ModbusAdapterBrakeTest(TWriteModbusRegister&& write_modbus_register_func,
                                               const ModbusApiSpec& read_api_spec, const ModbusApiSpec& write_api_spec)
  : api_spec_(read_api_spec)
  , reg_idx_cont_(getRegisters(write_api_spec))
  , reg_start_idx_(getMinRegisterIdx(reg_idx_cont_))
  , reg_block_size_(getRegisterBlockSize(reg_idx_cont_))
  , write_modbus_register_func_(write_modbus_register_func)
{
}

ModbusAdapterBrakeTest::TRegIdxCont ModbusAdapterBrakeTest::getRegisters(const ModbusApiSpec& write_api_spec)
{
  TRegIdxCont reg_idx_cont;

  if (!write_api_spec.hasRegisterDefinition(modbus_api_spec::BRAKETEST_PERFORMED))
  {
    throw ModbusAdapterBrakeTestException("Failed to read API spec for BRAKETEST_PERFORMED");
  }
  reg_idx_cont[modbus_api_spec::BRAKETEST_PERFORMED] =
      write_api_spec.getRegisterDefinition(modbus_api_spec::BRAKETEST_PERFORMED);

  if (!write_api_spec.hasRegisterDefinition(modbus_api_spec::BRAKETEST_RESULT))
  {
    throw ModbusAdapterBrakeTestException("Failed to read API spec for BRAKETEST_RESULT");
  }
  reg_idx_cont[modbus_api_spec::BRAKETEST_RESULT] =
      write_api_spec.getRegisterDefinition(modbus_api_spec::BRAKETEST_RESULT);

  if (abs(reg_idx_cont.at(modbus_api_spec::BRAKETEST_PERFORMED) - reg_idx_cont.at(modbus_api_spec::BRAKETEST_RESULT)) !=
      1)
  {
    std::ostringstream os;
    os << "Registers of BRAKETEST_PERFORMED and BRAKETEST_RESULT need to be 1 apart";
    os << " (distance: " << abs(reg_idx_cont.cbegin()->second - reg_idx_cont.cend()->second) << ")";
    // Both registers need to be one apart, so that we can write them in one cycle
    throw ModbusAdapterBrakeTestException(os.str());
  }

  return reg_idx_cont;
}

void ModbusAdapterBrakeTest::modbusMsgCallback(const ModbusMsgInStampedConstPtr& msg_raw)
{
  ModbusMsgBrakeTestWrapper msg{ msg_raw, api_spec_ };

  if (msg.isDisconnect())
  {
    return;
  }

  try
  {
    msg.checkStructuralIntegrity();
  }
  catch (const ModbusMsgWrapperException& ex)
  {
    ROS_ERROR_STREAM(ex.what());
    return;
  }

  if (msg.getVersion() != MODBUS_API_VERSION_REQUIRED)
  {
    std::ostringstream os;
    os << "Received Modbus message of unsupported API Version: " << msg.getVersion()
       << ", required Version: " << MODBUS_API_VERSION_REQUIRED;
    os << "\n";
    os << "Can not determine from Modbus message if brake-test is required.";
    ROS_ERROR_STREAM(os.str());
    return;
  }

  updateBrakeTestRequiredState(msg.getBrakeTestRequirementStatus());
}

void ModbusAdapterBrakeTest::updateBrakeTestRequiredState(TBrakeTestRequired brake_test_required)
{
  TBrakeTestRequired last_brake_test_flag{ brake_test_required_ };
  brake_test_required_ = brake_test_required;
  if (brake_test_required_ == pilz_msgs::IsBrakeTestRequiredResult::REQUIRED &&
      last_brake_test_flag != pilz_msgs::IsBrakeTestRequiredResult::REQUIRED)
  {
    ROS_INFO("Brake Test required.");
  }
}

bool ModbusAdapterBrakeTest::sendBrakeTestResult(SendBrakeTestResult::Request& req, SendBrakeTestResult::Response& res)
{
  if (!write_modbus_register_func_)
  {
    res.error_msg = "No callback available to send brake test result to FS control";
    res.success = false;
    return true;
  }

  RegCont reg_cont(reg_block_size_, 0);
  // Note: The FS controller needs a positive edge, so we first reset the registers
  reg_cont.at(reg_idx_cont_.at(modbus_api_spec::BRAKETEST_PERFORMED) - reg_start_idx_) =
      MODBUS_BRAKE_TEST_NOT_PERFORMED;
  reg_cont.at(reg_idx_cont_.at(modbus_api_spec::BRAKETEST_RESULT) - reg_start_idx_) = MODBUS_BRAKE_TEST_NOT_PASSED;

  if (!write_modbus_register_func_(reg_start_idx_, reg_cont))
  {
    res.error_msg = "Resetting of modus registers failed";
    res.success = false;
    return true;
  }

  reg_cont.at(reg_idx_cont_.at(modbus_api_spec::BRAKETEST_PERFORMED) - reg_start_idx_) = MODBUS_BRAKE_TEST_PERFORMED;
  if (req.result)
  {
    reg_cont.at(reg_idx_cont_.at(modbus_api_spec::BRAKETEST_RESULT) - reg_start_idx_) = MODBUS_BRAKE_TEST_PASSED;
  }
  if (!write_modbus_register_func_(reg_start_idx_, reg_cont))
  {
    res.error_msg = "Sending of brake test result to FS control failed";
    res.success = false;
    return true;
  }

  res.success = true;
  return true;
}

}  // namespace prbt_hardware_support
