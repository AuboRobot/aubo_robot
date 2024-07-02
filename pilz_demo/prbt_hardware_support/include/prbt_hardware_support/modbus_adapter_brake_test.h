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

#ifndef MODBUS_ADAPTER_BRAKE_TEST_H
#define MODBUS_ADAPTER_BRAKE_TEST_H

#include <memory>
#include <map>
#include <string>
#include <functional>

#include <ros/ros.h>

#include <pilz_msgs/IsBrakeTestRequired.h>
#include <pilz_msgs/IsBrakeTestRequiredResult.h>

#include <prbt_hardware_support/ModbusMsgInStamped.h>
#include <prbt_hardware_support/modbus_api_spec.h>
#include <prbt_hardware_support/SendBrakeTestResult.h>
#include <prbt_hardware_support/register_container.h>

namespace prbt_hardware_support
{
using TWriteModbusRegister = std::function<bool(const uint16_t&, const RegCont&)>;

/**
 * @brief Listens to the modbus_read topic and publishes a message
 * informing about a required brake test.
 */
class ModbusAdapterBrakeTest
{
public:
  ModbusAdapterBrakeTest(TWriteModbusRegister&& write_modbus_register_func, const ModbusApiSpec& read_api_spec,
                         const ModbusApiSpec& write_api_spec);

public:
  /**
   * @brief Called whenever a new modbus message arrives.
   *
   * @note Filters like for example the UpdateFilter can restrict
   * the number of incoming messages.
   */
  void modbusMsgCallback(const ModbusMsgInStampedConstPtr& msg_raw);

  /**
   * @brief Stores the brake test required flag and
   * initializes the brake test service,
   * the first time the function is called.
   */
  bool isBrakeTestRequired(pilz_msgs::IsBrakeTestRequired::Request&, pilz_msgs::IsBrakeTestRequired::Response& res);

  /**
   * @brief Sends the brake test result data to the modbus client.
   */
  bool sendBrakeTestResult(SendBrakeTestResult::Request& req, SendBrakeTestResult::Response& res);

private:
  using TBrakeTestRequired = pilz_msgs::IsBrakeTestRequiredResult::_value_type;
  void updateBrakeTestRequiredState(TBrakeTestRequired brake_test_required);

private:
  using TRegIdx = uint16_t;
  using TRegIdxCont = std::map<std::string, TRegIdx>;
  /**
   * @returns the indicies of the modbus registers, needed to write
   * the brake test results back to the modbus.
   */
  static TRegIdxCont getRegisters(const ModbusApiSpec& write_api_spec);

  //! @returns the lowest register index stored in the specified container.
  static const TRegIdxCont::mapped_type& getMinRegisterIdx(const TRegIdxCont& reg_idx_cont);
  //! @returns the highest register index stored in the specified container.
  static const TRegIdxCont::mapped_type& getMaxRegisterIdx(const TRegIdxCont& reg_idx_cont);

  //! @returns the number of registers specified by the given register index
  //! container.
  static unsigned long getRegisterBlockSize(const TRegIdxCont& reg_idx_cont);

private:
  const ModbusApiSpec api_spec_;

  //! Store the current state of whether a brake test is required
  TBrakeTestRequired brake_test_required_{ pilz_msgs::IsBrakeTestRequiredResult::UNKNOWN };

  //! Contains the indicies of the modbus registers, needed to write
  //! the brake test results back to the modbus.
  const TRegIdxCont reg_idx_cont_;

  const TRegIdx reg_start_idx_{ getMinRegisterIdx(reg_idx_cont_) };
  const RegCont::size_type reg_block_size_{ getRegisterBlockSize(reg_idx_cont_) };

  TWriteModbusRegister write_modbus_register_func_;
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

inline bool ModbusAdapterBrakeTest::isBrakeTestRequired(pilz_msgs::IsBrakeTestRequired::Request& /*req*/,
                                                        pilz_msgs::IsBrakeTestRequired::Response& res)
{
  res.result.value = brake_test_required_;
  return true;
}

// The following is excluded because lambda functions are not marked properly with gcc-7
// see https://github.com/gcc-mirror/gcc/commit/7de708f
// LCOV_EXCL_START
inline const ModbusAdapterBrakeTest::TRegIdxCont::mapped_type&
ModbusAdapterBrakeTest::getMinRegisterIdx(const TRegIdxCont& reg_idx_cont)
{
  return std::min_element(reg_idx_cont.cbegin(), reg_idx_cont.cend(),
                          [](const TRegIdxCont::value_type& l, const TRegIdxCont::value_type& r) -> bool {
                            return l.second < r.second;
                          })
      ->second;
}

inline const ModbusAdapterBrakeTest::TRegIdxCont::mapped_type&
ModbusAdapterBrakeTest::getMaxRegisterIdx(const TRegIdxCont& reg_idx_cont)
{
  return std::max_element(reg_idx_cont.cbegin(), reg_idx_cont.cend(),
                          [](const TRegIdxCont::value_type& l, const TRegIdxCont::value_type& r) -> bool {
                            return l.second < r.second;
                          })
      ->second;
}
// LCOV_EXCL_STOP

inline unsigned long
ModbusAdapterBrakeTest::getRegisterBlockSize(const ModbusAdapterBrakeTest::TRegIdxCont& reg_idx_cont)
{
  return static_cast<unsigned long>(std::abs(getMaxRegisterIdx(reg_idx_cont) - getMinRegisterIdx(reg_idx_cont)) + 1);
}

}  // namespace prbt_hardware_support
#endif  // MODBUS_ADAPTER_BRAKE_TEST_H
