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

#ifndef PRBT_HARDWARE_SUPPORT_PILZ_MODBUS_SERVER_MOCK_H
#define PRBT_HARDWARE_SUPPORT_PILZ_MODBUS_SERVER_MOCK_H

#include <mutex>
#include <atomic>
#include <cstdint>
#include <vector>
#include <thread>
#include <condition_variable>

#include <modbus/modbus.h>

#include <ros/console.h>

#include "prbt_hardware_support/register_container.h"

namespace prbt_hardware_support
{
/**
 * @class PilzModbusServerMock
 * @brief Offers a modbus server and read/write functionality via subscription/publication.
 */
class PilzModbusServerMock
{
public:
  PilzModbusServerMock(const unsigned int& holding_register_size);

  ~PilzModbusServerMock();

  /**
   * @brief Start the modbus server and make it accessible for clients
   *
   * @param ip The ip the server attaches to
   * @param port The used port
   */
  void start(const char* ip, const unsigned int port);

  /**
   * @brief Start the modbus server asynchronously and make it accessible for clients
   *
   * @param ip The ip the server attaches to
   * @param port The used port
   *
   * Returns as soon as it is running
   */
  void startAsync(const char* ip, const unsigned int port);

  void setHoldingRegister(std::initializer_list<std::pair<unsigned int, uint16_t> > reg_list);

  /**
   * @brief Set the values in the holding register
   *
   * @param data The values in the holding register
   * @param start_index The index from where the value is set, other values remain untouched.
   */
  void setHoldingRegister(const RegCont& data, unsigned int start_index);

  /**
   * @brief Reads the specified number of registers, beginning at the specified
   * start point from the holding register.
   *
   * @param start_index Index of first register to read.
   * @param num_reg_to_read Number of registers to read.
   *
   * @returns the registers which were read.
   */
  RegCont readHoldingRegister(const RegCont::size_type start_index, const RegCont::size_type num_reg_to_read);

  /**
   * @brief Terminate the Server. Reading or connecting to it will fail.
   */
  void terminate();

  void setTerminateFlag();

  /**
   * @brief Allocates needed resources for running the server.
   *
   * This needs to be called before a successful run()
   *
   * @param ip The ip the server attaches to
   * @param port The used port
   * @return True on success, false otherwise
   */
  bool init(const char* ip, unsigned int port);

  /**
   * @brief Run the server and publish the register values as messages.
   * The value of the modbus register is read in a loop.
   * Once in a loop pass the value is published as a message.
   * In order for clients to differentiate between messages notifying about
   * changes in the register the timestamp of the message is only changed
   * if a register value changed.
   */
  void run();

private:
  /**
   * @returns True if the server is told to shutdown via Modbus msg,
   * false otherwise.
   */
  bool shutdownSignalReceived();

private:
  const unsigned int holding_register_size_;
  //! Index of register for server shutdown signal
  const unsigned int terminate_register_idx_;

private:
  //! Modbus
  int socket_{ -1 };
  modbus_t* modbus_connection_{ nullptr };
  modbus_mapping_t* mb_mapping_{ nullptr };

  std::atomic_bool terminate_{ false };

  std::mutex modbus_register_access_mutex;

  std::mutex running_mutex_;
  std::condition_variable running_cv_;

  std::thread thread_;

private:
  static constexpr uint32_t DISCONNECT_TIMEOUT_IN_SEC{ 1 };
  static constexpr uint32_t DISCONNECT_TIMEOUT_IN_USEC{ 0 };

  static constexpr uint32_t RESPONSE_TIMEOUT_IN_SEC{ 0 };
  static constexpr uint32_t RESPONSE_TIMEOUT_IN_USEC{ 10000 };

  //! Register value which indicates that server has to shutdown.
  static constexpr uint16_t TERMINATE_SIGNAL{ 1 };
};

inline void PilzModbusServerMock::setTerminateFlag()
{
  ROS_INFO_NAMED("ServerMock", "Set terminate to true.");
  terminate_ = true;
}

inline void PilzModbusServerMock::terminate()
{
  ROS_INFO_NAMED("ServerMock", "Terminate called on ServerMock.");
  setTerminateFlag();

  if (thread_.joinable())
  {
    ROS_DEBUG_NAMED("ServerMock", "Waiting for worker Thread of ServerMock to be joined.");
    thread_.join();
    ROS_DEBUG_NAMED("ServerMock", "Waiting for worker Thread of ServerMock joined.");
  }
}

inline bool PilzModbusServerMock::shutdownSignalReceived()
{
  return readHoldingRegister(terminate_register_idx_, 1).back() == TERMINATE_SIGNAL;
}

}  // namespace prbt_hardware_support
#endif  // PRBT_HARDWARE_SUPPORT_PILZ_MODBUS_SERVER_MOCK_H
