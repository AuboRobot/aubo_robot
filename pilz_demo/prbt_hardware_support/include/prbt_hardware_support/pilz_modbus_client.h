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

#ifndef PRBT_HARDWARE_SUPPORT_CLIENT_NODE_H
#define PRBT_HARDWARE_SUPPORT_CLIENT_NODE_H

#include <atomic>
#include <mutex>
#include <queue>

#include <boost/optional.hpp>

#include <ros/ros.h>
#include <std_msgs/UInt16MultiArray.h>

#include <prbt_hardware_support/modbus_client.h>
#include <prbt_hardware_support/register_container.h>
#include <prbt_hardware_support/WriteModbusRegister.h>

namespace prbt_hardware_support
{
/**
 * @brief Connects to a modbus server and publishes the received data into ROS.
 */
class PilzModbusClient
{
  typedef std::unique_ptr<ModbusClient> ModbusClientUniquePtr;

public:
  /**
   * @brief Sets up publisher. To open the modbus connection call PilzModbusClient::init.
   * @param nh Node handle.
   * @param registers_to_read Registers which have to be read.
   * @param modbus_client ModbusClient to use
   * @param response_timeout_ms Time to wait for a response from Modbus server.
   * @param modbus_read_topic_name Name of the topic to which is published.
   * @param modbus_write_service_name Name under which the modbus write service is advertised.
   * @param read_frequency_hz Defines how often Modbus registers are read in.
   */
  PilzModbusClient(ros::NodeHandle& nh, const std::vector<unsigned short>& registers_to_read,
                   ModbusClientUniquePtr modbus_client, unsigned int response_timeout_ms,
                   const std::string& modbus_read_topic_name, const std::string& modbus_write_service_name,
                   double read_frequency_hz = DEFAULT_MODBUS_READ_FREQUENCY_HZ);

public:
  /**
   * @brief Tries to connect to a modbus server.
   * @param ip
   * @param port
   * @return True if a connection is established, false otherwise.
   */
  bool init(const char* ip, unsigned int port);

  /**
   * @brief Tries to connect to a modbus server.
   * @param ip
   * @param port
   * @param retries Number of retries getting a connection to the server. Set -1 for infinite retries.
   * @param timeout_ms between retries
   * @return True if a connection is established, false otherwise.
   */
  bool init(const char* ip, unsigned int port, int retries, const ros::Duration& timeout_ms);

  /**
   * @brief Publishes the register values as messages.
   *
   * The value of the modbus register is read in a loop.
   * Once a loop the value is published as a message.
   * In order for clients to differentiate between messages notifying about
   * changes in the register the timestamp of the message is only changed
   * if a register value changed.
   */
  void run();

  /**
   * @brief Ends the infinite loop started in method 'run()'.
   */
  void terminate();

  /**
   * @brief True if 'run()' method is active, false if 'run()' method is not active or,
   * currently, finishing.
   */
  bool isRunning();

  /**
   * @brief Splits a vector of integers into a vector of vectors with consecutive groups
   */
  std::vector<std::vector<unsigned short>> static splitIntoBlocks(std::vector<unsigned short>& in);

private:
  void sendDisconnectMsg();

  /**
   * @brief Stores the register which have to be send to the modbus server
   * in a local buffer for further processing by the modbus thread.
   */
  bool modbus_write_service_cb(WriteModbusRegister::Request& req, WriteModbusRegister::Response& res);

private:
  /**
   * @brief States of the Modbus-client.
   */
  enum State
  {
    not_initialized,
    initializing,
    initialized,
    running,
  };

  //! Registers which have to be read.
  std::vector<unsigned short> registers_to_read_;

  //! Defines how long we wait for a response from the Modbus-server.
  const unsigned int RESPONSE_TIMEOUT_MS;
  //! Defines how often the Modbus registers are read in.
  const double READ_FREQUENCY_HZ;

private:
  static constexpr double DEFAULT_MODBUS_READ_FREQUENCY_HZ{ 500 };
  static constexpr int DEFAULT_QUEUE_SIZE_MODBUS{ 1 };

private:
  std::atomic<State> state_{ State::not_initialized };
  std::atomic_bool stop_run_{ false };
  ModbusClientUniquePtr modbus_client_;
  ros::Publisher modbus_read_pub_;
  ros::Subscriber modbus_write_sub_;
  ros::ServiceServer modbus_write_service_;

  std::mutex write_reg_blocks_mutex_;
  std::queue<ModbusRegisterBlock> write_reg_blocks_;
};

inline void PilzModbusClient::terminate()
{
  stop_run_ = true;
}

inline bool PilzModbusClient::isRunning()
{
  return state_.load() == State::running;
}

inline bool PilzModbusClient::modbus_write_service_cb(WriteModbusRegister::Request& req,
                                                      WriteModbusRegister::Response& res)
{
  std::lock_guard<std::mutex> lock(write_reg_blocks_mutex_);
  write_reg_blocks_.emplace(req.holding_register_block);
  res.success = true;
  return true;
}

}  // namespace prbt_hardware_support

#endif  // PRBT_HARDWARE_SUPPORT_CLIENT_NODE_H
