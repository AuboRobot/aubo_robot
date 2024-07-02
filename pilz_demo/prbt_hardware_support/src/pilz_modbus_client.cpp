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

#include <prbt_hardware_support/pilz_modbus_client.h>

#include <prbt_hardware_support/ModbusMsgInStamped.h>
#include <prbt_hardware_support/modbus_msg_in_builder.h>
#include <prbt_hardware_support/pilz_modbus_exceptions.h>
#include <prbt_hardware_support/pilz_modbus_client_exception.h>

namespace prbt_hardware_support
{
PilzModbusClient::PilzModbusClient(ros::NodeHandle& nh, const std::vector<unsigned short>& registers_to_read,
                                   ModbusClientUniquePtr modbus_client, unsigned int response_timeout_ms,
                                   const std::string& modbus_read_topic_name,
                                   const std::string& modbus_write_service_name, double read_frequency_hz)
  : registers_to_read_(registers_to_read)
  , RESPONSE_TIMEOUT_MS(response_timeout_ms)
  , READ_FREQUENCY_HZ(read_frequency_hz)
  , modbus_client_(std::move(modbus_client))
  , modbus_read_pub_(nh.advertise<ModbusMsgInStamped>(modbus_read_topic_name, DEFAULT_QUEUE_SIZE_MODBUS))
  , modbus_write_service_(
        nh.advertiseService(modbus_write_service_name, &PilzModbusClient::modbus_write_service_cb, this))
{
}

bool PilzModbusClient::init(const char* ip, unsigned int port, int retries, const ros::Duration& timeout)
{
  size_t retry_n = 0;
  while (ros::ok() && (retries == -1 || static_cast<int>(retry_n) < retries))
  {
    ++retry_n;

    if (init(ip, port))
    {
      // The following warning needs only to be shown to complete the warnings shown if this didn't work on the first
      // try
      ROS_WARN_STREAM_COND(retry_n > 1, "Connection to " << ip << ":" << port << " successful.");
      return true;
    }

    ROS_WARN_STREAM_COND(retries == -1, "Connection to "
                                            << ip << ":" << port
                                            << " failed. Make sure that your cables are connected properly and that "
                                               "you have set the correct ip address and port.");
    ROS_WARN_STREAM_COND(retries != -1, "Connection to "
                                            << ip << ":" << port << " failed. Try(" << retry_n << "/" << retries
                                            << "). Make sure that your cables are connected properly and that "
                                               "you have set the correct ip address and port.");
    timeout.sleep();
  }

  return false;
}

bool PilzModbusClient::init(const char* ip, unsigned int port)
{
  State expected_state{ State::not_initialized };
  if (!state_.compare_exchange_strong(expected_state, State::initializing))
  {
    ROS_ERROR_STREAM("Modbus-client not in correct state: " << state_ << "expected:" << State::initializing);
    state_ = State::not_initialized;
    return false;
  }

  if (!modbus_client_->init(ip, port))
  {
    ROS_DEBUG("Init failed");
    state_ = State::not_initialized;
    return false;
  }

  modbus_client_->setResponseTimeoutInMs(RESPONSE_TIMEOUT_MS);

  state_ = State::initialized;
  ROS_DEBUG_STREAM("Connection to " << ip << ":" << port << " established");
  return true;
}

void PilzModbusClient::sendDisconnectMsg()
{
  ModbusMsgInStamped msg;
  msg.disconnect.data = true;
  msg.header.stamp = ros::Time::now();
  modbus_read_pub_.publish(msg);
  ros::spinOnce();
}

void PilzModbusClient::run()
{
  State expected_state{ State::initialized };
  if (!state_.compare_exchange_strong(expected_state, State::running))
  {
    ROS_ERROR_STREAM("Modbus-client not in correct state: " << state_ << "expected:" << State::running);
    throw PilzModbusClientException("Modbus-client not in correct state.");
  }

  RegCont holding_register;
  RegCont last_holding_register;
  ros::Time last_update{ ros::Time::now() };
  state_ = State::running;
  ros::Rate rate(READ_FREQUENCY_HZ);
  while (ros::ok() && !stop_run_.load())
  {
    // Work with local copy of buffer to ensure that the service callback
    // function does not become blocked
    boost::optional<ModbusRegisterBlock> write_reg_bock{ boost::none };
    {
      std::lock_guard<std::mutex> lock(write_reg_blocks_mutex_);
      if (!write_reg_blocks_.empty())
      {
        write_reg_bock = write_reg_blocks_.front();
        // Mark data as send/processed, by "deleting" them from memory
        write_reg_blocks_.pop();
      }
    }

    std::vector<std::vector<unsigned short>> blocks = splitIntoBlocks(registers_to_read_);

    unsigned short index_of_first_register = *std::min_element(registers_to_read_.begin(), registers_to_read_.end());
    int num_registers =
        *std::max_element(registers_to_read_.begin(), registers_to_read_.end()) - index_of_first_register + 1;
    holding_register = RegCont(static_cast<unsigned long>(num_registers), 0);

    ROS_DEBUG("blocks.size() %zu", blocks.size());
    try
    {
      for (auto& block : blocks)
      {
        ROS_DEBUG("block.size() %zu", block.size());
        unsigned short index_of_first_register_block = *(block.begin());
        unsigned long num_registers_block = block.size();
        RegCont block_holding_register;
        if (write_reg_bock)
        {
          block_holding_register = modbus_client_->writeReadHoldingRegister(
              static_cast<int>(write_reg_bock->start_idx), write_reg_bock->values,
              static_cast<int>(index_of_first_register_block), static_cast<int>(num_registers_block));
          // write only once:
          write_reg_bock = boost::none;
        }
        else
        {
          block_holding_register = modbus_client_->readHoldingRegister(static_cast<int>(index_of_first_register_block),
                                                                       static_cast<int>(num_registers_block));
        }
        for (uint i = 0; i < num_registers_block; i++)
          holding_register[i + index_of_first_register_block - index_of_first_register] = block_holding_register[i];
      }
    }
    catch (ModbusExceptionDisconnect& e)
    {
      ROS_ERROR_STREAM("Modbus disconnect: " << e.what());
      sendDisconnectMsg();
      break;
    }

    // LCOV_EXCL_START
    // The following line has no hit in the coverage report. Has to be a bug of gcov.
    ModbusMsgInStampedPtr msg{ ModbusMsgInBuilder::createDefaultModbusMsgIn(index_of_first_register,
                                                                            // LCOV_EXCL_END
                                                                            holding_register) };

    // Publish the received data into ROS
    if (holding_register != last_holding_register)
    {
      ROS_DEBUG_STREAM("Sending new ROS-message.");
      msg->header.stamp = ros::Time::now();
      last_update = msg->header.stamp;
      last_holding_register = holding_register;
    }
    else
    {
      msg->header.stamp = last_update;
    }
    modbus_read_pub_.publish(msg);

    ros::spinOnce();
    rate.sleep();
  }

  stop_run_ = false;
  state_ = State::not_initialized;
}

std::vector<std::vector<unsigned short>> PilzModbusClient::splitIntoBlocks(std::vector<unsigned short>& in)
{
  std::vector<std::vector<unsigned short>> out;
  std::sort(in.begin(), in.end());  // sort just in case to be more user-friendly
  unsigned short prev{ 0 };
  std::vector<unsigned short> current_block;
  for (auto& reg : in)
  {
    if (reg == prev)
    {
      throw PilzModbusClientException("List elemts must be unique.");
    }
    else if (reg == prev + 1)
    {
      current_block.push_back(reg);
    }
    else
    {  // *it >= prev + 1
      std::vector<unsigned short> to_out(current_block);
      if (!to_out.empty())
      {
        out.push_back(to_out);
      }
      current_block.clear();
      current_block.push_back(reg);
    }
    prev = reg;
  }
  std::vector<unsigned short> to_out(current_block);
  if (!to_out.empty())
  {
    out.push_back(to_out);
  }
  return out;
}

}  // namespace prbt_hardware_support
