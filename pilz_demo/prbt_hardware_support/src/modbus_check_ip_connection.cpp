/*
 * Copyright (c) 2020 Pilz GmbH & Co. KG
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

#include <cstring>
#include <chrono>
#include <thread>
#include <fcntl.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <prbt_hardware_support/modbus_check_ip_connection.h>

namespace prbt_hardware_support
{
timeval initTimeout(const unsigned int& secs, const unsigned int& usecs)
{
  timeval ret;
  ret.tv_sec = secs;
  ret.tv_usec = usecs;
  return ret;
}

sockaddr_in initSockAddrIn(const char* ip, const unsigned int& port)
{
  sockaddr_in ret;
  std::memset((char*)&ret, 0, sizeof(ret));
  ret.sin_family = AF_INET;
  ret.sin_port = htons((short unsigned int)port);
  ret.sin_addr.s_addr = inet_addr(ip);
  return ret;
}

void setConnectionToNonBlocking(const int& sockfd)
{
  int file_descr_flags{ fcntl(sockfd, F_GETFL, NULL) };
  file_descr_flags |= O_NONBLOCK;
  fcntl(sockfd, F_SETFL, file_descr_flags);
}

bool isSocketReadyForWriteOp(const int& sockfd)
{
  fd_set writeset;
  FD_ZERO(&writeset);
  FD_SET(sockfd, &writeset);
  timeval timeout_in_s{ initTimeout(1, 0) };
  const int socket_ready_for_writing{ select(sockfd + 1, nullptr, &writeset, nullptr, &timeout_in_s) };
  return socket_ready_for_writing > 0;
}

bool hasSocketPendingErrors(const int& sockfd)
{
  int optval;
  socklen_t optlen{ sizeof(optval) };
  const int read_pending_errors_failed{ getsockopt(sockfd, SOL_SOCKET, SO_ERROR, (void*)&optval, &optlen) };
  return !(read_pending_errors_failed == 0) || !(optval == 0);
}

bool checkIPConnection(const char* ip, const unsigned int& port)
{
  const int sockfd{ socket(AF_INET, SOCK_STREAM, 0) };
  const sockaddr_in serv_addr{ initSockAddrIn(ip, port) };

  setConnectionToNonBlocking(sockfd);
  connect(sockfd, (const sockaddr*)&serv_addr, sizeof(serv_addr));

  const bool connection_ok{ isSocketReadyForWriteOp(sockfd) && !hasSocketPendingErrors(sockfd) };

  close(sockfd);
  // wait one second to grant a free port
  std::this_thread::sleep_for(std::chrono::duration<double>(1));

  return connection_ok;
}

}  // namespace prbt_hardware_support
