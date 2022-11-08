// Copyright 2022 RoboCup Federation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef HLVS_PLAYER__NETWORK_CLIENT_HPP_
#define HLVS_PLAYER__NETWORK_CLIENT_HPP_

#ifdef _WIN32
#include <winsock.h>
#define sleep(x) Sleep(x)
#else
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#endif

#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>

#include <deque>
#include <memory>
#include <cmath>
#include <stdexcept>
#include <string>

#include "messages.pb.h"

#include <opencv2/highgui.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>

class NetworkClient
{
public:
  NetworkClient(const std::string & host, int port, int verbosity, rclcpp::Logger ros_logger);

  /**
   * Close socket if opened and free all resources associated to the current connection
   * Returns true on success.
   */
  bool connectClient();

  /**
   * Close socket if opened and free all resources associated to the current connection
   */
  void disconnectClient();

  /**
   * Send the provided message to the simulator.
   * On failure, the client is closed and a runtime_error is thrown afterwards.
   */
  void sendRequest(const ActuatorRequests & actuator_request);

  /**
   * Returns next sensor message received, or an empty pointer on failure. This call is blocking.
   * On failure, the client is closed and a runtime_error is thrown afterwards.
   */
  SensorMeasurements receive();

  /**
   * Returns true if the client is connected and no error has been detected
   */
  bool isOk();

  static ActuatorRequests buildRequestMessage(const std::string & path);

private:
  /**
   * Host address
   */
  std::string host_;

  /**
   * The destination port to establish connection
   */
  int port_;

  /**
   * The file descriptor for the socket: -1 if connection is not established
   */
  int socket_fd_;

  /**
   * 0: Silent mode, no error messages, even when the connection breaks
   * 1: Only print error messages
   * 2: Print messages on successful connection and warnings + errors from the simulator
   * 3: Print statistics over messages received
   * 4: Print all messages received
   */
  int verbosity_;

  struct MessageProperty
  {
    uint32_t simulated_time;  // [ms]
    uint64_t real_time;       // [ms]
    uint32_t msg_size;        // number of bytes in the message
  };
  std::deque<MessageProperty> msg_history_;

  uint64_t history_total_size_;
  uint64_t client_start_;
  uint64_t last_history_print_;

  /**
   * The period (in seconds) used to estimated bandwidth and real_time factor.
   */
  static float history_period_;

  /**
   * The maximal size of messages that can be received, if the announced size of a message is larger than this, the
   * connection will be considered as out of sync and the connection will automatically be closed.
   */
  static int max_answer_size_;

  /**
   * The number of connection attempts before giving up
   */
  static int max_attempts_;

  /**
   * The number of seconds to wait between two connection attempts
   */
  static int wait_time_sec_;


  rclcpp::Logger logger_;

  /**
   * Throws logic_error if the connection has not been started.
   * In case an error occurs during reception, the connection is ended and a runtime_error is thrown
   */
  void receiveData(char * buffer, int bytes);

  /**
   * Update the message history with a message
   */
  void updateHistory(const SensorMeasurements & sensors);

  void printMessages(const SensorMeasurements & sensor_measurements);
};

#endif  // HLVS_PLAYER__NETWORK_CLIENT_HPP_
