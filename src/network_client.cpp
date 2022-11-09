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

#include "hlvs_player/network_client.hpp"

float NetworkClient::history_period_ = 5;
// Adding some margin for other data than image
int NetworkClient::max_answer_size_ = 1920 * 1080 * 3 + 1000;
int NetworkClient::max_attempts_ = 20;
int NetworkClient::wait_time_sec_ = 1;

static void close_socket(int fd)
{
#ifdef _WIN32
  closesocket(fd);
#else
  close(fd);
#endif
}

static char * read_file(const char * filename)
{
  char * buffer = NULL;
  FILE * fp = fopen(filename, "r");
  if (!fp) {
    return NULL;
  }
  if (fseek(fp, 0L, SEEK_END) == 0) {
    const int size = ftell(fp);
    assert(size != -1);
    buffer = reinterpret_cast<char *>(malloc(sizeof(char) * (size + 1)));
    fseek(fp, 0L, SEEK_SET);
    const size_t len = fread(buffer, sizeof(char), size, fp);
    buffer[len] = '\0';
  }
  fclose(fp);
  return buffer;
}

void NetworkClient::printMessages(const SensorMeasurements & sensor_measurements)
{
  for (int i = 0; i < sensor_measurements.messages_size(); i++) {
    const Message & msg = sensor_measurements.messages(i);
    std::string prefix = Message_MessageType_Name(msg.message_type());
    RCLCPP_ERROR(logger_, "%s: %s\n", prefix.c_str(), msg.text().c_str());
  }
}

NetworkClient::NetworkClient(
  const std::string & host, int port, int verbosity,
  rclcpp::Logger ros_logger)
: host_(host),
  port_(port),
  socket_fd_(-1),
  verbosity_(verbosity),
  history_total_size_(0),
  client_start_(0),
  last_history_print_(0),
  logger_(ros_logger) {}

bool NetworkClient::connectClient()
{
  int return_code;
  struct hostent * server = gethostbyname(host_.c_str());
  struct sockaddr_in address;
  memset(&address, 0, sizeof(struct sockaddr_in));
  address.sin_family = AF_INET;
  address.sin_port = htons(port_);
  if (server) {
    memcpy(
      reinterpret_cast<char *>(&address.sin_addr.s_addr),
      reinterpret_cast<char *>(server->h_addr), server->h_length);
  } else {
    fprintf(stderr, "Cannot resolve server name: %s\n", host_.c_str());
    return false;
  }
#ifdef _WIN32
  WSADATA info;
  return_code = WSAStartup(MAKEWORD(2, 2), &info);  // Winsock 2.2
  if (return_code != 0) {
    if (verbosity_ > 0) {
      fprintf(stderr, "Cannot initialize Winsock\n");
    }
    return false;
  }
#endif
  socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_fd_ == -1) {
    if (verbosity_ > 0) {
      fprintf(stderr, "Cannot create socket\n");
    }
    return false;
  }
  int attempt = 1;
  bool connected = false;
  while (attempt <= max_attempts_) {
    if (!rclcpp::ok()) {
      exit(0);
    }
    return_code = connect(socket_fd_, (struct sockaddr *)&address, sizeof(struct sockaddr));
    if (return_code == 0) {
      connected = true;
      break;
    }
    fprintf(
      stderr, "Failed to connect to %s:%d (attempt %d / %d)\n",
      host_.c_str(), port_, attempt, max_attempts_);
    attempt++;
    sleep(wait_time_sec_);
  }
  if (!connected) {
    if (verbosity_ > 0) {
      fprintf(stderr, "Failed to connect after %d attempts. Giving up on connection\n", attempt);
    }
    disconnectClient();
    return false;
  }
  // Receiving the 'welcome message'
  char answer[8];
  receiveData(answer, 8);
  if (verbosity_ >= 4) {
    printf("Welcome message: %s\n", answer);
  }
  if (strncmp(answer, "Welcome", 8) != 0) {
    if (verbosity_ > 0) {
      if (strncmp(answer, "Refused", 8) == 0) {
        fprintf(
          stderr,
          "Connection to %s:%d refused: your IP address is not allowed in the game.json "
          "configuration file.\n",
          host_.c_str(), port_);
      } else {
        fprintf(stderr, "Received unknown answer from server: %s\n", answer);
      }
    }
    disconnectClient();
    return false;
  }
  if (verbosity_ >= 2) {
    printf("Connected to %s:%d\n", host_.c_str(), port_);
  }
  return true;
}

void NetworkClient::disconnectClient()
{
  if (socket_fd_ == -1 && verbosity_ > 0) {
    fprintf(stderr, "NetworkClient is already disconnected\n");
    return;
  }
  close_socket(socket_fd_);
  socket_fd_ = -1;
}

void NetworkClient::sendRequest(const ActuatorRequests & actuator_request)
{
  if (socket_fd_ == -1) {
    throw std::logic_error("NetworkClient is not connected");
  }
  const uint32_t size = htonl(actuator_request.ByteSizeLong());
#ifndef _WIN32
  // This doesn't work on Windows, we should implement SocketOutputStream to make it work
  // efficiently on Windows.
  // See https://stackoverflow.com/questions/23280457/c-google-protocol-buffers-open-http-socket
  if (send(
      socket_fd_,
      const_cast<char *>(reinterpret_cast<const char *>(&size)), sizeof(uint32_t), 0) == -1)
  {
    std::string error_msg = "Failed to send message of size: " + std::to_string(size) + " errno: " +
      std::to_string(errno);
    if (errno == ECONNRESET) {
      error_msg = "Simulator interrupted the connection";
    }
    disconnectClient();
    throw std::runtime_error(error_msg);
  }
  google::protobuf::io::ZeroCopyOutputStream * zeroCopyStream =
    new google::protobuf::io::FileOutputStream(socket_fd_);
  actuator_request.SerializeToZeroCopyStream(zeroCopyStream);
  delete zeroCopyStream;
#else  // here we make a useless malloc, copy and free
  char * output = reinterpret_cast<char *>(malloc(sizeof(int) + size));
  uint32_t * content_size = reinterpret_cast<uint32_t *>(output);
  *content_size = size;
  uint32_t total_size = sizeof(uint32_t) + *content_size;
  actuator_request.SerializeToArray(&output[sizeof(uint32_t)], *content_size);
  if (send(socket_fd_, output, total_size, 0) == -1) {
    std::string error_msg =
      "Failed to send message of size: " + std::to_string(total_size) + " errno: " + std::to_string(
      errno);
    free(output);
    disconnectClient();
    throw std::runtime_error(error_msg);
  }
  free(output);
#endif
}

SensorMeasurements NetworkClient::receive()
{
  uint32_t content_size_network;
  receiveData(reinterpret_cast<char *>(&content_size_network), sizeof(uint32_t));
  const int answer_size = ntohl(content_size_network);
  if (answer_size > max_answer_size_ || answer_size == 0) {
    disconnectClient();
    throw std::logic_error(
            "Unexpected size for the answer: " + std::to_string(
              answer_size) + " (probably out of sync)");
  }
  SensorMeasurements sensor_measurements;
  char * buffer = reinterpret_cast<char *>(malloc(answer_size));
  receiveData(buffer, answer_size);
  sensor_measurements.ParseFromArray(buffer, answer_size);
  free(buffer);
  // History is only updated when printing it
  if (verbosity_ >= 2) {
    printMessages(sensor_measurements);
  }
  if (verbosity_ >= 3) {
    updateHistory(sensor_measurements);
  }
  if (verbosity_ >= 4) {
    std::string printout;
    google::protobuf::TextFormat::PrintToString(sensor_measurements, &printout);
    std::cout << printout << std::endl;
  }
  return sensor_measurements;
}

bool NetworkClient::isOk()
{
  return socket_fd_ != -1;
}

ActuatorRequests NetworkClient::buildRequestMessage(const std::string & path)
{
  const char * message = read_file(path.c_str());
  if (message == nullptr) {
    throw std::runtime_error("File '" + path + "' does not exist");
  }
  ActuatorRequests actuator_request;
  google::protobuf::TextFormat::ParseFromString(message, &actuator_request);
  return actuator_request;
}

void NetworkClient::receiveData(char * buffer, int bytes)
{
  if (socket_fd_ == -1) {
    throw std::logic_error("NetworkClient is not connected");
  }
  int received = 0;
  while (received < bytes) {
    int n = recv(socket_fd_, buffer + received, bytes - received, 0);
    if (n == -1) {
      std::string error_msg = "Failed to send message of size: " + std::to_string(bytes) +
        " errno: " + std::to_string(errno);
      if (errno == ECONNRESET) {
        error_msg = "Simulator interrupted the connection";
      }
      disconnectClient();
      throw std::runtime_error(error_msg);
    }
    received += n;
  }
}

void NetworkClient::updateHistory(const SensorMeasurements & sensors)
{
  uint32_t msg_size = sensors.ByteSizeLong();
  MessageProperty new_msg;
  new_msg.simulated_time = sensors.time();
  new_msg.real_time = sensors.real_time();
  new_msg.msg_size = msg_size;
  history_total_size_ += msg_size;
  if (client_start_ == 0) {
    client_start_ = new_msg.real_time;
    last_history_print_ = new_msg.real_time;
  }
  // Adding new message + removing old ones
  msg_history_.push_front(new_msg);
  while (msg_history_.front().real_time - msg_history_.back().real_time > history_period_ * 1000) {
    history_total_size_ -= msg_history_.back().msg_size;
    msg_history_.pop_back();
  }
  // Printing at fixed frequency
  if (new_msg.real_time - last_history_print_ > history_period_ * 1000) {
    if (msg_history_.size() == 1) {
      printf("Cannot compute stats on 1 message\n");
    } else {
      const MessageProperty & old_msg = msg_history_.back();
      double real_time_elapsed = new_msg.real_time - old_msg.real_time;
      double simulated_time_elapsed = new_msg.simulated_time - old_msg.simulated_time;
      double real_time_factor = simulated_time_elapsed / real_time_elapsed;
      double bandwidth = history_total_size_ / (real_time_elapsed / 1000) / std::pow(2, 20);
      printf(
        "[%08.3f|%08.3f] real_time_factor: %f, bandwidth: %f MB/s\n",
        (new_msg.real_time - client_start_) / 1000.0,
        new_msg.simulated_time / 1000.0, real_time_factor, bandwidth);
    }
    last_history_print_ = new_msg.real_time;
  }
}
