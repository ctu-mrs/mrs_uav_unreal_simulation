// Copyright [2022] <Jakub Jirkal>
// This code is licensed under MIT license (see LICENSE for details)

#pragma once

#include <memory>
#include <queue>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <typeinfo>

#include <cereal/archives/binary.hpp>
#include <kissnet/kissnet.hpp>
#include <ueds_connector/serialization/serializable-extended.h>

#define LOCALHOST "127.0.0.1"
#define DEFAULT_PORT 8080
#define BUFFER_SIZE 1024

#define END_OF_MESSAGE '$'

namespace ueds_connector {

class SocketClient {
 public:
  SocketClient();
  SocketClient(const std::string& address, uint16_t port);
  ~SocketClient();

  kissnet::socket_status::values Connect();
  bool ConnectSimple();
  bool Disconnect();

  std::tuple<uint32_t, kissnet::socket_status> ReceiveMessage();

  bool Ping();

  template <typename TRequest>
  std::tuple<uint32_t, kissnet::socket_status> SendMessage(TRequest& message) {
    std::stringstream outputStream;

    try {
      cereal::BinaryOutputArchive oa(outputStream);
      oa(message);
    } catch (cereal::Exception& exception) {
      std::cerr << "Serialization error: " << exception.what() << std::endl;
      return std::make_tuple(0, kissnet::socket_status::errored);
    }

    std::string str = outputStream.str();
    return SendMessage_(reinterpret_cast<const std::byte*>(str.c_str()), str.size());
  }

  template <typename TRequest, typename TResponse>
  bool Request(TRequest& message, TResponse& response) {
    const auto [send_size, send_status] = SendMessage<TRequest>(message);

    if (send_status != kissnet::socket_status::valid || send_size == 0) {
      return false;
    }

    std::string response_str;
    if (!GetMessage(response_str)) {
      return false;
    }

//    std::cout << "m.length " << response_str.length() << std::endl;
//    std::cout << typeid(response).name() << std::endl;
    //std::cout << typeid(Serializable::Drone::GetLidarData::Response).name() << std::endl;


    try {
      std::stringstream ss2(response_str);
      cereal::BinaryInputArchive ia(ss2);
      ia(response);

      return true;
    } catch (cereal::Exception& exception) {
      // TODO
      std::cout << "SOCKET-CLIENT serialization crashed!!!" << exception.what() << std::endl;
    }
    return false;
  }

  uint16_t getPort() const {
    return port_;
  }

  std::string getAddress() const {
    return address_;
  }

 private:
  uint16_t port_ = DEFAULT_PORT;
  std::string address_ = LOCALHOST;
  std::unique_ptr<kissnet::tcp_socket> socket_ = nullptr;

  std::queue<std::unique_ptr<std::vector<std::byte>>> in_queue_;

 protected:
  void PushToInQueue_(std::unique_ptr<std::vector<std::byte>> item);
  std::unique_ptr<std::vector<std::byte>> PopFromInQueue_();
  void ClearInQueue_();
  [[nodiscard]] bool InQueueEmpty_() const;
  [[nodiscard]] bool IsSocketValid_() const;
  [[nodiscard]] bool IsReadyToSend_() const;
  [[nodiscard]] std::tuple<uint32_t, kissnet::socket_status> SendMessage_(const std::byte* buffer, uint32_t size) const;

  bool GetMessage(std::string& message);
};

}  // namespace ueds_connector
