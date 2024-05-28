// Copyright [2022] <Jakub Jirkal>
// This code is licensed under MIT license (see LICENSE for details)

#pragma once

#include <string>
#include <vector>

#include <ueds_connector/data-types.h>
#include <ueds_connector/socket-client.h>

namespace ueds_connector
{

class UedsConnector : public SocketClient {

public:
  UedsConnector() : SocketClient() {
  }

  UedsConnector(const std::string& address, uint16_t port) : SocketClient(address, port) {
  }

  std::pair<bool, Coordinates> GetLocation();

  std::tuple<bool, Coordinates, bool, Coordinates> SetLocation(const Coordinates& coordinates, bool checkCollisions);

  std::tuple<bool, std::vector<unsigned char>, uint32_t> GetLeftCameraData();

  std::tuple<bool, std::vector<unsigned char>, uint32_t> GetRightCameraData();

  std::tuple<bool, std::vector<unsigned char>, uint32_t> GetCameraDepth();

  std::tuple<bool, std::vector<unsigned char>, uint32_t> GetCameraSeg();

  std::tuple<bool, std::vector<unsigned char>, uint32_t> GetCameraColorDepth();

  std::pair<bool, Rotation> GetRotation();

  std::tuple<bool, Rotation, bool, Coordinates> SetRotation(const Rotation& rotation);

  std::tuple<bool, Coordinates, Rotation, bool, Coordinates> SetLocationAndRotation(const Coordinates& coordinate, const Rotation& rotation);

  std::tuple<bool, std::vector<LidarData>, Coordinates> GetLidarData();

  std::tuple<bool, std::vector<LidarSegData>, Coordinates> GetLidarSegData();

  std::pair<bool, LidarConfig> GetLidarConfig();

  bool SetLidarConfig(const LidarConfig& config);

  std::pair<bool, CameraConfig> GetCameraConfig();

  bool SetCameraConfig(const CameraConfig& config);

  std::pair<bool, bool> GetMoveLineVisible();

  bool SetMoveLineVisible(bool visible);
};

}  // namespace ueds_connector
