// Copyright [2022] <Jakub Jirkal>
// This code is licensed under MIT license (see LICENSE for details)

#pragma once

#include <string>
#include <vector>

#include <ueds_connector/data_types.h>
#include <ueds_connector/socket_client.h>

namespace ueds_connector
{

class UedsConnector : public SocketClient {

public:
  UedsConnector() : SocketClient() {
  }

  UedsConnector(const std::string& address, uint16_t port) : SocketClient(address, port) {
  }

  std::pair<bool, Coordinates> GetLocation();

  std::pair<bool, bool> GetCrashState();

  std::tuple<bool, Coordinates, bool, Coordinates> SetLocation(const Coordinates& coordinates, bool checkCollisions);

  std::tuple<bool, std::vector<unsigned char>, double, uint32_t> GetRgbCameraData();

  std::tuple<bool, std::vector<unsigned char>, std::vector<unsigned char>, double> GetStereoCameraData();

  std::tuple<bool, std::vector<unsigned char>, double, uint32_t> GetRgbSegmented();

  std::pair<bool, Rotation> GetRotation();

  std::tuple<bool, Rotation, bool, Coordinates> SetRotation(const Rotation& rotation);

  std::tuple<bool, Coordinates, Rotation, bool, Coordinates> SetLocationAndRotation(const Coordinates& coordinate, const Rotation& rotation, const bool should_collide);

  std::tuple<bool> SetLocationAndRotationAsync(const Coordinates& coordinate, const Rotation& rotation, const bool should_collide);

  std::tuple<bool, std::vector<LidarData>, Coordinates> GetLidarData();

  std::tuple<bool, std::vector<LidarSegData>, Coordinates> GetLidarSegData();

  std::pair<bool, LidarConfig> GetLidarConfig();

  bool SetLidarConfig(const LidarConfig& config);

  std::pair<bool, RgbCameraConfig> GetRgbCameraConfig();

  bool SetRgbCameraConfig(const RgbCameraConfig& config);

  std::pair<bool, StereoCameraConfig> GetStereoCameraConfig();

  bool SetStereoCameraConfig(const StereoCameraConfig& config);

  std::pair<bool, bool> GetMoveLineVisible();

  bool SetMoveLineVisible(bool visible);
};

}  // namespace ueds_connector
