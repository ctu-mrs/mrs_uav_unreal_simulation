// Copyright [2022] <Jakub Jirkal>
// This code is licensed under MIT license (see LICENSE for details)

#pragma once

#include <string>
#include <vector>

#include <ueds_connector/socket_client.h>

namespace ueds_connector
{

class GameModeController : public SocketClient {
public:
  GameModeController() : SocketClient() {
  }
  GameModeController(const std::string& address, uint16_t port) : SocketClient(address, port) {
  }

  std::pair<bool, std::vector<int>> GetDrones();

  std::pair<bool, int> SpawnDrone();

  bool RemoveDrone(const int port);

  std::pair<bool, CameraCaptureModeEnum> GetCameraCaptureMode();

  bool SetCameraCaptureMode(const CameraCaptureModeEnum& cameraCaptureMode);

  std::pair<bool, float> GetFps();
};

}  // namespace ueds_connector
