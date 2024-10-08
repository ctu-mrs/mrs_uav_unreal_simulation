// Copyright [2022] <Jakub Jirkal>
// This code is licensed under MIT license (see LICENSE for details)

#pragma once

#include <string>
#include <vector>

#include <ueds_connector/data_types.h>
#include <ueds_connector/socket_client.h>
#include <ueds_connector/serialization/serializable_shared.h>

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

  std::pair<bool, int> SpawnDroneAtLocation(ueds_connector::Coordinates &Location, std::string &TypeUav);

  bool RemoveDrone(const int port);

  std::pair<bool, CameraCaptureModeEnum> GetCameraCaptureMode();

  bool SetCameraCaptureMode(const CameraCaptureModeEnum& cameraCaptureMode);

  std::pair<bool, float> GetFps();

  std::pair<bool, int> GetApiVersion();

  std::pair<bool, double> GetTime();
  
  bool SetGraphicsSettings(const Serializable::GameMode::GraphicsSettingsEnum& graphicsSettings);

  bool SwitchWorldLevel(const Serializable::GameMode::WorldLevelEnum& worldLevelEnum);

  bool SetForestDensity(const int DensityLevel);

  bool SetForestHillyLevel(const int HillyLevel);

  std::pair<bool, ueds_connector::Coordinates> GetWorldOrigin();
};

}  // namespace ueds_connector
