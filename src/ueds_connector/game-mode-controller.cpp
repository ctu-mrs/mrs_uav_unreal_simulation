// Copyright [2022] <Jakub Jirkal>
// This code is licensed under MIT license (see LICENSE for details)

#include <ueds_connector/game-mode-controller.h>

#include <sstream>

using kissnet::socket_status;
using ueds_connector::CameraCaptureModeEnum;
using ueds_connector::GameModeController;

/* getDrones() //{ */

std::pair<bool, std::vector<int>> GameModeController::GetDrones() {

  Serializable::GameMode::GetDrones::Request request{};

  Serializable::GameMode::GetDrones::Response response{};
  const auto                                  status  = Request(request, response);
  const auto                                  success = status && response.status;

  return std::make_pair(success, success ? response.ports : std::vector<int>());
}

//}

/* spawnDrone() //{ */

std::pair<bool, int> GameModeController::SpawnDrone() {

  Serializable::GameMode::SpawnDrone::Request request{};

  Serializable::GameMode::SpawnDrone::Response response{};
  const auto                                   status  = Request(request, response);
  const auto                                   success = status && response.status;

  return std::make_pair(success, success ? response.port : 0);
}

//}

/* removeDrone() //{ */

bool GameModeController::RemoveDrone(const int port) {

  Serializable::GameMode::RemoveDrone::Request request{};
  request.port = port;

  Serializable::GameMode::RemoveDrone::Response response{};
  const auto                                    status  = Request(request, response);
  const auto                                    success = status && response.status;

  return success;
}

//}

/* getCameraCaptureMode() //{ */

std::pair<bool, CameraCaptureModeEnum> GameModeController::GetCameraCaptureMode() {

  Serializable::GameMode::GetCameraCaptureMode::Request request{};

  Serializable::GameMode::GetCameraCaptureMode::Response response{};
  const auto                                             status  = Request(request, response);
  const auto                                             success = status && response.status;

  CameraCaptureModeEnum captureMode = CameraCaptureModeEnum::CAPTURE_ALL_FRAMES;
  if (success) {
    captureMode = static_cast<CameraCaptureModeEnum>(response.cameraCaptureMode);
  }

  return std::make_pair(success, captureMode);
}

//}

/* setCameraCaptureMode() //{ */

bool GameModeController::SetCameraCaptureMode(const CameraCaptureModeEnum& cameraCaptureMode) {

  Serializable::GameMode::SetCameraCaptureMode::Request request{};
  request.cameraCaptureMode = static_cast<Serializable::GameMode::CameraCaptureModeEnum>(cameraCaptureMode);

  Serializable::GameMode::SetCameraCaptureMode::Response response{};
  const auto                                             status  = Request(request, response);
  const auto                                             success = status && response.status;

  return success;
}

//}

/* getFps() //{ */

std::pair<bool, float> GameModeController::GetFps() {

  Serializable::GameMode::GetFps::Request request{};

  Serializable::GameMode::GetFps::Response response{};
  const auto                               status  = Request(request, response);
  const auto                               success = status && response.status;

  return std::make_pair(success, success ? response.fps : 0);
}

//}
