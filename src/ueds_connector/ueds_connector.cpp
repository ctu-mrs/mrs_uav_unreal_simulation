// Copyright [2022] <Jakub Jirkal>
// This code is licensed under MIT license (see LICENSE for details)

#include <ueds_connector/ueds_connector.h>

#include <sstream>

using kissnet::socket_status;
using ueds_connector::CameraConfig;
using ueds_connector::Coordinates;
using ueds_connector::LidarConfig;
using ueds_connector::LidarData;
using ueds_connector::LidarSegData;
using ueds_connector::Rotation;
using ueds_connector::UedsConnector;

/* getLocation() //{ */

std::pair<bool, Coordinates> UedsConnector::GetLocation() {

  Serializable::Drone::GetLocation::Request request{};

  Serializable::Drone::GetLocation::Response response{};
  const auto                                 status  = Request(request, response);
  const auto                                 success = status && response.status;

  Coordinates coordinates{};

  if (success) {
    coordinates.x = response.x;
    coordinates.y = response.y;
    coordinates.z = response.z;
  }

  return std::make_pair(success, coordinates);
}

//}

/* setLocation() //{ */

std::tuple<bool, Coordinates, bool, Coordinates> UedsConnector::SetLocation(const Coordinates& coordinate, bool checkCollisions) {

  Serializable::Drone::SetLocation::Request request{};
  request.x               = coordinate.x;
  request.y               = coordinate.y;
  request.z               = coordinate.z;
  request.checkCollisions = checkCollisions;

  Serializable::Drone::SetLocation::Response response{};
  const auto                                 status  = Request(request, response);
  const auto                                 success = status && response.status;

  Coordinates teleportedTo{};
  Coordinates impactPoint{};
  if (success) {
    teleportedTo.x = response.teleportedToX;
    teleportedTo.y = response.teleportedToY;
    teleportedTo.z = response.teleportedToZ;
    impactPoint.x  = response.impactPointX;
    impactPoint.y  = response.impactPointY;
    impactPoint.z  = response.impactPointZ;
  }

  return std::make_tuple(success, teleportedTo, response.isHit, impactPoint);
}

//}

/* GetCameraData() //{ */

std::tuple<bool, std::vector<unsigned char>, uint32_t> UedsConnector::GetCameraData() {

  Serializable::Drone::GetCameraData::Request request{};

  Serializable::Drone::GetCameraData::Response response{};
  const auto                                   status  = Request(request, response);
  const auto                                   success = status && response.status;

  return std::make_tuple(success, success ? response.imageData : std::vector<unsigned char>(), success ? response.imageData.size() : 0);
}

//}

/* GetCameraDepth() //{ */
std::tuple<bool, std::vector<unsigned char>, uint32_t> UedsConnector::GetCameraDepth() {
  Serializable::Drone::GetCameraDepth::Request request{};

  Serializable::Drone::GetCameraDepth::Response response{};
  const auto status = Request(request, response);
  const auto success = status && response.status;

  return std::make_tuple(success, success ? response.imageData : std::vector<unsigned char>(),
                         success ? response.imageData.size() : 0);
}
//}

/* GetCameraSeg() //{ */
std::tuple<bool, std::vector<unsigned char>, uint32_t> UedsConnector::GetCameraSeg() {
  Serializable::Drone::GetCameraSeg::Request request{};

  Serializable::Drone::GetCameraSeg::Response response{};
  const auto status = Request(request, response);
  const auto success = status && response.status;

  return std::make_tuple(success, success ? response.imageData : std::vector<unsigned char>(),
                         success ? response.imageData.size() : 0);
}
//}

/* GetCameraColorDepth() //{ */
std::tuple<bool, std::vector<unsigned char>, uint32_t> UedsConnector::GetCameraColorDepth() {
  Serializable::Drone::GetCameraColorDepth::Request request{};

  Serializable::Drone::GetCameraColorDepth::Response response{};
  const auto status = Request(request, response);
  const auto success = status && response.status;

  return std::make_tuple(success, success ? response.imageData : std::vector<unsigned char>(),
                         success ? response.imageData.size() : 0);
}
//}

/* getRotation() //{ */

std::pair<bool, Rotation> UedsConnector::GetRotation() {

  Serializable::Drone::GetRotation::Request request{};

  Serializable::Drone::GetRotation::Response response{};
  const auto                                 status  = Request(request, response);
  const auto                                 success = status && response.status;

  Rotation rotation{};
  if (success) {
    rotation.pitch = response.pitch;
    rotation.yaw   = response.yaw;
    rotation.roll  = response.roll;
  }

  return std::make_pair(success, rotation);
}

//}

/* setRotation() //{ */

std::tuple<bool, Rotation, bool, Coordinates> UedsConnector::SetRotation(const Rotation& rotation) {

  Serializable::Drone::SetRotation::Request request{};
  request.pitch = rotation.pitch;
  request.yaw   = rotation.yaw;
  request.roll  = rotation.roll;

  Serializable::Drone::SetRotation::Response response{};
  const auto                                 status  = Request(request, response);
  const auto                                 success = status && response.status;

  Rotation    rotatedTo{};
  Coordinates impactPoint{};
  bool        isHit = false;
  if (success) {
    rotatedTo.pitch = response.rotatedToPitch;
    rotatedTo.yaw   = response.rotatedToYaw;
    rotatedTo.roll  = response.rotatedToRoll;
    isHit           = response.isHit;
    impactPoint.x   = response.impactPointX;
    impactPoint.y   = response.impactPointY;
    impactPoint.z   = response.impactPointZ;
  }

  return std::make_tuple(success, rotatedTo, isHit, impactPoint);
}

//}

/* setLocationAndRotation() //{ */

std::tuple<bool, Coordinates, Rotation, bool, Coordinates> UedsConnector::SetLocationAndRotation(const Coordinates& coordinate, const Rotation& rotation) {

  Serializable::Drone::SetLocationAndRotation::Request request{};
  request.x     = coordinate.x;
  request.y     = coordinate.y;
  request.z     = coordinate.z;
  request.pitch = rotation.pitch;
  request.yaw   = rotation.yaw;
  request.roll  = rotation.roll;

  Serializable::Drone::SetLocationAndRotation::Response response{};
  const auto                                            status  = Request(request, response);
  const auto                                            success = status && response.status;

  Coordinates teleportedTo{};
  Rotation    rotatedTo{};
  Coordinates impactPoint{};
  bool        isHit = false;

  if (success) {
    teleportedTo.x  = response.teleportedToX;
    teleportedTo.y  = response.teleportedToY;
    teleportedTo.z  = response.teleportedToZ;
    rotatedTo.pitch = response.rotatedToPitch;
    rotatedTo.yaw   = response.rotatedToYaw;
    rotatedTo.roll  = response.rotatedToRoll;
    isHit           = response.isHit;
    impactPoint.x   = response.impactPointX;
    impactPoint.y   = response.impactPointY;
    impactPoint.z   = response.impactPointZ;
  }

  return std::make_tuple(success, teleportedTo, rotatedTo, isHit, impactPoint);
}

//}

/* getLidarData() //{ */

std::tuple<bool, std::vector<LidarData>, Coordinates> UedsConnector::GetLidarData() {

  Serializable::Drone::GetLidarData::Request request{};

  Serializable::Drone::GetLidarData::Response response{};
  const auto                                  status  = Request(request, response);
  const auto                                  success = status && response.status;
  std::vector<LidarData>                      lidarData;
  Coordinates                                 start{};
  if (success) {
    const auto arrSize = response.lidarData.size();
    lidarData.resize(arrSize);
    for (int i = 0; i < arrSize; i++) {
      lidarData[i]            = LidarData{};
      lidarData[i].distance   = response.lidarData[i].distance;
      lidarData[i].directionX = response.lidarData[i].directionX;
      lidarData[i].directionY = response.lidarData[i].directionY;
      lidarData[i].directionZ = response.lidarData[i].directionZ;
    }

    start.x = response.startX;
    start.y = response.startY;
    start.z = response.startZ;
  }

  // std::cout << "Get lidar data drone controller: " << success << std::endl;
  return std::make_tuple(success, lidarData, start);
}

//}

/* getLidarSegData() //{ */

std::tuple<bool, std::vector<LidarSegData>, Coordinates> UedsConnector::GetLidarSegData() {
  Serializable::Drone::GetLidarSegData::Request request{};

  Serializable::Drone::GetLidarSegData::Response response{};
  const auto status = Request(request, response);
  const auto success = status && response.status;
  std::vector<LidarSegData> lidarSegData;
  Coordinates start{};
  if (success) {
    const auto arrSize = response.lidarSegData.size();
    lidarSegData.resize(arrSize);
    for (int i = 0; i < arrSize; i++) {
      lidarSegData[i] = LidarSegData{};
      lidarSegData[i].distance = response.lidarSegData[i].distance;
      lidarSegData[i].directionX = response.lidarSegData[i].directionX;
      lidarSegData[i].directionY = response.lidarSegData[i].directionY;
      lidarSegData[i].directionZ = response.lidarSegData[i].directionZ;
      lidarSegData[i].segmentation = response.lidarSegData[i].segmentation;
    }

    start.x = response.startX;
    start.y = response.startY;
    start.z = response.startZ;
  }
  //std::cout << "Get lidar data drone controller: " << success << std::endl;
  return std::make_tuple(success, lidarSegData, start);
}
//}

/* getLidarConfig() //{ */

std::pair<bool, LidarConfig> UedsConnector::GetLidarConfig() {

  Serializable::Drone::GetLidarConfig::Request request{};

  Serializable::Drone::GetLidarConfig::Response response{};
  const auto                                    status  = Request(request, response);
  const auto                                    success = status && response.status;

  /* bool success = true; */
  LidarConfig config{};

  if (success) {
    config.Enable       = response.config.Enable;
    config.showBeams    = response.config.ShowBeams;
    config.BeamHorRays  = response.config.BeamHorRays;
    config.BeamVertRays = response.config.BeamVertRays;
    config.beamLength   = response.config.BeamLength;
    config.Frequency    = response.config.Frequency;
    config.offset       = Coordinates{response.config.OffsetX, response.config.OffsetY, response.config.OffsetZ};

    config.orientation = Rotation{response.config.OrientationPitch, response.config.OrientationYaw, response.config.OrientationRoll};
    config.FOVHor      = response.config.FOVHor;
    config.FOVVert     = response.config.FOVVert;
  }

  return std::make_pair(success, config);
}

//}

/* setLidarConfig() //{ */

bool UedsConnector::SetLidarConfig(const LidarConfig& config) {

  Serializable::Drone::SetLidarConfig::Request request{};

  request.config              = Serializable::Drone::LidarConfig{};
  request.config.Enable       = config.Enable;
  request.config.ShowBeams    = config.showBeams;
  request.config.BeamHorRays  = config.BeamHorRays;
  request.config.BeamVertRays = config.BeamVertRays;
  request.config.BeamLength   = config.beamLength;
  request.config.Frequency    = config.Frequency;

  request.config.OffsetX = config.offset.x;
  request.config.OffsetY = config.offset.y;
  request.config.OffsetZ = config.offset.z;

  request.config.OrientationPitch = config.orientation.pitch;
  request.config.OrientationYaw   = config.orientation.yaw;
  request.config.OrientationRoll  = config.orientation.roll;

  request.config.FOVHor  = config.FOVHor;
  request.config.FOVVert = config.FOVVert;

  Serializable::Drone::SetLidarConfig::Response response{};

  const auto status  = Request(request, response);
  const auto success = status && response.status;

  return success;
}

//}

/* getCameraConfig() //{ */

std::pair<bool, CameraConfig> UedsConnector::GetCameraConfig() {

  Serializable::Drone::GetCameraConfig::Request request{};

  Serializable::Drone::GetCameraConfig::Response response{};
  const auto                                     status  = Request(request, response);
  const auto                                     success = status && response.status;

  CameraConfig config{};

  if (success) {
    config.showDebugCamera = response.config.showDebugCamera;

    config.angleFOV = response.config.angleFOV;

    config.offset = Coordinates{response.config.offsetX, response.config.offsetY, response.config.offsetZ};

    config.orientation = Rotation{response.config.orientationPitch, response.config.orientationYaw, response.config.orientationRoll};
  }

  return std::make_pair(success, config);
}

//}

/* setCameraConfig() //{ */

bool UedsConnector::SetCameraConfig(const CameraConfig& config) {

  Serializable::Drone::SetCameraConfig::Request request{};

  request.config                 = Serializable::Drone::CameraConfig{};
  request.config.showDebugCamera = config.showDebugCamera;

  request.config.angleFOV = config.angleFOV;

  request.config.offsetX = config.offset.x;
  request.config.offsetY = config.offset.y;
  request.config.offsetZ = config.offset.z;

  request.config.orientationPitch = config.orientation.pitch;
  request.config.orientationYaw   = config.orientation.yaw;
  request.config.orientationRoll  = config.orientation.roll;

  Serializable::Drone::SetCameraConfig::Response response{};

  const auto status  = Request(request, response);
  const auto success = status && response.status;

  return success;
}

//}

/* getMoveLineVisible() //{ */

std::pair<bool, bool> UedsConnector::GetMoveLineVisible() {

  Serializable::Drone::GetMoveLineVisible::Request request{};

  Serializable::Drone::GetMoveLineVisible::Response response{};
  const auto                                        status  = Request(request, response);
  const auto                                        success = status && response.status;

  return std::make_pair(success, success ? response.visible : false);
}

//}

/* setMoveLineVisible() //{ */

bool UedsConnector::SetMoveLineVisible(bool visible) {

  Serializable::Drone::SetMoveLineVisible::Request request{};
  request.visible = visible;

  Serializable::Drone::SetMoveLineVisible::Response response{};
  const auto                                        status  = Request(request, response);
  const auto                                        success = status && response.status;

  return success;
}

//}
