// Copyright [2022] <Jakub Jirkal>
// This code is licensed under MIT license (see LICENSE for details)

#include <ueds_connector/ueds_connector.h>

#include <sstream>

using kissnet::socket_status;
using ueds_connector::Coordinates;
using ueds_connector::LidarConfig;
using ueds_connector::LidarData;
using ueds_connector::LidarIntData;
using ueds_connector::LidarSegData;
using ueds_connector::RgbCameraConfig;
using ueds_connector::Rotation;
using ueds_connector::StereoCameraConfig;
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

/* getCrashState() //{ */

std::pair<bool, bool> UedsConnector::GetCrashState() {

  Serializable::Drone::GetCrashState::Request request{};

  Serializable::Drone::GetCrashState::Response response{};
  const auto                                   status  = Request(request, response);
  const auto                                   success = status && response.status;

  bool crashed = response.crashed;

  return std::make_pair(success, crashed);
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

/* GetRgbCameraData() //{ */

std::tuple<bool, std::vector<unsigned char>, double, uint32_t> UedsConnector::GetRgbCameraData() {

  Serializable::Drone::GetRgbCameraData::Request request{};

  Serializable::Drone::GetRgbCameraData::Response response{};

  const auto status  = Request(request, response);
  const auto success = status && response.status;

  return std::make_tuple(success, success ? response.image_ : std::vector<unsigned char>(), success ? response.stamp_ : 0.0,
                         success ? response.image_.size() : 0);
}

//}

/* GetStereoCameraData() //{ */

std::tuple<bool, std::vector<unsigned char>, std::vector<unsigned char>, double> UedsConnector::GetStereoCameraData() {

  Serializable::Drone::GetStereoCameraData::Request request{};

  Serializable::Drone::GetStereoCameraData::Response response{};

  const auto status  = Request(request, response);
  const auto success = status && response.status;

  return std::make_tuple(success, success ? response.image_left_ : std::vector<unsigned char>(), success ? response.image_right_ : std::vector<unsigned char>(),
                         success ? response.stamp_ : 0.0);
}

//}

/* GetRgbSegmented() //{ */

std::tuple<bool, std::vector<unsigned char>, double, uint32_t> UedsConnector::GetRgbSegmented() {

  Serializable::Drone::GetRgbSegCameraData::Request request{};

  Serializable::Drone::GetRgbSegCameraData::Response response{};
  const auto                                         status  = Request(request, response);
  const auto                                         success = status && response.status;

  return std::make_tuple(success, success ? response.image_ : std::vector<unsigned char>(), success ? response.stamp_ : 0.0,
                         success ? response.image_.size() : 0);
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

std::tuple<bool, Coordinates, Rotation, bool, Coordinates> UedsConnector::SetLocationAndRotation(const Coordinates& coordinate, const Rotation& rotation,
                                                                                                 const bool should_collide) {

  Serializable::Drone::SetLocationAndRotation::Request request{};

  request.x = coordinate.x;
  request.y = coordinate.y;
  request.z = coordinate.z;

  request.pitch = rotation.pitch;
  request.yaw   = rotation.yaw;
  request.roll  = rotation.roll;

  request.should_collide = should_collide;

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

/* setLocationAndRotationAsync() //{ */

std::tuple<bool> UedsConnector::SetLocationAndRotationAsync(const Coordinates& coordinate, const Rotation& rotation, const bool should_collide) {

  Serializable::Drone::SetLocationAndRotationAsync::Request request{};

  request.x = coordinate.x;
  request.y = coordinate.y;
  request.z = coordinate.z;

  request.pitch = rotation.pitch;
  request.yaw   = rotation.yaw;
  request.roll  = rotation.roll;

  request.should_collide = should_collide;

  Serializable::Drone::SetLocationAndRotationAsync::Response response{};
  const auto                                                 status  = Request(request, response);
  const auto                                                 success = status && response.status;

  return std::make_tuple(success);
}

//}

/* getLidarData() //{ */

std::tuple<bool, std::vector<LidarData>, Coordinates> UedsConnector::GetLidarData() {

  /* std::cout << "GetLidarData()" << std::endl; */
  Serializable::Drone::GetLidarData::Request request{};

  Serializable::Drone::GetLidarData::Response response{};
  const auto                                  status  = Request(request, response);
  const auto                                  success = status && response.status;
  std::vector<LidarData>                      lidarData;
  Coordinates                                 start{};

  if (success) {

    const auto arrSize = response.lidarData.size();
    lidarData.resize(arrSize);

    for (size_t i = 0; i < arrSize; i++) {
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

std::tuple<bool, double> ueds_connector::UedsConnector::GetRangefinderData()
{
  Serializable::Drone::GetRangefinderData::Request request{};

  Serializable::Drone::GetRangefinderData::Response response{};
  const auto                                        status  = Request(request, response);
  const auto                                        success = status && response.status;

  return std::make_pair(success, success ? response.range : -1);
} //}

/* getLidarSegData() //{ */

std::tuple<bool, std::vector<LidarSegData>, Coordinates> UedsConnector::GetLidarSegData() {
  Serializable::Drone::GetLidarSegData::Request request{};

  Serializable::Drone::GetLidarSegData::Response response{};
  const auto                                     status  = Request(request, response);
  const auto                                     success = status && response.status;
  std::vector<LidarSegData>                      lidarSegData;
  Coordinates                                    start{};

  if (success) {

    const auto arrSize = response.lidarSegData.size();
    lidarSegData.resize(arrSize);

    for (size_t i = 0; i < arrSize; i++) {
      lidarSegData[i]              = LidarSegData{};
      lidarSegData[i].distance     = response.lidarSegData[i].distance;
      lidarSegData[i].directionX   = response.lidarSegData[i].directionX;
      lidarSegData[i].directionY   = response.lidarSegData[i].directionY;
      lidarSegData[i].directionZ   = response.lidarSegData[i].directionZ;
      lidarSegData[i].segmentation = response.lidarSegData[i].segmentation;
    }

    start.x = response.startX;
    start.y = response.startY;
    start.z = response.startZ;
  }

  // std::cout << "Get lidar data drone controller: " << success << std::endl;
  return std::make_tuple(success, lidarSegData, start);
}
//}

/* getLidarIntData() //{ */

std::tuple<bool, std::vector<LidarIntData>, Coordinates> UedsConnector::GetLidarIntData() {

  /* std::cout << "GetLidarIntData()" << std::endl; */

  Serializable::Drone::GetLidarIntData::Request request{};

  Serializable::Drone::GetLidarIntData::Response response{};
  const auto                                     status  = Request(request, response);
  const auto                                     success = status && response.status;
  std::vector<LidarIntData>                      lidarIntData;
  Coordinates                                    start{};

  if (success) {

    const auto arrSize = response.lidarIntData.size();
    lidarIntData.resize(arrSize);

    for (size_t i = 0; i < arrSize; i++) {
      lidarIntData[i]            = LidarIntData{};
      lidarIntData[i].distance   = response.lidarIntData[i].distance;
      lidarIntData[i].directionX = response.lidarIntData[i].directionX;
      lidarIntData[i].directionY = response.lidarIntData[i].directionY;
      lidarIntData[i].directionZ = response.lidarIntData[i].directionZ;
      lidarIntData[i].intensity  = response.lidarIntData[i].intensity;
    }

    start.x = response.startX;
    start.y = response.startY;
    start.z = response.startZ;
  }

  return std::make_tuple(success, lidarIntData, start);
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

    config.FOVHor  = response.config.FOVHor;
    config.FOVVert = response.config.FOVVert;
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

/* getRgbCameraConfig() //{ */

std::pair<bool, RgbCameraConfig> UedsConnector::GetRgbCameraConfig() {

  Serializable::Drone::GetRgbCameraConfig::Request request{};

  Serializable::Drone::GetRgbCameraConfig::Response response{};
  const auto                                        status  = Request(request, response);
  const auto                                        success = status && response.status;

  RgbCameraConfig config{};

  if (success) {

    config.show_debug_camera_ = response.config.show_debug_camera_;

    config.fov_ = response.config.fov_;

    config.offset_ = Coordinates{response.config.offset_x_, response.config.offset_y_, response.config.offset_z_};

    config.orientation_ = Rotation{response.config.rotation_pitch_, response.config.rotation_yaw_, response.config.rotation_roll_};

    config.width_  = response.config.width_;
    config.height_ = response.config.height_;

    config.enable_temporal_aa_ = response.config.enable_temporal_aa_;
    config.enable_hdr_         = response.config.enable_hdr_;
    config.enable_raytracing_  = response.config.enable_raytracing_;
  }

  return std::make_pair(success, config);
}

//}

/* getStereoCameraConfig() //{ */

std::pair<bool, StereoCameraConfig> UedsConnector::GetStereoCameraConfig() {

  Serializable::Drone::GetStereoCameraConfig::Request request{};

  Serializable::Drone::GetStereoCameraConfig::Response response{};
  const auto                                           status  = Request(request, response);
  const auto                                           success = status && response.status;

  StereoCameraConfig config{};

  if (success) {

    config.show_debug_camera_ = response.config.show_debug_camera_;

    config.fov_ = response.config.fov_;

    config.offset_ = Coordinates{response.config.offset_x_, response.config.offset_y_, response.config.offset_z_};

    config.orientation_ = Rotation{response.config.rotation_pitch_, response.config.rotation_yaw_, response.config.rotation_roll_};

    config.width_  = response.config.width_;
    config.height_ = response.config.height_;

    config.baseline_ = response.config.baseline_;

    config.enable_temporal_aa_ = response.config.enable_temporal_aa_;
    config.enable_hdr_         = response.config.enable_hdr_;
    config.enable_raytracing_  = response.config.enable_raytracing_;
  }

  return std::make_pair(success, config);
}

//}

/* setRgbCameraConfig() //{ */

bool UedsConnector::SetRgbCameraConfig(const RgbCameraConfig& config) {

  Serializable::Drone::SetRgbCameraConfig::Request request{};

  request.config                    = Serializable::Drone::RgbCameraConfig{};
  request.config.show_debug_camera_ = config.show_debug_camera_;

  request.config.fov_ = config.fov_;

  request.config.offset_x_ = config.offset_.x;
  request.config.offset_y_ = config.offset_.y;
  request.config.offset_z_ = config.offset_.z;

  request.config.rotation_pitch_ = config.orientation_.pitch;
  request.config.rotation_yaw_   = config.orientation_.yaw;
  request.config.rotation_roll_  = config.orientation_.roll;

  request.config.width_  = config.width_;
  request.config.height_ = config.height_;

  request.config.enable_temporal_aa_ = config.enable_temporal_aa_;
  request.config.enable_hdr_         = config.enable_hdr_;
  request.config.enable_raytracing_  = config.enable_raytracing_;

  Serializable::Drone::SetRgbCameraConfig::Response response{};

  const auto status  = Request(request, response);
  const auto success = status && response.status;

  return success;
}

//}

/* setStereoCameraConfig() //{ */

bool UedsConnector::SetStereoCameraConfig(const StereoCameraConfig& config) {

  Serializable::Drone::SetStereoCameraConfig::Request request{};

  request.config                    = Serializable::Drone::StereoCameraConfig{};
  request.config.show_debug_camera_ = config.show_debug_camera_;

  request.config.fov_ = config.fov_;

  request.config.offset_x_ = config.offset_.x;
  request.config.offset_y_ = config.offset_.y;
  request.config.offset_z_ = config.offset_.z;

  request.config.rotation_pitch_ = config.orientation_.pitch;
  request.config.rotation_yaw_   = config.orientation_.yaw;
  request.config.rotation_roll_  = config.orientation_.roll;

  request.config.width_  = config.width_;
  request.config.height_ = config.height_;

  request.config.baseline_ = config.baseline_;

  request.config.enable_temporal_aa_ = config.enable_temporal_aa_;
  request.config.enable_hdr_         = config.enable_hdr_;
  request.config.enable_raytracing_  = config.enable_raytracing_;

  Serializable::Drone::SetStereoCameraConfig::Response response{};

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
