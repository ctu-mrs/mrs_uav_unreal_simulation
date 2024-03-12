// Copyright [2022] <Jakub Jirkal>
// This code is licensed under MIT license (see LICENSE for details)

#pragma once

#include <vector>

#include <cereal/types/vector.hpp>

namespace Serializable {
namespace Common {
enum MessageType : unsigned short { ping = 0x1 };

struct NetworkRequest {
  NetworkRequest() = default;
  explicit NetworkRequest(unsigned short _type) : type(_type) {}

  unsigned short type;

  template <class Archive>
  void serialize(Archive& archive) {
    archive(type);
  }
};

struct NetworkResponse {
  NetworkResponse() = default;
  explicit NetworkResponse(unsigned short _type) : type(_type), status(true) {}
  explicit NetworkResponse(bool _status) : status(_status) {}
  explicit NetworkResponse(unsigned short _type, bool _status) : type(_type), status(_status) {}

  unsigned short type;
  bool status;

  template <class Archive>
  void serialize(Archive& archive) {
    archive(type, status);
  }
};

namespace Ping {
struct Request : public Common::NetworkRequest {
  Request() : Common::NetworkRequest(static_cast<unsigned short>(MessageType::ping)) {}
};

struct Response : public Common::NetworkResponse {
  Response() : Common::NetworkResponse(static_cast<unsigned short>(MessageType::ping)) {}
  explicit Response(bool _status) : Common::NetworkResponse(MessageType::ping, _status) {}
};
}  // namespace Ping
}  // namespace Common

namespace Drone {
enum MessageType : unsigned short {
  get_location = 0x2,
  set_location = 0x3,
  get_camera_data = 0x4,
  get_rotation = 0x5,
  set_rotation = 0x6,
  set_location_and_rotation = 0x7,
  get_lidar_data = 0x8,
  get_lidar_config = 0x9,
  set_lidar_config = 0x10,
  get_camera_config = 0x11,
  set_camera_config = 0x12,
  get_move_line_visible = 0x13,
  set_move_line_visible = 0x14,
  get_camera_depth = 0x15,
  get_camera_seg =  0x16,
  get_lidar_seg = 0x17,
  get_camera_color_depth = 0x18,
};

struct LidarConfig
{
  bool Enable;
  bool ShowBeams;
  double BeamLength;

  double BeamHorRays;
  double BeamVertRays;

  double Frequency;
  double OffsetX;
  double OffsetY;
  double OffsetZ;

  double OrientationPitch;
  double OrientationYaw;
  double OrientationRoll;

  double FOVHor;
  double FOVVert;

//  double vertRayDiff;
//  double horRayDif;

  template <class Archive>
  void serialize(Archive& archive) {
    archive(Enable, ShowBeams, BeamLength, BeamHorRays, BeamVertRays, Frequency, OffsetX, OffsetY, OffsetZ, OrientationPitch, OrientationYaw, OrientationRoll,FOVHor,FOVVert);
  }
};

struct CameraConfig
{
  bool showDebugCamera;

  double offsetX;
  double offsetY;
  double offsetZ;

  double orientationPitch;
  double orientationYaw;
  double orientationRoll;

  double angleFOV;
  
  int Width;
  int Height;

  template <class Archive>
  void serialize(Archive& archive) {
    archive(showDebugCamera, offsetX, offsetY, offsetZ, orientationPitch, orientationYaw, orientationRoll, angleFOV, Width, Height);
  }
};

namespace GetLocation {
struct Request : public Common::NetworkRequest {
  Request() : Common::NetworkRequest(static_cast<unsigned short>(MessageType::get_location)) {}
};

struct Response : public Common::NetworkResponse {
  Response() : Common::NetworkResponse(static_cast<unsigned short>(MessageType::get_location)) {}
  explicit Response(bool _status) : Common::NetworkResponse(MessageType::get_location, _status) {}

  double x;
  double y;
  double z;

  template <class Archive>
  void serialize(Archive& archive) {
    archive(cereal::base_class<Common::NetworkResponse>(this), x, y, z);
  }
};
}  // namespace GetLocation

namespace SetLocation {
struct Request : public Common::NetworkRequest {
  Request() : Common::NetworkRequest(static_cast<unsigned short>(MessageType::set_location)) {}

  double x;
  double y;
  double z;
  bool checkCollisions;

  template <class Archive>
  void serialize(Archive& archive) {
    archive(cereal::base_class<Common::NetworkRequest>(this), x, y, z, checkCollisions);
  }
};

struct Response : public Common::NetworkResponse {
  Response() : Common::NetworkResponse(static_cast<unsigned short>(MessageType::set_location)) {}
  explicit Response(bool _status) : Common::NetworkResponse(MessageType::set_location, _status) {}

  double teleportedToX;
  double teleportedToY;
  double teleportedToZ;

  bool isHit;
  double impactPointX;
  double impactPointY;
  double impactPointZ;

  template <class Archive>
  void serialize(Archive& archive) {
    archive(cereal::base_class<Common::NetworkResponse>(this), teleportedToX, teleportedToY, teleportedToZ, isHit, impactPointX, impactPointY, impactPointZ);
  }
};
}  // namespace SetLocation

namespace GetCameraData {
struct Request : public Common::NetworkRequest {
  Request() : Common::NetworkRequest(static_cast<unsigned short>(MessageType::get_camera_data)) {}
};

struct Response : public Common::NetworkResponse {
  Response() : Common::NetworkResponse(static_cast<unsigned short>(MessageType::get_camera_data)) {}
  explicit Response(bool _status) : Common::NetworkResponse(MessageType::get_camera_data, _status) {}

  std::vector<unsigned char> imageData;

  template <class Archive>
  void serialize(Archive& archive) {
    archive(cereal::base_class<Common::NetworkResponse>(this), imageData);
  }
};
}  // namespace GetCameraData

namespace GetCameraDepth {
struct Request : public Common::NetworkRequest {
  Request() : Common::NetworkRequest(static_cast<unsigned short>(MessageType::get_camera_depth)) {}
};

struct Response : public Common::NetworkResponse {
  Response() : Common::NetworkResponse(static_cast<unsigned short>(MessageType::get_camera_depth)) {}
  explicit Response(bool _status) : Common::NetworkResponse(MessageType::get_camera_depth, _status) {}

  std::vector<unsigned char> imageData;

  template <class Archive>
  void serialize(Archive& archive) {
    archive(cereal::base_class<Common::NetworkResponse>(this), imageData);
  }
};
}  // namespace GetCameraDepth

namespace GetCameraSeg {
struct Request : public Common::NetworkRequest {
  Request() : Common::NetworkRequest(static_cast<unsigned short>(MessageType::get_camera_seg)) {}
};

struct Response : public Common::NetworkResponse {
  Response() : Common::NetworkResponse(static_cast<unsigned short>(MessageType::get_camera_seg)) {}
  explicit Response(bool _status) : Common::NetworkResponse(MessageType::get_camera_seg, _status) {}

  std::vector<unsigned char> imageData;

  template <class Archive>
  void serialize(Archive& archive) {
    archive(cereal::base_class<Common::NetworkResponse>(this), imageData);
  }
};
}  // namespace GetCameraSeg

namespace GetCameraColorDepth {
struct Request : public Common::NetworkRequest {
  Request() : Common::NetworkRequest(static_cast<unsigned short>(MessageType::get_camera_color_depth)) {}
};

struct Response : public Common::NetworkResponse {
  Response() : Common::NetworkResponse(static_cast<unsigned short>(MessageType::get_camera_color_depth)) {}
  explicit Response(bool _status) : Common::NetworkResponse(MessageType::get_camera_color_depth, _status) {}

  std::vector<unsigned char> imageData;

  template <class Archive>
  void serialize(Archive& archive) {
    archive(cereal::base_class<Common::NetworkResponse>(this), imageData);
  }
};
}  // namespace GetCameraColorDepth

namespace GetRotation {
struct Request : public Common::NetworkRequest {
  Request() : Common::NetworkRequest(static_cast<unsigned short>(MessageType::get_rotation)) {}
};

struct Response : public Common::NetworkResponse {
  Response() : Common::NetworkResponse(static_cast<unsigned short>(MessageType::get_rotation)) {}
  explicit Response(bool _status) : Common::NetworkResponse(MessageType::get_rotation, _status) {}

  double pitch;
  double yaw;
  double roll;

  template <class Archive>
  void serialize(Archive& archive) {
    archive(cereal::base_class<Common::NetworkResponse>(this), pitch, yaw, roll);
  }
};
}  // namespace GetRotation

namespace SetRotation {
struct Request : public Common::NetworkRequest {
  Request() : Common::NetworkRequest(static_cast<unsigned short>(MessageType::set_rotation)) {}

  double pitch;
  double yaw;
  double roll;

  template <class Archive>
  void serialize(Archive& archive) {
    archive(cereal::base_class<Common::NetworkRequest>(this), pitch, yaw, roll);
  }
};

struct Response : public Common::NetworkResponse {
  Response() : Common::NetworkResponse(static_cast<unsigned short>(MessageType::set_rotation)) {}
  explicit Response(bool _status) : Common::NetworkResponse(MessageType::set_rotation, _status) {}

  double rotatedToPitch;
  double rotatedToYaw;
  double rotatedToRoll;

  bool isHit;
  double impactPointX;
  double impactPointY;
  double impactPointZ;

  template <class Archive>
  void serialize(Archive& archive) {
    archive(cereal::base_class<Common::NetworkResponse>(this), rotatedToPitch, rotatedToYaw, rotatedToRoll, isHit, impactPointX, impactPointY, impactPointZ);
  }
};
}  // namespace SetRotation

namespace SetLocationAndRotation {
struct Request : public Common::NetworkRequest {
  Request() : Common::NetworkRequest(static_cast<unsigned short>(MessageType::set_location_and_rotation)) {}

  double x;
  double y;
  double z;
  double pitch;
  double yaw;
  double roll;

  template <class Archive>
  void serialize(Archive& archive) {
    archive(cereal::base_class<Common::NetworkRequest>(this), x, y, z, pitch, yaw, roll);
  }
};

struct Response : public Common::NetworkResponse {
  Response() : Common::NetworkResponse(static_cast<unsigned short>(MessageType::set_location_and_rotation)) {}
  explicit Response(bool _status) : Common::NetworkResponse(MessageType::set_location_and_rotation, _status) {}

  double teleportedToX;
  double teleportedToY;
  double teleportedToZ;
  
  double rotatedToPitch;
  double rotatedToYaw;
  double rotatedToRoll;

  bool isHit;
  double impactPointX;
  double impactPointY;
  double impactPointZ;

  template <class Archive>
  void serialize(Archive& archive) {
    archive(cereal::base_class<Common::NetworkResponse>(this), teleportedToX, teleportedToY, teleportedToZ, rotatedToPitch, rotatedToYaw, rotatedToRoll, isHit, impactPointX, impactPointY, impactPointZ);
  }
};
}  // namespace SetLocationAndRotation

namespace GetLidarData {
struct LidarData {
  LidarData() = default;

  double distance;
  double directionX;
  double directionY;
  double directionZ;

  template <class Archive>
  void serialize(Archive& archive) {
    archive(distance, directionX, directionY, directionZ);
  }
};

struct Request : public Common::NetworkRequest {
  Request() : Common::NetworkRequest(static_cast<unsigned short>(MessageType::get_lidar_data)) {}
};

struct Response : public Common::NetworkResponse {
  Response() : Common::NetworkResponse(static_cast<unsigned short>(MessageType::get_lidar_data)) {}
  explicit Response(bool _status) : Common::NetworkResponse(MessageType::get_lidar_data, _status) {}

  double startX;
  double startY;
  double startZ;

  std::vector<LidarData> lidarData;

  template <class Archive>
  void serialize(Archive& archive) {
    archive(cereal::base_class<Common::NetworkResponse>(this), startX, startY, startZ, lidarData);
  }
};
}  // namespace GetLidarData

namespace GetLidarSegData {
struct LidarSegData {
  LidarSegData() = default;

  double distance;
  double directionX;
  double directionY;
  double directionZ;
  int segmentation;
  template <class Archive>
  void serialize(Archive& archive) {
    archive(distance, directionX, directionY, directionZ, segmentation);
  }
};

struct Request : public Common::NetworkRequest {
  Request() : Common::NetworkRequest(static_cast<unsigned short>(MessageType::get_lidar_seg)) {}
};

struct Response : public Common::NetworkResponse {
  Response() : Common::NetworkResponse(static_cast<unsigned short>(MessageType::get_lidar_seg)) {}
  explicit Response(bool _status) : Common::NetworkResponse(MessageType::get_lidar_seg, _status) {}

  double startX;
  double startY;
  double startZ;

  std::vector<LidarSegData> lidarSegData;

  template <class Archive>
  void serialize(Archive& archive) {
    archive(cereal::base_class<Common::NetworkResponse>(this), startX, startY, startZ, lidarSegData);
  }
};
}  // namespace GetLidarSegData

namespace GetLidarConfig {
struct Request : public Common::NetworkRequest {
  Request() : Common::NetworkRequest(static_cast<unsigned short>(MessageType::get_lidar_config)){};
};

struct Response : public Common::NetworkResponse {
  Response() : Common::NetworkResponse(static_cast<unsigned short>(MessageType::get_lidar_config)){};
  explicit Response(bool _status) : Common::NetworkResponse(MessageType::get_lidar_config, _status){};

  LidarConfig config;

  template <class Archive>
  void serialize(Archive& archive) {
    archive(cereal::base_class<Common::NetworkResponse>(this), config);
  }
};
}  // namespace GetLidarConfig

namespace SetLidarConfig {
struct Request : public Common::NetworkRequest {
  Request() : Common::NetworkRequest(static_cast<unsigned short>(MessageType::set_lidar_config)){};

  LidarConfig config;

  template <class Archive>
  void serialize(Archive& archive) {
    archive(cereal::base_class<Common::NetworkRequest>(this), config);
  }
};

struct Response : public Common::NetworkResponse {
  Response() : Common::NetworkResponse(static_cast<unsigned short>(MessageType::set_lidar_config)){};
  explicit Response(bool _status) : Common::NetworkResponse(MessageType::set_lidar_config, _status){};
};
}  // namespace SetLidarConfig

namespace GetCameraConfig {
struct Request : public Common::NetworkRequest {
  Request() : Common::NetworkRequest(static_cast<unsigned short>(MessageType::get_camera_config)){};
};

struct Response : public Common::NetworkResponse {
  Response() : Common::NetworkResponse(static_cast<unsigned short>(MessageType::get_camera_config)){};
  explicit Response(bool _status) : Common::NetworkResponse(MessageType::get_camera_config, _status){};

  CameraConfig config;

  template <class Archive>
  void serialize(Archive& archive) {
    archive(cereal::base_class<Common::NetworkResponse>(this), config);
  }
};
}  // namespace GetCameraConfig

namespace SetCameraConfig {
struct Request : public Common::NetworkRequest {
  Request() : Common::NetworkRequest(static_cast<unsigned short>(MessageType::set_camera_config)){};

  CameraConfig config;

  template <class Archive>
  void serialize(Archive& archive) {
    archive(cereal::base_class<Common::NetworkRequest>(this), config);
  }
};

struct Response : public Common::NetworkResponse {
  Response() : Common::NetworkResponse(static_cast<unsigned short>(MessageType::set_camera_config)){};
  explicit Response(bool _status) : Common::NetworkResponse(MessageType::set_camera_config, _status){};
};
}  // namespace SetCameraConfig

namespace GetMoveLineVisible {
struct Request : public Common::NetworkRequest {
  Request() : Common::NetworkRequest(static_cast<unsigned short>(MessageType::get_move_line_visible)){};
};

struct Response : public Common::NetworkResponse {
  Response() : Common::NetworkResponse(static_cast<unsigned short>(MessageType::get_move_line_visible)){};
  explicit Response(bool _status) : Common::NetworkResponse(MessageType::get_move_line_visible, _status){};

  bool visible;

  template <class Archive>
  void serialize(Archive& archive) {
    archive(cereal::base_class<Common::NetworkResponse>(this), visible);
  }
};
}  // namespace GetMoveLineVisible

namespace SetMoveLineVisible {
struct Request : public Common::NetworkRequest {
  Request() : Common::NetworkRequest(static_cast<unsigned short>(MessageType::set_move_line_visible)){};

  bool visible;

  template <class Archive>
  void serialize(Archive& archive) {
    archive(cereal::base_class<Common::NetworkRequest>(this), visible);
  }
};

struct Response : public Common::NetworkResponse {
  Response() : Common::NetworkResponse(static_cast<unsigned short>(MessageType::set_move_line_visible)){};
  explicit Response(bool _status) : Common::NetworkResponse(MessageType::set_move_line_visible, _status){};
};
}  // namespace SetMoveLineVisible

}  // namespace Drone

namespace GameMode {
enum MessageType : unsigned short {
  get_drones = 0x2,
  spawn_drone = 0x3,
  remove_drone = 0x4,
  get_camera_capture_mode = 0x5,
  set_camera_capture_mode = 0x6,
  get_fps = 0x7
};

namespace GetDrones {
struct Request : public Common::NetworkRequest {
  Request(): Common::NetworkRequest(static_cast<unsigned short>(MessageType::get_drones)) {}
};

struct Response : public Common::NetworkResponse {
  Response() : Common::NetworkResponse(static_cast<unsigned short>(MessageType::get_drones)) {}
  explicit Response(bool _status) : Common::NetworkResponse(MessageType::get_drones, _status) {}

  std::vector<int> ports;

  template <class Archive>
  void serialize(Archive& archive) {
    archive(cereal::base_class<Common::NetworkResponse>(this), ports);
  }
};
}  // namespace GetDrones

namespace SpawnDrone {
struct Request : public Common::NetworkRequest {
  Request(): Common::NetworkRequest(static_cast<unsigned short>(MessageType::spawn_drone)) {}
};

struct Response : public Common::NetworkResponse {
  Response() : Common::NetworkResponse(static_cast<unsigned short>(MessageType::spawn_drone)) {}
  explicit Response(bool _status) : Common::NetworkResponse(MessageType::spawn_drone, _status) {}

  int port;

  template <class Archive>
  void serialize(Archive& archive) {
    archive(cereal::base_class<Common::NetworkResponse>(this), port);
  }
};
}  // namespace SpawnDrone

namespace RemoveDrone {
struct Request : public Common::NetworkRequest {
  Request(): Common::NetworkRequest(MessageType::remove_drone) {}

  int port;

  template <class Archive>
  void serialize(Archive& archive) {
    archive(cereal::base_class<Common::NetworkRequest>(this), port);
  }
};

struct Response : public Common::NetworkResponse {
  Response() : Common::NetworkResponse(static_cast<unsigned short>(MessageType::remove_drone)) {}
  explicit Response(bool _status) : Common::NetworkResponse(MessageType::remove_drone, _status) {}
};
}  // namespace RemoveDrone

enum CameraCaptureModeEnum : unsigned short
{
  CAPTURE_ALL_FRAMES = 0x0,
  CAPTURE_ON_MOVEMENT = 0x1,
  CAPTURE_ON_DEMAND = 0x2,
};

namespace GetCameraCaptureMode {
struct Request : public Common::NetworkRequest {
  Request(): Common::NetworkRequest(MessageType::get_camera_capture_mode){};
};

struct Response : public Common::NetworkResponse {
  Response() : Common::NetworkResponse(static_cast<unsigned short>(MessageType::get_camera_capture_mode)){};
  explicit Response(bool _status) : Common::NetworkResponse(MessageType::get_camera_capture_mode, _status){};

  CameraCaptureModeEnum cameraCaptureMode;

  template <class Archive>
  void serialize(Archive& archive) {
    archive(cereal::base_class<Common::NetworkResponse>(this), cameraCaptureMode);
  }
};
}  // namespace GetCameraCaptureMode

namespace SetCameraCaptureMode {
struct Request : public Common::NetworkRequest {
  Request(): Common::NetworkRequest(MessageType::set_camera_capture_mode){};

  CameraCaptureModeEnum cameraCaptureMode;

  template <class Archive>
  void serialize(Archive& archive) {
    archive(cereal::base_class<Common::NetworkRequest>(this), cameraCaptureMode);
  }
};

struct Response : public Common::NetworkResponse {
  Response() : Common::NetworkResponse(static_cast<unsigned short>(MessageType::set_camera_capture_mode)){};
  explicit Response(bool _status) : Common::NetworkResponse(MessageType::set_camera_capture_mode, _status){};
};
}  // namespace SetCameraCaptureMode

namespace GetFps {
struct Request : public Common::NetworkRequest {
  Request(): Common::NetworkRequest(MessageType::get_fps){};
};

struct Response : public Common::NetworkResponse {
  Response() : Common::NetworkResponse(static_cast<unsigned short>(MessageType::get_fps)){};
  explicit Response(bool _status) : Common::NetworkResponse(MessageType::get_fps, _status){};

  float fps;

  template <class Archive>
  void serialize(Archive& archive) {
    archive(cereal::base_class<Common::NetworkResponse>(this), fps);
  }
};
}  // namespace GetFps
}  // namespace GameMode
}  // namespace Serializable
