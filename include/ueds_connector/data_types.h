// Copyright [2022] <Jakub Jirkal>
// This code is licensed under MIT license (see LICENSE for details)

#pragma once

#include <string>
#include <map>

namespace ueds_connector
{
  
struct UavFrameType
{
    static const std::map<std::string, int>& Type2IdMesh() {
        static const std::map<std::string, int> map = {
            {"x500", 0},
            {"t650", 1},
            {"a300", 2},
            {"robofly", 3}
        };
        return map;
    }
};

struct Coordinates
{
  Coordinates() = default;
  Coordinates(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {
  }

  double x;
  double y;
  double z;

  std::string toString() const {
    return "(x: " + std::to_string(x) + ", y: " + std::to_string(y) + ", z: " + std::to_string(z) + ")";
  }
};

struct Rotation
{
  Rotation() = default;
  Rotation(double _pitch, double _yaw, double _roll) : pitch(_pitch), yaw(_yaw), roll(_roll) {
  }

  double pitch;
  double yaw;
  double roll;

  std::string toString() const {
    return "(pitch: " + std::to_string(pitch) + ", yaw: " + std::to_string(yaw) + ", roll: " + std::to_string(roll) + ")";
  }
};

struct LidarData
{
  LidarData() = default;

  double distance;
  double directionX;
  double directionY;
  double directionZ;

  std::string toString() const {
    return "(distance: " + std::to_string(distance) + ", directionX: " + std::to_string(directionX) + ", directionY: " + std::to_string(directionY) +
           ", directionZ: " + std::to_string(directionZ) + ")";
  }
};

struct LidarSegData
{
  LidarSegData() = default;

  double      distance;
  double      directionX;
  double      directionY;
  double      directionZ;
  int         segmentation;
  std::string toString() const {
    return "(distance: " + std::to_string(distance) + ", directionX: " + std::to_string(directionX) + ", directionY: " + std::to_string(directionY) +
           ", directionZ: " + std::to_string(directionZ) + ", segmentation: " + std::to_string(segmentation) + ")";
  }
};

struct LidarIntData
{
  LidarIntData() = default;

  double      distance;
  double      directionX;
  double      directionY;
  double      directionZ;
  int      intensity;
  std::string toString() const {
    return "(distance: " + std::to_string(distance) + ", directionX: " + std::to_string(directionX) + ", directionY: " + std::to_string(directionY) +
           ", directionZ: " + std::to_string(directionZ) + ", intensity: " + std::to_string(intensity) + ")";
  }
};

struct LidarConfig
{
  LidarConfig() = default;
  LidarConfig(bool Enable, bool showBeams, double BeamHorRays, double BeamVertRays, double beamLength, double Frequency, const Coordinates offset,
              const Rotation orientation, double FOVHor, double FOVVert)
      : Enable(Enable),
        showBeams(showBeams),
        beamLength(beamLength),
        BeamHorRays(BeamHorRays),
        BeamVertRays(BeamVertRays),
        Frequency(Frequency),
        offset(offset),
        orientation(orientation),
        FOVHor(FOVHor),
        FOVVert(FOVVert) {
  }

  bool        Enable;
  bool        showBeams;
  double      beamLength;
  double      BeamHorRays;
  double      BeamVertRays;
  double      Frequency;
  Coordinates offset;
  Rotation    orientation;
  double      FOVHor;
  double      FOVVert;

  std::string toString() const {
    return "(showBeams: " + std::to_string(showBeams) + ", beamLength: " + std::to_string(beamLength) + ", offset: " + offset.toString() +
           ", orientation: " + orientation.toString() + ")";
  }
};

enum CameraCaptureModeEnum : unsigned short
{
  CAPTURE_ALL_FRAMES  = 0x0,
  CAPTURE_ON_MOVEMENT = 0x1,
  CAPTURE_ON_DEMAND   = 0x2,
};

struct RgbCameraConfig
{
  RgbCameraConfig() = default;
  RgbCameraConfig(bool show_debug_camera, const Coordinates offset, const Rotation orientation, double fov, int width, int height, bool enable_temporal_aa,
                  bool enable_raytracing, bool enable_hdr)
      : show_debug_camera_(show_debug_camera),
        offset_(offset),
        orientation_(orientation),
        fov_(fov),
        width_(width),
        height_(height),
        enable_temporal_aa_(enable_temporal_aa),
        enable_raytracing_(enable_raytracing),
        enable_hdr_(enable_hdr) {
  }

  bool show_debug_camera_;

  Coordinates offset_;

  Rotation orientation_;

  double fov_;

  int width_;
  int height_;

  bool enable_temporal_aa_;
  bool enable_raytracing_;
  bool enable_hdr_;
};

struct StereoCameraConfig
{
  StereoCameraConfig() = default;
  StereoCameraConfig(bool show_debug_camera, const Coordinates offset, const Rotation orientation, double fov, int width, int height, double baseline,
                     bool enable_temporal_aa, bool enable_raytracing, bool enable_hdr)
      : show_debug_camera_(show_debug_camera),
        offset_(offset),
        orientation_(orientation),
        fov_(fov),
        width_(width),
        height_(height),
        baseline_(baseline),
        enable_temporal_aa_(enable_temporal_aa),
        enable_raytracing_(enable_raytracing),
        enable_hdr_(enable_hdr) {
  }

  bool show_debug_camera_;

  Coordinates offset_;

  Rotation orientation_;

  double fov_;

  int width_;
  int height_;

  double baseline_;

  bool enable_temporal_aa_;
  bool enable_raytracing_;
  bool enable_hdr_;
};

}  // namespace ueds_connector
