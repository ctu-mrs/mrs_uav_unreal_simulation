// Copyright [2022] <Jakub Jirkal>
// This code is licensed under MIT license (see LICENSE for details)

#pragma once

#include "string"

namespace ueds_connector
{
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

struct CameraConfig
{
  CameraConfig() = default;
  CameraConfig(bool showDebugCamera, double angleFov, const Coordinates offset, const Rotation orientation, int Width, int Height, double baseline)
      : showDebugCamera(showDebugCamera), angleFOV(angleFov), offset(offset), orientation(orientation), Width(Width), Height(Height), baseline(baseline) {
  }

  bool showDebugCamera;

  double angleFOV;

  Coordinates offset;

  Rotation orientation;

  int Width;

  int Height;

  double baseline;

  std::string toString() const {
    return "(showDebugCamera: " + std::to_string(showDebugCamera) + ", angleFOV: " + std::to_string(angleFOV) + ", offset: " + offset.toString() +
           ", directionZ: " + orientation.toString() + ", Width: " + std::to_string(Width) + ", Height: " + std::to_string(Height) + ")";
  }
};
}  // namespace ueds_connector
