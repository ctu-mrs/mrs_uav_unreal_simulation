/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <mrs_multirotor_simulator/uav_system_ros.h>

#include <rosgraph_msgs/msg/clock.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/timer_handler.h>
#include <mrs_lib/scope_timer.h>
#include <mrs_lib/transform_broadcaster.h>
#include <mrs_lib/attitude_converter.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <Eigen/Dense>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <flight_forge_connector/flight_forge_connector.h>
#include <flight_forge_connector/game_mode_controller.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <random>
#include <vector>
#include <chrono>

#include <mrs_msgs/srv/float64_srv.hpp>
#include <mrs_uav_flightforge_simulation/srv/set_orientation.hpp>

//}

/* defines //{ */

using namespace std::chrono_literals;

using PCLPoint               = pcl::PointXYZ;
using PCLPointCloud          = pcl::PointCloud<PCLPoint>;
using PCLPointCloudColor     = pcl::PointCloud<pcl::PointXYZRGB>;
using PCLPointCloudIntensity = pcl::PointCloud<pcl::PointXYZI>;

//}

namespace mrs_uav_flightforge_simulation
{

/* class FlightforgeSimulator //{ */

class FlightforgeSimulator : public rclcpp::Node {

public:
  FlightforgeSimulator(rclcpp::NodeOptions options);

private:
  // | --------------------- ROS 2 infrastructure --------------------- |
  rclcpp::CallbackGroup::SharedPtr cbgrp_main_;
  rclcpp::CallbackGroup::SharedPtr cbgrp_sensors_;
  rclcpp::CallbackGroup::SharedPtr cbgrp_status_;

  rclcpp::TimerBase::SharedPtr timer_init_;
  void                         timerInit();

  std::atomic<bool> is_initialized_ = false;

  std::shared_ptr<mrs_lib::ScopeTimerLogger>       scope_timer_logger_;
  std::shared_ptr<image_transport::ImageTransport> it_;

  // | ------------------------- params ------------------------- |
  double      _simulation_rate_;
  double      _clock_rate_;
  std::string _world_frame_name_;
  bool        _collisions_;

  // | --------------------- dynamic params --------------------- |
  double realtime_factor_;
  bool   dynamic_rtf_;
  bool   paused_;

  bool   rangefinder_enabled_;
  double rangefinder_rate_;

  bool   lidar_enabled_;
  double lidar_rate_;
  bool   lidar_seg_enabled_;
  double lidar_seg_rate_;
  bool   lidar_int_enabled_;
  double lidar_int_rate_;
  bool   lidar_noise_enabled_;
  double lidar_std_at_1m_;
  double lidar_std_slope_;

  bool   rgb_enabled_;
  double rgb_rate_;
  bool   rgb_segmented_enabled_;
  double rgb_segmented_rate_;
  bool   rgb_depth_enabled_;
  double rgb_depth_rate_;

  bool   stereo_enabled_;
  double stereo_rate_;

  double lidar_int_value_grass_;
  double lidar_int_value_road_;
  double lidar_int_value_tree_;
  double lidar_int_value_building_;
  double lidar_int_value_fence_;
  double lidar_int_value_dirt_road_;
  double lidar_int_value_other_;
  bool   lidar_int_noise_enabled_;
  double lidar_int_std_at_1m_;
  double lidar_int_std_slope_;

  // | -------------------- parameter callbacks ------------------- |
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters);
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // | -------------------------- time -------------------------- |
  rclcpp::Time sim_time_;
  std::mutex   mutex_sim_time_;
  double       _clock_min_dt_;

  // | ------------------------- timers ------------------------- |
  rclcpp::TimerBase::SharedPtr timer_dynamics_;
  void                         timerDynamics();

  rclcpp::TimerBase::SharedPtr timer_status_;
  void                         timerStatus();

  rclcpp::TimerBase::SharedPtr timer_time_sync_;
  void                         timerTimeSync();

  rclcpp::TimerBase::SharedPtr timer_unreal_sync_;
  void                         timerUnrealSync();

  rclcpp::TimerBase::SharedPtr timer_rangefinder_;
  void                         timerRangefinder();

  rclcpp::TimerBase::SharedPtr timer_lidar_;
  void                         timerLidar();

  rclcpp::TimerBase::SharedPtr timer_seg_lidar_;
  void                         timerSegLidar();

  rclcpp::TimerBase::SharedPtr timer_int_lidar_;
  void                         timerIntLidar();

  rclcpp::TimerBase::SharedPtr timer_rgb_;
  void                         timerRgb();

  rclcpp::TimerBase::SharedPtr timer_rgb_segmented_;
  void                         timerRgbSegmented();

  rclcpp::TimerBase::SharedPtr timer_depth_;
  void                         timerDepth();

  rclcpp::TimerBase::SharedPtr timer_stereo_;
  void                         timerStereo();

  // | --------------------- service servers -------------------- |
  rclcpp::Service<mrs_msgs::srv::Float64Srv>::SharedPtr service_server_realtime_factor_;
  void callbackSetRealtimeFactor(const std::shared_ptr<mrs_msgs::srv::Float64Srv::Request> request, std::shared_ptr<mrs_msgs::srv::Float64Srv::Response> response);

  std::vector<rclcpp::Client<mrs_msgs::srv::Float64Srv>::SharedPtr> set_ground_z_clients_;

  std::vector<rclcpp::Service<tutorial_interfaces::srv::SetOrientation>::SharedPtr> service_gimbal_control_servers_;
  void                                                                            callbackSetGimbalOrientation(
      const std::shared_ptr<tutorial_interfaces::srv::SetOrientation::Request> request, std::shared_ptr<tutorial_interfaces::srv::SetOrientation::Response> response,
      int uav_index);

  // | --------------------------- tfs -------------------------- |
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster>       dynamic_broadcaster_;

  // | ----------------------- camera info ---------------------- |
  sensor_msgs::msg::CameraInfo rgb_camera_info_;
  sensor_msgs::msg::CameraInfo stereo_camera_info_;

  geometry_msgs::msg::TransformStamped rgb_camera_tf_;
  geometry_msgs::msg::TransformStamped stereo_camera_tf_;

  // | --------- store current camera orientation -------- |
  std::vector<Eigen::Quaterniond> rgb_camera_orientations_;

  // | --------------------------- rtf -------------------------- |
  double       actual_rtf_ = 1.0;
  rclcpp::Time last_sim_time_status_;

  // | ------------------------- sensors ------------------------ |
  double lidar_horizontal_fov_left_;
  double lidar_horizontal_fov_right_;
  double lidar_vertical_fov_up_;
  double lidar_vertical_fov_down_;
  int    lidar_horizontal_rays_;
  int    lidar_vertical_rays_;
  double lidar_offset_x_;
  double lidar_offset_y_;
  double lidar_offset_z_;
  double lidar_rotation_pitch_;
  double lidar_rotation_roll_;
  double lidar_rotation_yaw_;
  double lidar_beam_length_;
  bool   lidar_show_beams_;
  bool   lidar_livox_;

  int    rgb_width_;
  int    rgb_height_;
  double rgb_fov_;
  double rgb_offset_x_;
  double rgb_offset_y_;
  double rgb_offset_z_;
  double rgb_rotation_pitch_;
  double rgb_rotation_yaw_;
  double rgb_rotation_roll_;
  bool   rgb_enable_hdr_;
  bool   rgb_enable_temporal_aa_;
  bool   rgb_enable_raytracing_;
  bool   rgb_enable_motion_blur_;
  double rgb_motion_blur_amount_;
  double rgb_motion_blur_distortion_;

  double stereo_baseline_;
  int    stereo_width_;
  int    stereo_height_;
  double stereo_fov_;
  double stereo_offset_x_;
  double stereo_offset_y_;
  double stereo_offset_z_;
  double stereo_rotation_pitch_;
  double stereo_rotation_yaw_;
  double stereo_rotation_roll_;
  bool   stereo_enable_hdr_;
  bool   stereo_enable_temporal_aa_;
  bool   stereo_enable_raytracing_;

  // segmentation decode array//{
  uint8_t seg_rgb_[256][3] = {
      255, 255, 255, 153, 108, 6,   112, 105, 191, 89,  121, 72,  190, 225, 64,  206, 190, 59,  81,  13,  36,  115, 176, 195, 161, 171, 27,  135, 169,
      180, 29,  26,  199, 102, 16,  239, 242, 107, 146, 156, 198, 23,  49,  89,  160, 68,  218, 116, 11,  236, 9,   196, 30,  8,   121, 67,  28,  0,   53,
      65,  146, 52,  70,  226, 149, 143, 151, 126, 171, 194, 39,  7,   205, 120, 161, 212, 51,  60,  211, 80,  208, 189, 135, 188, 54,  72,  205, 103, 252,
      157, 124, 21,  123, 19,  132, 69,  195, 237, 132, 94,  253, 175, 182, 251, 87,  90,  162, 242, 199, 29,  1,   254, 12,  229, 35,  196, 244, 220, 163,
      49,  86,  254, 214, 152, 3,   129, 92,  31,  106, 207, 229, 90,  125, 75,  48,  98,  55,  74,  126, 129, 238, 222, 153, 109, 85,  152, 34,  173, 69,
      31,  37,  128, 125, 58,  19,  33,  134, 57,  119, 218, 124, 115, 120, 0,   200, 225, 131, 92,  246, 90,  16,  51,  155, 241, 202, 97,  155, 184, 145,
      182, 96,  232, 44,  133, 244, 133, 180, 191, 29,  1,   222, 192, 99,  242, 104, 91,  168, 219, 65,  54,  217, 148, 66,  130, 203, 102, 204, 216, 78,
      75,  234, 20,  250, 109, 206, 24,  164, 194, 17,  157, 23,  236, 158, 114, 88,  245, 22,  110, 67,  17,  35,  181, 213, 93,  170, 179, 42,  52,  187,
      148, 247, 200, 111, 25,  62,  174, 100, 25,  240, 191, 195, 144, 252, 36,  67,  241, 77,  149, 237, 33,  141, 119, 230, 85,  28,  34,  108, 78,  98,
      254, 114, 161, 30,  75,  50,  243, 66,  226, 253, 46,  104, 76,  8,   234, 216, 15,  241, 102, 93,  14,  71,  192, 255, 193, 253, 41,  164, 24,  175,
      120, 185, 243, 231, 169, 233, 97,  243, 215, 145, 72,  137, 21,  160, 113, 101, 214, 92,  13,  167, 140, 147, 101, 109, 181, 53,  118, 126, 3,   177,
      32,  40,  63,  99,  186, 139, 153, 88,  207, 100, 71,  146, 227, 236, 38,  187, 215, 4,   215, 18,  211, 66,  113, 49,  134, 47,  42,  63,  219, 103,
      127, 57,  240, 137, 227, 133, 211, 145, 71,  201, 217, 173, 183, 250, 40,  113, 208, 125, 68,  224, 186, 249, 69,  148, 46,  239, 85,  20,  108, 116,
      224, 56,  214, 26,  179, 147, 43,  48,  188, 172, 221, 83,  47,  155, 166, 218, 62,  217, 189, 198, 180, 122, 201, 144, 169, 132, 2,   14,  128, 189,
      114, 163, 227, 112, 45,  157, 177, 64,  86,  142, 118, 193, 163, 14,  32,  79,  200, 45,  170, 74,  81,  2,   59,  37,  212, 73,  35,  225, 95,  224,
      39,  84,  170, 220, 159, 58,  173, 17,  91,  237, 31,  95,  84,  34,  201, 248, 63,  73,  209, 129, 235, 107, 231, 115, 40,  36,  74,  95,  238, 228,
      154, 61,  212, 54,  13,  94,  165, 141, 174, 0,   140, 167, 255, 117, 93,  91,  183, 10,  186, 165, 28,  61,  144, 238, 194, 12,  158, 41,  76,  110,
      234, 150, 9,   121, 142, 1,   246, 230, 136, 198, 5,   60,  233, 232, 250, 80,  143, 112, 56,  187, 70,  156, 2,   185, 62,  138, 223, 226, 122, 183,
      222, 166, 245, 3,   175, 6,   140, 240, 59,  210, 248, 44,  10,  83,  82,  52,  223, 248, 167, 87,  15,  150, 111, 178, 117, 197, 84,  22,  235, 208,
      124, 9,   76,  45,  176, 24,  50,  154, 159, 251, 149, 111, 207, 168, 231, 15,  209, 247, 202, 80,  205, 152, 178, 221, 213, 27,  8,   38,  244, 117,
      51,  107, 68,  190, 23,  199, 139, 171, 88,  168, 136, 202, 58,  6,   46,  86,  105, 127, 176, 174, 249, 197, 172, 172, 138, 228, 142, 81,  7,   204,
      185, 22,  61,  247, 233, 100, 78,  127, 65,  105, 33,  87,  158, 139, 156, 252, 42,  7,   136, 20,  99,  179, 79,  150, 223, 131, 182, 184, 110, 123,
      37,  60,  138, 96,  210, 96,  94,  123, 48,  18,  137, 197, 162, 188, 18,  5,   39,  219, 151, 204, 143, 135, 249, 79,  73,  77,  64,  178, 41,  246,
      77,  16,  154, 4,   116, 134, 19,  4,   122, 235, 177, 106, 230, 21,  119, 12,  104, 5,   98,  50,  130, 53,  30,  192, 25,  26,  165, 166, 10,  160,
      82,  106, 43,  131, 44,  216, 103, 255, 101, 221, 32,  151, 196, 213, 220, 89,  70,  209, 228, 97,  184, 83,  82,  239, 232, 251, 164, 128, 193, 11,
      245, 38,  27,  159, 229, 141, 203, 130, 56,  55,  147, 210, 11,  162, 203, 118, 0,   0,   0};
  /*//}*/

  // | ----------------------- publishers ----------------------- |
  mrs_lib::PublisherHandler<rosgraph_msgs::msg::Clock>     ph_clock_;
  mrs_lib::PublisherHandler<geometry_msgs::msg::PoseArray> ph_poses_;

  std::vector<mrs_lib::PublisherHandler<sensor_msgs::msg::Range>> ph_rangefinders_;

  std::vector<mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2>> ph_lidars_;
  std::vector<mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2>> ph_seg_lidars_;
  std::vector<mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2>> ph_int_lidars_;

  std::vector<image_transport::Publisher> imp_rgb_;
  std::vector<image_transport::Publisher> imp_stereo_left_;
  std::vector<image_transport::Publisher> imp_stereo_right_;
  std::vector<image_transport::Publisher> imp_rgbd_segmented_;
  std::vector<image_transport::Publisher> imp_depth_;

  std::vector<mrs_lib::PublisherHandler<sensor_msgs::msg::CameraInfo>> ph_rgb_camera_info_;
  std::vector<mrs_lib::PublisherHandler<sensor_msgs::msg::CameraInfo>> ph_rgb_seg_camera_info_;
  std::vector<mrs_lib::PublisherHandler<sensor_msgs::msg::CameraInfo>> ph_stereo_left_camera_info_;
  std::vector<mrs_lib::PublisherHandler<sensor_msgs::msg::CameraInfo>> ph_stereo_right_camera_info_;

  // | ------------------------- system ------------------------- |
  std::vector<std::shared_ptr<mrs_multirotor_simulator::UavSystemRos>> uavs_;

  // | -------------------------- time -------------------------- |
  rclcpp::Time last_published_time_;

  // | ------------------------- methods ------------------------ |
  void publishPoses(void);

  // | ------------------------- Unreal ------------------------- |
  std::unique_ptr<ueds_connector::GameModeController>                 ueds_game_controller_;
  std::vector<std::shared_ptr<ueds_connector::UedsConnector>> ueds_connectors_;
  std::mutex                                                          mutex_ueds_;

  ueds_connector::Coordinates              ueds_world_origin_;
  std::vector<ueds_connector::Coordinates> ueds_world_origins_;

  ueds_connector::Coordinates position2ue(const Eigen::Vector3d& pos, const ueds_connector::Coordinates& ueds_world_origin);

  void updateUnrealPoses(const bool teleport_without_collision);
  void checkForCrash(void);
  void fabricateCamInfo(void);
  void publishStaticTfs(void);
  void publishCameraTf(const int& uav_index);

  // how much to add to unreal time to get to our wall time
  double     wall_time_offset_             = 0;
  double     wall_time_offset_drift_slope_ = 0;
  rclcpp::Time last_sync_time_;
  std::mutex   mutex_wall_time_offset_;

  double      ueds_fps_ = 0;
  std::string ueds_world_level_name_enum_;
  std::string ueds_graphics_settings_enum_;
  int         ueds_forest_density_     = 5;
  int         ueds_forest_hilly_level_ = 3;
  std::string weather_type_;
  ueds_connector::Daytime daytime_;
  bool                    uavs_mutual_visibility_ = true;

  std::vector<double> last_rgb_ue_stamp_;
  std::vector<double> last_rgb_seg_ue_stamp_;
  std::vector<double> last_stereo_ue_stamp_;

  double uedsToWallTime(const double ueds_time);

  std::default_random_engine rng;
};

//}

/* FlightforgeSimulator() //{ */
FlightforgeSimulator::FlightforgeSimulator(rclcpp::NodeOptions options) : Node("mrs_uav_flightforge_simulation", options) {
  timer_init_ = create_wall_timer(100ms, std::bind(&FlightforgeSimulator::timerInit, this));
}
/*//}*/

/* timerInit() //{ */
void FlightforgeSimulator::timerInit() {

  if (is_initialized_) {
    return;
  }

  RCLCPP_INFO(get_logger(), "[FlightforgeSimulator]: initializing");

  cbgrp_main_    = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbgrp_sensors_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbgrp_status_  = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  mrs_lib::ParamLoader param_loader(shared_from_this(), "FlightforgeSimulator");

  std::string custom_config_path;
  param_loader.loadParam("custom_config", custom_config_path, std::string(""));
  if (custom_config_path != "") {
    param_loader.addYamlFile(custom_config_path);
  }
  param_loader.addYamlFileFromParam("config");
  param_loader.addYamlFileFromParam("config_uavs");

  // | ------------------- load static parameters ------------------- |
  param_loader.loadParam("simulation_rate", _simulation_rate_);
  param_loader.loadParam("clock_rate", _clock_rate_);
  param_loader.loadParam("frames/world/name", _world_frame_name_);
  param_loader.loadParam("collisions", _collisions_);

  // | ----------------- load and set dynamic params ---------------- |
  param_loader.loadParam("realtime_factor", realtime_factor_);
  param_loader.loadParam("dynamic_rtf", dynamic_rtf_);
  param_loader.loadParam("paused", paused_);
  param_loader.loadParam("sensors/rangefinder/enabled", rangefinder_enabled_);
  param_loader.loadParam("sensors/rangefinder/rate", rangefinder_rate_);
  param_loader.loadParam("sensors/lidar/enabled", lidar_enabled_);
  param_loader.loadParam("sensors/lidar/rate", lidar_rate_);
  param_loader.loadParam("sensors/lidar/lidar_segmented/enabled", lidar_seg_enabled_);
  param_loader.loadParam("sensors/lidar/lidar_segmented/rate", lidar_seg_rate_);
  param_loader.loadParam("sensors/lidar/lidar_intensity/enabled", lidar_int_enabled_);
  param_loader.loadParam("sensors/lidar/lidar_intensity/rate", lidar_int_rate_);
  param_loader.loadParam("sensors/lidar/noise/enabled", lidar_noise_enabled_);
  param_loader.loadParam("sensors/lidar/noise/std_at_1m", lidar_std_at_1m_);
  param_loader.loadParam("sensors/lidar/noise/std_slope", lidar_std_slope_);
  param_loader.loadParam("sensors/rgb/enabled", rgb_enabled_);
  param_loader.loadParam("sensors/rgb/rate", rgb_rate_);
  param_loader.loadParam("sensors/rgb/rgb_segmented/enabled", rgb_segmented_enabled_);
  param_loader.loadParam("sensors/rgb/rgb_segmented/rate", rgb_segmented_rate_);
  param_loader.loadParam("sensors/rgb/depth/enabled", rgb_depth_enabled_);
  param_loader.loadParam("sensors/rgb/depth/rate", rgb_depth_rate_);
  param_loader.loadParam("sensors/stereo/enabled", stereo_enabled_);
  param_loader.loadParam("sensors/stereo/rate", stereo_rate_);
  param_loader.loadParam("sensors/lidar/lidar_intensity/values/grass", lidar_int_value_grass_);
  param_loader.loadParam("sensors/lidar/lidar_intensity/values/road", lidar_int_value_road_);
  param_loader.loadParam("sensors/lidar/lidar_intensity/values/tree", lidar_int_value_tree_);
  param_loader.loadParam("sensors/lidar/lidar_intensity/values/building", lidar_int_value_building_);
  param_loader.loadParam("sensors/lidar/lidar_intensity/values/fence", lidar_int_value_fence_);
  param_loader.loadParam("sensors/lidar/lidar_intensity/values/dirt_road", lidar_int_value_dirt_road_);
  param_loader.loadParam("sensors/lidar/lidar_intensity/values/other", lidar_int_value_other_);
  param_loader.loadParam("sensors/lidar/lidar_intensity/noise/enabled", lidar_int_noise_enabled_);
  param_loader.loadParam("sensors/lidar/lidar_intensity/noise/std_at_1m", lidar_int_std_at_1m_);
  param_loader.loadParam("sensors/lidar/lidar_intensity/noise/std_slope", lidar_int_std_slope_);

  this->declare_parameter("realtime_factor", realtime_factor_);
  this->declare_parameter("dynamic_rtf", dynamic_rtf_);
  this->declare_parameter("paused", paused_);

  param_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&FlightforgeSimulator::parametersCallback, this, std::placeholders::_1));

  // | --------------------- sensor parameters -------------------- |
  param_loader.loadParam("sensors/lidar/horizontal_fov_left", lidar_horizontal_fov_left_);
  param_loader.loadParam("sensors/lidar/horizontal_fov_right", lidar_horizontal_fov_right_);
  param_loader.loadParam("sensors/lidar/vertical_fov_up", lidar_vertical_fov_up_);
  param_loader.loadParam("sensors/lidar/vertical_fov_down", lidar_vertical_fov_down_);
  param_loader.loadParam("sensors/lidar/horizontal_rays", lidar_horizontal_rays_);
  param_loader.loadParam("sensors/lidar/vertical_rays", lidar_vertical_rays_);
  param_loader.loadParam("sensors/lidar/offset_x", lidar_offset_x_);
  param_loader.loadParam("sensors/lidar/offset_y", lidar_offset_y_);
  param_loader.loadParam("sensors/lidar/offset_z", lidar_offset_z_);
  param_loader.loadParam("sensors/lidar/rotation_pitch", lidar_rotation_pitch_);
  param_loader.loadParam("sensors/lidar/rotation_roll", lidar_rotation_roll_);
  param_loader.loadParam("sensors/lidar/rotation_yaw", lidar_rotation_yaw_);
  param_loader.loadParam("sensors/lidar/beam_length", lidar_beam_length_);
  param_loader.loadParam("sensors/lidar/show_beams", lidar_show_beams_);
  param_loader.loadParam("sensors/lidar/livox", lidar_livox_);

  param_loader.loadParam("sensors/rgb/width", rgb_width_);
  param_loader.loadParam("sensors/rgb/height", rgb_height_);
  param_loader.loadParam("sensors/rgb/fov", rgb_fov_);
  param_loader.loadParam("sensors/rgb/offset_x", rgb_offset_x_);
  param_loader.loadParam("sensors/rgb/offset_y", rgb_offset_y_);
  param_loader.loadParam("sensors/rgb/offset_z", rgb_offset_z_);
  param_loader.loadParam("sensors/rgb/rotation_pitch", rgb_rotation_pitch_);
  param_loader.loadParam("sensors/rgb/rotation_roll", rgb_rotation_roll_);
  param_loader.loadParam("sensors/rgb/rotation_yaw", rgb_rotation_yaw_);
  param_loader.loadParam("sensors/rgb/enable_hdr", rgb_enable_hdr_);
  param_loader.loadParam("sensors/rgb/enable_temporal_aa", rgb_enable_temporal_aa_);
  param_loader.loadParam("sensors/rgb/enable_raytracing", rgb_enable_raytracing_);
  param_loader.loadParam("sensors/rgb/enable_motion_blur", rgb_enable_motion_blur_);
  param_loader.loadParam("sensors/rgb/motion_blur_amount", rgb_motion_blur_amount_);
  param_loader.loadParam("sensors/rgb/motion_blur_distortion", rgb_motion_blur_distortion_);

  param_loader.loadParam("sensors/stereo/width", stereo_width_);
  param_loader.loadParam("sensors/stereo/height", stereo_height_);
  param_loader.loadParam("sensors/stereo/fov", stereo_fov_);
  param_loader.loadParam("sensors/stereo/baseline", stereo_baseline_);
  param_loader.loadParam("sensors/stereo/offset_x", stereo_offset_x_);
  param_loader.loadParam("sensors/stereo/offset_y", stereo_offset_y_);
  param_loader.loadParam("sensors/stereo/offset_z", stereo_offset_z_);
  param_loader.loadParam("sensors/stereo/rotation_pitch", stereo_rotation_pitch_);
  param_loader.loadParam("sensors/stereo/rotation_roll", stereo_rotation_roll_);
  param_loader.loadParam("sensors/stereo/rotation_yaw", stereo_rotation_yaw_);
  param_loader.loadParam("sensors/stereo/enable_hdr", stereo_enable_hdr_);
  param_loader.loadParam("sensors/stereo/enable_temporal_aa", stereo_enable_temporal_aa_);
  param_loader.loadParam("sensors/stereo/enable_raytracing", stereo_enable_raytracing_);

  // | -------------------- unreal parameters ------------------- |
  param_loader.loadParam("graphics_settings", ueds_graphics_settings_enum_);
  param_loader.loadParam("world_name", ueds_world_level_name_enum_);
  param_loader.loadParam("ueds_forest_density", ueds_forest_density_);
  param_loader.loadParam("ueds_forest_hilly_level", ueds_forest_hilly_level_);
  param_loader.loadParam("weather_type", weather_type_);
  param_loader.loadParam("daytime/hour", daytime_.hour);
  param_loader.loadParam("daytime/minute", daytime_.minute);
  param_loader.loadParam("uavs_mutual_visibility", uavs_mutual_visibility_);

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(get_logger(), "[FlightforgeSimulator]: could not load all parameters!");
    rclcpp::shutdown();
    return;
  }

  // | --------------------- initialize time -------------------- |
  sim_time_             = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  last_published_time_  = sim_time_;
  last_sim_time_status_ = sim_time_;
  _clock_min_dt_        = 1.0 / _clock_rate_;

  // | ------------------- initialize libraries ------------------- |
  it_                  = std::make_shared<image_transport::ImageTransport>(shared_from_this());
  static_broadcaster_  = std::make_shared<tf2_ros::StaticTransformBroadcaster>(shared_from_this());
  dynamic_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());

  // | ------------------------ uavs ------------------------ |
  std::vector<std::string> uav_names;
  param_loader.loadParam("uav_names", uav_names);

  for (auto& uav_name : uav_names) {
    RCLCPP_INFO(get_logger(), "[FlightforgeSimulator]: initializing '%s'", uav_name.c_str());
    mrs_multirotor_simulator::UavSystemRos_CommonHandlers_t common_handlers;
    common_handlers.node                  = shared_from_this();
    common_handlers.uav_name              = uav_name;
    common_handlers.transform_broadcaster = dynamic_broadcaster_;
    uavs_.push_back(std::make_shared<mrs_multirotor_simulator::UavSystemRos>(common_handlers));
  }

  // | ----------- initialize the Unreal Sim connector ---------- |
  ueds_game_controller_ = std::make_unique<ueds_connector::GameModeController>(LOCALHOST, 8551);
  while (rclcpp::ok()) {
    if (ueds_game_controller_->Connect() == 1) {
      break;
    }
    RCLCPP_ERROR(get_logger(), "[FlightforgeSimulator]: Error connecting to Unreal's game mode controller");
    std::this_thread::sleep_for(1s);
  }

  // | ------------------ check the API version ----------------- |
  auto [res, api_version]                     = ueds_game_controller_->GetApiVersion();
  auto [api_version_major, api_version_minor] = api_version;
  if (!res || api_version_major != API_VERSION_MAJOR || api_version_minor != API_VERSION_MINOR) {
    RCLCPP_ERROR(get_logger(), "[FlightforgeSimulator]: the API versions don't match! (ROS side '%d.%d' != Unreal side '%d.%d')", API_VERSION_MAJOR,
                 API_VERSION_MINOR, api_version_major, api_version_minor);
    rclcpp::shutdown();
    return;
  }

  // | --------------------- Set graphical settings and choose World Level --------------------- |
  ueds_game_controller_->SwitchWorldLevel(ueds_connector::WorldName::Name2Id().at(ueds_world_level_name_enum_));
  ueds_game_controller_->Disconnect();
  std::this_thread::sleep_for(5s);
  while (rclcpp::ok()) {
    if (ueds_game_controller_->Connect() == 1) {
      break;
    }
    RCLCPP_ERROR(get_logger(), "[FlightforgeSimulator]: Error reconnecting to Unreal's game mode controller");
    std::this_thread::sleep_for(1s);
  }
  ueds_game_controller_->SetGraphicsSettings(ueds_connector::GraphicsSettings::Name2Id().at(ueds_graphics_settings_enum_));
  ueds_game_controller_->SetMutualDroneVisibility(uavs_mutual_visibility_);
  ueds_game_controller_->SetWeather(ueds_connector::WeatherType::Type2Id().at(weather_type_));
  ueds_game_controller_->SetDatetime(daytime_.hour, daytime_.minute);
  ueds_game_controller_->SetForestDensity(ueds_forest_density_);
  ueds_game_controller_->SetForestHillyLevel(ueds_forest_hilly_level_);
  std::this_thread::sleep_for(1s);

  // | --------------------- Spawn the UAVs --------------------- |
  const auto [result, world_origin] = ueds_game_controller_->GetWorldOrigin();
  if (!result) {
    RCLCPP_ERROR(get_logger(), "[FlightforgeSimulator]: GameError: getting world origin");
    rclcpp::shutdown();
    return;
  }
  ueds_world_origin_ = world_origin;

  for (size_t i = 0; i < uav_names.size(); i++) {
    const std::string&          uav_name  = uav_names[i];
    auto                        uav_state = uavs_[i]->getState();
    ueds_connector::Coordinates pos       = position2ue(uav_state.x, ueds_world_origin_);
    RCLCPP_INFO(get_logger(), "[FlightforgeSimulator]: %s spawning at [%.2lf, %.2lf, %.2lf] ...", uav_name.c_str(), uav_state.x.x(), uav_state.x.y(),
                uav_state.x.z());

    std::string uav_frame;
    param_loader.loadParam(uav_names[i] + "/frame", uav_frame);
    int uav_frame_id = ueds_connector::UavFrameType::Type2IdMesh().at(uav_frame);

    auto [resSpawn, port] = ueds_game_controller_->SpawnDroneAtLocation(pos, uav_frame_id);
    if (!resSpawn) {
      RCLCPP_ERROR(get_logger(), "[FlightforgeSimulator]: failed to spawn %s", uav_names[i].c_str());
      rclcpp::shutdown();
      return;
    }

    auto ueds_connector = std::make_shared<ueds_connector::UedsConnector>(LOCALHOST, port);
    ueds_connectors_.push_back(ueds_connector);
    if (ueds_connector->Connect() != 1) {
      RCLCPP_ERROR(get_logger(), "[FlightforgeSimulator]: %s - Error connecting to drone controller", uav_name.c_str());
      rclcpp::shutdown();
      return;
    }

    // | ------------------ initialize publishers ----------------- |
    ph_rangefinders_.push_back(mrs_lib::PublisherHandler<sensor_msgs::msg::Range>(shared_from_this(), "/" + uav_name + "/rangefinder"));
    ph_lidars_.push_back(mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2>(shared_from_this(), "/" + uav_name + "/lidar/points"));
    ph_seg_lidars_.push_back(mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2>(shared_from_this(), "/" + uav_name + "/lidar_segmented/points"));
    ph_int_lidars_.push_back(mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2>(shared_from_this(), "/" + uav_name + "/lidar_intensity/points"));
    imp_rgb_.push_back(it_->advertise("/" + uav_name + "/rgb/image_raw", 1));
    imp_stereo_left_.push_back(it_->advertise("/" + uav_name + "/stereo/left/image_raw", 1));
    imp_stereo_right_.push_back(it_->advertise("/" + uav_name + "/stereo/right/image_raw", 1));
    imp_rgbd_segmented_.push_back(it_->advertise("/" + uav_name + "/rgb_segmented/image_raw", 1));
    imp_depth_.push_back(it_->advertise("/" + uav_name + "/depth/image_raw", 1));
    ph_rgb_camera_info_.push_back(mrs_lib::PublisherHandler<sensor_msgs::msg::CameraInfo>(shared_from_this(), "/" + uav_name + "/rgb/camera_info"));
    ph_rgb_seg_camera_info_.push_back(mrs_lib::PublisherHandler<sensor_msgs::msg::CameraInfo>(shared_from_this(), "/" + uav_name + "/rgb_segmented/camera_info"));
    ph_stereo_left_camera_info_.push_back(
        mrs_lib::PublisherHandler<sensor_msgs::msg::CameraInfo>(shared_from_this(), "/" + uav_name + "/stereo/left/camera_info"));
    ph_stereo_right_camera_info_.push_back(
        mrs_lib::PublisherHandler<sensor_msgs::msg::CameraInfo>(shared_from_this(), "/" + uav_name + "/stereo/right/camera_info"));

    // | ------------------ set sensor configs ----------------- |
    {
      ueds_connector::RgbCameraConfig cameraConfig{};
      cameraConfig.width_                  = rgb_width_;
      cameraConfig.height_                 = rgb_height_;
      cameraConfig.fov_                    = rgb_fov_;
      cameraConfig.offset_                 = ueds_connector::Coordinates(rgb_offset_x_ * 100.0, rgb_offset_y_ * 100.0, rgb_offset_z_ * 100.0);
      cameraConfig.orientation_            = ueds_connector::Rotation(-rgb_rotation_pitch_, rgb_rotation_yaw_, rgb_rotation_roll_);
      cameraConfig.enable_raytracing_      = rgb_enable_raytracing_;
      cameraConfig.enable_hdr_             = rgb_enable_hdr_;
      cameraConfig.enable_temporal_aa_     = rgb_enable_temporal_aa_;
      cameraConfig.enable_motion_blur_     = rgb_enable_motion_blur_;
      cameraConfig.motion_blur_amount_     = rgb_motion_blur_amount_;
      cameraConfig.motion_blur_distortion_ = rgb_motion_blur_distortion_;
      ueds_connectors_[i]->SetRgbCameraConfig(cameraConfig);
      last_rgb_ue_stamp_.push_back(0.0);
      last_rgb_seg_ue_stamp_.push_back(0.0);
    }
    {
      ueds_connector::StereoCameraConfig cameraConfig{};
      cameraConfig.width_              = stereo_width_;
      cameraConfig.height_             = stereo_height_;
      cameraConfig.fov_                = stereo_fov_;
      cameraConfig.baseline_           = stereo_baseline_;
      cameraConfig.offset_             = ueds_connector::Coordinates(stereo_offset_x_ * 100.0, stereo_offset_y_ * 100.0, stereo_offset_z_ * 100.0);
      cameraConfig.orientation_        = ueds_connector::Rotation(-stereo_rotation_pitch_, stereo_rotation_yaw_, stereo_rotation_roll_);
      cameraConfig.enable_raytracing_  = stereo_enable_raytracing_;
      cameraConfig.enable_hdr_         = stereo_enable_hdr_;
      cameraConfig.enable_temporal_aa_ = stereo_enable_temporal_aa_;
      ueds_connectors_[i]->SetStereoCameraConfig(cameraConfig);
      last_stereo_ue_stamp_.push_back(0.0);
    }
    {
      ueds_connector::LidarConfig lidarConfig{};
      lidarConfig.BeamHorRays  = lidar_horizontal_rays_;
      lidarConfig.BeamVertRays = lidar_vertical_rays_;
      lidarConfig.FOVHorLeft   = lidar_horizontal_fov_left_;
      lidarConfig.FOVHorRight  = lidar_horizontal_fov_right_;
      lidarConfig.FOVVertUp    = lidar_vertical_fov_up_;
      lidarConfig.FOVVertDown  = lidar_vertical_fov_down_;
      lidarConfig.beamLength   = lidar_beam_length_ * 100.0;
      lidarConfig.offset       = ueds_connector::Coordinates(lidar_offset_x_ * 100.0, lidar_offset_y_ * 100.0, lidar_offset_z_ * 100.0);
      lidarConfig.orientation  = ueds_connector::Rotation(-lidar_rotation_pitch_, lidar_rotation_yaw_, lidar_rotation_roll_);
      lidarConfig.showBeams    = lidar_show_beams_;
      lidarConfig.Livox        = lidar_livox_;
      ueds_connectors_[i]->SetLidarConfig(lidarConfig);
    }
  }

  // | ----------------------- publishers ----------------------- |
  ph_clock_ = mrs_lib::PublisherHandler<rosgraph_msgs::msg::Clock>(shared_from_this(), "~/clock_out");
  ph_poses_ = mrs_lib::PublisherHandler<geometry_msgs::msg::PoseArray>(shared_from_this(), "~/uav_poses_out");

  // | --------------------- service servers -------------------- |
  service_server_realtime_factor_ = this->create_service<mrs_msgs::srv::Float64Srv>(
      "~/set_realtime_factor_in", std::bind(&FlightforgeSimulator::callbackSetRealtimeFactor, this, std::placeholders::_1, std::placeholders::_2));

  // | ------------------------- timers ------------------------- |
  timer_dynamics_ =
      create_wall_timer(std::chrono::duration<double>(1.0 / (_simulation_rate_ * realtime_factor_)), std::bind(&FlightforgeSimulator::timerDynamics, this), cbgrp_main_);
  timer_status_      = create_wall_timer(1s, std::bind(&FlightforgeSimulator::timerStatus, this), cbgrp_status_);
  timer_time_sync_   = create_wall_timer(1s, std::bind(&FlightforgeSimulator::timerTimeSync, this), cbgrp_status_);
  timer_unreal_sync_ = create_wall_timer(std::chrono::duration<double>(1.0 / _simulation_rate_), std::bind(&FlightforgeSimulator::timerUnrealSync, this), cbgrp_main_);

  timer_rangefinder_ =
      create_wall_timer(std::chrono::duration<double>(1.0 / rangefinder_rate_), std::bind(&FlightforgeSimulator::timerRangefinder, this), cbgrp_sensors_);
  timer_lidar_ = create_wall_timer(std::chrono::duration<double>(1.0 / lidar_rate_), std::bind(&FlightforgeSimulator::timerLidar, this), cbgrp_sensors_);
  timer_seg_lidar_ =
      create_wall_timer(std::chrono::duration<double>(1.0 / lidar_seg_rate_), std::bind(&FlightforgeSimulator::timerSegLidar, this), cbgrp_sensors_);
  timer_int_lidar_ =
      create_wall_timer(std::chrono::duration<double>(1.0 / lidar_int_rate_), std::bind(&FlightforgeSimulator::timerIntLidar, this), cbgrp_sensors_);
  timer_rgb_ = create_wall_timer(std::chrono::duration<double>(1.0 / rgb_rate_), std::bind(&FlightforgeSimulator::timerRgb, this), cbgrp_sensors_);
  timer_stereo_ = create_wall_timer(std::chrono::duration<double>(1.0 / stereo_rate_), std::bind(&FlightforgeSimulator::timerStereo, this), cbgrp_sensors_);
  timer_rgb_segmented_ =
      create_wall_timer(std::chrono::duration<double>(1.0 / rgb_segmented_rate_), std::bind(&FlightforgeSimulator::timerRgbSegmented, this), cbgrp_sensors_);
  timer_depth_ = create_wall_timer(std::chrono::duration<double>(1.0 / rgb_depth_rate_), std::bind(&FlightforgeSimulator::timerDepth, this), cbgrp_sensors_);

  rgb_camera_orientations_.resize(uavs_.size());
  set_ground_z_clients_.resize(uav_names.size());
  for (size_t i = 0; i < uav_names.size(); i++) {
    std::string service_name        = "/" + uav_names[i] + "/set_ground_z";
    set_ground_z_clients_[i]        = this->create_client<mrs_msgs::srv::Float64Srv>(service_name);
    std::string gimbal_service_name = "/" + uav_names[i] + "/set_gimbal_orientation";
    service_gimbal_control_servers_.push_back(this->create_service<tutorial_interfaces::srv::SetOrientation>(
        gimbal_service_name, [this, i](const std::shared_ptr<tutorial_interfaces::srv::SetOrientation::Request> request,
                                       std::shared_ptr<tutorial_interfaces::srv::SetOrientation::Response>      response) {
          this->callbackSetGimbalOrientation(request, response, i);
        }));
  }

  // | -------------------- finishing methods ------------------- |
  fabricateCamInfo();
  publishStaticTfs();

  is_initialized_ = true;
  RCLCPP_INFO(get_logger(), "[FlightforgeSimulator]: initialized");
  timer_init_->cancel();
}
//}

/* parametersCallback() //{ */

rcl_interfaces::msg::SetParametersResult FlightforgeSimulator::parametersCallback(const std::vector<rclcpp::Parameter>& parameters) {

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto& param : parameters) {
    const auto& param_name = param.get_name();
    if (param_name == "realtime_factor") {
      realtime_factor_ = param.as_double();
    } else if (param_name == "dynamic_rtf") {
      dynamic_rtf_ = param.as_bool();
    } else if (param_name == "paused") {
      paused_ = param.as_bool();
    }
  }

  return result;
}

//}

/* timerDynamics() //{ */

void FlightforgeSimulator::timerDynamics() {

  if (!is_initialized_ || paused_) {
    return;
  }

  mrs_lib::ScopeTimer scope_timer = mrs_lib::ScopeTimer("timerDynamics", scope_timer_logger_, true);

  // | ------------------ make simulation step ------------------ |
  double           simulation_step_size = 1.0 / _simulation_rate_;
  rclcpp::Duration simulation_step_size_duration(static_cast<int64_t>(simulation_step_size * 1e9));
  sim_time_ += simulation_step_size_duration;

  for (size_t i = 0; i < uavs_.size(); i++) {
    if (!uavs_[i]->hasCrashed()) {
      uavs_[i]->makeStep(simulation_step_size, sim_time_.seconds());
    }
  }

  publishPoses();

  // | ---------------------- publish time ---------------------- |
  if ((sim_time_ - last_published_time_).seconds() >= _clock_min_dt_) {
    rosgraph_msgs::msg::Clock ros_time;
    ros_time.clock       = sim_time_;
    ph_clock_.publish(ros_time);
    last_published_time_ = sim_time_;
  }
}

//}

/* timerStatus() //{ */

void FlightforgeSimulator::timerStatus() {

  if (!is_initialized_) {
    return;
  }

  auto             sim_time        = mrs_lib::get_mutexed(mutex_sim_time_, sim_time_);
  rclcpp::Duration last_sec_sim_dt = sim_time - last_sim_time_status_;
  last_sim_time_status_            = sim_time;
  double last_sec_rtf              = last_sec_sim_dt.seconds() / 1.0;
  actual_rtf_                      = 0.9 * actual_rtf_ + 0.1 * last_sec_rtf;

  double fps;
  {
    std::scoped_lock lock(mutex_ueds_);
    auto [res, fps_val] = ueds_game_controller_->GetFps();
    if (!res) {
      RCLCPP_ERROR(get_logger(), "[FlightforgeSimulator]: failed to get FPS from ueds");
      return;
    }
    fps = fps_val;
  }

  const double fps_filter_const = 0.9;
  ueds_fps_                     = fps_filter_const * ueds_fps_ + (1.0 - fps_filter_const) * fps;

  double highest_fps = 0;
  if (rgb_enabled_ && rgb_rate_ > highest_fps)
    highest_fps = rgb_rate_;
  if (stereo_enabled_ && stereo_rate_ > highest_fps)
    highest_fps = stereo_rate_;
  if (lidar_enabled_ && lidar_rate_ > highest_fps)
    highest_fps = lidar_rate_;
  if (rgb_segmented_enabled_ && rgb_segmented_rate_ > highest_fps)
    highest_fps = rgb_segmented_rate_;
  if (lidar_seg_enabled_ && lidar_seg_rate_ > highest_fps)
    highest_fps = lidar_seg_rate_;
  if (lidar_int_enabled_ && lidar_int_rate_ > highest_fps)
    highest_fps = lidar_int_rate_;

  const double ueds_rtf_   = ueds_fps_ / highest_fps;
  const double desired_rtf = (dynamic_rtf_ && ueds_rtf_ < realtime_factor_) ? ueds_rtf_ : realtime_factor_;

  timer_dynamics_->cancel();
  timer_dynamics_ = create_wall_timer(std::chrono::duration<double>(1.0 / (_simulation_rate_ * desired_rtf)), std::bind(&FlightforgeSimulator::timerDynamics, this),
                                      cbgrp_main_);

  if (_collisions_) {
    checkForCrash();
  }

  RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000, "[FlightforgeSimulator]: %s, desired RTF = %.2f, actual RTF = %.2f, ueds FPS = %.2f",
                       paused_ ? "paused" : "running", desired_rtf, actual_rtf_, fps);
}
//}

/* timerUnrealSync() //{ */

void FlightforgeSimulator::timerUnrealSync() {
  if (!is_initialized_) {
    return;
  }
  updateUnrealPoses(false);
}

//}

/* timerTimeSync() //{ */

void FlightforgeSimulator::timerTimeSync() {
  if (!is_initialized_) {
    return;
  }
  // Not implemented in detail, just a placeholder
}
//}

/* timerRangefinder() //{ */
void FlightforgeSimulator::timerRangefinder() {
  if (!is_initialized_ || !rangefinder_enabled_) {
    return;
  }

  for (size_t i = 0; i < uavs_.size(); i++) {
    bool   res;
    double range;
    {
      std::scoped_lock lock(mutex_ueds_);
      std::tie(res, range) = ueds_connectors_[i]->GetRangefinderData();
    }

    if (!res) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *this->get_clock(), 1000, "[FlightforgeSimulator]: [uav%d] - ERROR GetRangefinderData", int(i));
      continue;
    }

    sensor_msgs::msg::Range msg_range;
    msg_range.header.stamp    = this->get_clock()->now();
    msg_range.header.frame_id = "uav" + std::to_string(i + 1) + "/fcu";
    msg_range.radiation_type  = sensor_msgs::msg::Range::INFRARED;
    msg_range.min_range       = 0.1;
    msg_range.max_range       = 30;
    msg_range.range           = range / 100.0;

    ph_rangefinders_[i].publish(msg_range);
  }
}
/*//}*/

/* timerLidar() //{ */

void FlightforgeSimulator::timerLidar() {
  if (!is_initialized_ || !lidar_enabled_) {
    return;
  }

  for (size_t i = 0; i < uavs_.size(); i++) {
    bool                                   res;
    std::vector<ueds_connector::LidarData> lidarData;
    ueds_connector::Coordinates            start;
    {
      std::scoped_lock lock(mutex_ueds_);
      std::tie(res, lidarData, start) = ueds_connectors_[i]->GetLidarData();
    }

    if (!res) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *this->get_clock(), 1000, "[FlightforgeSimulator]: [uav%d] - ERROR getLidarData", int(i));
      continue;
    }

    sensor_msgs::msg::PointCloud2    pcl_msg;
    sensor_msgs::PointCloud2Modifier modifier(pcl_msg);
    modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                  sensor_msgs::msg::PointField::FLOAT32, "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    pcl_msg.header.stamp    = this->get_clock()->now();
    pcl_msg.header.frame_id = "uav" + std::to_string(i + 1) + "/lidar";
    pcl_msg.height          = lidar_vertical_rays_;
    pcl_msg.width           = lidar_horizontal_rays_;
    pcl_msg.is_dense        = true;
    pcl_msg.point_step      = 16;
    pcl_msg.row_step        = pcl_msg.point_step * pcl_msg.width;
    pcl_msg.data.resize(pcl_msg.row_step * pcl_msg.height);

    sensor_msgs::PointCloud2Iterator<float> iterX(pcl_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iterY(pcl_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iterZ(pcl_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iterIntensity(pcl_msg, "intensity");

    for (const auto& ray : lidarData) {
      tf2::Vector3 dir(ray.directionX, ray.directionY, ray.directionZ);
      double       ray_distance = ray.distance / 100.0;
      if (lidar_noise_enabled_ && ray_distance > 0) {
        const double                     std = ray_distance * lidar_std_slope_ * lidar_std_at_1m_;
        std::normal_distribution<double> distribution(0, std);
        ray_distance += distribution(rng);
      }
      dir              = dir.normalized() * ray_distance;
      *iterX           = dir.x();
      *iterY           = -dir.y();
      *iterZ           = dir.z();
      *iterIntensity   = ray.distance;
      ++iterX;
      ++iterY;
      ++iterZ;
      ++iterIntensity;
    }
    ph_lidars_[i].publish(pcl_msg);
  }
}

//}

/* timerSegLidar() //{ */

void FlightforgeSimulator::timerSegLidar() {
  if (!is_initialized_ || !lidar_seg_enabled_) {
    return;
  }

  for (size_t i = 0; i < uavs_.size(); i++) {
    bool                                      res;
    std::vector<ueds_connector::LidarSegData> lidarSegData;
    ueds_connector::Coordinates               start;
    {
      std::scoped_lock lock(mutex_ueds_);
      std::tie(res, lidarSegData, start) = ueds_connectors_[i]->GetLidarSegData();
    }

    if (!res) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *this->get_clock(), 1000, "[FlightforgeSimulator]: [uav%d] - ERROR getLidarSegData", int(i));
      continue;
    }

    PCLPointCloudColor pcl_cloud;
    for (const auto& ray : lidarSegData) {
      pcl::PointXYZRGB point;
      tf2::Vector3     dir(ray.directionX, ray.directionY, ray.directionZ);
      double           ray_distance = ray.distance / 100.0;
      if (lidar_noise_enabled_ && ray_distance > 0) {
        const double                     std = ray_distance * lidar_std_slope_ * lidar_std_at_1m_;
        std::normal_distribution<double> distribution(0, std);
        ray_distance += distribution(rng);
      }
      dir     = dir.normalized() * ray_distance;
      point.x = dir.x();
      point.y = -dir.y();
      point.z = dir.z();
      point.r = seg_rgb_[ray.segmentation][0];
      point.g = seg_rgb_[ray.segmentation][1];
      point.b = seg_rgb_[ray.segmentation][2];
      pcl_cloud.push_back(point);
    }

    sensor_msgs::msg::PointCloud2 pcl_msg;
    pcl::toROSMsg(pcl_cloud, pcl_msg);
    pcl_msg.header.stamp    = this->get_clock()->now();
    pcl_msg.header.frame_id = "uav" + std::to_string(i + 1) + "/lidar";
    ph_seg_lidars_[i].publish(pcl_msg);
  }
}

//}

/* timerIntLidar() //{ */

void FlightforgeSimulator::timerIntLidar() {
  if (!is_initialized_ || !lidar_int_enabled_) {
    return;
  }

  for (size_t i = 0; i < uavs_.size(); i++) {
    bool                                      res;
    std::vector<ueds_connector::LidarIntData> lidarIntData;
    ueds_connector::Coordinates               start;
    {
      std::scoped_lock lock(mutex_ueds_);
      std::tie(res, lidarIntData, start) = ueds_connectors_[i]->GetLidarIntData();
    }

    if (!res) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *this->get_clock(), 1000, "[FlightforgeSimulator]: [uav%d] - ERROR getLidarIntData", int(i));
      continue;
    }

    PCLPointCloudIntensity pcl_cloud;
    for (const auto& ray : lidarIntData) {
      pcl::PointXYZI point;
      tf2::Vector3   dir(ray.directionX, ray.directionY, ray.directionZ);
      double         ray_distance = ray.distance / 100.0;
      if (lidar_noise_enabled_ && ray_distance > 0) {
        const double                     std = ray_distance * lidar_std_slope_ * lidar_std_at_1m_;
        std::normal_distribution<double> distribution(0, std);
        ray_distance += distribution(rng);
      }
      dir     = dir.normalized() * ray_distance;
      point.x = dir.x();
      point.y = -dir.y();
      point.z = dir.z();

      switch (ray.intensity) {
        case 0:
          point.intensity = lidar_int_value_other_;
          break;
        case 1:
          point.intensity = lidar_int_value_grass_;
          break;
        case 2:
          point.intensity = lidar_int_value_road_;
          break;
        case 3:
          point.intensity = lidar_int_value_tree_;
          break;
        case 4:
          point.intensity = lidar_int_value_building_;
          break;
        case 5:
          point.intensity = lidar_int_value_fence_;
          break;
        case 6:
          point.intensity = lidar_int_value_dirt_road_;
          break;
      }
      if (lidar_int_noise_enabled_ && ray_distance > 0) {
        const double                     intensity_std = ray_distance * lidar_int_std_slope_ * lidar_int_std_at_1m_;
        std::normal_distribution<double> intensity_distribution(0, intensity_std);
        point.intensity += intensity_distribution(rng);
        point.intensity = std::clamp(point.intensity, 0.0f, 255.0f);
      }
      pcl_cloud.push_back(point);
    }

    sensor_msgs::msg::PointCloud2 pcl_msg;
    pcl::toROSMsg(pcl_cloud, pcl_msg);
    pcl_msg.header.stamp    = this->get_clock()->now();
    pcl_msg.header.frame_id = "uav" + std::to_string(i + 1) + "/lidar";
    ph_int_lidars_[i].publish(pcl_msg);
  }
}

//}

/* timerRgb() //{ */

void FlightforgeSimulator::timerRgb() {
  if (!is_initialized_ || !rgb_enabled_) {
    return;
  }

  for (size_t i = 0; i < uavs_.size(); i++) {
    bool                       res;
    std::vector<unsigned char> cameraData;
    uint32_t                   size;
    double                     stamp;
    {
      std::scoped_lock lock(mutex_ueds_);
      std::tie(res, cameraData, stamp, size) = ueds_connectors_[i]->GetRgbCameraData();
    }

    if (abs(stamp - last_rgb_ue_stamp_.at(i)) < 0.001) {
      return;
    }
    last_rgb_ue_stamp_.at(i) = stamp;

    if (!res || cameraData.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000, "[FlightforgeSimulator]: failed to obtain rgb camera from uav%lu", i + 1);
      continue;
    }

    cv::Mat image = cv::imdecode(cameraData, cv::IMREAD_COLOR);
    auto    msg   = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
    msg->header.frame_id = "uav" + std::to_string(i + 1) + "/rgb";
    msg->header.stamp    = this->get_clock()->now();  // Simplified timestamping

    imp_rgb_[i].publish(*msg);

    auto camera_info   = rgb_camera_info_;
    camera_info.header = msg->header;
    ph_rgb_camera_info_[i].publish(camera_info);
  }
}

//}

/* timerStereo() //{ */

void FlightforgeSimulator::timerStereo() {
  if (!is_initialized_ || !stereo_enabled_) {
    return;
  }

  for (size_t i = 0; i < uavs_.size(); i++) {
    bool                       res;
    std::vector<unsigned char> image_left;
    std::vector<unsigned char> image_right;
    double                     stamp;
    {
      std::scoped_lock lock(mutex_ueds_);
      std::tie(res, image_left, image_right, stamp) = ueds_connectors_[i]->GetStereoCameraData();
    }

    if (abs(stamp - last_stereo_ue_stamp_.at(i)) < 0.001) {
      return;
    }
    last_stereo_ue_stamp_.at(i) = stamp;

    if (!res || image_left.empty() || image_right.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000, "[FlightforgeSimulator]: failed to obtain stereo camera from uav%lu", i + 1);
      continue;
    }

    cv::Mat cv_left  = cv::imdecode(image_left, cv::IMREAD_COLOR);
    cv::Mat cv_right = cv::imdecode(image_right, cv::IMREAD_COLOR);

    auto msg_left  = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cv_left).toImageMsg();
    auto msg_right = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cv_right).toImageMsg();

    msg_left->header.stamp    = this->get_clock()->now();
    msg_left->header.frame_id = "uav" + std::to_string(i + 1) + "/stereo_left";
    msg_right->header.stamp    = msg_left->header.stamp;
    msg_right->header.frame_id = "uav" + std::to_string(i + 1) + "/stereo_right";

    imp_stereo_left_[i].publish(*msg_left);
    imp_stereo_right_[i].publish(*msg_right);

    {
      auto camera_info   = stereo_camera_info_;
      camera_info.header = msg_left->header;
      ph_stereo_left_camera_info_[i].publish(camera_info);
    }
    {
      auto camera_info   = stereo_camera_info_;
      camera_info.header = msg_right->header;
      camera_info.p[3]   = -camera_info.p[0] * stereo_baseline_;
      ph_stereo_right_camera_info_[i].publish(camera_info);
    }
  }
}

//}

/* timerRgbSegmented() //{ */

void FlightforgeSimulator::timerRgbSegmented() {
  if (!is_initialized_ || !rgb_segmented_enabled_) {
    return;
  }
  // Implementation similar to timerRgb
}

//}

/* timerDepth() //{ */

void FlightforgeSimulator::timerDepth() {
  if (!is_initialized_ || !rgb_depth_enabled_) {
    return;
  }
  // Implementation similar to timerRgb, but for depth images
}

//}

/* callbackSetRealtimeFactor() //{ */

void FlightforgeSimulator::callbackSetRealtimeFactor(const std::shared_ptr<mrs_msgs::srv::Float64Srv::Request>   request,
                                                     std::shared_ptr<mrs_msgs::srv::Float64Srv::Response> response) {
  if (!is_initialized_) {
    response->success = false;
    response->message = "not initialized";
    return;
  }
  realtime_factor_  = request->value;
  response->success = true;
  response->message = "realtime factor set";
}

//}

/* callbackSetGimbalOrientation() //{ */

void FlightforgeSimulator::callbackSetGimbalOrientation(
    const std::shared_ptr<tutorial_interfaces::srv::SetOrientation::Request> request, std::shared_ptr<tutorial_interfaces::srv::SetOrientation::Response> response,
    int uav_index) {
  if (!is_initialized_) {
    response->success = false;
    response->message = "not initialized";
    return;
  }

  if (uav_index < 0 || uav_index >= (int)ueds_connectors_.size()) {
    response->success = false;
    response->message = "invalid uav_index";
    return;
  }

  ueds_connector::Rotation rot(request->roll, request->pitch, request->yaw);
  bool                     res = ueds_connectors_[uav_index]->SetGimbalOrientation(rot);

  response->success = res;
  if (res) {
    response->message = "Gimbal orientation set.";
  } else {
    response->message = "Failed to set gimbal orientation.";
  }
}

//}

/* updateUnrealPoses() //{ */

void FlightforgeSimulator::updateUnrealPoses(const bool teleport_without_collision) {
  std::scoped_lock lock(mutex_ueds_);
  for (size_t i = 0; i < uavs_.size(); i++) {
    if (uavs_[i]->hasCrashed()) {
      continue;
    }
    auto                       state = uavs_[i]->getState();
    auto                       pos   = position2ue(state.x, ueds_world_origin_);
    mrs_lib::AttitudeConverter rot(state.R);
    ueds_connectors_[i]->SetPose(pos, ueds_connector::Rotation(rot.getRoll(), rot.getPitch(), rot.getYaw()), teleport_without_collision);
  }
}

//}

/* other methods... */
void FlightforgeSimulator::publishPoses(void) {
  if (!ph_poses_.isLatched()) {
    return;
  }

  auto pose_arr_msg            = geometry_msgs::msg::PoseArray();
  pose_arr_msg.header.stamp    = sim_time_;
  pose_arr_msg.header.frame_id = _world_frame_name_;

  for (auto& uav : uavs_) {
    auto                     state = uav->getState();
    geometry_msgs::msg::Pose pose;
    pose.position.x    = state.x.x();
    pose.position.y    = state.x.y();
    pose.position.z    = state.x.z();
    pose.orientation   = mrs_lib::AttitudeConverter(state.R);
    pose_arr_msg.poses.push_back(pose);
  }
  ph_poses_.publish(pose_arr_msg);
}

void FlightforgeSimulator::checkForCrash(void) {
  for (size_t i = 0; i < uavs_.size(); i++) {
    if (uavs_[i]->hasCrashed()) {
      continue;
    }
    auto [res, has_crashed] = ueds_connectors_[i]->HasCrashed();
    if (res && has_crashed) {
      uavs_[i]->crash();
      RCLCPP_ERROR(get_logger(), "[FlightforgeSimulator]: uav%d has crashed!", (int)i);
    }
  }
}

void FlightforgeSimulator::fabricateCamInfo(void) {
  // RGB
  rgb_camera_info_.header.frame_id = "rgb_camera";
  rgb_camera_info_.width           = rgb_width_;
  rgb_camera_info_.height          = rgb_height_;
  double fx                        = (rgb_width_ / 2.0) / tan((rgb_fov_ / 2.0) * (M_PI / 180.0));
  rgb_camera_info_.k[0]            = fx;
  rgb_camera_info_.k[2]            = rgb_width_ / 2.0;
  rgb_camera_info_.k[4]            = fx;
  rgb_camera_info_.k[5]            = rgb_height_ / 2.0;
  rgb_camera_info_.k[8]            = 1.0;
  rgb_camera_info_.p[0]            = fx;
  rgb_camera_info_.p[2]            = rgb_width_ / 2.0;
  rgb_camera_info_.p[5]            = fx;
  rgb_camera_info_.p[6]            = rgb_height_ / 2.0;
  rgb_camera_info_.p[10]           = 1.0;

  // Stereo
  stereo_camera_info_.header.frame_id = "stereo_camera";
  stereo_camera_info_.width           = stereo_width_;
  stereo_camera_info_.height          = stereo_height_;
  double stereo_fx                    = (stereo_width_ / 2.0) / tan((stereo_fov_ / 2.0) * (M_PI / 180.0));
  stereo_camera_info_.k[0]            = stereo_fx;
  stereo_camera_info_.k[2]            = stereo_width_ / 2.0;
  stereo_camera_info_.k[4]            = stereo_fx;
  stereo_camera_info_.k[5]            = stereo_height_ / 2.0;
  stereo_camera_info_.k[8]            = 1.0;
  stereo_camera_info_.p[0]            = stereo_fx;
  stereo_camera_info_.p[2]            = stereo_width_ / 2.0;
  stereo_camera_info_.p[5]            = stereo_fx;
  stereo_camera_info_.p[6]            = stereo_height_ / 2.0;
  stereo_camera_info_.p[10]           = 1.0;
}

void FlightforgeSimulator::publishStaticTfs(void) {
  // Not implemented
}

void FlightforgeSimulator::publishCameraTf(const int& uav_index) {
  // Not implemented
}

ueds_connector::Coordinates FlightforgeSimulator::position2ue(const Eigen::Vector3d& pos, const ueds_connector::Coordinates& ueds_world_origin) {
  return ueds_connector::Coordinates(pos.x() * 100.0 + ueds_world_origin.x, -(pos.y() * 100.0 - ueds_world_origin.y),
                                     pos.z() * 100.0 + ueds_world_origin.z);
}

double FlightforgeSimulator::uedsToWallTime(const double ueds_time) {
  std::scoped_lock lock(mutex_wall_time_offset_);
  return ueds_time + wall_time_offset_;
}

}  // namespace mrs_uav_flightforge_simulation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mrs_uav_flightforge_simulation::FlightforgeSimulator)
