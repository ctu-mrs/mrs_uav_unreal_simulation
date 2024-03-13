/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_multirotor_simulator/uav_system_ros.h>

#include <rosgraph_msgs/Clock.h>

#include <geometry_msgs/PoseArray.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/scope_timer.h>

#include <dynamic_reconfigure/server.h>
#include <mrs_uav_unreal_simulation/unreal_simulatorConfig.h>

#include <Eigen/Dense>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>

#include <ueds_connector/ueds_connector.h>
#include <ueds_connector/game-mode-controller.h>

#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

//}

using PCLPoint           = pcl::PointXYZ;
using PCLPointCloud      = pcl::PointCloud<PCLPoint>;
using PCLPointCloudColor = pcl::PointCloud<pcl::PointXYZRGB>;
namespace mrs_uav_unreal_simulation
{

/* class UnrealSimulator //{ */

class UnrealSimulator : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle   nh_;
  std::atomic<bool> is_initialized_;

  // | ------------------------- params ------------------------- |

  double _simulation_rate_;

  ros::Time  sim_time_;
  std::mutex mutex_sim_time_;

  double _clock_min_dt_;

  std::string _world_frame_name_;

  // | ------------------------- timers ------------------------- |

  ros::WallTimer timer_main_;
  void           timerMain(const ros::WallTimerEvent& event);

  ros::WallTimer timer_status_;
  void           timerStatus(const ros::WallTimerEvent& event);

  ros::Timer timer_lidar_;
  void       timerLidar(const ros::TimerEvent& event);

  ros::Timer timer_seg_lidar_;
  void       timerSegLidar(const ros::TimerEvent& event);

  ros::Timer timer_rgb_;
  void       timerRgb(const ros::TimerEvent& event);

  ros::Timer timer_depth_;
  void       timerDepth(const ros::TimerEvent& event);

  ros::Timer timer_seg_;
  void       timerSeg(const ros::TimerEvent& event);

  ros::Timer timer_color_depth_;
  void       timerColorDepth(const ros::TimerEvent& event);

  // | ------------------------ rtf check ----------------------- |

  double    actual_rtf_ = 1.0;
  ros::Time last_sim_time_status_;

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<rosgraph_msgs::Clock>     ph_clock_;
  mrs_lib::PublisherHandler<geometry_msgs::PoseArray> ph_poses_;

  std::vector<mrs_lib::PublisherHandler<sensor_msgs::PointCloud2>>     ph_lidars_;
  std::vector<mrs_lib::PublisherHandler<sensor_msgs::PointCloud2>>     ph_seg_lidars_;
  std::vector<mrs_lib::PublisherHandler<sensor_msgs::CompressedImage>> ph_rgbs_;
  std::vector<mrs_lib::PublisherHandler<sensor_msgs::CompressedImage>> ph_depths_;
  std::vector<mrs_lib::PublisherHandler<sensor_msgs::CompressedImage>> ph_segs_;
  std::vector<mrs_lib::PublisherHandler<sensor_msgs::CompressedImage>> ph_color_depths_;

  // | ------------------------- system ------------------------- |

  std::vector<std::shared_ptr<mrs_multirotor_simulator::UavSystemRos>> uavs_;

  // | -------------------------- time -------------------------- |

  ros::Time last_published_time_;

  // | ------------------------- methods ------------------------ |

  void publishPoses(void);

  // | --------------- dynamic reconfigure server --------------- |

  boost::recursive_mutex                                    mutex_drs_;
  typedef mrs_uav_unreal_simulation::unreal_simulatorConfig DrsConfig_t;
  typedef dynamic_reconfigure::Server<DrsConfig_t>          Drs_t;
  boost::shared_ptr<Drs_t>                                  drs_;
  void                                                      callbackDrs(mrs_uav_unreal_simulation::unreal_simulatorConfig& config, uint32_t level);
  DrsConfig_t                                               drs_params_;
  std::mutex                                                mutex_drs_params_;

  // | ------------------------- Unreal ------------------------- |

  std::unique_ptr<ueds_connector::GameModeController> ueds_game_controller_;

  std::vector<std::shared_ptr<ueds_connector::UedsConnector>> ueds_connectors_;

  std::mutex mutex_ueds_;

  std::vector<ueds_connector::Coordinates> ueds_world_origins_;

  void updateUnrealPoses(void);
};

//}

/* onInit() //{ */

void UnrealSimulator::onInit() {

  is_initialized_ = false;

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  if (!(nh_.hasParam("/use_sim_time"))) {
    nh_.setParam("/use_sim_time", true);
  }

  srand(time(NULL));

  sim_time_            = ros::Time(0);
  last_published_time_ = ros::Time(0);

  mrs_lib::ParamLoader param_loader(nh_, "UnrealSimulator");

  std::string custom_config_path;

  param_loader.loadParam("custom_config", custom_config_path);

  if (custom_config_path != "") {
    param_loader.addYamlFile(custom_config_path);
  }

  param_loader.addYamlFileFromParam("config");
  param_loader.addYamlFileFromParam("config_uavs");

  param_loader.loadParam("simulation_rate", _simulation_rate_);
  param_loader.loadParam("realtime_factor", drs_params_.realtime_factor);
  param_loader.loadParam("frames/world/name", _world_frame_name_);

  param_loader.loadParam("sensors/lidar/enabled", drs_params_.lidar_enabled);
  param_loader.loadParam("sensors/lidar/rate", drs_params_.lidar_rate);

  param_loader.loadParam("sensors/lidar_seg/enabled", drs_params_.lidar_seg_enabled);
  param_loader.loadParam("sensors/lidar_seg/rate", drs_params_.lidar_seg_rate);

  param_loader.loadParam("sensors/rgb/enabled", drs_params_.rgb_enabled);
  param_loader.loadParam("sensors/rgb/rate", drs_params_.rgb_rate);

  param_loader.loadParam("sensors/depth/enabled", drs_params_.depth_enabled);
  param_loader.loadParam("sensors/depth/rate", drs_params_.depth_rate);

  param_loader.loadParam("sensors/seg/enabled", drs_params_.seg_enabled);
  param_loader.loadParam("sensors/seg/rate", drs_params_.seg_rate);

  param_loader.loadParam("sensors/color_depth/enabled", drs_params_.color_depth_enabled);
  param_loader.loadParam("sensors/color_depth/rate", drs_params_.color_depth_rate);

  double clock_rate;
  param_loader.loadParam("clock_rate", clock_rate);

  drs_params_.paused = false;

  std::vector<std::string> uav_names;

  param_loader.loadParam("uav_names", uav_names);

  for (size_t i = 0; i < uav_names.size(); i++) {

    std::string uav_name = uav_names[i];

    ROS_INFO("[UnrealSimulator]: initializing '%s'", uav_name.c_str());

    uavs_.push_back(std::make_unique<mrs_multirotor_simulator::UavSystemRos>(nh_, uav_name));
  }

  // | ----------------------- Unreal sim ----------------------- |

  ueds_game_controller_ = std::make_unique<ueds_connector::GameModeController>(LOCALHOST, 8000);

  while (true) {

    bool connect_result = ueds_game_controller_->Connect();

    if (connect_result != 1) {
      ROS_ERROR("[UnrealSimulator]: Error connecting to Unreal's game mode controller, connect_result was %d", connect_result);
    } else {
      break;
    }

    ros::Duration(1.0).sleep();
  }

  for (size_t i = 0; i < uav_names.size(); i++) {

    const std::string uav_name = uav_names[i];

    const auto [resSpawn, port] = ueds_game_controller_->SpawnDrone();

    if (!resSpawn) {
      ROS_ERROR("[UnrealSimulator]: failed to spawn %s", uav_names[i].c_str());
      ros::shutdown();
    }

    ROS_INFO("[UnrealSimulator]: %s spawned", uav_name.c_str());

    std::shared_ptr<ueds_connector::UedsConnector> ueds_connector = std::make_shared<ueds_connector::UedsConnector>(LOCALHOST, port);

    ueds_connectors_.push_back(ueds_connector);

    auto connect_result = ueds_connector->Connect();

    if (connect_result != 1) {

      ROS_ERROR("[UnrealSimulator]: %s - Error connecting to drone controller, connect_result was %d", uav_name.c_str(), connect_result);
      ros::shutdown();

    } else {

      ROS_INFO("[UnrealSimulator]: %s - Connection succeed: %d", uav_name.c_str(), connect_result);

      const auto [res, location] = ueds_connector->GetLocation();

      if (!res) {
        ROS_ERROR("[UnrealSimulator]: %s - DroneError: getting location", uav_name.c_str());
        ros::shutdown();
      } else {
        ueds_world_origins_.push_back(location);
      }
    }

    ph_lidars_.push_back(mrs_lib::PublisherHandler<sensor_msgs::PointCloud2>(nh_, "/" + uav_name + "/lidar/points", 10));
    ph_seg_lidars_.push_back(mrs_lib::PublisherHandler<sensor_msgs::PointCloud2>(nh_, "/" + uav_name + "/lidar_seg/points", 10));
    ph_rgbs_.push_back(mrs_lib::PublisherHandler<sensor_msgs::CompressedImage>(nh_, "/" + uav_name + "/rgbd/image_raw/compressed", 10));
    ph_depths_.push_back(mrs_lib::PublisherHandler<sensor_msgs::CompressedImage>(nh_, "/" + uav_name + "/depth/image_raw/compressed", 10));
    ph_segs_.push_back(mrs_lib::PublisherHandler<sensor_msgs::CompressedImage>(nh_, "/" + uav_name + "/seg/image_raw/compressed", 10));
    ph_color_depths_.push_back(mrs_lib::PublisherHandler<sensor_msgs::CompressedImage>(nh_, "/" + uav_name + "/depth_color/image_raw/compressed", 10));
  }

  // | --------------- dynamic reconfigure server --------------- |

  drs_.reset(new Drs_t(mutex_drs_, nh_));
  drs_->updateConfig(drs_params_);
  Drs_t::CallbackType f = boost::bind(&UnrealSimulator::callbackDrs, this, _1, _2);
  drs_->setCallback(f);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[UnrealSimulator]: could not load all parameters!");
    ros::shutdown();
  }

  _clock_min_dt_ = 1.0 / clock_rate;

  // | ----------------------- publishers ----------------------- |

  ph_clock_ = mrs_lib::PublisherHandler<rosgraph_msgs::Clock>(nh_, "clock_out", 10, false);

  ph_poses_ = mrs_lib::PublisherHandler<geometry_msgs::PoseArray>(nh_, "uav_poses_out", 10, false);

  // | ------------------------- timers ------------------------- |

  timer_main_ = nh_.createWallTimer(ros::WallDuration(1.0 / (_simulation_rate_ * drs_params_.realtime_factor)), &UnrealSimulator::timerMain, this);

  timer_status_ = nh_.createWallTimer(ros::WallDuration(1.0), &UnrealSimulator::timerStatus, this);

  timer_lidar_ = nh_.createTimer(ros::Duration(1.0 / drs_params_.lidar_rate), &UnrealSimulator::timerLidar, this);

  timer_seg_lidar_ = nh_.createTimer(ros::Duration(1.0 / drs_params_.lidar_rate), &UnrealSimulator::timerSegLidar, this);

  timer_rgb_ = nh_.createTimer(ros::Duration(1.0 / drs_params_.rgb_rate), &UnrealSimulator::timerRgb, this);

  timer_depth_ = nh_.createTimer(ros::Duration(1.0 / drs_params_.depth_rate), &UnrealSimulator::timerDepth, this);

  timer_seg_ = nh_.createTimer(ros::Duration(1.0 / drs_params_.seg_rate), &UnrealSimulator::timerSeg, this);

  timer_color_depth_ = nh_.createTimer(ros::Duration(1.0 / drs_params_.color_depth_rate), &UnrealSimulator::timerColorDepth, this);

  // | ----------------------- finish init ---------------------- |

  is_initialized_ = true;

  ROS_INFO("[UnrealSimulator]: initialized");
}

//}

// | ------------------------- timers ------------------------- |

/* timerMain() //{ */

void UnrealSimulator::timerMain([[maybe_unused]] const ros::WallTimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[UnrealSimulator]: main timer spinning");

  // | ------------------ make simulation step ------------------ |

  double simulation_step_size = 1.0 / _simulation_rate_;

  sim_time_ = sim_time_ + ros::Duration(simulation_step_size);

  for (size_t i = 0; i < uavs_.size(); i++) {
    uavs_[i]->makeStep(simulation_step_size);
  }

  publishPoses();

  // | ---------------------- publish time ---------------------- |

  if ((sim_time_ - last_published_time_).toSec() >= _clock_min_dt_) {

    rosgraph_msgs::Clock ros_time;

    ros_time.clock.fromSec(sim_time_.toSec());

    ph_clock_.publish(ros_time);

    last_published_time_ = sim_time_;
  }
}

//}

/* timerStatus() //{ */

void UnrealSimulator::timerStatus([[maybe_unused]] const ros::WallTimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  auto sim_time   = mrs_lib::get_mutexed(mutex_sim_time_, sim_time_);
  auto drs_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);

  ros::Duration last_sec_sim_dt = sim_time - last_sim_time_status_;

  last_sim_time_status_ = sim_time;

  double last_sec_rtf = last_sec_sim_dt.toSec() / 1.0;

  actual_rtf_ = 0.9 * actual_rtf_ + 0.1 * last_sec_rtf;

  ROS_INFO_THROTTLE(0.1, "[UnrealSimulator]: %s, desired RTF = %.2f, actual RTF = %.2f", drs_params.paused ? "paused" : "running", drs_params.realtime_factor,
                    actual_rtf_);
}

//}

/* timerLidar() //{ */

void UnrealSimulator::timerLidar([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  /* mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("timerLidar()"); */

  auto drs_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);

  if (!drs_params_.lidar_enabled) {
    ROS_INFO_THROTTLE(1.0, "[UnrealSimulator]: LiDAR sensor disabled");
    return;
  }

  updateUnrealPoses();

  for (size_t i = 0; i < uavs_.size(); i++) {

    mrs_multirotor_simulator::MultirotorModel::State state = uavs_[i]->getState();

    bool                                   res;
    std::vector<ueds_connector::LidarData> lidarData;
    ueds_connector::LidarConfig            lidarConfig;
    ueds_connector::Coordinates            start;

    {
      std::scoped_lock lock(mutex_ueds_);

      std::tie(res, lidarConfig) = ueds_connectors_[i]->GetLidarConfig();
    }

    if (!res) {
      ROS_ERROR_THROTTLE(1.0, "[UnrealSimulator]: [uav%d] - ERROR getLidarConfig", int(i));
      continue;
    }

    {
      std::scoped_lock lock(mutex_ueds_);

      std::tie(res, lidarData, start) = ueds_connectors_[i]->GetLidarData();
    }

    if (!res) {
      ROS_ERROR_THROTTLE(1.0, "[UnrealSimulator]: [uav%d] - ERROR getLidarData", int(i));
      continue;
    }

    sensor_msgs::PointCloud2 pcl_msg;

    // Modifier to describe what the fields are.
    sensor_msgs::PointCloud2Modifier modifier(pcl_msg);
    modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1, sensor_msgs::PointField::FLOAT32, "z", 1,
                                  sensor_msgs::PointField::FLOAT32, "intensity", 1, sensor_msgs::PointField::FLOAT32);
    // Msg header
    pcl_msg.header.stamp    = ros::Time::now();
    pcl_msg.header.frame_id = "uav"+ std::to_string(i+1) +"/fcu";

    pcl_msg.height   = lidarConfig.BeamVertRays;
    pcl_msg.width    = lidarConfig.BeamHorRays;
    pcl_msg.is_dense = true;

    // Total number of bytes per point
    pcl_msg.point_step = 16;
    pcl_msg.row_step   = pcl_msg.point_step * pcl_msg.width;
    pcl_msg.data.resize(pcl_msg.row_step * pcl_msg.height);

    sensor_msgs::PointCloud2Iterator<float> iterX(pcl_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iterY(pcl_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iterZ(pcl_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iterIntensity(pcl_msg, "intensity");

    for (const ueds_connector::LidarData& ray : lidarData) {

      tf::Vector3 dir = tf::Vector3(ray.directionX, ray.directionY, ray.directionZ);

      dir = dir.normalized() * (ray.distance / 100.0);

      *iterX         = dir.x();
      *iterY         = -dir.y();  // convert left-hand to right-hand coordinates
      *iterZ         = dir.z();
      *iterIntensity = ray.distance;

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

void UnrealSimulator::timerSegLidar([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  /* mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("timerLidar()"); */

  auto drs_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);

  if (!drs_params_.lidar_seg_enabled) {
    ROS_INFO_THROTTLE(1.0, "[UnrealSimulator]: Segmentation LiDAR sensor disabled");
    return;
  }

  updateUnrealPoses();

  for (size_t i = 0; i < uavs_.size(); i++) {

    mrs_multirotor_simulator::MultirotorModel::State state = uavs_[i]->getState();

    bool                                      res;
    std::vector<ueds_connector::LidarSegData> lidarSegData;
    ueds_connector::LidarConfig               lidarConfig;
    ueds_connector::Coordinates               start;

    {
      std::scoped_lock lock(mutex_ueds_);

      std::tie(res, lidarConfig) = ueds_connectors_[i]->GetLidarConfig();
    }

    if (!res) {
      ROS_ERROR("[UnrealSimulator]: [uav%d] - ERROR getLidarConfig", int(i));
      continue;
    }

    {
      std::scoped_lock lock(mutex_ueds_);

      std::tie(res, lidarSegData, start) = ueds_connectors_[i]->GetLidarSegData();
    }

    if (!res) {
      ROS_ERROR("[UnrealSimulator]: [uav%d] - ERROR getLidarSegData", int(i));
      continue;
    }

    PCLPointCloudColor pcl_cloud;
    for (const ueds_connector::LidarSegData& ray : lidarSegData) {
      pcl::PointXYZRGB point;
      tf::Vector3      dir = tf::Vector3(ray.directionX, ray.directionY, ray.directionZ);
      dir                  = dir.normalized() * (ray.distance / 100.0);

      point.x = dir.x();
      point.y = -dir.y();  // convert left-hand to right-hand coordinates
      point.z = dir.z();
      switch (ray.segmentation) {
        case 0: {
          point.r = 0;
          point.g = 0;
          point.b = 255;
          break;
        }
        case 1: {
          point.r = 0;
          point.g = 0;
          point.b = 255;
          break;
        }
        case 2: {
          point.r = 255;
          point.g = 0;
          point.b = 0;
          break;
        }
        case 3: {
          point.r = 0;
          point.g = 255;
          point.b = 0;
          break;
        }
        case 4: {
          point.r = 120;
          point.g = 255;
          point.b = 0;
          break;
        }
        case 5: {
          point.r = 255;
          point.g = 150;
          point.b = 255;
          break;
        }
        case 6: {
          point.r = 255;
          point.g = 75;
          point.b = 0;
          break;
        }
        case 7: {
          point.r = 0;
          point.g = 20;
          point.b = 255;
          break;
        }
        default: {
          point.r = 0;
          point.g = 0;
          point.b = 0;
          break;
        }
      }

      pcl_cloud.push_back(point);
    }


    sensor_msgs::PointCloud2 pcl_msg;
    pcl::toROSMsg(pcl_cloud, pcl_msg);
    pcl_msg.header.stamp    = ros::Time::now();
    pcl_msg.header.frame_id = "uav"+ std::to_string(i+1) +"/fcu";

    ph_seg_lidars_[i].publish(pcl_msg);
  }
}  // namespace mrs_uav_unreal_simulation

//}

/* timerRgb() //{ */

void UnrealSimulator::timerRgb([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  /* mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("timerRgb()"); */

  auto drs_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);

  if (!drs_params_.rgb_enabled) {
    ROS_INFO_THROTTLE(1.0, "[UnrealSimulator]: RGB sensor disabled");
    return;
  }

  updateUnrealPoses();

  for (size_t i = 0; i < uavs_.size(); i++) {

    bool                       res;
    std::vector<unsigned char> cameraData;
    uint32_t                   size;

    /* timer.checkpoint("before_getting_data"); */

    {
      std::scoped_lock lock(mutex_ueds_);

      std::tie(res, cameraData, size) = ueds_connectors_[i]->GetCameraData();
    }

    /* timer.checkpoint("after_getting_data"); */

    // ROS_WARN("Unreal: send camera msg");
    sensor_msgs::CompressedImage img_msg;
    img_msg.header.stamp = ros::Time::now();
    img_msg.format       = "jpeg";

    img_msg.data = cameraData;

    ph_rgbs_[i].publish(img_msg);
  }
}

//}

/* timerDepth() //{ */

void UnrealSimulator::timerDepth([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  /* mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("timerRgb()"); */

  auto drs_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);

  if (!drs_params_.depth_enabled) {
    ROS_INFO_THROTTLE(1.0, "[UnrealSimulator]: Depth sensor disabled");
    return;
  }

  updateUnrealPoses();

  for (size_t i = 0; i < uavs_.size(); i++) {

    bool                       res;
    std::vector<unsigned char> cameraData;
    uint32_t                   size;

    /* timer.checkpoint("before_getting_data"); */

    {
      std::scoped_lock lock(mutex_ueds_);

      std::tie(res, cameraData, size) = ueds_connectors_[i]->GetCameraDepth();
    }

    /* timer.checkpoint("after_getting_data"); */

    // ROS_WARN("Unreal: send camera msg");
    sensor_msgs::CompressedImage img_msg;
    img_msg.header.stamp = ros::Time::now();
    img_msg.format       = "jpeg";

    img_msg.data = cameraData;

    ph_depths_[i].publish(img_msg);
  }
}

//}

/* timerSeg() //{ */

void UnrealSimulator::timerSeg([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  /* mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("timerRgb()"); */

  auto drs_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);

  if (!drs_params_.seg_enabled) {
    ROS_INFO_THROTTLE(1.0, "[UnrealSimulator]: Seg sensor disabled");
    return;
  }

  updateUnrealPoses();

  for (size_t i = 0; i < uavs_.size(); i++) {

    bool                       res;
    std::vector<unsigned char> cameraData;
    uint32_t                   size;

    /* timer.checkpoint("before_getting_data"); */

    {
      std::scoped_lock lock(mutex_ueds_);

      std::tie(res, cameraData, size) = ueds_connectors_[i]->GetCameraSeg();
    }

    /* timer.checkpoint("after_getting_data"); */

    // ROS_WARN("Unreal: send camera msg");
    sensor_msgs::CompressedImage img_msg;
    img_msg.header.stamp = ros::Time::now();
    img_msg.format       = "jpeg";

    img_msg.data = cameraData;

    ph_segs_[i].publish(img_msg);
  }
}

//}

/* timerColorDepth() //{ */

void UnrealSimulator::timerColorDepth([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  /* mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("timerRgb()"); */

  auto drs_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);

  if (!drs_params_.color_depth_enabled) {
    ROS_INFO_THROTTLE(1.0, "[UnrealSimulator]: Color depth sensor disabled");
    return;
  }

  updateUnrealPoses();

  for (size_t i = 0; i < uavs_.size(); i++) {

    bool                       res;
    std::vector<unsigned char> cameraData;
    uint32_t                   size;

    /* timer.checkpoint("before_getting_data"); */

    {
      std::scoped_lock lock(mutex_ueds_);

      std::tie(res, cameraData, size) = ueds_connectors_[i]->GetCameraColorDepth();
    }

    /* timer.checkpoint("after_getting_data"); */

    // ROS_WARN("Unreal: send camera msg");
    sensor_msgs::CompressedImage img_msg;
    img_msg.header.stamp = ros::Time::now();
    img_msg.format       = "jpeg";

    img_msg.data = cameraData;

    ph_color_depths_[i].publish(img_msg);

    /* if (!drs_params_.color_depth_PC_enabled) { */
    /*   ROS_INFO_THROTTLE(1.0, "[UnrealSimulator]: Color depth PC sensor disabled"); */
    /*   return; */
    /* } */

    /* PCLPointCloud pcl_cloud; */
    /* float         disp_min = 30; */
    /* float         disp_max = 550; */
    /* for (uint32_t i = 0; i < size; i++) { */
    /*   // retrive the pixel data from the image and get the individual color values */
    /*   uint8_t r = cameraData[i * 4]; */
    /*   uint8_t g = cameraData[i * 4 + 1]; */
    /*   uint8_t b = cameraData[i * 4 + 2]; */

    /*   float d_normal; */

    /*   if (r >= g && r >= b && g >= b) { */
    /*     d_normal = g - b; */
    /*   } else if (r >= g && r >= b && g >= b) { */
    /*     d_normal = g - b + 1529 */
    /*   } else if (g >= r && g >= b) { */
    /*     d_normal = b - r + 510; */
    /*   } else if (b >= g && b >= r) { */
    /*     d_normal = r - g + 1020; */
    /*   } */
    /*   depth_recovered = 1529 / (1529 * disp_min + (disp_max - disp_min) * d_normal); */
    /* } */
  }
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackDrs() //{ */

void UnrealSimulator::callbackDrs(mrs_uav_unreal_simulation::unreal_simulatorConfig& config, [[maybe_unused]] uint32_t level) {

  {
    // | ----------------- pausing the simulation ----------------- |

    auto old_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);

    if (!old_params.paused && config.paused) {
      timer_main_.stop();
    } else if (old_params.paused && !config.paused) {
      timer_main_.start();
    }
  }

  // | --------------------- save the params -------------------- |

  {
    std::scoped_lock lock(mutex_drs_params_);

    drs_params_ = config;
  }

  // | ----------------- set the realtime factor ---------------- |

  timer_main_.setPeriod(ros::WallDuration(1.0 / (_simulation_rate_ * config.realtime_factor)), true);

  // | ------------------ set the camera rates ------------------ |

  timer_rgb_.setPeriod(ros::Duration(1.0 / config.rgb_rate));

  timer_lidar_.setPeriod(ros::Duration(1.0 / config.lidar_rate));
  
  
  ueds_connector::CameraConfig cameraConfig{};
  cameraConfig.Width = config.rgb_width;
  cameraConfig.Height = config.rgb_height;
  cameraConfig.angleFOV = config.rgb_fov;
  cameraConfig.offset = ueds_connector::Coordinates(config.rgb_offset_x, config.rgb_offset_y, config.rgb_offset_z);
  cameraConfig.orientation = ueds_connector::Rotation(config.rgb_rotation_pitch, config.rgb_rotation_roll, config.rgb_rotation_yaw);
  for (size_t i = 0; i < uavs_.size(); i++) {
    const auto res = ueds_connectors_[i]->SetCameraConfig(cameraConfig);
    if (!res) {
      ROS_ERROR("[UnrealSimulator]: failed to set camera config for uav %d", i);
    } 
  }

  ueds_connector::LidarConfig lidarConfig{};
  lidarConfig.BeamHorRays = config.lidar_horizontal_rays;
  lidarConfig.BeamVertRays = config.lidar_vertical_rays;
  lidarConfig.FOVHor = config.lidar_horizontal_fov;
  lidarConfig.FOVVert = config.lidar_vertical_fov;
  lidarConfig.beamLength = config.lidar_beam_length;
  lidarConfig.offset = ueds_connector::Coordinates(config.lidar_offset_x, config.lidar_offset_y, config.lidar_offset_z);
  lidarConfig.orientation = ueds_connector::Rotation(config.lidar_rotation_pitch, config.lidar_rotation_roll, config.lidar_rotation_yaw);
  for (size_t i = 0; i < uavs_.size(); i++) {
    const auto res = ueds_connectors_[i]->SetLidarConfig(lidarConfig);
    if (!res) {
      ROS_ERROR("[UnrealSimulator]: failed to set lidar config for uav %d", i);
    } 
  } 


  ROS_INFO("[UnrealSimulator]: DRS updated params");
}

//}

// | ------------------------ routines ------------------------ |

/* publishPoses() //{ */

void UnrealSimulator::publishPoses(void) {

  auto sim_time = mrs_lib::get_mutexed(mutex_sim_time_, sim_time_);

  geometry_msgs::PoseArray pose_array;

  pose_array.header.stamp    = sim_time;
  pose_array.header.frame_id = _world_frame_name_;

  for (size_t i = 0; i < uavs_.size(); i++) {

    auto state = uavs_[i]->getState();

    geometry_msgs::Pose pose;

    pose.position.x  = state.x[0];
    pose.position.y  = state.x[1];
    pose.position.z  = state.x[2];
    pose.orientation = mrs_lib::AttitudeConverter(state.R);

    pose_array.poses.push_back(pose);
  }

  ph_poses_.publish(pose_array);
}

//}

/* updateUnrealPoses() //{ */

void UnrealSimulator::updateUnrealPoses(void) {

  // | ------------ set each UAV's position in unreal ----------- |

  {
    std::scoped_lock lock(mutex_ueds_);

    for (size_t i = 0; i < uavs_.size(); i++) {

      mrs_multirotor_simulator::MultirotorModel::State state = uavs_[i]->getState();

      auto [roll, pitch, yaw] = mrs_lib::AttitudeConverter(state.R).getExtrinsicRPY();

      ueds_connector::Coordinates pos;

      pos.x = ueds_world_origins_[i].x + state.x.x() * 100.0;
      pos.y = ueds_world_origins_[i].y - state.x.y() * 100.0;
      pos.z = ueds_world_origins_[i].z + state.x.z() * 100.0;

      ueds_connector::Rotation rot;
      rot.pitch = 180.0 * (-pitch / M_PI);
      rot.roll  = 180.0 * (roll / M_PI);
      rot.yaw   = 180.0 * (-yaw / M_PI);

      const auto [res, teleportedTo, rotatedTo, isHit, impactPoint] = ueds_connectors_[i]->SetLocationAndRotation(pos, rot);

      if (isHit) {
        if (!uavs_[i]->hasCrashed()) {
          uavs_[i]->crash();
        }
      }
    }
  }
}

//}

}  // namespace mrs_uav_unreal_simulation

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_unreal_simulation::UnrealSimulator, nodelet::Nodelet)
