#include <drone_controller_ros.h>

// namespace ueds_uav_api
namespace mrs_multirotor_simulator
{

/* constructor DroneControllerRos() //{ */

DroneControllerRos::DroneControllerRos(ros::NodeHandle &nh, double simulation_rate, std::shared_ptr<UavSystemRos> uav_system, const std::string uav_name,
                                       int port, bool oneUAVsim) {

  this->uav_system_      = uav_system;
  this->uav_name_        = uav_name;
  this->oneUAVsim_       = oneUAVsim;
  this->simulation_rate_ = simulation_rate;

  drone_controller_ = std::make_unique<ueds_connector::DroneController>(LOCALHOST, port);

  auto connect_result = drone_controller_->Connect();

  if (connect_result != 1) {

    ROS_ERROR("%s - Error connecting to drone controller. connect_result was %d", uav_name_.c_str(), connect_result);
    ros::shutdown();

  } else {

    ROS_INFO("%s - Connection succeed: %d", uav_name_.c_str(), connect_result);

    const auto [res, location] = drone_controller_->GetLocation();

    if (!res) {
      ROS_ERROR("%s - DroneError: getting location", uav_name_.c_str());
      ros::shutdown();
    } else {
      ueds_world_frame_ = location;
      ROS_INFO("ueds_world_frame_: x[%lf] y[%lf] z[%lf]", ueds_world_frame_.x, ueds_world_frame_.y, ueds_world_frame_.z);
    }
  }

  ueds_timer_ = nh.createTimer(ros::Duration(1.0 / simulation_rate_), std::bind(&DroneControllerRos::uedsTimer, this));

  if (USE_LIDAR) {
    lidar_pub_ = nh.advertise<sensor_msgs::PointCloud2>("ueds/" + uav_name_ + "/lidar", 10);
  }

  if (USE_CAMERA) {
    camera_pub_ = nh.advertise<sensor_msgs::CompressedImage>("ueds/" + uav_name_ + "/camera/image/compressed", 10);
  }
}

//}

/* getName() //{ */

std::string DroneControllerRos::getName() {
  return uav_name_;
}

//}

/* uedsTimer() //{ */

void DroneControllerRos::uedsTimer() {

  uedsSetPositionAndRotation();

  if (USE_LIDAR) {
    uedsPublishLidar();
  }

  if (USE_CAMERA) {
    uedsPubllishCamera();
  }
}

//}

/* uedsSetPositionAndRotation() //{ */

void DroneControllerRos::uedsSetPositionAndRotation() {

  MultirotorModel::State state = uav_system_->getState();

  // ROS_WARN("uavPose: %lf %lf %lf", state.x.x(), state.x.y(), state.x.z());


  auto [roll, pitch, yaw] = mrs_lib::AttitudeConverter(state.R).getExtrinsicRPY();

  const auto [res, teleportedTo, rotatedTo, isHit, impactPoint] = drone_controller_->SetLocationAndRotation(
      ueds_connector::Coordinates(ueds_world_frame_.x + state.x.x() * 100, ueds_world_frame_.y - state.x.y() * 100, ueds_world_frame_.z + state.x.z() * 100),
      ueds_connector::Rotation(-pitch / M_PI * 180, -yaw / M_PI * 180, roll / M_PI * 180));

  if (isHit) {
    if (!uav_system_->hasCrashed()) {
      uav_system_->crash();
    }
  }
}

//}

/* uedsPublishLidar() //{ */

void DroneControllerRos::uedsPublishLidar() {

  const auto [res, lidarData, start] = drone_controller_->GetLidarData();

  if (!res) {
    ROS_ERROR("Unreal: [%s] - ERROR getLidarData", uav_name_.c_str());
  } else {
    // ROS_INFO("Lidar Start: x[%lf] y[%lf] z[%lf]",start.x, start.y, start.z);
    sensor_msgs::PointCloud2 pcl_msg;

    // Modifier to describe what the fields are.
    sensor_msgs::PointCloud2Modifier modifier(pcl_msg);
    modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1, sensor_msgs::PointField::FLOAT32, "z", 1,
                                  sensor_msgs::PointField::FLOAT32, "intensity", 1, sensor_msgs::PointField::FLOAT32);
    // Msg header
    pcl_msg.header       = std_msgs::Header();
    pcl_msg.header.stamp = ros::Time::now();

    if (oneUAVsim_) {
      pcl_msg.header.frame_id = uav_name_ + "/world_origin";
    } else {
      pcl_msg.header.frame_id = "simulator_origin";
    }


    pcl_msg.height   = 1;                 // unordered 1D data array points cloud
    pcl_msg.width    = lidarData.size();  // 360; //num_of_points
    pcl_msg.is_dense = true;
    // Total number of bytes per point
    pcl_msg.point_step = 16;
    pcl_msg.row_step   = pcl_msg.point_step * pcl_msg.width;
    pcl_msg.data.resize(pcl_msg.row_step);
    // Iterators for PointCloud msg
    sensor_msgs::PointCloud2Iterator<float> iterX(pcl_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iterY(pcl_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iterZ(pcl_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iterIntensity(pcl_msg, "intensity");

    for (const ueds_connector::LidarData &i : lidarData) {
      tf::Vector3 dir = tf::Vector3(i.directionX, i.directionY, i.directionZ);
      dir             = dir.normalize() * (i.distance / 100.0);
      tf::Vector3 lidarTransform =
          tf::Vector3((start.x - ueds_world_frame_.x) / 100, (start.y - ueds_world_frame_.y) / 100, (start.z - ueds_world_frame_.z) / 100);
      *iterX         = lidarTransform.x() + dir.x();
      *iterY         = -lidarTransform.y() - dir.y();  // convert left-hand to right-hand coordinates
      *iterZ         = lidarTransform.z() + dir.z();
      *iterIntensity = i.distance;
      // ROS_WARN("UEDworldStart: %lf %lf %lf", ueds_world_frame_.x, ueds_world_frame_.y, ueds_world_frame_.z);
      // ROS_WARN("UEDstartStart: %lf %lf %lf", start.x, start.y, start.z);
      // ROS_WARN("UEDSlidarStart: %lf %lf %lf", lidarTransform.x(), lidarTransform.y(), lidarTransform.z());
      // ROS_WARN("lidarStart: %f %f %f", *iterX, *iterY, *iterZ);
      ++iterX;
      ++iterY;
      ++iterZ;
      ++iterIntensity;
    }
    // *iterX = 10;
    // *iterY = 0; //convert left-hand to right-hand coordinates
    // *iterZ = 0;
    // *iterIntensity = 10;
    lidar_pub_.publish(pcl_msg);
  }
}

//}

/* uedsPublishCamera() //{ */

void DroneControllerRos::uedsPubllishCamera() {

  const auto [res, cameraData, size] = drone_controller_->GetCameraData();

  if (true) {
    // ROS_WARN("Unreal: send camera msg");
    sensor_msgs::CompressedImage img_msg;
    img_msg.header.stamp = ros::Time::now();
    img_msg.format       = "jpeg";

    img_msg.data = cameraData;

    camera_pub_.publish(img_msg);
  } else {
    ROS_ERROR("Unreal: [%s] can not send camera msg", uav_name_.c_str());
  }
}

//}

}  // namespace mrs_multirotor_simulator
