#ifndef DRONE_CONTROLLER_ROS_H
#define DRONE_CONTROLLER_ROS_H

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>

#include <uav_system_ros.h>
#include <mrs_lib/attitude_converter.h>
#include <ueds_connector/ueds_connector.h>

#define USE_LIDAR true
#define USE_CAMERA true

//namespace ueds_uav_api
namespace mrs_multirotor_simulator
{

class DroneControllerRos
{
private:
    std::string uav_name_;

    double simulation_rate_;

    bool oneUAVsim_;

    std::shared_ptr<UavSystemRos> uav_system_;

    ueds_connector::Coordinates ueds_world_frame_;

    std::unique_ptr<ueds_connector::DroneController> drone_controller_;

    ros::Timer ueds_timer_;
    ros::Publisher lidar_pub_;
    ros::Publisher camera_pub_;
public:
    DroneControllerRos();
    DroneControllerRos(ros::NodeHandle& nh, double simulation_rate, std::shared_ptr<UavSystemRos> uav_system, const std::string name, int port, bool oneUAVsim);

    ~DroneControllerRos();

    std::string getName();

    void uedsTimer();

    void uedsSetPositionAndRotation();

    void uedsPublishLidar();

    void uedsPubllishCamera();
    
};


    
} // namespace name

#endif
