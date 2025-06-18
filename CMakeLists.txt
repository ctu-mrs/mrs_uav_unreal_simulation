cmake_minimum_required(VERSION 3.5)
project(mrs_uav_flightforge_simulation)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(USE_ROS_TIMER 0)

if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[mrs_uav_managers]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

if(ENABLE_COVERAGE)
  message(WARNING "building for coverage, the performance might be limited")

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs")

  add_definitions("-O1")
  add_definitions("-g")
endif()

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  mrs_msgs
  mrs_lib
  pluginlib
  geometry_msgs
  mrs_lib
  mrs_msgs
  mrs_uav_hw_api
  nav_msgs
  rosgraph_msgs
  sensor_msgs
  std_msgs
  std_srvs
  mrs_multirotor_simulator
  image_transport
  cv_bridge
  pcl_ros
  pcl_conversions
  Eigen3
  )


set(LIBRARIES
  MrsUavFlightforgeSimulation_FlightforgeSimulator
  MrsUavFlightforgeSimulation_HwApiPlugin
  MrsUavFlightforgeSimulation_FlightforgeConnector
  )

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

include_directories(
  include
  ${rclpp_INCLUDE_DIRS}
  ${mrs_lib_INCLUDE_DIRS}
  ${mrs_uav_hw_api_INCLUDE_DIRS}
  )


## | --------------------- FlightForge connector --------------------- |

add_library(MrsUavFlightforgeSimulation_FlightforgeConnector
  src/flight_forge_connector/socket_client.cpp
  src/flight_forge_connector/flight_forge_connector.cpp
  src/flight_forge_connector/game_mode_controller.cpp
  )

## | ------------------------ FlightForge Simulator ----------------------- |

add_library(MrsUavFlightforgeSimulation_FlightforgeSimulator
  src/flightforge_simulator.cpp
  )

# add_dependencies(MrsUavFlightforgeSimulation_flightforgeSimulator
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
#   )

target_link_libraries(MrsUavFlightforgeSimulation_FlightforgeSimulator
  MrsUavFlightforgeSimulation_FlightforgeConnector
  ${LIB_FLIGHTFORGE_CONNECTOR}
  ${Eigen_LIBRARIES}
  )

ament_target_dependencies(MrsUavFlightforgeSimulation_FlightforgeSimulator
  rclcpp
  rclcpp_components
  mrs_lib
  sensor_msgs
  std_msgs
  mrs_msgs
  geometry_msgs
  nav_msgs
  image_transport
  cv_bridge
  pcl_ros
  pcl_conversions
)
## | ---------------------- HW Api Plugin --------------------- |

add_library(MrsUavFlightforgeSimulation_HwApiPlugin
  src/hw_api_plugin.cpp
  )

ament_target_dependencies(MrsUavFlightforgeSimulation_HwApiPlugin
  rclcpp
  rclcpp_components
  mrs_lib
  sensor_msgs
  std_msgs
  mrs_msgs
  geometry_msgs
  nav_msgs
  image_transport
  cv_bridge
  pcl_ros
  pcl_conversions
  ${DEPENDENCIES}
)

target_compile_definitions(MrsUavFlightforgeSimulation_HwApiPlugin PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

## --------------------------------------------------------------
## |                       Export plugins                       |
## --------------------------------------------------------------

# <package of the base class>, <relative path to the xml>
pluginlib_export_plugin_description_file(mrs_uav_hw_api plugins.xml)

# add_service_files(DIRECTORY srv/ FILES
#   FILES
#   SetOrientation.srv
# )

# generate_messages(
#   DEPENDENCIES
#   geometry_msgs
#   std_msgs
# )

# generate_dynamic_reconfigure_options(
#   cfg/unreal_simulator.cfg
#   )

# catkin_package(
#   LIBRARIES ${LIBRARIES}
#   INCLUDE_DIRS include
#   CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
#   )

# find_package(rosidl_default_generators REQUIRED)
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "srv/SetOrientation.srv"
# )


## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

ament_export_libraries(
  ${LIBRARIES}
)

install(
  TARGETS ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include
  DESTINATION .
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY tmux
  USE_SOURCE_PERMISSIONS
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
