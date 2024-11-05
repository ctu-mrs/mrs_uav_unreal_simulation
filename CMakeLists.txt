cmake_minimum_required(VERSION 3.5)
project(mrs_uav_unreal_simulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall)
add_definitions(-Wextra)

set(CATKIN_DEPENDENCIES
  cmake_modules
  dynamic_reconfigure
  geometry_msgs
  mrs_lib
  mrs_msgs
  mrs_uav_hw_api
  nav_msgs
  nodelet
  roscpp
  rosgraph_msgs
  sensor_msgs
  std_msgs
  std_srvs
  mrs_multirotor_simulator
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

generate_dynamic_reconfigure_options(
  cfg/unreal_simulator.cfg
  )

set(LIBRARIES
  MrsUavUnrealSimulation_Simulator
  MrsUavUnrealSimulation_HwApiPlugin
  MrsUavUnrealSimulation_UedsConnector
  )

catkin_package(
  LIBRARIES ${LIBRARIES}
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  )

## | --------------------- UEDS connector --------------------- |

add_library(MrsUavUnrealSimulation_UedsConnector
  src/flight_forge_connector/socket_client.cpp
  src/flight_forge_connector/flight_forge_connector.cpp
  src/flight_forge_connector/game_mode_controller.cpp
  )

## | ------------------------ Simulator ----------------------- |

add_library(MrsUavUnrealSimulation_Simulator
  src/unreal_simulator.cpp
  )

add_dependencies(MrsUavUnrealSimulation_Simulator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsUavUnrealSimulation_Simulator
  MrsUavUnrealSimulation_UedsConnector
  ${LIB_UEDS_CONNECTOR}
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

## | ---------------------- HW Api Plugin --------------------- |

add_library(MrsUavUnrealSimulation_HwApiPlugin
  src/hw_api_plugin.cpp
  )

add_dependencies(MrsUavUnrealSimulation_HwApiPlugin
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsUavUnrealSimulation_HwApiPlugin
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY tmux/
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/tmux
  )

install(DIRECTORY scripts/
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY scripts
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(FILES nodelets.xml plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
