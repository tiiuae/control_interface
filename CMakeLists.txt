cmake_minimum_required(VERSION 3.5)
project(control_interface)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(fog_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(MAVSDK REQUIRED)
find_package(Threads REQUIRED)

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES})

## --------------------------------------------------------------
## |                       compile                              |
## --------------------------------------------------------------

add_library(control_interface SHARED
  src/control_interface.cpp
  )

target_compile_definitions(control_interface
  PRIVATE "${PROJECT_NAME}_BUILDING_DLL")

ament_target_dependencies(control_interface
  rclcpp
  rclcpp_components
  std_msgs
  nav_msgs
  std_srvs
  fog_msgs
  px4_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  MAVSDK
  Threads
  )

target_link_libraries(control_interface
  MAVSDK::mavsdk_action
  MAVSDK::mavsdk_mission
  MAVSDK::mavsdk_telemetry
  MAVSDK::mavsdk
  )

rclcpp_components_register_nodes(control_interface PLUGIN "${PROJECT_NAME}::ControlInterface" EXECUTABLE control_interface)

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

install(TARGETS
  control_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  )

ament_package()
