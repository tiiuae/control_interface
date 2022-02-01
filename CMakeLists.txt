cmake_minimum_required(VERSION 3.5)
project(control_interface)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions("-Wall")
add_definitions("-Wextra")
add_definitions("-Wpedantic")
add_definitions("-g")
# add_definitions("-O3")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(fog_msgs 0.0.8 EXACT REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(MAVSDK 0.42.0 EXACT REQUIRED)
find_package(Threads REQUIRED)

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
  MAVSDK::mavsdk
  )

rclcpp_components_register_nodes(control_interface PLUGIN "${PROJECT_NAME}::ControlInterface" EXECUTABLE control_interface)

add_library(mission_result_relayer SHARED
  src/control_px4_subscriber.cpp
  )

target_compile_definitions(mission_result_relayer
  PRIVATE "${PROJECT_NAME}_BUILDING_DLL")

ament_target_dependencies(mission_result_relayer
  rclcpp
  rclcpp_components
  std_msgs
  px4_msgs
  )

rclcpp_components_register_nodes(mission_result_relayer PLUGIN "${PROJECT_NAME}::MissionResultRelayer" EXECUTABLE mission_result_relayer)

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

install(TARGETS
  control_interface mission_result_relayer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
