cmake_minimum_required(VERSION 3.12)
project(control_interface)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions("-Wall")
add_definitions("-Wextra")
add_definitions("-Wpedantic")
add_definitions("-g")
add_definitions("-O3")
# add_definitions(-fsanitize=address,undefined,alignment,bounds)
# add_link_options(-fsanitize=address,undefined,alignment,bounds)
if ("$ENV{ROS_DISTRO}" STREQUAL "foxy") # to select the correct version of declare_parameter() and avoid errors/warnings
  add_definitions("-DROS_FOXY")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(fog_msgs 0.0.8 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(MAVSDK 0.42.0 EXACT REQUIRED)
find_package(Threads REQUIRED)

find_package(fog_lib REQUIRED)

include_directories(
  include
)

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
  rcl_interfaces
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
  fog_lib
  )

target_link_libraries(control_interface
  MAVSDK::mavsdk_action
  MAVSDK::mavsdk_mission
  MAVSDK::mavsdk_telemetry
  MAVSDK::mavsdk_param
  MAVSDK::mavsdk
  )

rclcpp_components_register_nodes(control_interface PLUGIN "${PROJECT_NAME}::ControlInterface" EXECUTABLE control_interface)

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

ament_export_include_directories(include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config 
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY scripts
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  control_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
