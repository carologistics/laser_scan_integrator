cmake_minimum_required(VERSION 3.5)
project(laser_scan_integrator)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rcutils REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(laser_scan_integrator_msg REQUIRED)
find_package(visualization_msgs REQUIRED)


# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Executable
add_executable(laser_scan_integrator src/main.cpp)
ament_target_dependencies(laser_scan_integrator
  rclcpp
  std_msgs
  sensor_msgs
  tf2_sensor_msgs
  rcl_interfaces
  rcutils
  tf2_geometry_msgs
  tf2
  tf2_ros
  geometry_msgs
  Boost
  PCL
  pcl_conversions
  laser_scan_integrator_msg
  visualization_msgs
)

# Link PCL librariesrosidl_generate_interfaces
target_link_libraries(laser_scan_integrator ${PCL_LIBRARIES})


# Export dependencies
ament_export_dependencies(laser_scan_integrator_msg)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install executable
install(TARGETS
  laser_scan_integrator
  DESTINATION lib/${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
