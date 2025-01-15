cmake_minimum_required(VERSION 3.8)
project(laser_scan_mapper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(laser_scan_integrator_msg REQUIRED)

add_executable(tf_publisher src/tf_publisher.cpp)
add_executable(mapper src/mapper.cpp)

ament_target_dependencies(
tf_publisher 
rclcpp 
tf2_ros 
geometry_msgs
)

ament_target_dependencies(
mapper 
rclcpp 
tf2_ros 
tf2 
tf2_geometry_msgs 
geometry_msgs
laser_scan_integrator_msg
)


install(TARGETS 
tf_publisher 
mapper 
DESTINATION lib/${PROJECT_NAME}
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
