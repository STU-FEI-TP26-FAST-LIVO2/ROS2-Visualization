cmake_minimum_required(VERSION 3.8)
project(pointcloud_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

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

add_executable(pointcloud_publisher_node
  src/pointcloud_publisher.cpp)

ament_target_dependencies(
  pointcloud_publisher_node
  rclcpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)


add_executable(camera_publisher_node src/camera_publisher.cpp)

ament_target_dependencies(camera_publisher_node
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(camera_publisher_node
  ${OpenCV_LIBRARIES}
)

add_executable(map_builder_node
  src/map_builder.cpp
)

ament_target_dependencies(
  map_builder_node
  rclcpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

target_include_directories(map_builder_node PUBLIC
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(map_builder_node
  ${PCL_LIBRARIES}
)

add_executable(system_controller_node
  src/system_controller.cpp
)

ament_target_dependencies(
  system_controller_node
  rclcpp
  std_msgs
  std_srvs
)

install(TARGETS
  pointcloud_publisher_node
  camera_publisher_node
  map_builder_node
  system_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
