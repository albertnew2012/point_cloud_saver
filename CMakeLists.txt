cmake_minimum_required(VERSION 3.5)
project(point_cloud_saver)

set(CMAKE_CXX_STANDARD 17)  # to use filesystem
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

# Add executable
add_executable(point_cloud_saver src/point_cloud_saver.cpp)

# Link libraries
ament_target_dependencies(point_cloud_saver rclcpp sensor_msgs pcl_conversions PCL)

target_link_libraries(point_cloud_saver ${PCL_LIBRARIES} stdc++fs)

# Install executable
install(TARGETS
  point_cloud_saver
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files if any
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
