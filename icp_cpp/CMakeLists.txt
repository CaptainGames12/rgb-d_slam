cmake_minimum_required(VERSION 3.8)
project(icp_cpp)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

add_executable(icp_node src/icp_node.cpp)

ament_target_dependencies(icp_node
  rclcpp
  sensor_msgs
  geometry_msgs
  pcl_ros
  pcl_conversions
  cv_bridge
)

target_link_libraries(icp_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  Eigen3::Eigen
)

install(TARGETS icp_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()