cmake_minimum_required(VERSION 3.18)
project(open3d_slam_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Шукаємо компоненти ROS 2 та наше щойно створене ядро
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization filesystem timer system)
find_package(open3d_slam_core REQUIRED) # <--- ОСЬ ВОНО!

# Додаємо твою майбутню ROS 2 ноду (наприклад, open3d_slam_node.cpp)
add_executable(open3d_slam_node src/open3d_slam_node.cpp)

# Додаємо шляхи до локальних хедерів обгортки
target_include_directories(open3d_slam_node PRIVATE include)

# Прив'язуємо наше ядро напряму через створений нами експортний target namespace
target_link_libraries(open3d_slam_node
  open3d_slam_core::open3d_slam_core
)

# Прив'язуємо залежності ROS 2
ament_target_dependencies(open3d_slam_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
)

# Інсталяція виконуваних файлів ROS 2 ноди
install(TARGETS open3d_slam_node
  DESTINATION lib/${PROJECT_NAME}
)

# Інсталяція папок розширень (launch-файли та конфіги)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install Python scripts
install(FILES
  open3d_slam_ros2/evaluate_trajectory.py
  open3d_slam_ros2/plot_trajectory.py
  open3d_slam_ros2/evaluate_and_plot.py
  DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
              GROUP_READ GROUP_EXECUTE
              WORLD_READ WORLD_EXECUTE
)

ament_package()
