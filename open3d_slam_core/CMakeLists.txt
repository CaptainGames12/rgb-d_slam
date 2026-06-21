cmake_minimum_required(VERSION 3.18)
project(open3d_slam_core)

# Встановлюємо стандарт C++ (необхідний для сучасного Open3D та ROS 2)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)

# Шукаємо залежності ядра
find_package(ament_cmake REQUIRED) # Використовуємо суто для експорту пакету в colcon
find_package(Open3D REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization filesystem system)

# Шукаємо системні залежності для інтегрованого Lua-парсера
find_package(Lua REQUIRED)
find_package(glog REQUIRED)

# Збираємо список усіх файлів строго за скріншотом твоєї папки src/
set(SRC_FILES
  src/AdjacencyMatrix.cpp
  src/CloudRegistration.cpp
  src/LuaLoader.cpp
  src/Mapper.cpp
  src/MotionCompensation.cpp
  src/Odometry.cpp
  src/OptimizationProblem.cpp
  src/PlaceRecognition.cpp
  src/ScanToMapRegistration.cpp
  src/SlamWrapper.cpp
  src/Submap.cpp
  src/SubmapCollection.cpp
  src/Transform.cpp
  src/TransformInterpolationBuffer.cpp
  src/Voxel.cpp
  src/VoxelHashMap.cpp
  src/configuration_file_resolver.cc
  src/constraint_builders.cpp
  src/croppers.cpp
  src/helpers.cpp
  src/lua_parameter_dictionary.cc
  src/math.cpp
  src/output.cpp
  src/parameter_loaders.cpp
  src/time.cpp
)

# Створюємо динамічну бібліотеку (.so)
add_library(${PROJECT_NAME} SHARED ${SRC_FILES})

# Налаштовуємо папки з хедер-файлами (для збірки та для інсталяції)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${LUA_INCLUDE_DIR}
)

# Лінкуємо математичні ліби та нові системні бібліотеки Lua/glog до нашого ядра
target_link_libraries(${PROJECT_NAME} PUBLIC
  Open3D::Open3D
  yaml-cpp
  Boost::serialization
  Boost::filesystem
  Boost::system
  ${LUA_LIBRARIES}
  glog::glog
)

# --- Магія експорту ROS 2 (щоб пакет open3d_slam_ros2 міг зробити find_package) ---
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(Open3D Eigen3 Boost Lua glog)

# Інсталяція бінарників
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Інсталяція хедерів (папка include)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY
  include/example_param
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
