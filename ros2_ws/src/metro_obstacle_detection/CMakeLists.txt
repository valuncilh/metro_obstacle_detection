cmake_minimum_required(VERSION 3.18)
project(metro_obstacle_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(USE_CUDA "Enable GPU acceleration" OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation search)
find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)

if(USE_CUDA)
  find_package(CUDAToolkit REQUIRED)
  enable_language(CUDA)
  set(CMAKE_CUDA_STANDARD 17)
  set(CMAKE_CUDA_ARCHITECTURES "89")
  message(STATUS "CUDA enabled: ${CUDAToolkit_VERSION}")
endif()

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(cluster_cpu src/cluster_cpu.cpp)
target_link_libraries(cluster_cpu ${PCL_LIBRARIES})

if(USE_CUDA)
  add_library(cluster_gpu src/cluster_gpu.cu)
  set_target_properties(cluster_gpu PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
  target_include_directories(cluster_gpu PRIVATE
    include
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
  )
  target_link_libraries(cluster_gpu CUDA::cudart)
endif()

add_executable(preprocessor src/preprocessor.cpp)
target_link_libraries(preprocessor ${PCL_LIBRARIES} OpenMP::OpenMP_CXX)
ament_target_dependencies(preprocessor rclcpp sensor_msgs std_msgs pcl_ros pcl_conversions)

add_executable(gauge_filter src/gauge_filter.cpp)
target_link_libraries(gauge_filter ${PCL_LIBRARIES} OpenMP::OpenMP_CXX)
ament_target_dependencies(gauge_filter rclcpp sensor_msgs std_msgs pcl_ros pcl_conversions)

add_executable(anomaly_detector src/anomaly_detector.cpp)
target_link_libraries(anomaly_detector cluster_cpu)
if(USE_CUDA)
  target_link_libraries(anomaly_detector cluster_gpu)
  target_compile_definitions(anomaly_detector PRIVATE USE_CUDA)
endif()
target_link_libraries(anomaly_detector ${PCL_LIBRARIES} OpenMP::OpenMP_CXX)
ament_target_dependencies(anomaly_detector rclcpp sensor_msgs std_msgs visualization_msgs pcl_ros pcl_conversions)

add_executable(decision_maker src/decision_maker.cpp)
ament_target_dependencies(decision_maker rclcpp std_msgs)

add_executable(metrics_logger src/metrics_logger.cpp)
ament_target_dependencies(metrics_logger rclcpp sensor_msgs std_msgs visualization_msgs)

install(TARGETS
  preprocessor gauge_filter anomaly_detector decision_maker metrics_logger
  cluster_cpu
  DESTINATION lib/${PROJECT_NAME}
)

if(USE_CUDA)
  install(TARGETS cluster_gpu LIBRARY DESTINATION lib/${PROJECT_NAME})
endif()

install(DIRECTORY config/ launch/ DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_package()
