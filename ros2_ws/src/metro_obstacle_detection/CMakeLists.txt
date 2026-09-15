cmake_minimum_required(VERSION 3.8)
project(metro_obstacle_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation search)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Preprocessor
add_executable(preprocessor src/preprocessor.cpp)
target_link_libraries(preprocessor ${PCL_LIBRARIES})
ament_target_dependencies(preprocessor rclcpp sensor_msgs std_msgs pcl_ros pcl_conversions)

# Gauge filter
add_executable(gauge_filter src/gauge_filter.cpp)
target_link_libraries(gauge_filter ${PCL_LIBRARIES})
ament_target_dependencies(gauge_filter rclcpp sensor_msgs std_msgs pcl_ros pcl_conversions)

# Anomaly detector
add_executable(anomaly_detector src/anomaly_detector.cpp)
target_link_libraries(anomaly_detector ${PCL_LIBRARIES})
ament_target_dependencies(anomaly_detector rclcpp sensor_msgs std_msgs visualization_msgs pcl_ros pcl_conversions)

# Decision maker
add_executable(decision_maker src/decision_maker.cpp)
ament_target_dependencies(decision_maker rclcpp std_msgs)

# Debug logger
add_executable(debug_logger src/debug_logger.cpp)
ament_target_dependencies(debug_logger rclcpp sensor_msgs std_msgs)

install(TARGETS
  preprocessor
  gauge_filter
  anomaly_detector
  decision_maker
  debug_logger
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()
