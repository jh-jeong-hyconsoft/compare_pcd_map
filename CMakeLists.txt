cmake_minimum_required(VERSION 3.8)
project(pcd_compare)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io registration)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(pcd_map_publisher src/pcd_map_publisher.cpp)
target_include_directories(pcd_map_publisher PRIVATE include ${PCL_INCLUDE_DIRS})
target_compile_definitions(pcd_map_publisher PRIVATE ${PCL_DEFINITIONS})
ament_target_dependencies(
  pcd_map_publisher ament_index_cpp rclcpp sensor_msgs pcl_conversions)
target_link_libraries(pcd_map_publisher ${PCL_LIBRARIES})

install(TARGETS pcd_map_publisher DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config data DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)

ament_package()
