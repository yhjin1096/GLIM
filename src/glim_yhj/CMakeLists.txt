cmake_minimum_required(VERSION 3.0.2)
project(glim_yhj)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  pcl_ros
  tf
)

find_package(OpenCV)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES glim_yhj
 CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs pcl_ros tf
 DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/main.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)