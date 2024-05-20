cmake_minimum_required(VERSION 3.0.2)
project(24_localization)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ransac
#  CATKIN_DEPENDS pcl pcl_converisons roscpp sensor_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(plane_ex src/plane_extract.cpp)
target_link_libraries(plane_ex ${catkin_LIBRARIES})
