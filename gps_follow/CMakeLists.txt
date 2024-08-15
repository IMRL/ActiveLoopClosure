cmake_minimum_required(VERSION 3.5)

project(gps_follow)

set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set( CMAKE_CXX_FLAGS "-std=c++14 " )
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  sensor_msgs
  geometry_msgs
  nav_msgs
  pcl_ros
  livox_ros_driver2
  message_generation
)

add_message_files(
  FILES
  NavSatFixArray.msg # 自定义消息文件
)

add_service_files(
  FILES
  gpsTomap.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gps_follow
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)
# catkin_install_python(PROGRAMS scripts/osm_ros_gw.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
add_executable(ini_gps src/waypoint_gps.cpp)
target_link_libraries(ini_gps ${catkin_LIBRARIES})

add_executable(active_loop_laplacian src/active_loop_laplacian.cpp src/rppSolver.cpp)
target_link_libraries(active_loop_laplacian ${catkin_LIBRARIES})

add_executable(laplacian_graph src/laplacian_graph.cpp)
target_link_libraries(laplacian_graph ${catkin_LIBRARIES})

# catkin_package(
#   CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
#   DEPENDS 
#   INCLUDE_DIRS
# )
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

