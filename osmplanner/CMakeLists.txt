cmake_minimum_required(VERSION 3.5)
project(osmplanner)

set(CMAKE_CXX_STANDARD 14) # 或更高版本

set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
  pcl_ros
  cv_bridge
  tf

)
# set(PCL_DIR "/usr/local/include/pcl-1.13")
find_package(PCL REQUIRED)
# find_package(PCL 1.13.1 REQUIRED)
find_package(Eigen REQUIRED)
find_package(octomap REQUIRED)
find_package(OpenCV REQUIRED)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES gw_tools
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
# INCLUDE_DIRS
)

include_directories(
  # include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /usr/include/opencv4
)


add_library(osmPlannerLib  src/grid_gw.cpp)
# add_dependencies(osmPlannerLib ${PROJECT_NAME}_EXPORTED_TARGETS )
target_link_libraries(osmPlannerLib ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})


add_executable(osm_grid src/grid_gw.cpp)
target_link_libraries(osm_grid ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


add_executable(osm_waypoint src/osmPlanner_gw.cpp)
target_link_libraries(osm_waypoint ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
