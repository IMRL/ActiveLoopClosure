cmake_minimum_required(VERSION 3.0.2)
project(terrain_rf)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 ")
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  std_msgs
  cv_bridge
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
# find_package(MLPACK REQUIRED)
# find_package(Armadillo REQUIRED)
# find_package(Boost COMPONENTS filesystem serialization program_options REQUIRED)

find_package(OpenMP REQUIRED)
# Add OpenMP flags if found
if(OPENMP_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES extract_pcl_feature
#  CATKIN_DEPENDS pcl_ros roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


add_executable(terrain_rf src/terrain.cpp)
target_link_libraries(terrain_rf ${catkin_LIBRARIES} ${PCL_LIBRARIES} )