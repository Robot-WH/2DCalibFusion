cmake_minimum_required(VERSION 3.0.2)
project(ros_wrapper)
# ## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
# set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3")

# List C++ dependencies on ros packages

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES} )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread signals)
# include_directories()
find_package(PCL REQUIRED QUIET)
# Find Eigen3 (from http://wiki.ros.org/jade/Migration)
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## occupancy_grid
add_executable(occupancy_grid_node
  src/occupancy_grid/occupancy_grid.cc
)
add_dependencies(occupancy_grid_node 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(occupancy_grid_node
  ${catkin_LIBRARIES}
)


## SLAM
add_executable(slam2d_node
  src/slam2d_node.cc
)

add_dependencies(slam2d_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(slam2d_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  yaml-cpp
  ${libnabo_LIBRARIES}
)

