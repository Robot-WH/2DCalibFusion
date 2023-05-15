cmake_minimum_required(VERSION 3.0.2)
project(calib_fusion_2d)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# List C++ dependencies on ros packages
set( ROS_CXX_DEPENDENCIES
  nav_msgs
  roscpp
  sensor_msgs
  tf
  tf_conversions
  laser_geometry
)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES} )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS 
 CATKIN_DEPENDS ${ROS_CXX_DEPENDENCIES}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

# message("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%catkin_INCLUDE_DIRS:" ${catkin_INCLUDE_DIRS})

# add_subdirectory(src/Lib/libpointmatcher-master)
add_subdirectory(src/FrontEnd2D)
add_subdirectory(src/ros_wrapper)

include_directories(
  ${FrontEnd2D_INCLUDE_DIR}
)

add_executable(test_rot_2d
  src/test/test_rot_2d.cpp
)
target_include_directories(test_rot_2d PRIVATE
  ${catkin_INCLUDE_DIRS}    # 包含 /usr/include/eigen3 
)

add_executable(test_data_dispatcher
  src/test/test_data_dispatcher.cpp
)

target_link_libraries(test_data_dispatcher
  ${catkin_LIBRARIES}
)

  #############
  ## INSTALL ##
  #############

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
