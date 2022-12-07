cmake_minimum_required(VERSION 3.0.2)
project(calib_fusion_2d)
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


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   nav_msgs#   sensor_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
 INCLUDE_DIRS include
#  LIBRARIES lesson4
 CATKIN_DEPENDS ${ROS_CXX_DEPENDENCIES}
#  DEPENDS system_lib
)

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
add_executable(${PROJECT_NAME}_occupancy_grid_node
  src/occupancy_grid/occupancy_grid.cc
)
add_dependencies(${PROJECT_NAME}_occupancy_grid_node 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_occupancy_grid_node
  ${catkin_LIBRARIES}
)

## Hector map
add_executable(${PROJECT_NAME}_hector_mapping_node
  src/hector_mapping/hector_mapping.cc
)

add_dependencies(${PROJECT_NAME}_hector_mapping_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_hector_mapping_node
  ${catkin_LIBRARIES}
  # ${Boost_LIBRARIES}
)

## Hector SLAM
add_executable(${PROJECT_NAME}_hector_slam_node
  src/hector_mapping/hector_slam.cc

)

add_dependencies(${PROJECT_NAME}_hector_slam_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_hector_slam_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  yaml-cpp
)


add_executable(preprocess_node
  src/preprocess/laser_preprocess_node.cpp
)

add_dependencies(preprocess_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(preprocess_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARY_DIRS}
)

add_executable(optimization_fusion_odometry_node
  src/sensor_fusion/optimization_fusion_odometry_node.cpp
)

add_dependencies(optimization_fusion_odometry_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(optimization_fusion_odometry_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARY_DIRS}
)

# add_executable(filter_fusion_odometry_node
#   src/sensor_fusion/filter_fusion_odometry_node.cpp
# )

# add_dependencies(filter_fusion_odometry_node
#   ${catkin_EXPORTED_TARGETS}
# )

# target_link_libraries(filter_fusion_odometry_node
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   ${PCL_LIBRARY_DIRS}
# )

add_executable(simple_fusion_node
  src/sensor_fusion/simple_fusion_node.cpp
)

add_dependencies(simple_fusion_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(simple_fusion_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARY_DIRS}
)

add_executable(test_rot_2d
  src/test/test_rot_2d.cpp
)

target_link_libraries(test_rot_2d
)

add_executable(test_data_dispatcher
  src/test/test_data_dispatcher.cpp
)

target_link_libraries(test_data_dispatcher
  ${catkin_LIBRARIES}
)
