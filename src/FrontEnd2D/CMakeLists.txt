cmake_minimum_required(VERSION 3.0.2)
project(FrontEnd2D)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
# set(CMAKE_FIND_DEBUG_MODE TRUE)    # cmake find_package() 搜索路径输出  

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread signals)
# include_directories()
find_package(PCL REQUIRED)

# Find Eigen3 (from http://wiki.ros.org/jade/Migration)
find_package(Eigen3 REQUIRED)    # 选择Eigen的最低版本，这里要求最低3.4 ,会去查找3.4版本的Eigen3Config.cmake文件
# if(NOT EIGEN3_FOUND)
#   # Fallback to cmake_modules
#   find_package(cmake_modules REQUIRED)
#   find_package(Eigen REQUIRED)
#   set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
#   set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
#   # Possibly map additional variables to the EIGEN3_ prefix.
# else()
if(EIGEN3_FOUND)
  message("Find Eigen, version: " ${EIGEN3_VERSION_STRING})
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
# find_package(libpointmatcher 1.1.0 REQUIRED)
# include_directories("${libpointmatcher_INCLUDE_DIRS}")
# message(STATUS "Using libpointmatcher version ${libpointmatcher_VERSION}")
# 若过使用sudo apt install libeigen3-dev安装的eigen, 那么${EIGEN3_INCLUDE_DIRS} = /usr/include/eigen3

# message("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%EIGEN3_INCLUDE_DIRS: " ${EIGEN3_INCLUDE_DIRS})
###########
## Build ##
###########
set(FrontEnd2D_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include" CACHE PATH 
    "project include location" FORCE)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${PCL_INCLUDE_DIRS}   # 包含 eigen3的路径  /usr/local/include/eigen3
  ${EIGEN3_INCLUDE_DIRS}  
  ${Boost_INCLUDE_DIRS}
  ${MSALib_INCLUDE_DIR}
)

# message("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})

# message("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Boost_INCLUDE_DIRS:" ${Boost_INCLUDE_DIRS})

# message("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%libnabo_INCLUDE_DIRS:" ${libnabo_INCLUDE_DIRS})
add_library(Estimator2D SHARED 
  src/estimator.cpp) 
  
target_link_libraries(Estimator2D PUBLIC
  yaml-cpp
  msa2d_lib
)

  #############
  ## INSTALL ##
  #############
  install(
    TARGETS
      Estimator2D
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

  # # install headers
  # install(DIRECTORY include/FrontEnd2D
  #         DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
