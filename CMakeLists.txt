cmake_minimum_required(VERSION 2.8.3)
project(esdf_planner)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  plan_env
  path_searching
  cv_bridge

  message_filters
)

# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
# INCLUDE_DIRS include
 LIBRARIES esdf_planner
 CATKIN_DEPENDS plan_env #path_searching
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)


add_executable(esdf_planner
  src/esdf_planner.cpp
  )

add_executable(test_synchronization
  src/test_synchronization.cpp
  )

add_executable(exp_sync
  src/sync_example.cpp
  )

target_link_libraries(esdf_planner 
  ${catkin_LIBRARIES}
  )

target_link_libraries(test_synchronization
  ${catkin_LIBRARIES}
  )

target_link_libraries(exp_sync
  ${catkin_LIBRARIES}
  )

#catkin_install_python(PROGRAMS scripts/getDepth.py scripts/depth_img.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)