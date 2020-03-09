cmake_minimum_required(VERSION 2.8.3)
project(anymal_sim_cdt)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf_conversions
  eigen_conversions
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES anymal_sim_drs
  CATKIN_DEPENDS
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  goal_placer_node
  src/goal_placer_node.cpp
  src/goal_placer.cpp
)

target_link_libraries(
  goal_placer_node
  ${catkin_LIBRARIES}
)

install(
  TARGETS goal_placer_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
