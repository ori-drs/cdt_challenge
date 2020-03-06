cmake_minimum_required(VERSION 2.8.3)
project(anymal_elevation_mapping_cdt)

find_package(catkin REQUIRED
  COMPONENTS
    elevation_mapping
)

catkin_package(
  CATKIN_DEPENDS
    elevation_mapping
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
