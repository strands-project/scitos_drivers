cmake_minimum_required(VERSION 2.8.3)
project(scitos_bringup)

find_package(catkin REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES scitos_bringup
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
