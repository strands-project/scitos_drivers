cmake_minimum_required(VERSION 2.8.3)
project(scitos_pc_monitor)

find_package(catkin REQUIRED COMPONENTS diagnostic_msgs rospy std_msgs)

catkin_python_setup()


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES scitos_pc_monitor
#  CATKIN_DEPENDS diagnostic_msgs rospy std_msgs
  DEPENDS lm-sensors sysstat
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/pc_monitor
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_scitos_pc_monitor.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
