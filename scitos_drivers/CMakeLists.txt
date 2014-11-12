cmake_minimum_required(VERSION 2.8.3)
project(scitos_drivers)
find_package(catkin REQUIRED)
#catkin_metapackage()
catkin_package()

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

