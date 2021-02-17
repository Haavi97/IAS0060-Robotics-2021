set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

cmake_minimum_required(VERSION 2.8.3)
project(uwsim_arrows)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  tf
  urdf
  message_generation
  kdl_conversions
  visualization_msgs
  uwsim_physics_msgs
  cv_bridge
  image_transport
  uwsim
  uwsim_osgocean
  uwsim_osgworks
  uwsim_osgbullet
  uwsim_bullet
  uwsim_physics
  pluginlib
  std_msgs
  tut_arrows_msgs
  sensor_msgs  
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#  Flippers.msg
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  geometry_msgs
#  std_msgs
#  sim_link
#)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES uwsim_arrows
  CATKIN_DEPENDS uwsim tut_arrows_msgs sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

find_package(uwsim_osgocean REQUIRED)
find_package(uwsim_osgworks REQUIRED)
find_package(uwsim_osgbullet REQUIRED)
find_package(uwsim_bullet REQUIRED)

find_package(PkgConfig)
pkg_check_modules(LIBXML++ REQUIRED libxml++-2.6)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${BULLET_INCLUDE_DIRS} ${OSGWORKS_INCLUDE_DIR} ${OSGBULLET_INCLUDE_DIR} ${OSGOCEAN_INCLUDE_DIRS} ${LIBXML++_INCLUDE_DIRS}
  ${uwsim_INCLUDE_DIRS}
  ${uwsim_physics_INCLUDE_DIRS}
  )
  
link_directories(${orocos_kdl_LIBRARY_DIRS})

add_library(uwsim_arrows
src/SimDev_DirectionalReceiver.cpp
src/SimDev_Flipper.cpp
src/SimDev_VehicleFin.cpp
src/SimDev_Depth2Sonar.cpp
src/SimDev_FluidPressure.cpp
src/SimDev_BeaconReceiver.cpp
src/uwsim_arrows.cpp
)

add_executable(ucat_buoyance_driver
	src/ucat_buoyance_driver.cpp
)

add_executable(imu2tf
	src/imu2tf.cpp
)

add_executable(SetTargetFromRviz
	src/SetTargetFromRviz.cpp
)

add_dependencies(ucat_buoyance_driver uwsim_physics_msgs_gencpp)
add_dependencies(SetTargetFromRviz tut_arrows_msgs_gencpp)

target_link_libraries(uwsim_arrows
	uwsim ${catkin_LIBRARIES}
)

target_link_libraries(ucat_buoyance_driver
	${catkin_LIBRARIES}
)

target_link_libraries(imu2tf
	${catkin_LIBRARIES}
)

target_link_libraries(SetTargetFromRviz
	${catkin_LIBRARIES}
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(uwsim_game_node uwsim_game_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(uwsim_game_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS uwsim_game uwsim_game_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_uwsim_game.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
