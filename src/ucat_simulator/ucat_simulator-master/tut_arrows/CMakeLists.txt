if(CMAKE_EXTRA_GENERATOR STREQUAL "Eclipse CDT4")
	message("Adding extra definitions for Eclipse")
	set(CMAKE_CXX_COMPILER_ARG1 -std=gnu++0x)
	set(CMAKE_C_COMPILER_ARG1 -std=gnu++0x)
endif()

add_definitions("-std=gnu++0x")

cmake_minimum_required(VERSION 2.8.3)
find_package(OpenCV REQUIRED)
project(tut_arrows)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tut_arrows_msgs
  std_msgs
  tf
  cv_bridge
#  pcl_ros
)

execute_process(
	OUTPUT_VARIABLE auv_msgs_DIR
	COMMAND rospack find auv_msgs
)
string(STRIP "${auv_msgs_DIR}" auv_msgs_DIR)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
#find_package(eigen REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

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
#  INCLUDE_DIRS include
#  LIBRARIES tut_arrows
  CATKIN_DEPENDS roscpp rospy tut_arrows_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${auv_msgs_INCLUDE_DIRS}
  ${auv_msgs_DIR}/msg_gen/cpp/include
)

## Declare a cpp library
# add_library(tut_arrows
#   src/${PROJECT_NAME}/tut_arrows.cpp
# )

## Declare a cpp executable
add_executable(WrenchDriver 
	src/WrenchDriver.cpp
)



add_executable(joy2wrench 
	src/joy2wrench.cpp
)
add_executable(RemoteDriver 
	src/RemoteDriver.cpp
)
#add_executable(ranges2pointcloud 
#	src/ranges2pointcloud.cpp
#)
add_executable(MainControl
	src/MainControl.cpp
	src/ObstacleAvoidance.cpp
	src/QuatUtils.cpp
	src/TrajectoryUtils.cpp
	src/UcatDynamics.cpp
)
add_executable(MotionEstimator
	src/MotionEstimator.cpp
	src/ekf.cpp
	src/UcatDynamics.cpp
	src/CppNumericalSolvers/Meta.cpp
	src/CppNumericalSolvers/ISolver.cpp
	src/CppNumericalSolvers/BfgsSolver.cpp
	src/QuatUtils.cpp
)
add_executable(WrenchToAcousticMsg 
	src/WrenchToAcousticMsg.cpp
)
add_executable(coordinate_converter 
	src/coordinateConverter.cpp
)
add_executable(ModemRangeDriver
	src/ModemRangeDriver.cpp
)
add_executable(TrajectoryManager
	src/Target.cpp
	src/Trajectory.cpp
	src/QuatUtils.cpp
	src/TrajectoryUtils.cpp
)
add_executable(Explorer
	src/ExplorerController.cpp
	)
add_executable(ModemPositionPublisher
	src/ModemPositionPublisher.cpp
	)
add_executable(RemoteController
	src/RemoteController.cpp
	src/TrajectoryUtils.cpp
	src/QuatUtils.cpp
	)
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(WrenchDriver tut_arrows_msgs_gencpp)
add_dependencies(joy2wrench tut_arrows_msgs_gencpp)
add_dependencies(RemoteDriver tut_arrows_msgs_gencpp)
add_dependencies(MainControl tut_arrows_msgs_gencpp)
add_dependencies(WrenchToAcousticMsg tut_arrows_msgs_gencpp)
add_dependencies(ModemRangeDriver tut_arrows_msgs_gencpp)
add_dependencies(TrajectoryManager tut_arrows_msgs_gencpp)
add_dependencies(ModemPositionPublisher tut_arrows_msgs_gencpp)
add_dependencies(RemoteController tut_arrows_msgs_gencpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(WrenchDriver
   ${catkin_LIBRARIES}
 )

 target_link_libraries(joy2wrench
   ${catkin_LIBRARIES}
 )
 target_link_libraries(RemoteDriver
   ${catkin_LIBRARIES}
 )
# target_link_libraries(ranges2pointcloud
#   ${catkin_LIBRARIES}
# )
 target_link_libraries(MainControl
   ${catkin_LIBRARIES}
 )
 target_link_libraries(MotionEstimator
   ${catkin_LIBRARIES}
 )
  target_link_libraries(WrenchToAcousticMsg
   ${catkin_LIBRARIES}
 )
 target_link_libraries(coordinate_converter
   ${catkin_LIBRARIES}
   Geographic
 )
 target_link_libraries(ModemRangeDriver
   ${catkin_LIBRARIES}
 )
  target_link_libraries(TrajectoryManager
   ${catkin_LIBRARIES}
 )
 target_link_libraries(Explorer
   ${catkin_LIBRARIES}
 )
  target_link_libraries(ModemPositionPublisher
   ${catkin_LIBRARIES}
 )
 target_link_libraries(RemoteController
   ${catkin_LIBRARIES}
 )
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
# install(TARGETS tut_arrows tut_arrows_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_tut_arrows.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
