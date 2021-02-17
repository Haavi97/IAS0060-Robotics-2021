set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

cmake_minimum_required(VERSION 2.8.3)
project(uwsim_physics)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  uwsim_physics_msgs
  uwsim
  uwsim_osgocean
  uwsim_osgworks
  uwsim_osgbullet
  uwsim_bullet
  pluginlib
  message_generation
  std_msgs
)


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
  LIBRARIES
  uwsim_physics
#  INCLUDE_DIRS include
#  LIBRARIES uwsim_game
#  CATKIN_DEPENDS geometry_msgs kdl roscpp rospy tf urdf
#  DEPENDS system_lib
)

###########
## Build ##
###########

#get_cmake_property(_variableNames VARIABLES)
#foreach (_variableName ${_variableNames})
#    #message(STATUS "${_variableName}=${${_variableName}}")
#message(STATUS "${_variableName}")
#endforeach()

find_package(Boost REQUIRED thread program_options system filesystem)
find_package(uwsim_osgocean REQUIRED)
find_package(uwsim_osgworks REQUIRED)
find_package(uwsim_osgbullet REQUIRED)
find_package(uwsim_bullet REQUIRED)

find_package(PkgConfig)
pkg_check_modules(LIBXML++ REQUIRED libxml++-2.6)

include_directories(
  include
  ${BULLET_INCLUDE_DIRS} ${OSGWORKS_INCLUDE_DIR} ${OSGBULLET_INCLUDE_DIR} ${OSGOCEAN_INCLUDE_DIRS} ${LIBXML++_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
  ${uwsim_INCLUDE_DIRS}
)

link_directories(${orocos_kdl_LIBRARY_DIRS})

add_library(uwsim_physics 
src/SimDev_VehiclePhysics.cpp
src/SimDev_VehiclePhysicsForce.cpp
src/SimDev_VehiclePhysicsForces.cpp
#src/SimDev_VehicleFin.cpp
#src/SimDev_DirectionalReceiver.cpp
#src/SimDev_SkeletonFin.cpp
#src/SimDev_PassiveFin.cpp
#src/SimDev_GameCamera.cpp
#src/SimDev_Game2D.cpp
#src/SimDev_Flipper.cpp
src/uwsim_physics.cpp
)

add_dependencies(uwsim_physics uwsim_physics_gencpp)

target_link_libraries(uwsim_physics 
uwsim ${catkin_LIBRARIES}
)

#####################################################################
#pkg_check_modules(LIBXML++ REQUIRED libxml++-2.6)
#INCLUDE_DIRECTORIES(${UWSIM_INCLUDE_DIRS} ${LIBXML++_INCLUDE_DIRS})
#LINK_DIRECTORIES(${UWSIM_LIBRARY_DIRS} ${LIBXML++_LIBRARY_DIRS})
#TARGET_LINK_LIBRARIES(uwsim_physics ${LIBXML++_LIBRARIES})
#####################################################################


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(uwsim_game_node uwsim_game_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(uwsim_game_node
#   ${catkin_LIBRARIES}
# )
