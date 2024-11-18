## cmake_minimum_required(VERSION 3.0.2)
## project(maximo_coding_test)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
## find_package(catkin REQUIRED COMPONENTS
##  geometry_msgs
##  nav_msgs
##  roscpp
##  actionlib
##  tf
##  tf2_ros
##  move_base_msgs
##  fiducial_msgs
## )

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   nav_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
## catkin_package(
##  INCLUDE_DIRS include
##  LIBRARIES ${PROJECT_NAME}
##  CATKIN_DEPENDS roscpp
## )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
## include_directories(
##   include
##   ${catkin_INCLUDE_DIRS}
## )

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
## set (EXPLORER_SOURCES 
##  src/explorer_node.cpp
##  src/explorer.cpp
##  src/sar_base.cpp
##  include/explorer.hpp
##  include/sar_base.hpp
## )
## add_executable(explorer_node ${EXPLORER_SOURCES})

## Specify libraries to link a library or executable target against
## target_link_libraries(explorer_node
##   ${catkin_LIBRARIES}
## )









cmake_minimum_required(VERSION 3.0.2)
project(maximo_coding_test)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  actionlib
  tf
  tf2_ros
  move_base_msgs
  fiducial_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
)

set (EXPLORER_SOURCES 
  src/explorer_node.cpp
  src/explorer.cpp
  src/sar_base.cpp
  include/maximo_coding_test/explorer.hpp
  include/maximo_coding_test/sar_base.hpp
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_explorer_node ${EXPLORER_SOURCES})

## Specify libraries to link the executable target against
target_link_libraries(${PROJECT_NAME}_explorer_node
  ${catkin_LIBRARIES}
)






