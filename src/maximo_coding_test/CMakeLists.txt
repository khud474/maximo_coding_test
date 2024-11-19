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
  message_generation
)

find_package(Eigen3 REQUIRED)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  FollowerGoals.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
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

set (FOLLOWER_SOURCES 
  src/follower_node.cpp
  src/follower.cpp
  src/sar_base.cpp
  include/maximo_coding_test/follower.hpp
  include/maximo_coding_test/sar_base.hpp
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_explorer_node ${EXPLORER_SOURCES})
add_executable(${PROJECT_NAME}_follower_node ${FOLLOWER_SOURCES})

## Specify libraries to link the executable target against
target_link_libraries(${PROJECT_NAME}_explorer_node
  ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_follower_node
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)






