cmake_minimum_required(VERSION 3.0.2)
project(caric_mission)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rotors_comm
  tcc
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(gazebo REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES caric_mission
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

## Declare a C++ executable
add_executable(${PROJECT_NAME}_mission_manager_node src/MissionManager.cpp)
add_dependencies(${PROJECT_NAME}_mission_manager_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_mission_manager_node
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES} pthread
)
