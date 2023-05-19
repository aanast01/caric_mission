cmake_minimum_required(VERSION 3.0.2)
project(caric_mission)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17 -Wfatal-errors")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  rotors_comm
  tcc
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(gazebo REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES caric_mission
 CATKIN_DEPENDS roscpp rospy std_msgs
 DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
# list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

## Declare a C++ executable
add_executable(${PROJECT_NAME}_mission_manager_node src/MissionManager.cpp)
add_dependencies(${PROJECT_NAME}_mission_manager_node ${catkin_EXPORTED_TARGETS})
target_compile_options(${PROJECT_NAME}_mission_manager_node PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_mission_manager_node
                      ${catkin_LIBRARIES} ${PCL_LIBRARIES}
                      ${GAZEBO_LIBRARIES} ${OpenMP_CXX_FLAGS})
