cmake_minimum_required(VERSION 2.8.3)
project(pumpkin_carving)

add_compile_options(-std=c++11)
find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  eigen_conversions
  geometry_msgs
  moveit_msgs
  roscpp
  rospy
  std_msgs
  tf
  tf_conversions
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pumpkin_carving
  CATKIN_DEPENDS moveit_core moveit_ros_planning_interface
  DEPENDS Eigen3
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

###########
## Build ##
###########

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(arm_mover src/arm_mover.cpp)
target_link_libraries(arm_mover ${catkin_LIBRARIES} ${Boost_LIBRARIES})
