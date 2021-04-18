cmake_minimum_required(VERSION 3.0.2)
project(nhr_sim)

find_package(catkin REQUIRED COMPONENTS
  nhr_control
  gazebo_ros
  turtlebot3_description
)

catkin_package(CATKIN_DEPENDS
  nhr_control
  gazebo_ros
  turtlebot3_description
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(
  DIRECTORY launch world
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
