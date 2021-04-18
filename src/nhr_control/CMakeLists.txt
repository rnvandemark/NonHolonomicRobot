cmake_minimum_required(VERSION 3.0.2)
project(nhr_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nhr_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs nhr_msgs
)

catkin_install_python(PROGRAMS
  nodes/Planner.py
  nodes/Visualizer.py
  nodes/PlanRequester.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
