cmake_minimum_required(VERSION 2.8.3)
project(perceptin_pynodes)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES perceptin_pynodes
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
   scripts/Perceptin_Client_node
   scripts/Perceptin_Server_node
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

