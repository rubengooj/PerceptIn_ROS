CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)

SET(OpenCV_DIR "/home/yanyan/opencv-2.4.9/build")

PROJECT(perceptin_server)

SET(CMAKE_BUILD_TYPE Release)

# Set build flags, set ARM_ARCHITECTURE environment variable on Odroid
#SET(CMAKE_CXX_FLAGS "-Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGssS} -mfpu=neon -march=armv7-a")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -lpthread")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)   
#FIND_PACKAGE(OpenGL REQUIRED)
#FIND_PACKAGE(CUDA REQUIRED)

ADD_DEFINITIONS("-DSLAM_TYPE_FLOAT")
ADD_DEFINITIONS("-D_DEBUG")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  image_transport
  cv_bridge
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

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

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

#add_message_files(
#   FILES
#   imu_msgs.msg
#)   

#generate_messages( 
#   DEPENDENCIES 
#   std_msgs 
#   geometry_msgs
#   sensor_msgs
#)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES senscape_slam
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
#  CATKIN_DEPENDS 
#  message_runtime
#)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
INCLUDE_DIRECTORIES(
  ${catkin_INCLUDE_DIRS}
)

# Include dirs
INCLUDE_DIRECTORIES(
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/perceptin_server
)

# Set link libraries
LIST(APPEND LINK_LIBS 
  ${EIGEN3_LIBS}
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES} 
  GL glut GLU
)


# Set source files
SET(SOURCE_FILES_PERCEPTIN_SERVER
  ${PROJECT_SOURCE_DIR}/src/perceptin_ros.cpp
  ${PROJECT_SOURCE_DIR}/src/perceptin_server.cpp
)

# Set source files
SET(SOURCE_FILES_PERCEPTIN_ROS_OUTPUT
  ${PROJECT_SOURCE_DIR}/src/perceptin_ros_output.cpp
)

## Declare a C++ library
# add_library(senscape_slam
#   src/${PROJECT_NAME}/senscape_slam.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(senscape_slam ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(senscape_slam_node src/senscape_slam_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(senscape_slam_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(senscape_slam_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS senscape_slam senscape_slam_node
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

# build shared library.
#ADD_LIBRARY(senscape_slam SHARED ${SOURCE_FILES})
ADD_EXECUTABLE(perceptin_server ${SOURCE_FILES_PERCEPTIN_SERVER})
#ADD_DEPENDENCIES(lsd_slam_imu_input imu_msgs_gencpp)

ADD_EXECUTABLE(perceptin_ros_output ${SOURCE_FILES_PERCEPTIN_ROS_OUTPUT})
#ADD_DEPENDENCIES(lsd_slam_imu_output imu_msgs_gencpp)

#ADD_EXECUTABLE(senscape_slam  test.cpp)      

TARGET_LINK_LIBRARIES(perceptin_server ${LINK_LIBS})
TARGET_LINK_LIBRARIES(perceptin_ros_output ${LINK_LIBS})

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_senscape_slam.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

