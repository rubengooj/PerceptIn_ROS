CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
SET(OpenCV_DIR "/home/yanyan/opencv-3.2.0/build")
PROJECT(perceptin_client)

SET(CMAKE_C_FLAGS "-std=c99")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3 -std=c++11")
SET(CMAKE_BUILD_TYPE Release)

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
endif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

### 3rd party ###
# OpenCV
FIND_PACKAGE(OpenCV REQUIRED)

# Boost
SET(Boost_LIBRARY_DIRS /usr/local/lib)
SET(Boost_USE_STATIC_LIBS        OFF)
SET(Boost_USE_MULTITHREADED      ON)
SET(Boost_USE_STATIC_RUNTIME     OFF)
FIND_PACKAGE(Boost REQUIRED COMPONENTS  filesystem system)

# Boost thread
set(BoostThreadLib boost_thread)

# LIBUSB
FIND_PACKAGE(PkgConfig)
pkg_check_modules(LIBUSB libusb-1.0)


# Include dirs
INCLUDE_DIRECTORIES(
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
)

# Set link libraries
LIST(APPEND LINK_LIBS 
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES} 
  ${Boost_PROGRAM_OPTIONS_LIBRARY} 
  ${BoostThreadLib}
  ${LIBUSB_LIBRARIES}
  -lpthread
  -lblas
  -llapack
)

LINK_DIRECTORIES(${PROJECT_SOURCE_DIR}/lib)

# Set source files
SET(SOURCE_FILES_PERCEPTIN_CLIENT
  ${PROJECT_SOURCE_DIR}/src/perceptin_stream.cpp
  ${PROJECT_SOURCE_DIR}/src/perceptin_client.cpp
)



# build shared library.
ADD_EXECUTABLE(perceptin_client ${SOURCE_FILES_PERCEPTIN_CLIENT}) 

TARGET_LINK_LIBRARIES(perceptin_client -lPerceptInPIRVS ${LINK_LIBS})


