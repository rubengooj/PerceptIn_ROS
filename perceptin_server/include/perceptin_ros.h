/**
*This file is part of LSD-SLAM fused with imu.
*
*This file defines the class PerceptInROS,
*class PerceptInROS is used to receive data from the PerceptIn devices through socket and then pulishs them using ROS.
*/

#ifndef LSD_SLAM_PERCEPTIN_ROS_
#define LSD_SLAM_PERCEPTIN_ROS_

#include <unistd.h>
#include <stdio.h>  
#include <sys/types.h>  
#include <sys/socket.h>  
#include <sys/un.h> 

#include <string>
#include <iostream>
#include <thread>
#include "opencv2/opencv.hpp"
#include "perceptin_data.h"

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

namespace lsd_slam {

/**
*Defines PerceptInROS class to  retransmit the data of device  PerceptIn.
*/
class PerceptInROS {
 public:
  //the default constructor
  inline PerceptInROS();

  //the default destructor
  inline ~PerceptInROS();

  //begins to receive data from the PerceptIn through socket and then publishs them using ROS.
  inline void PublishData();

 private:
  //open the Socket for imu data
  inline int OpenSocketImu();

  //close the Socket for imu data
  inline int CloseSocketImu();

  //open the Socket for stereo data
  inline int OpenSocketStereo();

  //close the Socket for stereo data
  inline int CloseSocketStereo();

  //retransmits the imu data
  void PublishImu();

  //retransmits the stereo data
  void PublishStereo();
  
 private:
  int socket_imu_; //the socket handle for imu data
  int client_imu_; //the client handle for imu data
  int socket_stereo_; //the socket handle for stereo data
  int client_stereo_; //the client handle for stereo data

  std::thread *thread_imu_;
  std::thread *thread_stereo_;
};

PerceptInROS::PerceptInROS() {
  socket_imu_ = -1;
  client_imu_ = -1;
  socket_stereo_ = -1;
  client_stereo_ = -1;

  thread_imu_ = NULL;
  thread_stereo_ = NULL;
}

PerceptInROS::~PerceptInROS() {
  CloseSocketImu();
  CloseSocketStereo();
}

void PerceptInROS::PublishData() {
  if (thread_imu_ == NULL)
    thread_imu_ = new std::thread(std::mem_fun(&PerceptInROS::PublishImu), this);
  if (thread_stereo_ == NULL)
    thread_stereo_ = new std::thread(std::mem_fun(&PerceptInROS::PublishStereo), this);

  thread_imu_->join();
  thread_stereo_->join();
}

int PerceptInROS::OpenSocketImu() {
  //creats the unix socket for imu data  
  socket_imu_ = socket(PF_UNIX, SOCK_STREAM, 0);
  if (socket_imu_ < 0) {
    printf("Failed to create communication socket for imu data.\n");
    return -1;
  }

  struct sockaddr_un server_addr;
  server_addr.sun_family = AF_UNIX;
  server_addr.sun_path[0] = 0;
  strcpy(server_addr.sun_path + 1, SERVER_IMU);
  socklen_t len = sizeof(server_addr.sun_family) + sizeof(SERVER_IMU);
  int ret = bind(socket_imu_, (struct sockaddr*)&server_addr, len);
  if (ret == -1) {
    printf("Failed to bind server socket for imu data.\n");  
    return -1;  
  } 
    
  ret = listen(socket_imu_, 1);  
  if( ret == -1) {
    printf("Failed to listen the client connect request for imu data.\n");   
    return -1;  
  }

  struct sockaddr_un client_addr;
  len = sizeof(client_addr);  
  client_imu_ = accept(socket_imu_, (struct sockaddr*)&client_addr, &len);  
  if (client_imu_ < 0) { 
    printf("Failed to accept client connect request for imu data.\n");  
    return 1;  
  }  
  return 0;
}

int PerceptInROS::CloseSocketImu() {
  if (client_imu_ > 0) {
    close(client_imu_);
    client_imu_ = -1;
  }

  if (socket_imu_ > 0) {
    close(socket_imu_);  
    socket_imu_ = -1;
  }
 
  return 0;
}

int PerceptInROS::OpenSocketStereo() {
  //creats the unix socket for stereo data
  socket_stereo_ = socket(PF_UNIX, SOCK_STREAM, 0);
  if (socket_stereo_ < 0) {
    printf("Failed to create communication socket for stereo data.\n");
    return -1;
  }

  struct sockaddr_un server_addr;
  server_addr.sun_family = AF_UNIX;
  server_addr.sun_path[0] = 0;
  strcpy(server_addr.sun_path + 1, SERVER_STEREO);
  socklen_t len = sizeof(server_addr.sun_family) + sizeof(SERVER_STEREO);
  int ret = bind(socket_stereo_, (struct sockaddr*)&server_addr, len);
  if (ret == -1) {
    printf("Failed to bind server socket for stereo data.\n");  
    return -1;  
  } 
    
  ret = listen(socket_stereo_, 1);  
  if( ret == -1) {
    printf("Failed to listen the client connect request for stereo data.\n");   
    return -1;  
  }

  struct sockaddr_un client_addr;
  len = sizeof(client_addr);  
  client_stereo_ = accept(socket_stereo_, (struct sockaddr*)&client_addr, &len);  
  if (client_stereo_ < 0) { 
    printf("Failed to accept client connect request for stereo data.\n");  
    return 1;  
  }  
  return 0;
}

int PerceptInROS::CloseSocketStereo() {
  if (client_stereo_ > 0) {
    close(client_stereo_);
    client_stereo_ = -1;
  }

  if (socket_stereo_ > 0) {
    close(socket_stereo_);  
    socket_stereo_ = -1;
  }
  return 0;
}

} //namespace lsd_slam


#endif //LSD_SLAM_PERCEPTIN_ROS_