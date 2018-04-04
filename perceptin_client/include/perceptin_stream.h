/**
*This file is part of LSD-SLAM fused with imu.
*
*This file defines the class PerceptInStream, class PerceptInStream is used to grab data from the PerceptIn devices 
*and then publishs them using socket.
*/

#ifndef LSD_SLAM_PERCEPTIN_STREAM_
#define LSD_SLAM_PERCEPTIN_STREAM_

#include <unistd.h>
#include <stdio.h>  
#include <sys/types.h>  
#include <sys/socket.h>  
#include <sys/un.h> 

#include <string>
#include <iostream>
#include <thread>
#include <mutex>
#include <deque>
#include <condition_variable>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include "pirvs.h"

#include "perceptin_data.h"

namespace lsd_slam {

/**
*Defines stream class to grab data from device  PerceptIn and publish data using socket.
*/
class PerceptInStream {
 public:
  //the default constructor
  inline PerceptInStream();

  //the default destructor
  inline ~PerceptInStream();

  //begins to grab data from the PerceptIn
  void PublishData( int exposure = 600 );

 private:
  //open the PerceptIn
  inline int OpenDevice();

  //close the PerceptIn
  inline int CloseDevice();

  //open the Socket imu
  inline int OpenSocketImu();

  //close the Socket imu
  inline int CloseSocketImu();

  //open the Socket stereo
  inline int OpenSocketStereo();

  //close the Socket stereo
  inline int CloseSocketStereo();

  //pushes imu data to buffer
  inline void PushImu(std::shared_ptr<const PIRVS::ImuData> _imu_data);

  //pushes stereo data to buffer
  inline void PushStereo(std::shared_ptr<const PIRVS::StereoData> _stereo_data);

  //thread function to publish imu data
  void PublishImu();

  //thread function to publish stereo data
  void PublishStereo();
  
 private:
  std::shared_ptr<PIRVS::PerceptInDevice> perceptin_; //the device PerceptIn

  //thread for publishing imu data
  std::thread *thread_imu_;
  std::mutex mutex_imu_;
  std::condition_variable condition_imu_;
  std::deque<std::shared_ptr<const PIRVS::ImuData>> buffer_imu_;
  int buffersize_imu_;
  int socket_imu_;

  //thread for publishing stereo data
  std::thread *thread_stereo_;
  std::mutex mutex_stereo_;
  std::condition_variable condition_stereo_;
  std::deque<std::shared_ptr<const PIRVS::StereoData>> buffer_stereo_;
  int buffersize_stereo_;
  int socket_stereo_;

  bool bgrab_; //the flag whether to grab
};

PerceptInStream::PerceptInStream() {
  thread_imu_ = NULL;
  buffersize_imu_ = 10;
  buffer_imu_.clear();
  socket_imu_ = -1;

  thread_stereo_ = NULL;
  buffersize_stereo_ = 10;
  buffer_stereo_.clear();
  socket_stereo_ = -1;

  bgrab_ = false;
}

PerceptInStream::~PerceptInStream() {
  CloseSocketImu();
  CloseSocketStereo();
  CloseDevice();
}

int PerceptInStream::OpenDevice() {
  if (!PIRVS::CreatePerceptInV1Device(&perceptin_) || !perceptin_) {
    printf("Failed to create PerceptIn device.\n");
    return -1;
  }
 
  if (!perceptin_->StartDevice()) {
    printf("Failed to start PerceptIn device.\n");
    return -1;
  }
  bgrab_ = true;
  return 0;
}

int PerceptInStream::CloseDevice() {
  if (bgrab_) {
    perceptin_->StopDevice();
    bgrab_ = false;
  }

  return 0;
}

int PerceptInStream::OpenSocketImu() {
  //creats the unix socket  for imu data
  socket_imu_ = socket(PF_UNIX, SOCK_STREAM, 0);
  if (socket_imu_ < 0) {
    printf("Failed to create communication socket for imu data.\n");
    return -1;
  }

  //connects to the server of imu data
  struct sockaddr_un server_addr;
  server_addr.sun_family = AF_UNIX;
  server_addr.sun_path[0] = 0;
  strcpy(server_addr.sun_path + 1, SERVER_IMU);
  socklen_t len = sizeof(server_addr.sun_family) + sizeof(SERVER_IMU);
  int ret = connect(socket_imu_, (struct sockaddr*)&server_addr, len);
  if (ret == -1) {
    printf("Failed to connect to the server of imu data.\n");
    return -1;
  }
  return 0;
}

int PerceptInStream::CloseSocketImu() {
  if (socket_imu_ > 0) {
    close(socket_imu_);  
    socket_imu_ = -1;
  }
  return 0;
}

int PerceptInStream::OpenSocketStereo() {
  //creats the unix socket  for stereo data
  socket_stereo_ = socket(PF_UNIX, SOCK_STREAM, 0);
  if (socket_stereo_ < 0) {
    printf("Failed to create communication socket for stereo data.\n");
    return -1;
  }

  //connects to the server of stereo data
  struct sockaddr_un server_addr;
  server_addr.sun_family = AF_UNIX;
  server_addr.sun_path[0] = 0;
  strcpy(server_addr.sun_path + 1, SERVER_STEREO);
  socklen_t len = sizeof(server_addr.sun_family) + sizeof(SERVER_STEREO);
  int ret = connect(socket_stereo_, (struct sockaddr*)&server_addr, len);
  if (ret == -1) {
    printf("Failed to connect to the server of stereo. data\n");
    return -1;
  }
  return 0;
}

int PerceptInStream::CloseSocketStereo() {
  if (socket_stereo_ > 0) {
    close(socket_stereo_);  
    socket_stereo_ = -1;
  }
  return 0;
}

void PerceptInStream::PushImu(std::shared_ptr<const PIRVS::ImuData> _imu_data) {
  std::unique_lock<std::mutex> lock(mutex_imu_);
  if (buffer_imu_.size() >= buffersize_imu_)
    return;

  buffer_imu_.push_back(_imu_data);
  condition_imu_.notify_one();
}

void PerceptInStream::PushStereo(std::shared_ptr<const PIRVS::StereoData> _stereo_data) {
  std::unique_lock<std::mutex> lock(mutex_stereo_);
  if (buffer_stereo_.size() >= buffersize_stereo_)
    return;

  buffer_stereo_.push_back(_stereo_data);
  condition_stereo_.notify_one();
}

} //namespace lsd_slam


#endif //LSD_SLAM_PERCEPTIN_STREAM_
