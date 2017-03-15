/**
*This file is part of LSD-SLAM fused with imu.
*
*This file defines the container used to save imu data and stereo data of the device PerceptIn.
*/

#ifndef LSD_SLAM_PERCEPTIN_DATA_
#define LSD_SLAM_PERCEPTIN_DATA_

#include <stdio.h>  
#include <string>

namespace lsd_slam {

#define SERVER_IMU "/tmp/server_imu"  //socket path for imu
#define SERVER_STEREO "/tmp/server_stereo"  //socket path for stereo
#define IMAGE_BUFFER_SIZE 4800  //image buffer size for socket

/**
*Defines the base container used to save the data of device PerceptIn.
*/
struct MsgBaseData {
  MsgBaseData() {
    timestamp = 0;
  }

  size_t timestamp; //timestamp
};

/**
*Defines the imu container used to save the imu data of device PerceptIn.
*/
struct MsgImuData : public MsgBaseData {
  MsgImuData() {
    memset(accel, 0, 3*sizeof(double));
    memset(gyros, 0, 3*sizeof(double));
  }

  double accel[3]; //the data of accelerometer
  double gyros[3]; //the data of gyroscope
};

/**
*Defines the stereo container used to save the stereo data of device PerceptIn.
*/
struct MsgStereoData : public MsgBaseData {
  MsgStereoData() {
    memset(image, 0, IMAGE_BUFFER_SIZE);
  }

  char image[IMAGE_BUFFER_SIZE]; //the data of left image
};

} //namespace lsd_slam

#endif  //LSD_SLAM_PERCEPTIN_DATA_