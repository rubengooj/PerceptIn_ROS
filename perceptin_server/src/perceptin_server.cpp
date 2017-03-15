/**
*This file is part of LSD-SLAM fused with imu.
*
*This file is used to grab imu data and stereo images from devices
*/

#include <iostream>
#include <ros/ros.h>

#include "perceptin_ros.h"

int main(int argc, char** argv) {
  using namespace lsd_slam;
  
   ros::init(argc, argv, "perceptin_ros");

  PerceptInROS perceptin_ros;
  perceptin_ros.PublishData();

  return 0;
}
