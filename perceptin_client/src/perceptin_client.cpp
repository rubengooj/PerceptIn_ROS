/**
*This file is part of LSD-SLAM fused with imu.
*
*This file is used to grab imu data and stereo images from device PerceptIn
*/

#include <iostream>

#include "perceptin_stream.h"

int main(int argc, char** argv) {
  using namespace lsd_slam;
  
  PerceptInStream perceptin_stream;
  perceptin_stream.PublishData();

  return 0;
}
