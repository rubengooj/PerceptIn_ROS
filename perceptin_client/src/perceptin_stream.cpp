/**
*This file is part of LSD-SLAM fused with imu.
*
*This file defines the class PerceptInStream, class PerceptInStream is used to grab data from the PerceptIn devices
*and then publishs them using socket.
*/

#include "perceptin_stream.h"

namespace lsd_slam {

void PerceptInStream::PublishData() {
  if (OpenDevice() < 0)   // opens the device PerceptIn
    return;

  if (thread_imu_ == NULL)
    thread_imu_ = new std::thread(std::mem_fun(&PerceptInStream::PublishImu), this);
  if (thread_stereo_ == NULL)
    thread_stereo_ = new std::thread(std::mem_fun(&PerceptInStream::PublishStereo), this);

  perceptin_->SetExposure(600);
  std::shared_ptr<const PIRVS::Data> data;
  std::shared_ptr<const PIRVS::ImuData> imu_data;
  std::shared_ptr<const PIRVS::StereoData> stereo_data;

  while (true) {
    if (!perceptin_->GetData(&data))  //grab data from device PerceptIn
      continue;
    
    imu_data = std::dynamic_pointer_cast<const PIRVS::ImuData>(data);  //publishs imu data
    if (imu_data) {
      PushImu(imu_data);
      continue;
    }

    stereo_data = std::dynamic_pointer_cast<const PIRVS::StereoData>(data); //publish stereo data
    if (stereo_data) 
      PushStereo(stereo_data);
  } //while
}

void PerceptInStream::PublishImu() {
  if (OpenSocketImu() < 0)   // opens the Socket for imu data
    return;

  std::shared_ptr<const PIRVS::ImuData> imu_data;
  MsgImuData msg_imu;
  int num = 0;
  std::unique_lock<std::mutex> lock(mutex_imu_);

  while (true) {  //the main loop
    condition_imu_.wait(lock);

    while (true) { //pubishes the buffer data
      if (buffer_imu_.empty())
        break;
      imu_data = buffer_imu_.front();
      buffer_imu_.pop_front();

     //publish the current data
      lock.unlock();
      msg_imu.timestamp = imu_data->timestamp;
      for (int i=0; i<3; ++i) {
        msg_imu.accel[i] = imu_data->accel[i];
        msg_imu.gyros[i] = imu_data->ang_v[i];
      }
      num = write(socket_imu_, &msg_imu, sizeof(MsgImuData)); 
      std::cout << "imu:  " << imu_data->timestamp << "   "
                     << imu_data->accel[0] << "   " << imu_data->accel[1] << "   " << imu_data->accel[2] << "   "
                     << imu_data->ang_v[0] << "   " << imu_data->ang_v[1] << "   " << imu_data->ang_v[2] << std::endl; 
      lock.lock();
    }
  }
}

void PerceptInStream::PublishStereo() {
  if (OpenSocketStereo() < 0)  //opens the socket for stereo data
    return;

  std::shared_ptr<const PIRVS::StereoData> stereo_data;
  MsgStereoData msg_stereo;
  const uchar *image_data=NULL, *image_data_end=NULL;
  int num = 0;
  std::unique_lock<std::mutex> lock(mutex_stereo_);

  while (true) { //the main loop
    condition_stereo_.wait(lock);

    while (true) { //publishes the buffer data
      if (buffer_stereo_.empty())
        break;
      stereo_data = buffer_stereo_.front();
      buffer_stereo_.pop_front();

      // publishes the current data
      lock.unlock();
      msg_stereo.timestamp = stereo_data->timestamp;
      image_data = stereo_data->img_l.ptr(0, 0);
      image_data_end = image_data + 640 * 480;
      while (image_data < image_data_end) {
        memcpy(&msg_stereo.image, image_data,  IMAGE_BUFFER_SIZE);
        num = write(socket_stereo_, &msg_stereo, sizeof(MsgStereoData)); 
        image_data += IMAGE_BUFFER_SIZE;
      }

      image_data = stereo_data->img_r.ptr(0, 0);
      image_data_end = image_data + 640 * 480;
      while (image_data < image_data_end) {
        memcpy(&msg_stereo.image, image_data,  IMAGE_BUFFER_SIZE);
        num = write(socket_stereo_, &msg_stereo, sizeof(MsgStereoData)); 
        image_data += IMAGE_BUFFER_SIZE;
      } 
      std::cout << "stereo_image:   " << stereo_data->timestamp << std::endl;
      lock.lock();
    }
  }
}

} //namespace lsd_slam