/**
*This file is part of LSD-SLAM fused with imu.
*
*This file defines the class PerceptInROS,
*class PerceptInROS is used to receive data from the PerceptIn devices through socket and then pulishs them using ROS.
*/

#include "perceptin_ros.h"

namespace lsd_slam {

void PerceptInROS::PublishImu() {
  if (OpenSocketImu() < 0)  // opens the socket for imu data
    return;

  //defines publishers
  ros::NodeHandle node_handle;
  ros::Publisher imu_pub = node_handle.advertise<geometry_msgs::TwistStamped>("imu", 1);
  ros::Rate ros_rate(10000);

  //defines message
  std_msgs::Header header;
  geometry_msgs::TwistStamped imu_msg;
  MsgImuData msg_imu;

  while (node_handle.ok()) {
    memset(&msg_imu, 0, sizeof(MsgImuData));  
    int num = read(client_imu_, &msg_imu, sizeof(MsgImuData));

    //publishs imu data
    header.stamp.sec = static_cast<int>(1e-3 * msg_imu.timestamp);
    header.stamp.nsec = 1e6 * static_cast<int>(msg_imu.timestamp - 1e3 * header.stamp.sec);
    imu_msg.header = header;
    imu_msg.twist.linear.x = msg_imu.accel[0];
    imu_msg.twist.linear.y = msg_imu.accel[1];
    imu_msg.twist.linear.z = msg_imu.accel[2];
    imu_msg.twist.angular.x = msg_imu.gyros[0];
    imu_msg.twist.angular.y = msg_imu.gyros[1];
    imu_msg.twist.angular.z = msg_imu.gyros[2];
    imu_pub.publish(imu_msg);

    std::cout << msg_imu.timestamp << "    "  
                   << msg_imu.accel[0] <<  "    " << msg_imu.accel[1] <<  "    "  << msg_imu.accel[2] <<  "    " 
                   << msg_imu.gyros[0] <<  "    " << msg_imu.gyros[1] <<  "    "  << msg_imu.gyros[2] << std::endl;
    ros::spinOnce();
    ros_rate.sleep();
  } //while
}

void PerceptInROS::PublishStereo() {
  if (OpenSocketStereo() < 0)  // opens the socket for stereo data
    return;

  //defines publishers
  ros::NodeHandle node_handle;
  //ros::Publisher image_left_pub  = node_handle.advertise<sensor_msgs::CompressedImage>("image_left", 1);
  //ros::Publisher image_right_pub = node_handle.advertise<sensor_msgs::CompressedImage>("image_right", 1);
  ros::Publisher image_left_cvpub  = node_handle.advertise<sensor_msgs::Image>("image_left", 1);
  ros::Publisher image_right_cvpub = node_handle.advertise<sensor_msgs::Image>("image_right", 1);
  ros::Rate ros_rate(10000);

  //defines message
  std_msgs::Header header;
  //sensor_msgs::CompressedImagePtr image_left_msg;
  //sensor_msgs::CompressedImagePtr image_right_msg;
  sensor_msgs::ImagePtr image_left_msg;
  sensor_msgs::ImagePtr image_right_msg;
  MsgStereoData msg_stereo;

  char *image_data = new char[640*480*2];
  char *image_index = image_data;
  char *image_data_end = image_data + 640*480*2;

  cv::namedWindow("image_left");
  cv::namedWindow("image_right");

  while (node_handle.ok()) {
    memset(&msg_stereo, 0, sizeof(MsgStereoData));  
    int num = read(client_stereo_, &msg_stereo, sizeof(MsgStereoData));

    memcpy(image_index, msg_stereo.image, IMAGE_BUFFER_SIZE);
    image_index += IMAGE_BUFFER_SIZE;

    if (image_index >= image_data_end) {
      image_index = image_data;

      //publishes stereo data
      cv::Mat image_left(480, 640, CV_8UC1, image_data);
      cv::Mat image_right(480, 640, CV_8UC1, image_data + 640*480);
      header.stamp.sec = static_cast<int>(1e-3 * msg_stereo.timestamp);
      header.stamp.nsec = 1e6 * static_cast<int>(msg_stereo.timestamp - 1e3 * header.stamp.sec);

      /*image_left_msg  = cv_bridge::CvImage(header, "mono8", image_left).toCompressedImageMsg();
      image_right_msg = cv_bridge::CvImage(header, "mono8", image_right).toCompressedImageMsg();
      image_left_pub.publish(image_left_msg);
      image_right_pub.publish(image_right_msg);*/

      image_left_msg  = cv_bridge::CvImage(header, "mono8", image_left).toImageMsg();
      image_right_msg = cv_bridge::CvImage(header, "mono8", image_right).toImageMsg();

      image_left_cvpub.publish(image_left_msg);
      image_right_cvpub.publish(image_right_msg);

      cv::imshow("image_left", image_left);
      cv::imshow("image_right", image_right);
      cv::waitKey(1);
    }

    ros::spinOnce();
    ros_rate.sleep();
  } //while

  delete image_data;
}

} //namespace lsd_slam
