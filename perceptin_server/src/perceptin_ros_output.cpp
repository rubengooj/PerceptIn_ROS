/*********************************************
This file is part of LSD-SLAM fused with imu.

This file is used to output imu data and stereo images
*********************************************/

#include <iostream>
#include <thread>
#include "opencv2/opencv.hpp"

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>


namespace lsd_slam {

// the imu subscribe callback function
void ImuCallback(const geometry_msgs::TwistStampedConstPtr &_imu_msg) {
  
  long long timestamp = 1e3 * _imu_msg->header.stamp.sec + 1e-6 * _imu_msg->header.stamp.nsec;

  std::cout << "imu:  "
            << timestamp << "    "
            << "accel:  " 
            << _imu_msg->twist.linear.x << "    "
            << _imu_msg->twist.linear.y << "    "
            << _imu_msg->twist.linear.z << "    "
            << "gyros:  " 
            << _imu_msg->twist.angular.x << "    "
            << _imu_msg->twist.angular.y << "    "
            << _imu_msg->twist.angular.z << std::endl;
}

// the stereo subscribe callback function
void StereoCallback(const sensor_msgs::CompressedImageConstPtr &_image_left,
  const sensor_msgs::CompressedImageConstPtr &_image_right) {
  
  cv_bridge::CvImagePtr cvimage_left = cv_bridge::toCvCopy(_image_left, "mono8");
  cv_bridge::CvImagePtr cvimage_right = cv_bridge::toCvCopy(_image_right, "mono8"); 

  long long timestamp_left, timestamp_right;
  timestamp_left = 1e3 * _image_left->header.stamp.sec + 1e-6 * _image_left->header.stamp.nsec;
  timestamp_right = 1e3 * _image_right->header.stamp.sec + 1e-6 * _image_right->header.stamp.nsec;

  std::cout << "image_left: "
            << timestamp_left << "     "
            << "image_right: "
            << timestamp_right << "    "
            << (timestamp_left - timestamp_right) << std::endl;

  cv::imshow("image_left_view", cvimage_left->image);
  cv::imshow("image_right_view", cvimage_right->image);
  cv::waitKey(1);
}

// the thread function to subscribe imu message
void OutputImu() {
  ros::NodeHandle node_handle;
  ros::Subscriber imu_sub = node_handle.subscribe("imu", 10, ImuCallback); 
  ros::Rate ros_rate(1000);

  while (node_handle.ok()) {
    ros::spinOnce();
    ros_rate.sleep();
  }
}

// the thread function to subscribe stereo message
void OutputStereo() {
  ros::NodeHandle node_handle;
  ros::Rate ros_rate(100);
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::CompressedImage,
                                                          sensor_msgs::CompressedImage> StereoSyncPolicy;

  message_filters::Subscriber<sensor_msgs::CompressedImage> image_left_sub(node_handle, "image_left", 1);  
  message_filters::Subscriber<sensor_msgs::CompressedImage> image_right_sub(node_handle, "image_right", 1);
  message_filters::Synchronizer<StereoSyncPolicy> sync(StereoSyncPolicy(10), image_left_sub, image_right_sub);
  sync.registerCallback(boost::bind(&StereoCallback, _1, _2));  

  while (node_handle.ok()) {
    ros::spinOnce();
    ros_rate.sleep();
  }
}

} //namespace lsd_slam

//the main function
int main(int argc, char** argv) {
  using namespace lsd_slam;

  ros::init(argc, argv, "perceptin_ros_output");

  std::thread thread_imu(&OutputImu); //the thread to subscribe imu message
  std::thread thread_stereo(&OutputStereo); //the thread  to subscribe stereo message

  thread_imu.join();
  thread_stereo.join();
  return 0;
}
