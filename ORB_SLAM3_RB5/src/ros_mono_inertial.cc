/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"


#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include "System.h"
#include "../include/ImuTypes.h"

using namespace std;
using std::placeholders::_1;

static int cameraWidth, cameraHeight;

class ImageImuGrabber : public rclcpp::Node
{
public:
  int count = 0;
  bool mbClahe;
  cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
  queue<sensor_msgs::msg::Imu::SharedPtr> imuBuf;
  std::mutex mBufMutexImage;
  std::mutex mBufMutexImu;
  queue<sensor_msgs::msg::Image::SharedPtr> img0Buf;

  void PublishPose(
      const Sophus::SE3f &T,
      const rclcpp::Time &stamp,
      const string &frame_id,
      const string &child_frame_id);

  void GrabImage(const sensor_msgs::msg::Image::SharedPtr img_msg);
  cv::Mat GetImage(const sensor_msgs::msg::Image::SharedPtr img_msg);
  void GrabImu(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
  void SyncWithImu();

  ORB_SLAM3::System *mpSLAM;

  ImageImuGrabber(ORB_SLAM3::System *pSLAM, const bool bClahe) : Node("Mono_Inertial")
  {
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera_0", 10, std::bind(&ImageImuGrabber::GrabImage, this, _1));

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/camera_pose", 10);

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", 1000, std::bind(&ImageImuGrabber::GrabImu, this, _1));

    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    mpSLAM = pSLAM;

    mbClahe = bClahe;
  };

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  bool bEqual = false;
  if (argc < 3 || argc > 4)
  {
    cerr << endl
         << "Usage: ros2 run ORB_SLAM3 Mono_Inertial path_to_vocabulary path_to_settings [do_equalize]" << endl;
    rclcpp::shutdown();
    return 1;
  }

  if (argc == 4)
  {
    std::string sbEqual(argv[3]);
    if (sbEqual == "true")
      bEqual = true;
  }

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR, true);

  // Load camera parameters from settings file
  cv::FileStorage fSettings(argv[2], cv::FileStorage::READ);
  cameraWidth = fSettings["Camera.width"].operator int();
  cameraHeight = fSettings["Camera.height"].operator int();

  auto node = std::make_shared<ImageImuGrabber>(&SLAM, bEqual);

  std::thread sync_thread(&ImageImuGrabber::SyncWithImu, node);

  rclcpp::spin(node);

  return 0;
}

void ImageImuGrabber::GrabImage(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
  mBufMutexImage.lock();
  if (!img0Buf.empty())
    img0Buf.pop();
  img0Buf.push(img_msg);
  mBufMutexImage.unlock();
}

cv::Mat ImageImuGrabber::GetImage(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
  // RCLCPP_INFO(this->get_logger(), "Resize Image");
  cv::Mat img;
  
  RCLCPP_INFO(this->get_logger(), "A message is received");
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(img_msg);
  }
  catch (cv_bridge::Exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return img;
  }

  if (cv_ptr->image.cols != cameraWidth or
      cv_ptr->image.rows != cameraHeight)
  {
    RCLCPP_INFO(this->get_logger(), "Resize image from (%d, %d) to (%d, %d).", cv_ptr->image.cols, cv_ptr->image.rows, cameraWidth, cameraHeight);
    cv::resize(cv_ptr->image, img, cv::Size(cameraWidth, cameraHeight), cv::INTER_LINEAR);
  }
  else
  {
    img = cv_ptr->image.clone();
  }

  if (cv_ptr->image.type() == 0)
  {
    return img;
  }
  else
  {
    std::cout << "Error type" << std::endl;
    return img;
  }
}

void ImageImuGrabber::SyncWithImu()
{
  while (1)
  {
    cv::Mat im;
    double tIm = 0;
    if (!img0Buf.empty() && !imuBuf.empty())
    {
      tIm = double(img0Buf.front()->header.stamp.sec) + double(img0Buf.front()->header.stamp.nanosec) * 1e-9;
      double tIm_back = double(img0Buf.back()->header.stamp.sec) + double(img0Buf.back()->header.stamp.nanosec) * 1e-9;
      if (tIm > tIm_back)
        continue;
      {
        this->mBufMutexImage.lock();
        im = GetImage(img0Buf.front());
        img0Buf.pop();
        this->mBufMutexImage.unlock();
      }

      vector<ORB_SLAM3::IMU::Point> vImuMeas;
      mBufMutexImu.lock();
      if (!imuBuf.empty())
      {
        // Load imu measurements from buffer
        vImuMeas.clear();
        double tImu = double(imuBuf.front()->header.stamp.sec) + double(imuBuf.front()->header.stamp.nanosec) * 1e-9;
        while (!imuBuf.empty() && tImu <= tIm)
        {
          cv::Point3f acc(imuBuf.front()->linear_acceleration.x, imuBuf.front()->linear_acceleration.y, imuBuf.front()->linear_acceleration.z);
          cv::Point3f gyr(imuBuf.front()->angular_velocity.x, imuBuf.front()->angular_velocity.y, imuBuf.front()->angular_velocity.z);
          vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, tImu));
          imuBuf.pop();
        }
      }
      mBufMutexImu.unlock();
      if (mbClahe)
        mClahe->apply(im, im);

      Sophus::SE3f se3_tf = mpSLAM->TrackMonocular(im, tIm, vImuMeas);

      cout << se3_tf.rotationMatrix() << endl;
      cout << se3_tf.translation() << endl;

      string frame_id("map");
      string child_frame_id("camera_");
      child_frame_id = child_frame_id + to_string(count);
      count += 1;
      rclcpp::Time stamp = img0Buf.front()->header.stamp;
      PublishPose(se3_tf, stamp, frame_id, child_frame_id);
    }

    std::chrono::milliseconds tSleep(1);
    std::this_thread::sleep_for(tSleep);
  }
}

void ImageImuGrabber::GrabImu(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
  mBufMutexImu.lock();
  imuBuf.push(imu_msg);
  mBufMutexImu.unlock();
  return;
}

void ImageImuGrabber::PublishPose(
    const Sophus::SE3f& T, 
    const rclcpp::Time& stamp,
    const string& frame_id, 
    const string& child_frame_id)
{
    Eigen::Quaternionf q(T.rotationMatrix());
    geometry_msgs::msg::TransformStamped tf_gm;
    geometry_msgs::msg::PoseStamped msg;
    
    tf_gm.header.stamp = stamp;
    tf_gm.header.frame_id = frame_id;
    tf_gm.child_frame_id = child_frame_id;

    tf_gm.transform.translation.x = T.translation().x();
    tf_gm.transform.translation.y = T.translation().y();
    tf_gm.transform.translation.z = T.translation().z();

    tf_gm.transform.rotation.x = q.x();
    tf_gm.transform.rotation.y = q.y();
    tf_gm.transform.rotation.z = q.z();
    tf_gm.transform.rotation.w = q.w();

    // populate msg
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.pose.position.x = T.translation().x();
    msg.pose.position.y = T.translation().y();
    msg.pose.position.z = T.translation().z();

    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();
    // output_array.push_back(msg);
    pose_pub_->publish(msg);
    // send TF
    tf_->sendTransform(tf_gm);
    RCLCPP_INFO(this->get_logger(), "Sending TF");
    //RCLCPP_INFO("Transformation published for marker.");
    
}

