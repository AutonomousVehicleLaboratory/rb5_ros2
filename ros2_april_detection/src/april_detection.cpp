#include "ros2_april_detection/april_detection.h"

AprilDetection::AprilDetection(){
  a_detector = apriltag_detector_create();

  // tag36h11
  tf = tag36h11_create();

  apriltag_detector_add_family(a_detector, tf);

  return;
}

AprilDetection::~AprilDetection(){
  return;
}

tuple<vector<apriltag_pose_t>, vector<int>, cv::Mat> AprilDetection::processImage(const cv::Mat image) const{

  cv::Mat image_gray; 
  cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);

  // TODO: modify C99 designed initializer
  image_u8_t im = { .width  = image_gray.cols,
                    .height = image_gray.rows,
                    .stride = image_gray.cols, 
                    .buf    = image_gray.data 
  };


  zarray_t * detections = apriltag_detector_detect(a_detector, &im);

  apriltag_detection_t *det;
  apriltag_detection_info_t tag_info; 
  vector<apriltag_pose_t> poses;
  vector<int> ids;

  tag_info.tagsize = 0.159;
  tag_info.fx = 663.57507; 
  tag_info.fy = 694.47272;
  tag_info.cx = 956.22994;
  tag_info.cy = 539.54574;

  for (int i=0; i<zarray_size(detections); i++){

    zarray_get(detections, i, &det);
    tag_info.det = det;
    apriltag_pose_t pose;

    // estimate SE(3) pose 
    estimate_tag_pose(&tag_info, &pose);
    poses.push_back(pose);
    ids.push_back(det->id);

  }
  
  return make_tuple(poses, ids, image);

}


