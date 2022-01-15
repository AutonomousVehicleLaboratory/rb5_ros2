#include "ros2_april_slam/april_slam.h"

AprilSlam::AprilSlam(){

  // initialize priors and graph
  init_prior = Pose2(0.0, 0.0, 0.0);
  init_prior_noise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
  imu_noise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.0));
  x0 = Symbol('x', 0);
  poses_[0] = x0;
  graph_.addPrior(x0, init_prior, init_prior_noise);

  return;
}

AprilSlam::~AprilSlam(){
  return;
}

void AprilSlam::update(vector<float> imu_z, vector<float> marker_z){
  return;
}