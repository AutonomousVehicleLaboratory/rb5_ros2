#include "ros2_april_slam/april_slam.h"

AprilSlam::AprilSlam(){

  current_state_ = 1;
  // initialize priors and graph
  init_prior = Pose2(0.0, 0.0, 0.0);
  init_prior_noise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
  imu_noise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.0));
  measurement_noise = noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.2));

  x0 = Symbol('x', 0);
  poses_[0] = x0;
  x_last = x0;
  graph_.addPrior(x0, init_prior, init_prior_noise);
  pose_estimates.insert(x0, Pose2(0.0, 0.0, 0.0));


  return;
}

AprilSlam::~AprilSlam(){
  return;
}

vector<float> AprilSlam::transformCoordinate(float theta, vector<float> t, vector<float> p){

  
  float x = p[0] * cos(theta) - p[1] * sin(theta) + t[0];
  float y = p[0] * sin(theta) + p[1] * cos(theta) + t[1]; 
  return vector<float>({x, y});
}

void AprilSlam::updateMeasurement(vector<float> imu_z, vector<float> marker_z, unsigned int marker_id){

  std::cout << "Updating measurement" << std::endl;
  float dx = imu_z[0], dy = imu_z[1], dtheta = imu_z[2];
  std::cout << "z: " << marker_z[0] << " " << marker_z[1]<< std::endl;
  // float range = marker_z[0], bearing = marker_z[1];
  float x_l = marker_z[0], y_l = marker_z[1];
  double bearing = atan(y_l / x_l);
  double range = sqrt( pow(x_l, 2) + pow(y_l, 2) );

  Symbol x_curr('x', current_state_);
  Pose2 odom(dx, dy, dtheta);

  Symbol l_curr;

  if (landmarks_.find(marker_id) == landmarks_.end()){
    landmarks_[marker_id] = Symbol('l', marker_id);
  }

  l_curr = landmarks_[marker_id];  


  // add odometry measurement
  graph_.emplace_shared<BetweenFactor<Pose2> >(x_last, x_curr, odom, imu_noise);

  // add landmark observation  
  graph_.emplace_shared<BearingRangeFactor<Pose2, Point2> >(x_curr, l_curr, bearing, range, measurement_noise);

  // update initial pose estimates for graph optimization
  running_estimate[0] += dx;
  running_estimate[1] += dy;
  running_estimate[2] += dtheta;

  vector<float> l_estimate = transformCoordinate(running_estimate[2], running_estimate, vector<float>({x_l, y_l}));


  // TODO: provide a pose estimate for landmarks in addition to poses
  if (!pose_estimates.exists(x_curr.key())){
    pose_estimates.insert(x_curr, Pose2(running_estimate[0],
                                        running_estimate[1],
                                        running_estimate[2]));
  }
  if (!pose_estimates.exists(l_curr.key())){
    pose_estimates.insert(l_curr, Point2(l_estimate[0], l_estimate[1]));
  }

  ++current_state_;
  x_last = x_curr;
  return;
}

void AprilSlam::optimizeGraph(){
  LevenbergMarquardtOptimizer optimizer(graph_, pose_estimates);

  Values result = optimizer.optimize();
  result.print("Final Result:\n");

  // TODO: update pose estimates
  pose_estimates = Values();
  return;
}