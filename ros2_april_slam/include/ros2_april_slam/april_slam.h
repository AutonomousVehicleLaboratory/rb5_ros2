#ifndef APRIL_SLAM_H
#define APRIL_SLAM_H


#include <math.h>
#include <bits/stdc++.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>


using namespace gtsam;
using namespace std;

class AprilSlam{
  public:
    AprilSlam();
    ~AprilSlam();
    
    // methods
    void updateMeasurement(vector<float> imu_z, vector<float> marker_z, unsigned int marker_id);
    void optimizeGraph();
  private:

      // 
      vector<float> transformCoordinate(float theta, vector<float> t, vector<float> p);

      // gtsam
      NonlinearFactorGraph graph_;
      Symbol x0;
      unordered_map<int, Symbol> landmarks_;
      unordered_map<int, Symbol> poses_;

      unsigned long long current_state_;
      vector<float> running_estimate = {0.0, 0.0, 0.0};
      Values pose_estimates;

      // priors
      Pose2 init_prior;
      noiseModel::Diagonal::shared_ptr init_prior_noise, imu_noise, measurement_noise;

      // variables
      Symbol x_last;

      

      
     


};

#endif // APRIL_SLAM_H