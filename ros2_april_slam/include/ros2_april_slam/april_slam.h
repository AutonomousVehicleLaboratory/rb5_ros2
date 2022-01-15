#ifndef APRIL_SLAM_H
#define APRIL_SLAM_H

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

  private:

      // gtsam
      NonlinearFactorGraph graph_;
      Symbol x0;
      unordered_map<string, Symbol> landmarks_;
      unordered_map<int, Symbol> poses_;

      // priors
      Pose2 init_prior;
      noiseModel::Diagonal::shared_ptr init_prior_noise, imu_noise;

      // methods
      void update(vector<float> imu_z, vector<float> marker_z);
     


};

#endif // APRIL_SLAM_H