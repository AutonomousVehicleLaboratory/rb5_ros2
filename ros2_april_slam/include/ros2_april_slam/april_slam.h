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
      NonlinearFactorGraph graph_;
      unordered_map<string, Symbol> landmarks_;

};

#endif // APRIL_SLAM_H