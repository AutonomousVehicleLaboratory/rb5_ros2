#include "ros2_april_slam/april_slam.h"
#include <rclcpp/rclcpp.hpp>


class AprilSlamNode : public rclcpp::Node{
  public:
    AprilSlamNode() : Node("april_slam"){
      return;
    }
};

int main(int argc, char* argv[]){
  auto node = std::make_shared<AprilSlamNode>();
  std::cout << argc << argv[0] << std::endl;
}