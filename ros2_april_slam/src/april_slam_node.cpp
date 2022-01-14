#include "ros2_april_slam/april_slam.h"
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"


// using std::placeholders::_1;

class AprilSlamNode : public rclcpp::Node{
  public:
    AprilSlamNode() : Node("april_slam"){

      // pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      //   "/april_poses", 10, boost::bind(&AprilSlamNode::aprilCallback, this, _1));
      pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/april_poses", 10, [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {aprilCallback(msg); } );
      return;
    }



  private:
    AprilSlam env;

    // subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

    // publishers

    //callbacks
    void aprilCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
      RCLCPP_INFO(this->get_logger(), "Receiving Marker Pose");
      // unsigned int marker_id = msg->header.seq;
      return;
    }

    

};

int main(int argc, char* argv[]){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AprilSlamNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  // std::cout << argc << argv[0] << std::endl;
  return 0;
}