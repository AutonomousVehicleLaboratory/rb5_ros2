#include "ros2_april_detection/april_detection.h"
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
using std::placeholders::_1;

AprilDetection det; 

class AprilDetectionNode : public rclcpp::Node{
  public:
    AprilDetectionNode() : Node("april_detection"){
      image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera_0", 10, std::bind(&AprilDetectionNode::imageCallback, this, _1));

      pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/april_poses", 10);
      
    }

    
    

  private:
    // subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    // publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

    // callbacks
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) const{
      RCLCPP_INFO(this->get_logger(), "Received image.");

      return;
    }

};

int main(int argc, char *argv[]){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AprilDetectionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;

}
