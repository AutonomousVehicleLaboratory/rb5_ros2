#include "ros2_april_slam/april_slam.h"
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <math.h>


// using std::placeholders::_1;

class AprilSlamNode : public rclcpp::Node{
  public:
    AprilSlamNode() : Node("april_slam"){

      // pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      //   "/april_poses", 10, boost::bind(&AprilSlamNode::aprilCallback, this, _1));
      pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/april_poses", 10, [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {aprilCallback(msg); } );
      
      imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", 1, [this](sensor_msgs::msg::Imu::SharedPtr msg) {imuCallback(msg); } );
      
      imu_init = false;
      return;
    }



  private:

    // gtsam optimization 
    AprilSlam env;
    vector<float> odom = {0.0, 0.0, 0.0};
    bool imu_init;
    double t_prev;

    // subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    // publishers

    //callbacks
    void aprilCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
      
      double range = sqrt( pow(msg->pose.position.x, 2) + 
                           pow(msg->pose.position.z, 2) );

      double bearing = atan(msg->pose.position.z / msg->pose.position.x);


      cout << "Marker (" << msg->header.frame_id << ") |";
      RCLCPP_INFO(this->get_logger(), "r: %f, bearing: %f", range, bearing);

      return;
    }
    
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg){

      if (!imu_init) {
        imu_init = true;
        t_prev = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        return;
      }
      
      
      double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
      double dt = t - t_prev;
      t_prev = t;

      // RCLCPP_INFO(this->get_logger(), "Received: %f", dt);
      double w_z = msg->angular_velocity.z;
      double a_x = msg->linear_acceleration.x;
      double a_y = msg->linear_acceleration.y;
      double a_z = msg->linear_acceleration.z;

      odom[0] +=  a_x * dt * dt; // x = x(0) + a_x * dt^2
      odom[1] +=  a_y * dt * dt; // y = y(0) + a_y * dt^2
      odom[2] +=  w_z * dt; // yaw

      // RCLCPP_INFO(this->get_logger(), "a_x: %f, a_y: %f, a_z: %f", a_x, a_y, a_z);
      // RCLCPP_INFO(this->get_logger(), "Receiving IMU data: %f, %f, %f", odom[0], odom[1], odom[2]);

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