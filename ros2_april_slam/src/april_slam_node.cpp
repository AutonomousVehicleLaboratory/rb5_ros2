#include "ros2_april_slam/april_slam.h"
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <math.h>
#include <chrono>


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
    AprilSlam map;
    Vector3 odom = {0.0, 0.0, 0.0};
    Vector2 velocities = {0.0, 0.0};
    int optimizer_trigger = 0, imu_trigger = 0;

    bool imu_init;
    double t_prev;

    // subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    // publishers

    //callbacks
    void aprilCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
      
      ++optimizer_trigger;
      double x = msg->pose.position.x;
      double y = msg->pose.position.z;
      // RCLCPP_INFO(this->get_logger(), "r: %f, bearing: %f", range, bearing);

      auto t_start = std::chrono::high_resolution_clock::now();
      map.updateMeasurement(odom, Vector2(x, y), stoi(msg->header.frame_id));
      auto t_stop = std::chrono::high_resolution_clock::now();
      odom[0] = odom[1] = odom[2] = 0.0;
      auto update_dt = std::chrono::duration_cast<std::chrono::microseconds>(t_stop - t_start);

      if (optimizer_trigger % 20 == 0 ){
        t_start = std::chrono::high_resolution_clock::now();
        gtsam::Values res_states = map.optimizeGraph();
        int graph_size = res_states.size();
        t_stop = std::chrono::high_resolution_clock::now();
        update_dt = std::chrono::duration_cast<std::chrono::microseconds>(t_stop - t_start);
        std::cout << "Optimization dt: " << update_dt.count() << " | ";
        std::cout << "Graph size: " << graph_size << std::endl;
        optimizer_trigger = 0;
      }


      return;
    }
    
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg){

      ++imu_trigger;
      if (!imu_init) {
        imu_init = true;
        t_prev = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        return;
      }
      
      
      double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
      double dt = t - t_prev;
      t_prev = t;

      double w_z = msg->angular_velocity.z;
      double a_x = msg->linear_acceleration.x;
      double a_y = msg->linear_acceleration.y;

      velocities[0] += a_x * dt;
      velocities[1] += a_y * dt;

      odom[0] +=  velocities[0] * dt; // x = x(0) + a_x * dt^2
      odom[1] +=  velocities[1] * dt; // y = y(0) + a_y * dt^2
      odom[2] +=  w_z * dt; // yaw

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