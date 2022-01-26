#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <math.h>
#include <chrono>
#include <bits/stdc++.h>

using namespace std;

class StateFromImu : public rclcpp::Node{
  public:
    StateFromImu() : Node("state_from_imu"){
      imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", 1, [this](sensor_msgs::msg::Imu::SharedPtr msg) {imuCallback(msg); } );

    }

  private:
    vector<float> odom = {0.0, 0.0, 0.0};
    vector<float> accel_sum = {0.0, 0.0};
    vector<float> vel_sum = {0.0, 0.0};

    long unsigned int imu_count = 0;

    bool imu_init = false;
    double t_prev;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg){
      if (!imu_init){
        imu_init = true;
        t_prev = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        return;
      }

      ++imu_count;
      double w_z = msg->angular_velocity.z;
      double a_x = msg->linear_acceleration.x;
      double a_y = msg->linear_acceleration.y;
      // double a_z = msg->linear_acceleration.z;

      double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
      double dt = t - t_prev;
      t_prev = t;

      accel_sum[0] += a_x;
      accel_sum[1] += a_y;


      vel_sum[0] += a_x * dt;
      vel_sum[1] += a_y * dt;

      odom[0] +=  vel_sum[0] * dt; // x = x(0) + a_x * dt^2
      odom[1] +=  vel_sum[1] * dt; // y = y(0) + a_y * dt^2
      odom[2] +=  w_z * dt; // yaw
      std::cout << "a_x_avg: " << accel_sum[0] / imu_count << ", a_y_avg: " << accel_sum[1] / imu_count << std::endl;
      std::cout << "v_x: " << vel_sum[0] << ", v_y: " << vel_sum[1] << std::endl; 
      std::cout << "x_x: " << odom[0] << ", x_y: " << odom[1] << std::endl; 
      // std::cout << "x: " << odom[0] << ", y: " << odom[1] << ", theta: " << odom[2] << ", dt: " << dt << std::endl; 

    }

};

int main(int argc, char* argv[]){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StateFromImu>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}