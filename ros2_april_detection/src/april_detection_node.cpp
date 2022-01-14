#include "ros2_april_detection/april_detection.h"
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
//#include <tf2_ros/transform_datatypes.h>

using std::placeholders::_1;
// camera parameters
double distortion_coeff[5] = {0.022327, 
                            -0.019742, 
                            -0.000961, 
                            0.000625, 
                            0.000000};

double intrinsics[9] = {691.01615,    0.     ,  954.51,
                      0.     ,  690.10114,  540.77467,
                      0.     ,    0.     ,    1.};

const cv::Mat d(cv::Size(1, 5), CV_64FC1, distortion_coeff);
const cv::Mat K(cv::Size(3, 3), CV_64FC1, intrinsics);



class AprilDetectionNode : public rclcpp::Node{
  public:

    
    AprilDetectionNode() : Node("april_detection"){
      image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera_0", 10, std::bind(&AprilDetectionNode::imageCallback, this, _1));

      pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/april_poses", 10);

      tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    }

    // undistort raw image data
    cv::Mat rectify(const cv::Mat image) const {
      cv::Mat image_rect = image.clone();
      const cv::Mat new_K = cv::getOptimalNewCameraMatrix(K, d, image.size(), 1.0); 
      cv::undistort(image, image_rect, K, d, new_K); 

      return image_rect;

    }
    

  private:

    // apriltag lib
    AprilDetection det;

    // subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    // publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

    // tf broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;

    void publishTransforms(vector<apriltag_pose_t> poses, vector<int> ids){
      tf2::Quaternion q;
      tf2::Matrix3x3 so3_mat;
      //tf2::Transform tf;
      // tf2_ros::TransformBroadcaster br;

      geometry_msgs::msg::TransformStamped tf_gm;
      vector<geometry_msgs::msg::PoseStamped> output_array;      

      for (unsigned int i=0; i<poses.size(); i++){
        string marker_name = "marker_" + to_string(ids[i]);
        geometry_msgs::msg::TransformStamped tf_gm;
        geometry_msgs::msg::PoseStamped msg;
        
        
        tf_gm.header.stamp = this->get_clock()->now();
        tf_gm.header.frame_id = "camera";
        tf_gm.child_frame_id = marker_name;
        
        // translation
        // tf.setOrigin(tf::Vector3(poses[i].t->data[0],
        //                         poses[i].t->data[1],
        //                         poses[i].t->data[2]));


        tf_gm.transform.translation.x = poses[i].t->data[0];
        tf_gm.transform.translation.y = poses[i].t->data[1];
        tf_gm.transform.translation.z = poses[i].t->data[2];

        // orientation - SO(3)
        so3_mat.setValue(poses[i].R->data[0], poses[i].R->data[1], poses[i].R->data[2],
                        poses[i].R->data[3], poses[i].R->data[4], poses[i].R->data[5], 
                        poses[i].R->data[6], poses[i].R->data[7], poses[i].R->data[8]);

        double roll, pitch, yaw; 

        // orientation - q
        so3_mat.getRPY(roll, pitch, yaw); // so3 to RPY
        q.setRPY(roll, pitch, yaw);
        tf_gm.transform.rotation.x = q.x();
        tf_gm.transform.rotation.y = q.y();
        tf_gm.transform.rotation.z = q.z();
        tf_gm.transform.rotation.w = q.w();

        // populate msg
        msg.header.stamp = this->now();
        msg.header.frame_id = to_string(ids[i]);
        msg.pose.position.x = poses[i].t->data[0];
        msg.pose.position.y = poses[i].t->data[1];
        msg.pose.position.z = poses[i].t->data[2];

        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();
        msg.pose.orientation.w = q.w();
        // output_array.push_back(msg);
        pose_pub_->publish(msg);
        // send TF
        tf_->sendTransform(tf_gm);
        RCLCPP_INFO(this->get_logger(), "Sending TF");
        //RCLCPP_INFO("Transformation published for marker.");
      }
      
    }
    // callbacks
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg){
      
      cv_bridge::CvImagePtr img_cv = cv_bridge::toCvCopy(msg);

      // rectify and run detection (pair<vector<apriltag_pose_t>, cv::Mat>)
      auto april_obj =  det.processImage(rectify(img_cv->image));

      // TODO: return geometry_msgs::PoseStamped
      publishTransforms(get<0>(april_obj), get<1>(april_obj));
      // geometry_msgs::msg::PoseStamped pose_msg;
      // pose_pub_->publish(pose_msg);

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
