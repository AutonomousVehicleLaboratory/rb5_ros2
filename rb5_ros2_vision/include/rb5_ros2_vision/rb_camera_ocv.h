#ifndef RB5_CAMERA_OCV
#define RB5_CAMERA_OCV
#include <stdio.h>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <gst/gst.h>
#include <glib.h>
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <string>
#include <cstring>
#include <turbojpeg.h>

class RbCamera : public rclcpp::Node
{
  public:
    RbCamera(const std::string & name)
      : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true);
    ~RbCamera();
    GstFlowReturn processData(GstElement * sink, RbCamera::CustomData * data);

    rclcpp::Publisher<std_msgs::msg::Image>::SharedPtr pub_;

    // data
    typedef struct _CustomData
    {
      GstElement *pipeline;
      GstElement *source;
      GstElement *convert;
      GstElement *capsfiltersrc;
      GstElement *capsfilterapp;
      GstElement *appsink;
    } CustomData;

    void init();

    GstMessage *msg;
    GstBus *bus;
    GstStateChangeReturn ret;
    guint bus_id;

    int camera_id;
    int width;
    int height;
    int frame_rate;
    bool image_compress;

    std::string input_format;
    std::string output_format;
    std::string topic_name;

    CustomData data;
};

#endif
