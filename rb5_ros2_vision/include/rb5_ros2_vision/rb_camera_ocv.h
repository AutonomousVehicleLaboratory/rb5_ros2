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

#include "camera_parameter.h"

class RbCamera : public rclcpp::Node
{
  public:
    
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
    
    RbCamera(const std::string & name);
    ~RbCamera();
    static GstFlowReturn processData(GstElement * sink, RbCamera* node);

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;

    GstMessage *msg;
    GstBus *bus;
    GstStateChangeReturn ret;
    guint bus_id;

    int camera_id;
    int width;
    int height;
    int frame_rate;
    bool image_compress;
    bool image_rectify;

    std::string input_format;
    std::string output_format;
    std::string topic_name;

    CameraParameter cam_param;
    Mat map1;
    Mat map2;

    CustomData data;
};

#endif
