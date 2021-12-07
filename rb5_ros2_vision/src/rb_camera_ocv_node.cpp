#include "rb_camera_ocv.h"
#include <sys/time.h>

RbCamera::RbCamera(const std::string & name)
  : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
{
  // Handle parameters
  rclcpp::Parameter _camera_id;
  rclcpp::Parameter _frame_rate;
  rclcpp::Parameter _width;
  rclcpp::Parameter _height;
  rclcpp::Parameter _input_format;
  rclcpp::Parameter _output_format;
  rclcpp::Parameter _topic_name;
  rclcpp::Parameter _image_compress;

  this->declare_parameter<int>("camera_id", 0);
  this->declare_parameter<int>("frame_rate", 30);
  this->declare_parameter<int>("width", 1920);
  this->declare_parameter<int>("height", 1080);
  this->declare_parameter<std::string>("input_format", "NV12");
  this->declare_parameter<std::string>("output_format", "RGB");
  this->declare_parameter<std::string>("topic_name", "camera_0");
  this->declare_parameter<bool>("image_compress", false);

  this->get_parameter("camera_id", _camera_id);
  RCLCPP_INFO(this->get_logger(), "camera_id: %s", _camera_id.value_to_string().c_str());
  this->get_parameter("frame_rate", _frame_rate);
  RCLCPP_INFO(this->get_logger(), "frame_rate: %s", _frame_rate.value_to_string().c_str());
  this->get_parameter("width", _width);
  RCLCPP_INFO(this->get_logger(), "width: %s", _width.value_to_string().c_str());
  this->get_parameter("height", _height);
  RCLCPP_INFO(this->get_logger(), "height: %s", _height.value_to_string().c_str());
  this->get_parameter("input_format", _input_format);
  RCLCPP_INFO(this->get_logger(), "input_format: %s", _input_format.as_string().c_str());
  this->get_parameter("output_format", _output_format);
  RCLCPP_INFO(this->get_logger(), "output_format: %s", _output_format.as_string().c_str());
  this->get_parameter("topic_name", _topic_name);
  RCLCPP_INFO(this->get_logger(), "topic_name: %s", _topic_name.as_string().c_str());
  this->get_parameter("image_compress", _image_compress);
  RCLCPP_INFO(this->get_logger(), "image_compress: %s", _image_compress.value_to_string().c_str());  

  // convert parameter type
  camera_id = _camera_id.as_int();
  width = _width.as_int();
  height = _height.as_int();
  frame_rate = _frame_rate.as_int();
  input_format = _input_format.as_string();
  output_format = _output_format.as_string();
  topic_name = _topic_name.as_string();
  image_compress = _image_compress.as_bool();

  // Create a publisher on the output topic.
  pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name, 10);
  std::weak_ptr<std::remove_pointer<decltype(pub_.get())>::type> captured_pub = pub_;

  const string inputSettingsFile = "/root/dev/ros2ws/src/rb5_ros2/rb5_ros2_vision/config/camera_parameter.yaml";
  std::cout << "input file name: " << inputSettingsFile << std::endl;
  FileStorage fs(inputSettingsFile, FileStorage::READ); // Read the settings
  if (!fs.isOpened())
  {
      cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
      return;
  }

  cam_param.read(fs);
  fs.release();    

  Mat cameraMatrix = cam_param.camera_matrix.clone();
  Mat distCoeffs = cam_param.distortion_coefficients.clone();
  Size imageSize;
  imageSize.width = cam_param.image_width;
  imageSize.height = cam_param.image_height;

  initUndistortRectifyMap(
    cameraMatrix, distCoeffs, Mat(),
    getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 0, imageSize, 0), imageSize,
    CV_16SC2, map1, map2);

  // Gstreamer pipeline definition and setup.
  gst_init(0, nullptr);

  std::string input_caps = "video/x-raw,format=" + input_format + 
                           ",framerate=" + std::to_string(frame_rate) + "/1" + 
                           ",width=" + std::to_string(width) + 
                           ",height=" + std::to_string(height);
  
  std::string output_caps = "video/x-raw,format=" + output_format;

  std::cout << "input caps: " << input_caps << std::endl;
  std::cout << "output caps: " << output_caps << std::endl;

  if (camera_id == 0 or camera_id == 1) {
    data.source        = gst_element_factory_make("qtiqmmfsrc", "source");
  }
  else {
    data.source        = gst_element_factory_make("autovideosrc", "source");
  }

  data.capsfiltersrc = gst_element_factory_make("capsfilter", "capsfiltersrc");
  data.convert        = gst_element_factory_make ("videoconvert", "convert");
  data.capsfilterapp = gst_element_factory_make("capsfilter", "capsfilterapp");
  data.appsink       = gst_element_factory_make ("appsink", "sink");
  data.pipeline      = gst_pipeline_new("rb5-camera");

  if (!data.pipeline || !data.convert || !data.source || !data.appsink ||
      !data.capsfiltersrc || !data.capsfilterapp ) {
    g_printerr ("Not all elements could be created.\n");
    return;
  }

  // Build pipeline
  gst_bin_add_many (GST_BIN (data.pipeline), data.source, data.convert, data.appsink, data.capsfiltersrc, data.capsfilterapp, nullptr);
  if (gst_element_link_many (data.source, data.capsfiltersrc, data.convert, data.capsfilterapp, data.appsink, nullptr) != TRUE) {
    g_printerr ("Elements could not be linked.\n");
    gst_object_unref (data.pipeline);
    return;
  }

  if (camera_id == 0 or camera_id == 1) {
    g_object_set (G_OBJECT(data.source), "camera", camera_id, nullptr);
  }

  g_object_set(G_OBJECT(data.capsfiltersrc), "caps",
               gst_caps_from_string(input_caps.c_str()), nullptr);
  g_object_set(G_OBJECT(data.capsfilterapp), "caps",
		           gst_caps_from_string(output_caps.c_str()), nullptr);


}


RbCamera::~RbCamera(){
  gst_object_unref(bus);
  gst_element_set_state(data.pipeline, GST_STATE_NULL);
  gst_object_unref(data.pipeline);
}


void RbCamera::init(){
  g_object_set(data.appsink, "emit-signals", TRUE, nullptr);
  if (camera_id < 2) {
    g_object_set(G_OBJECT(data.source), "camera", camera_id, NULL);
  }
  g_signal_connect(data.appsink, "new-sample", G_CALLBACK(this->processData), this);

  // play
  ret = gst_element_set_state(data.pipeline, GST_STATE_PLAYING);
  if(ret == GST_STATE_CHANGE_FAILURE){
    g_printerr ("Unable to set the pipeline to the playing state.\n");
    gst_object_unref (data.pipeline);
    return;
  }
  bus = gst_element_get_bus (data.pipeline);

    /* Wait until error or EOS */
  bus = gst_element_get_bus (data.pipeline);
  msg =
      gst_bus_timed_pop_filtered (bus, GST_CLOCK_TIME_NONE,
      (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));

  /* Parse message */
  if (msg != NULL) {
    GError *err;
    gchar *debug_info;

    switch (GST_MESSAGE_TYPE (msg)) {
      case GST_MESSAGE_ERROR:
        gst_message_parse_error (msg, &err, &debug_info);
        g_printerr ("Error received from element %s: %s\n",
            GST_OBJECT_NAME (msg->src), err->message);
        g_printerr ("Debugging information: %s\n",
            debug_info ? debug_info : "none");
        g_clear_error (&err);
        g_free (debug_info);
        break;
      case GST_MESSAGE_EOS:
        g_print ("End-Of-Stream reached.\n");
        break;
      default:
        /* We should not reach here because we only asked for ERRORs and EOS */
        g_printerr ("Unexpected message received.\n");
	break;
    }
    gst_message_unref (msg);
  }
}


/* Callback for appsink to parse the video stream and publish images. 
static function as it will be cast as a C CALLBACK function.*/
GstFlowReturn RbCamera::processData(GstElement * sink, RbCamera* node){

  GstSample *sample;
  GstBuffer *buffer;
  GstMapInfo map_info;

  // Retrieve buffer
  g_signal_emit_by_name (sink, "pull-sample", &sample);

  if(sample){
    buffer = gst_sample_get_buffer (sample);
    GstCaps *caps = gst_sample_get_caps(sample);
    GstStructure *caps_structure = gst_caps_get_structure(caps, 0);

    gboolean res;
    int width, height;
    // const gchar *format;
    res = gst_structure_get_int (caps_structure, "width", &width);
    res |= gst_structure_get_int (caps_structure, "height", &height);
    // format = gst_structure_get_string (caps_structure, "format");
    if (!res) {
        g_print("no dimensions");
    }

    // g_print("%s\n", gst_structure_to_string(caps_structure)); // print streaming format

    if (!gst_buffer_map ((buffer), &map_info, GST_MAP_READ)) {
      gst_buffer_unmap ((buffer), &map_info);
      gst_sample_unref(sample);

      return GST_FLOW_ERROR;
    }

    // print system time to show the streaming frequency.
    timeval current_time;
    gettimeofday(&current_time, 0);
    std::cout << current_time.tv_sec << "." << current_time.tv_usec << std::endl;

    // Parse data from buffer, depending on the format, conversion might be needed.
    // cv::Mat frame_rgb = cv::Mat::zeros(width, height, CV_8UC3);
    cv::Mat frame_rgb(cv::Size(width, height), CV_8UC3, (char*)map_info.data, cv::Mat::AUTO_STEP);
    // cv::cvtColor(frame_rgb, frame, cv::COLOR_RGB2);

    node->image_rectify = true;

    if (node->image_rectify){ 
      Mat frame_rgb_rect;
      remap(frame_rgb, frame_rgb_rect, node->map1, node->map2, INTER_LINEAR);
      frame_rgb = frame_rgb_rect;
    }

    // Pack the OpenCV image into the ROS image.
    sensor_msgs::msg::Image::UniquePtr cam_msg(new sensor_msgs::msg::Image());
    cam_msg->header.stamp = node->now();
    cam_msg->header.frame_id = "camera_frame";
    cam_msg->height = frame_rgb.rows;
    cam_msg->width = frame_rgb.cols;
    cam_msg->encoding = "rgb8";
    cam_msg->is_bigendian = false;
    cam_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame_rgb.step);
    cam_msg->data.assign(frame_rgb.datastart, frame_rgb.dataend);
    node->pub_->publish(std::move(cam_msg));  // Publish.
    
    /*
    // Publish camera frame 
    if (_image_compress) {
      sensor_msgs::CompressedImage cam_compress_msg;
      cam_compress_msg.header = header;

      // cv imencode compression
      // std::vector<int> p;
      // p.push_back(cv::IMWRITE_JPEG_QUALITY);
      // p.push_back(90);
      
      // cv::Mat frame_bgr = cv::Mat::zeros(width, height, CV_8UC3);
      // cv::cvtColor(frame_rgb, frame_bgr, cv::COLOR_RGB2BGR);
      // cv::imencode(".jpg", frame_bgr, cam_compress_msg.data, p);

      cam_compress_pub.publish(cam_compress_msg);
    }
    */

    // free resources
    gst_buffer_unmap(buffer, &map_info);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
  }


  return GST_FLOW_ERROR;
}


int main(int argc, char *argv[]){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RbCamera>("rb_camera");
  node->init();
}

