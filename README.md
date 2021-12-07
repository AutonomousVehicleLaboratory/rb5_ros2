# Rb5 related packages in ROS2

These packages and developed using ROS dashing on Ubuntu 18.04.
For ROS2 foxy or others, see notes below.

## 1. rb5_ros2_vision

use the following command to launch the main camera on rb5

``` bash
ros2 launch rb5_ros2_vision rb_camera_main_ocv.launch
```
make sure that 'use_rb_cam' parameter is set to true.

use the following command to launch on a webcam
``` bash
ros2 launch rb5_ros2_vision rb_camera_webcam_ocv.launch
```
make sure that 'use_rb_cam' parameter is set to false.

If 'image_rectify' is set to true, make sure you set the 'camera_parameter_path'
correctly to a xml or yaml/yml file. (tested on yaml) Notice that newer versions of opencv saves matrices with 'dt' representing data type. Also the file must start with the yaml version.

## Notes on other versions of ROS2

1. In launch files, dashing uses node_executable while foxy uses executable.
2. Dashing doesn't support command line arguements. 

