cmake_minimum_required(VERSION 3.5)
project(orb_slam3_rb5_ros2)

set(ORB_SLAM3_SOURCE_DIR "/path/to/ORB_SLAM3_RB5_LIB/")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Check C++14 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${ORB_SLAM3_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 3.2 QUIET)
if(NOT OpenCV_FOUND)
	message(FATAL_ERROR "OpenCV > 4.4 not found.")
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${ORB_SLAM3_SOURCE_DIR}/
${ORB_SLAM3_SOURCE_DIR}/include
${ORB_SLAM3_SOURCE_DIR}/include/CameraModels
${ORB_SLAM3_SOURCE_DIR}/Thirdparty/Sophus
${Pangolin_INCLUDE_DIRS}
)

set(LIBS 
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${ORB_SLAM3_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${ORB_SLAM3_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
${ORB_SLAM3_SOURCE_DIR}/lib/libORB_SLAM3.so
-lboost_system
)

# Node for monocular camera
add_executable(Mono
  src/ros_mono.cc)
ament_target_dependencies(Mono
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
  "cv_bridge"
  "image_transport"
  "tf2"
  "tf2_ros"
)


target_link_libraries(Mono
${LIBS}
${catkin_LIBRARIES}
)

# Node for monocular camera (Augmented Reality Demo)
# add_executable(MonoAR
# src/AR/ros_mono_ar.cc
# src/AR/ViewerAR.h
# src/AR/ViewerAR.cc
# )
# ament_target_dependencies(MonoAR
#   "rclcpp"
#   "sensor_msgs"
#   "cv_bridge"
#   "image_transport"
# )

# target_link_libraries(MonoAR
# ${LIBS}
# ${catkin_LIBRARIES}
# )

# Node for stereo camera
# add_executable(Stereo
# src/ros_stereo.cc
# )
# ament_target_dependencies(Stereo
#   "rclcpp"
#   "sensor_msgs"
#   "cv_bridge"
#   "image_transport"
# )

# target_link_libraries(Stereo
# ${LIBS}
# )

# Node for RGB-D camera
# add_executable(RGBD
# src/ros_rgbd.cc
# )
# ament_target_dependencies(RGBD
#   "rclcpp"
#   "sensor_msgs"
#   "cv_bridge"
#   "image_transport"
# )

# target_link_libraries(RGBD
# ${LIBS}
# )

# Node for monocular-inertial camera
add_executable(Mono_Inertial
src/ros_mono_inertial.cc
)
ament_target_dependencies(Mono_Inertial
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
  "cv_bridge"
  "tf2"
  "tf2_ros"
  "image_transport"
)

target_link_libraries(Mono_Inertial
${LIBS}
${catkin_LIBRARIES}
)

# Node for stereo-inertial camera
# add_executable(Stereo_Inertial
# src/ros_stereo_inertial.cc
# )
# ament_target_dependencies(Stereo_Inertial
#   "rclcpp"
#   "sensor_msgs"
#   "cv_bridge"
#   "tf2"
#   "tf2_ros"
#   "image_transport"
# )

# target_link_libraries(Stereo_Inertial
# ${LIBS}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  Mono
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  Mono_Inertial
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
