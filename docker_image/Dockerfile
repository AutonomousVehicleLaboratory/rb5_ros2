FROM ubuntu:bionic 

# Install ROS2 System Dependencies.
COPY install_ros2_system_deps.sh /root/install/
RUN /root/install/install_ros2_system_deps.sh

# Install ROS2.
COPY install_ros2.sh /root/install/
RUN /root/install/install_ros2.sh
ENV ROS_DISTRO dashing

# Prepare a ROS2 workspace.
COPY prepare_colcon_ws.sh /root/install/
RUN /root/install/prepare_colcon_ws.sh

# Install GTSAM
COPY install_gtsam.sh /root/install
RUN /root/install/install_gtsam.sh

COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
