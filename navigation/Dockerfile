FROM ros:jazzy-ros-base-noble
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git python3-colcon-common-extensions python3-yaml \
    python3-dev libapr1-dev ros-jazzy-ament-cmake-auto \
    ros-jazzy-pcl-ros ros-jazzy-pcl-conversions ros-jazzy-sensor-msgs-py \
    ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
    ros-jazzy-imu-complementary-filter ros-jazzy-pointcloud-to-laserscan \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/arx-navigation
COPY deps deps
COPY workspace/src workspace/src
COPY scripts scripts
RUN bash scripts/build.sh
COPY env.sh env.sh
ENV ROS_DOMAIN_ID=192
CMD ["bash"]
