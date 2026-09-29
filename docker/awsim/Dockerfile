# AWSIM v1.3.1 と一緒に使う ROS 2 Humble のコンテナ（docs/validation_awsim.md 3.2 節、tools/awsim/README.md）
# AWSIM が出すトピックの記録（ros2 bag record）と、車の走行（tools/awsim/awsim_drive.py）に使う。
# AWSIM 本体は ROS 2 を中に持っているので、ホスト（Ubuntu 22.04）で直接動かす。
#
#   docker compose -f docker/compose.yaml build awsim
#   docker compose -f docker/compose.yaml run --rm awsim
FROM ros:humble-ros-base
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-rmw-cyclonedds-cpp \
      ros-humble-autoware-auto-msgs \
      ros-humble-rosbag2-storage-mcap \
      python3-numpy python3-scipy \
    && rm -rf /var/lib/apt/lists/*
# AWSIM の Quick start demo と同じ通信の設定（ホストの AWSIM と同じ PC の中だけで通信する）
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_LOCALHOST_ONLY=1
WORKDIR /ws/src/gnss_and_lidar_localization
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
