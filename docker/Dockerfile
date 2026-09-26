# syntax=docker/dockerfile:1
#
# gnss_and_lidar_localization の開発・実行環境（設計書 7.8 節）
#
#   dev     : ビルド・テスト・デバッグ・データ変換用（ソースはボリュームでマウントする）
#   build   : ワークスペースを colcon でビルドする中間ステージ
#   runtime : 実行に必要なものだけを持つイメージ
#
# 例:
#   docker build -f docker/Dockerfile --target dev -t gll:dev .
#   docker build -f docker/Dockerfile --target runtime -t gll:runtime .

ARG ROS_DISTRO=jazzy

# ---------------------------------------------------------------------------
FROM ros:${ROS_DISTRO}-ros-base AS base
ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake ninja-build git \
      libeigen3-dev libgeographiclib-dev libgtest-dev \
      python3-colcon-common-extensions \
      ros-${ROS_DISTRO}-tf2-ros \
      ros-${ROS_DISTRO}-nav-msgs \
      ros-${ROS_DISTRO}-sensor-msgs \
      ros-${ROS_DISTRO}-geometry-msgs \
      ros-${ROS_DISTRO}-diagnostic-msgs \
      ros-${ROS_DISTRO}-ament-cmake-gtest \
    && rm -rf /var/lib/apt/lists/*

# Phase 2 で small_gicp（タグ固定でソースからビルド）と OpenMP を追加する。

# ---------------------------------------------------------------------------
FROM base AS dev
ARG ROS_DISTRO
RUN apt-get update && apt-get install -y --no-install-recommends \
      gdb less vim-tiny \
      python3-pip python3-numpy python3-matplotlib \
      ros-${ROS_DISTRO}-rviz2 \
      ros-${ROS_DISTRO}-ros2bag ros-${ROS_DISTRO}-rosbag2-storage-mcap \
    && rm -rf /var/lib/apt/lists/*
# i2Nav-Robot の ROS1 bag を ROS 2 に変換するためのツール（docs/validation_i2nav.md）
RUN pip3 install --no-cache-dir --break-system-packages rosbags

WORKDIR /ws
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
 && echo "[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash" >> /root/.bashrc
CMD ["bash"]

# ---------------------------------------------------------------------------
FROM base AS build
ARG ROS_DISTRO
WORKDIR /ws
COPY core src/gnss_and_lidar_localization/core
COPY ros2 src/gnss_and_lidar_localization/ros2
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DGLL_BUILD_TESTS=OFF

# ---------------------------------------------------------------------------
FROM ros:${ROS_DISTRO}-ros-core AS runtime
ARG ROS_DISTRO
RUN apt-get update && apt-get install -y --no-install-recommends \
      libgeographiclib-dev \
      ros-${ROS_DISTRO}-tf2-ros \
      ros-${ROS_DISTRO}-nav-msgs \
      ros-${ROS_DISTRO}-diagnostic-msgs \
    && rm -rf /var/lib/apt/lists/*
COPY --from=build /ws/install /ws/install
RUN echo "source /ws/install/setup.bash" >> /root/.bashrc
CMD ["bash", "-c", "source /ws/install/setup.bash && ros2 launch gll_ros2 localizer.launch.py"]
