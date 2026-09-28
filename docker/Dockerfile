# syntax=docker/dockerfile:1
#
# gnss_and_lidar_localization の開発・実行環境（設計書 7.8 節）
#
#   dev          : ビルド・テスト・デバッグ・データ変換用（ソースはボリュームでマウントする）
#   devcontainer : VS Code の Dev Container 用（dev に、root でない利用者 ubuntu と sudo などを足したもの。.devcontainer/）
#   build        : ワークスペースを colcon でビルドする中間ステージ
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
      libeigen3-dev libgeographiclib-dev libgtest-dev libyaml-cpp-dev \
      python3-colcon-common-extensions \
      ros-${ROS_DISTRO}-tf2-ros \
      ros-${ROS_DISTRO}-nav-msgs \
      ros-${ROS_DISTRO}-sensor-msgs \
      ros-${ROS_DISTRO}-geometry-msgs \
      ros-${ROS_DISTRO}-diagnostic-msgs \
      ros-${ROS_DISTRO}-ament-cmake-gtest \
    && rm -rf /var/lib/apt/lists/*

# small_gicp（スキャンマッチング。設計書 6 章）はタグを固定してソースからビルドする。
# 静的ライブラリにして、実行イメージに共有ライブラリを持ち込まなくて済むようにする。OpenMP は GCC の libgomp を使う。
ARG SMALL_GICP_VERSION=v1.0.1
RUN git clone --depth 1 --branch ${SMALL_GICP_VERSION} https://github.com/koide3/small_gicp.git /tmp/small_gicp \
 && cmake -S /tmp/small_gicp -B /tmp/small_gicp/build -G Ninja -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_HELPER=ON -DBUILD_WITH_OPENMP=ON -DBUILD_SHARED_LIBS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
 && cmake --build /tmp/small_gicp/build \
 && cmake --install /tmp/small_gicp/build \
 && rm -rf /tmp/small_gicp

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
# VS Code の Dev Container（.devcontainer/devcontainer.json）。ベースイメージ（Ubuntu 24.04）にある利用者 ubuntu で作業する。
# UID は VS Code がホストの利用者に合わせるので、マウントしたソースに作ったファイルが root のものにならない。
FROM dev AS devcontainer
ARG ROS_DISTRO
ARG USERNAME=ubuntu
RUN apt-get update && apt-get install -y --no-install-recommends \
      sudo bash-completion python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*
RUN id "${USERNAME}" \
 && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}" && chmod 0440 "/etc/sudoers.d/${USERNAME}" \
 && mkdir -p /ws/src /ws/build /ws/install /ws/log /data \
 && chown -R "${USERNAME}:${USERNAME}" /ws /data \
 && { echo "source /opt/ros/${ROS_DISTRO}/setup.bash"; \
      echo "[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash"; \
      echo "[ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ] && source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash"; \
    } >> "/home/${USERNAME}/.bashrc"
USER ${USERNAME}
WORKDIR /ws

# ---------------------------------------------------------------------------
FROM base AS build
ARG ROS_DISTRO
WORKDIR /ws
COPY tiled_pcd_map src/gnss_and_lidar_localization/tiled_pcd_map
COPY core src/gnss_and_lidar_localization/core
COPY ros2 src/gnss_and_lidar_localization/ros2
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DGLL_BUILD_TESTS=OFF -DTILED_PCD_MAP_BUILD_TESTS=OFF

# ---------------------------------------------------------------------------
FROM ros:${ROS_DISTRO}-ros-core AS runtime
ARG ROS_DISTRO
RUN apt-get update && apt-get install -y --no-install-recommends \
      libgeographiclib-dev libyaml-cpp0.8 libgomp1 \
      ros-${ROS_DISTRO}-tf2-ros \
      ros-${ROS_DISTRO}-nav-msgs \
      ros-${ROS_DISTRO}-diagnostic-msgs \
    && rm -rf /var/lib/apt/lists/*
COPY --from=build /ws/install /ws/install
RUN echo "source /ws/install/setup.bash" >> /root/.bashrc
CMD ["bash", "-c", "source /ws/install/setup.bash && ros2 launch gll_ros2 localizer.launch.py"]
