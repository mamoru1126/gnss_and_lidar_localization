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
      ros-humble-rosbag2-storage-mcap \
      python3-numpy python3-scipy \
      python3-colcon-common-extensions git \
    && rm -rf /var/lib/apt/lists/*
# AWSIM v1.x が使う autoware_auto_msgs（tier4/autoware_auto_msgs）。apt の ros-humble-autoware-auto-msgs は中身の無い
# パッケージなので、ソースから制御と車両のメッセージだけをビルドする（コミットを固定）
ARG AUTOWARE_AUTO_MSGS_COMMIT=624aae9feb142eb2a4e16069ed06bf4050ccc3ea
RUN mkdir -p /opt/autoware_auto_msgs/src && cd /opt/autoware_auto_msgs/src \
    && git init -q aam && cd aam \
    && git fetch -q --depth 1 https://github.com/tier4/autoware_auto_msgs ${AUTOWARE_AUTO_MSGS_COMMIT} \
    && git checkout -q FETCH_HEAD && cd /opt/autoware_auto_msgs \
    && source /opt/ros/humble/setup.bash \
    && colcon build --merge-install --install-base /opt/autoware_auto_msgs/install \
         --packages-up-to autoware_auto_control_msgs autoware_auto_vehicle_msgs \
         --cmake-args -DBUILD_TESTING=OFF \
    && rm -rf /opt/autoware_auto_msgs/build /opt/autoware_auto_msgs/log
# AWSIM の Quick start demo と同じ通信の設定（ホストの AWSIM と同じ PC の中だけで通信する）
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_LOCALHOST_ONLY=1
# ROS 2 と autoware_auto_msgs を読み込んでから、渡されたコマンド（既定は bash）を動かす。
# ホストの利用者の UID で動かしても（compose の user）、読み込まれるように .bashrc ではなくここで読む
RUN printf '#!/bin/bash\nsource /opt/ros/humble/setup.bash\nsource /opt/autoware_auto_msgs/install/setup.bash\nexec "$@"\n' \
      > /awsim_entrypoint.sh && chmod +x /awsim_entrypoint.sh
ENTRYPOINT ["/awsim_entrypoint.sh"]
CMD ["bash"]
# 利用者が root でなくても ROS 2 のログを書けるように
ENV HOME=/tmp/home ROS_LOG_DIR=/tmp/home/.ros/log
RUN mkdir -p /tmp/home && chmod 777 /tmp/home
WORKDIR /ws/src/gnss_and_lidar_localization
