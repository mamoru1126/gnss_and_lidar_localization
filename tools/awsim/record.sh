#!/usr/bin/env bash
# AWSIM のトピックを記録する（Humble のコンテナの中で）。Ctrl-C で止める。
#   tools/awsim/record.sh <名前>     → data/awsim/<名前>/（MCAP）
set -euo pipefail
[ $# -eq 1 ] || { echo "使い方: tools/awsim/record.sh <名前>"; exit 1; }
out="${GLL_DATA:?GLL_DATA が無い（tools/awsim/container.sh のコンテナの中で動かす）}/awsim/$1"
[ -e "$out" ] && { echo "$out はもうある。別の名前にする"; exit 1; }
exec ros2 bag record -s mcap -o "$out" \
  /awsim/ground_truth/vehicle/pose /sensing/imu/tamagawa/imu_raw \
  /vehicle/status/velocity_status /sensing/lidar/top/pointcloud_raw
