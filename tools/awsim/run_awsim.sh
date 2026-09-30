#!/usr/bin/env bash
# AWSIM v1.3.1 を起動する（ホストで）。Quick start demo の設定を、ここでまとめて行う:
#   - 環境変数 ROS_LOCALHOST_ONLY=1、RMW_IMPLEMENTATION=rmw_cyclonedds_cpp（この起動の中だけ。~/.profile は変えない）
#   - CycloneDDS のための net.core.rmem_max と、ループバックのマルチキャスト（再起動で戻るので、足りなければ sudo で設定する）
#
#   tools/awsim/run_awsim.sh [AWSIM に渡す引数...]
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
exe="$repo/data/awsim/AWSIM_v1.3.1/AWSIM.x86_64"
[ -x "$exe" ] || { echo "$exe が無い。先に tools/awsim/setup.sh を実行する"; exit 1; }

if [ "$(sysctl -n net.core.rmem_max)" -lt 2147483647 ]; then
  echo "net.core.rmem_max を上げる（sudo）"
  sudo sysctl -w net.core.rmem_max=2147483647 >/dev/null
fi
if ! ip link show lo | grep -q MULTICAST; then
  echo "ループバックのマルチキャストを有効にする（sudo）"
  sudo ip link set lo multicast on
fi

export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
exec "$exe" "$@"
