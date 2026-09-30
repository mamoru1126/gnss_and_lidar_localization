#!/usr/bin/env bash
# AWSIM の PC（Ubuntu 22.04）の準備。最初に 1 回だけ実行する（何度実行してもよい）。
#   1. AWSIM v1.3.1 と西新宿の地図を落として、リポジトリの data/awsim/ に展開する（済んでいれば何もしない）
#   2. 記録と走行に使う ROS 2 Humble のコンテナを作る
#
#   tools/awsim/setup.sh
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
dir="$repo/data/awsim"
mkdir -p "$dir"

fetch() {  # $1 = URL、$2 = 展開後にできるもの
  if [ -e "$dir/$2" ]; then
    echo "ok: data/awsim/$2"
    return
  fi
  local zip="$dir/$(basename "$1")"
  echo "download: $1"
  curl -fL --retry 3 -o "$zip.part" "$1"
  mv "$zip.part" "$zip"
  unzip -q -o "$zip" -d "$dir"
  rm -f "$zip"
  echo "ok: data/awsim/$2"
}
fetch https://github.com/tier4/AWSIM/releases/download/v1.3.1/AWSIM_v1.3.1.zip AWSIM_v1.3.1/AWSIM.x86_64
chmod +x "$dir/AWSIM_v1.3.1/AWSIM.x86_64"
# 西新宿の地図（点群と lanelet2）。CC BY-NC 4.0（非営利に限る。docs/validation_awsim.md 9 章）
fetch https://github.com/tier4/AWSIM/releases/download/v1.1.0/nishishinjuku_autoware_map.zip \
  nishishinjuku_autoware_map/lanelet2_map.osm

docker compose -f "$repo/docker/compose.yaml" build awsim
echo "準備ができた。次は tools/awsim/run_awsim.sh（AWSIM を起動）と tools/awsim/container.sh（Humble のコンテナに入る）"
