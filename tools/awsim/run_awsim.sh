#!/usr/bin/env bash
# AWSIM v1.3.1 を起動する（ホストで）。Quick start demo の設定を、ここでまとめて行う:
#   - 環境変数 ROS_LOCALHOST_ONLY=1、RMW_IMPLEMENTATION=rmw_cyclonedds_cpp（この起動の中だけ。~/.profile は変えない）
#   - CycloneDDS のための net.core.rmem_max と、ループバックのマルチキャスト（再起動で戻るので、足りなければ sudo で設定する）
#   - AWSIM の設定ファイル（--json_path）を data/awsim/awsim_config.json に書いて渡す。既定では
#     ほかの車（NPC）を出さない（awsim_drive.py は障害物を見ないので、NPC がいるとぶつかる）
#
#   tools/awsim/run_awsim.sh [--traffic N] [--seed S] [--start X,Y,Z,YAW_DEG] [-- AWSIM に渡す引数...]
#     --traffic N   ほかの車の最大の台数（既定 0 = 出さない。AWSIM の既定は 40）
#     --seed S      ほかの車の出方の乱数（既定 0）
#     --start ...   車の最初の位置（地図座標）と向き（東から反時計回り [deg]）。既定は AWSIM の最初の位置
#                   （81381.73,49920.19,41.58,35）。awsim_drive.py --print-pose で今の位置を出せる
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
exe="$repo/data/awsim/AWSIM_v1.3.1/AWSIM.x86_64"
[ -x "$exe" ] || { echo "$exe が無い。先に tools/awsim/setup.sh を実行する"; exit 1; }

traffic=0
seed=0
start="81381.7265625,49920.1890625,41.57674865722656,35.0"
while [ $# -gt 0 ]; do
  case "$1" in
    --traffic) traffic="$2"; shift 2 ;;
    --seed) seed="$2"; shift 2 ;;
    --start) start="$2"; shift 2 ;;
    --) shift; break ;;
    -h|--help) sed -n 2,13p "$0"; exit 0 ;;
    *) break ;;
  esac
done
IFS=, read -r sx sy sz syaw <<<"$start"
[ -n "${syaw:-}" ] || { echo "--start は X,Y,Z,YAW_DEG の 4 つ"; exit 1; }

config="$repo/data/awsim/awsim_config.json"
cat > "$config" <<JSON
{
    "TimeScale": 1.0,
    "TimeSource": "",
    "RandomTrafficSeed": $seed,
    "MaxVehicleCount": $traffic,
    "G29DevicePath": "",
    "Ego": {
        "Position": {"x": $sx, "y": $sy, "z": $sz},
        "EulerAngles": {"x": 0.0, "y": 0.0, "z": $syaw}
    }
}
JSON
echo "AWSIM の設定: ほかの車 $traffic 台、車の最初の位置 ($sx, $sy, $sz)、向き $syaw° → $config"

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
exec "$exe" --json_path "$config" "$@"
