#!/usr/bin/env bash
# AWSIM 用の Humble のコンテナ（docker/awsim/Dockerfile）で、記録と走行の道具が動くかを確かめる（CI の awsim_humble ジョブ）。
#   1. awsim_drive.py と記録が使う型（autoware_msgs）が読み込めること
#   2. ros2 bag record -s mcap で記録でき、ros2 bag info で読めること
#   3. awsim_drive.py が、AWSIM の車の代わり（fake_awsim_vehicle.py）を経路の終わりまで走らせること
set -eo pipefail
set -m
source /opt/ros/humble/setup.bash
source /opt/autoware_msgs/install/setup.bash
set -u
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
tools="$(dirname "$here")"
work="$(mktemp -d)"
log="$(mktemp)"
exec > >(tee "$log") 2>&1
on_exit() {
  rc=$?
  kill $(jobs -p) 2>/dev/null || true
  if [ "$rc" -ne 0 ]; then
    msg="$(tail -n 60 "$log" | sed -e 's/%/%25/g' -e 's/\r/%0D/g' | awk '{printf "%s%%0A", $0}')"
    echo "::error title=run_humble_check.sh failed (exit $rc)::$msg"
  fi
  rm -rf "$work" "$log"
}
trap on_exit EXIT
echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"

python3 -c "from autoware_control_msgs.msg import Control; \
from autoware_vehicle_msgs.msg import GearCommand, VelocityReport; print('autoware_msgs ok')"

# 30 m 進んで、半径 10 m で左に曲がる経路
printf '0 0\n10 0\n20 0\n30 0\n32.59 0.34\n35 1.34\n37.07 2.93\n38.66 5\n39.66 7.41\n40 10\n40 20\n' > "$work/route.txt"
python3 "$tools/awsim_drive.py" "$work/route.txt" --check

stamp() { echo "[$(date +%T)] $*"; }
stamp "start recording"
timeout -k 10 -s INT 240 ros2 bag record -s mcap -o "$work/bag" /awsim/ground_truth/vehicle/pose /control/command/control_cmd &
rec=$!
stamp "start the stand-in vehicle"
timeout -k 5 200 python3 "$here/fake_awsim_vehicle.py" --goal 40,20 --timeout 150 &
veh=$!
sleep 2
timeout 20 ros2 topic list --no-daemon || true
pose="$(timeout -k 5 20 python3 "$tools/awsim_drive.py" --print-pose)"
stamp "--print-pose: $pose"
test "$pose" = "0.00,0.00,0.0"
stamp "start awsim_drive.py"
timeout -k 5 60 python3 "$tools/awsim_drive.py" "$work/route.txt" --wait 3 --kmh 15 &
drv=$!
rc=0
wait "$veh" || rc=$?
stamp "vehicle finished (exit $rc)"
drc=0
wait "$drv" || drc=$?
stamp "awsim_drive.py finished (exit $drc。124 以上なら時間切れで止めた = 終わらなかった)"
kill -INT "$rec" 2>/dev/null || true
for _ in $(seq 1 20); do kill -0 "$rec" 2>/dev/null || break; sleep 1; done
kill -KILL "$rec" 2>/dev/null || true
wait "$rec" || true
stamp "recording stopped"
test "$rc" -eq 0 && test "$drc" -eq 0
timeout 30 ros2 bag info "$work/bag" | tee "$work/info.txt"
grep -q "Storage id:.*mcap" "$work/info.txt"
n=$(awk '/Topic: \/awsim\/ground_truth\/vehicle\/pose/ {for (i=1;i<=NF;i++) if ($i=="Count:") print $(i+1)}' "$work/info.txt")
echo "recorded ground truth messages: $n"
test "${n:-0}" -gt 1000
echo "humble check passed"
