#!/usr/bin/env bash
# AWSIM 用の Humble のコンテナ（docker/awsim/Dockerfile）で、記録と走行の道具が動くかを確かめる（CI の awsim_humble ジョブ）。
#   1. awsim_drive.py が使う型（autoware_auto_msgs）が読み込めること
#   2. ros2 bag record -s mcap で記録でき、ros2 bag info で読めること
#   3. awsim_drive.py が、AWSIM の車の代わり（fake_awsim_vehicle.py）を経路の終わりまで走らせること
set -eo pipefail
set -m
source /opt/ros/humble/setup.bash
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

ros2 interface list | grep -i -E "ackermann|gearcommand|velocityreport|engage" || true
ls /opt/ros/humble/share | grep -i autoware || true
python3 -c "from autoware_auto_control_msgs.msg import AckermannControlCommand; \
from autoware_auto_vehicle_msgs.msg import Engage, GearCommand, VelocityReport; print('autoware_auto_msgs ok')"

# 30 m 進んで、半径 10 m で左に曲がる経路
printf '0 0\n10 0\n20 0\n30 0\n32.59 0.34\n35 1.34\n37.07 2.93\n38.66 5\n39.66 7.41\n40 10\n40 20\n' > "$work/route.txt"
python3 "$tools/awsim_drive.py" "$work/route.txt" --check

ros2 bag record -s mcap -o "$work/bag" /awsim/ground_truth/vehicle/pose /control/command/control_cmd &
rec=$!
python3 "$here/fake_awsim_vehicle.py" --goal 40,20 --timeout 150 &
veh=$!
sleep 2
timeout 170 python3 "$tools/awsim_drive.py" "$work/route.txt" --wait 3 &
drv=$!
wait "$veh"
wait "$drv" || true
kill -INT "$rec"
wait "$rec" || true
ros2 bag info "$work/bag" | tee "$work/info.txt"
grep -q "Storage id:.*mcap" "$work/info.txt"
n=$(awk '/Topic: \/awsim\/ground_truth\/vehicle\/pose/ {for (i=1;i<=NF;i++) if ($i=="Count:") print $(i+1)}' "$work/info.txt")
echo "recorded ground truth messages: $n"
test "${n:-0}" -gt 1000
echo "humble check passed"
