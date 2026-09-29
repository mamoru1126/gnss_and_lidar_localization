#!/usr/bin/env bash
# 合成の AWSIM bag を変換し、推定ノード（gll_localizer）に通して真値と比べる（CI の ros2 ジョブ）。
# colcon build の後、ROS 2 と /ws/install を source した状態で実行する。
#   A: GNSS（途中 20 s だけ RTK-FIX を外す）+ LiDAR と地図
#   B: GNSS なし。/initialpose（1 m・10° ずらす）から地図の上で初期化し、LiDAR だけで追う
# 合成データなので誤差は小さいはず。ここで見るのは、変換・パラメータ・地図・起動手順がつながって動くこと。
set -euo pipefail
# ジョブ制御を有効にする（無効のままだと、バックグラウンドのノードが SIGINT を無視し、止められない）
set -m
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
tools="$(dirname "$here")"
work="$(mktemp -d)"
log="$(mktemp)"
exec > >(tee "$log") 2>&1
on_exit() {
  rc=$?
  if [ "$rc" -ne 0 ]; then
    msg="$(tail -n 80 "$log" | sed -e 's/%/%25/g' -e 's/\r/%0D/g' | awk '{printf "%s%%0A", $0}')"
    echo "::error title=run_localizer_check.sh failed (exit $rc)::$msg"
  fi
  pkill -INT -f localizer_node 2>/dev/null || true
  rm -rf "$work" "$log"
}
trap on_exit EXIT

tiler="$(command -v tiled_pcd_map_tiler || find /ws/install -type f -name tiled_pcd_map_tiler 2>/dev/null | head -1)"
[ -x "$tiler" ] || { echo "tiled_pcd_map_tiler not found"; exit 1; }
ros2 pkg prefix gll_ros2 >/dev/null || { echo "gll_ros2 is not built/sourced"; exit 1; }

python3 "$here/make_fake_awsim_bag.py" "$work/fake"
"$tiler" -i "$work/fake/map.pcd" -o "$work/tiles" --tile-size 20 --voxel-size 0.2
EXT=0.9,0,2.0,0,1,88
python3 "$tools/awsim_to_bag.py" "$work/fake/bag" "$work/a/a.mcap" --lidar-extrinsic "$EXT" --gnss-off-time 30:20 \
  --tiles "$work/tiles"
python3 "$tools/awsim_to_bag.py" "$work/fake/bag" "$work/b/b.mcap" --lidar-extrinsic "$EXT" --no-gnss \
  --initial-pose 1.0,10 --tiles "$work/tiles" --seed 3
python3 "$here/check_bag_ros2.py" "$work/a/a.mcap"
python3 "$here/check_bag_ros2.py" "$work/b/b.mcap" --no-gnss --initial-pose

run() {  # $1 = bag（<stem>_params.yaml が隣にある）
  local stem="${1%.mcap}"
  ros2 launch gll_ros2 localizer.launch.py params_file:="${stem}_params.yaml" use_sim_time:=true \
    > "${stem}_node.log" 2>&1 &
  local pid=$!
  sleep 5
  timeout 300 ros2 bag play "$1" --clock 100 --disable-keyboard-controls
  sleep 2
  # 止める: SIGINT → 30 s 待って SIGTERM → 30 s 待って SIGKILL（CSV は SIGINT で閉じられる）
  local sig
  for sig in INT TERM KILL; do
    kill -"$sig" -- -"$pid" 2>/dev/null || kill -"$sig" "$pid" 2>/dev/null || true
    for _ in $(seq 1 30); do kill -0 "$pid" 2>/dev/null || break; sleep 1; done
    kill -0 "$pid" 2>/dev/null || break
    echo "node did not stop on SIG$sig"
  done
  wait "$pid" 2>/dev/null || true
  echo "---- node log (last lines)"
  tail -n 25 "${stem}_node.log"
  test -s "${stem}_output.csv" || { echo "no output csv"; exit 1; }
}

run "$work/a/a.mcap"
python3 "$tools/evaluate.py" "$work/a/a_output.csv" "$work/a/a_groundtruth.csv" --title "A: GNSS + LiDAR" --out "$work/a.md" \
  --max xy_rms=0.15 yaw_rms_deg=1.0 lost_share=0 --min lidar_share=0.2

run "$work/b/b.mcap"
python3 "$tools/evaluate.py" "$work/b/b_output.csv" "$work/b/b_groundtruth.csv" --title "B: 地図だけ（初期姿勢 1 m・10° ずれ）" --out "$work/b.md" \
  --max xy_rms=0.15 yaw_rms_deg=1.0 lost_share=0 init_time=20 --min lidar_share=0.9
# 通ったときも、指標の表を GitHub Actions の注釈に出す（ログを開かなくても値が見えるように）
msg="$(cat "$work/a.md" "$work/b.md" | sed -e 's/%/%25/g' -e 's/\r/%0D/g' | awk '{printf "%s%%0A", $0}')"
echo "::notice title=AWSIM localizer check (synthetic data)::$msg"
echo "localizer check passed"
