#!/usr/bin/env bash
# 変換した bag（awsim_to_bag.py の出力）を推定ノード（gll_localizer）に通す。--record を付けると、推定の様子を
# rosbag（MCAP）に録る（Foxglove などで見る用）。dev コンテナで、ROS 2 と /ws/install を source した状態で使う。
#
#   tools/awsim/replay.sh <bag のフォルダ> [--record <録る bag のフォルダ>] [--rate 1] [--raw-points]
#
# パラメータは <bag のフォルダ>_params.yaml（awsim_to_bag.py が作る）。推定の CSV もそこに書かれた場所に出る。
# 録るトピック（表示は frame map_local で見る。map → map_local は /tf_static にある）:
#   /map/points                          全体の地図（awsim_to_bag.py の --tiles で入る。間引いたもの）
#   /gll_localizer/debug/map_points      いま照合に使っている部分の地図（タイルを読み込んだ範囲）
#   /gll_localizer/debug/scan_points     いまのスキャン（前処理の後）を照合の結果の姿勢で置いたもの
#   /gll_localizer/output/pose ほか       推定の出力、/groundtruth/pose（真値）、/sensing/gnss/fix、/tf、/tf_static など
#   --raw-points のとき /sensing/lidar/points（生の点群。大きい）も録る
set -euo pipefail
set -m  # ノードと録画をプロセスグループごと止められるように

bag="" rec="" rate=1 raw=0
while [ $# -gt 0 ]; do
  case "$1" in
    --record) rec="$2"; shift 2 ;;
    --rate) rate="$2"; shift 2 ;;
    --raw-points) raw=1; shift ;;
    -h|--help) sed -n '2,14p' "$0"; exit 0 ;;
    *) bag="${1%/}"; shift ;;
  esac
done
[ -n "$bag" ] || { sed -n '2,14p' "$0"; exit 1; }
bag="${bag%.mcap}"
params="${bag}_params.yaml"
[ -d "$bag" ] || { echo "$bag が無い（awsim_to_bag.py の出力のフォルダを指定する）"; exit 1; }
[ -f "$params" ] || { echo "$params が無い"; exit 1; }
ros2 pkg prefix gll_ros2 >/dev/null || { echo "gll_ros2 がビルド・source されていない"; exit 1; }

pids=()
stop() {  # SIGINT → 30 s 待って SIGTERM → 30 s 待って SIGKILL（CSV と録った bag は SIGINT で閉じられる）
  local pid sig
  for pid in "$@"; do
    for sig in INT TERM KILL; do
      kill -"$sig" -- -"$pid" 2>/dev/null || kill -"$sig" "$pid" 2>/dev/null || true
      for _ in $(seq 1 30); do kill -0 "$pid" 2>/dev/null || break; sleep 1; done
      kill -0 "$pid" 2>/dev/null || break
      echo "process $pid did not stop on SIG$sig"
    done
    wait "$pid" 2>/dev/null || true
  done
}
trap 'stop "${pids[@]}"' EXIT

if [ -n "$rec" ]; then
  [ -e "$rec" ] && { echo "$rec がもうある（別の名前にするか消す）"; exit 1; }
  topics=(/tf /tf_static /map/points /groundtruth/pose /sensing/gnss/fix /initialpose /diagnostics
    /gll_localizer/output/pose /gll_localizer/output/odometry /gll_localizer/output/status
    /gll_localizer/debug/raw_pose /gll_localizer/debug/lidar_pose /gll_localizer/debug/map_points
    /gll_localizer/debug/scan_points)
  [ "$raw" = 1 ] && topics+=(/sensing/lidar/points)
  # --use-sim-time は付けない（/clock が止まった後に SIGINT で止まらないことがある）。受信時刻で録る。
  # メッセージの header.stamp は bag の時刻なので、Foxglove では「header の時刻」で並べて見られる
  # 標準入力は /dev/null にする: 録画はキー操作（スペースで一時停止）のために端末を読もうとするので、端末から
  # 動かしたバックグラウンドのジョブだと SIGTTIN で止まったままになる（録画が始まらず、SIGINT でも止まらない）
  ros2 bag record -s mcap -o "$rec" --topics "${topics[@]}" < /dev/null > "${rec%/}.record.log" 2>&1 &
  pids+=($!)
  # 録画が始まったか（bag のフォルダができるか）を確かめる。始まらなければログを見せて止める
  for _ in $(seq 1 20); do [ -d "$rec" ] && break; kill -0 "${pids[-1]}" 2>/dev/null || break; sleep 0.5; done
  if [ ! -d "$rec" ]; then
    echo "!!!! 録画が始まらない（$rec ができない）。${rec%/}.record.log:"
    cat "${rec%/}.record.log"
    exit 1
  fi
fi

log="${bag}_node.log"
ros2 launch gll_ros2 localizer.launch.py params_file:="$params" use_sim_time:=true < /dev/null > "$log" 2>&1 &
pids=($! "${pids[@]}")  # 先にノードを止める（止まる前の出力も録る）
echo "推定ノードのログ: $log"
sleep 5

ros2 bag play "$bag" --clock 100 --rate "$rate" --disable-keyboard-controls
sleep 2
trap - EXIT
stop "${pids[@]}"
echo "---- node log (last lines)"
tail -n 20 "$log"
if [ -n "$rec" ]; then
  echo "---- recorded: $rec（録画のログ: ${rec%/}.record.log）"
  tail -n 5 "${rec%/}.record.log"
  if [ ! -f "$rec/metadata.yaml" ]; then
    # 録画がきれいに止まらなかったとき: 索引と metadata.yaml を作り直す
    echo "metadata.yaml が無いので作り直す（ros2 bag reindex）"
    ros2 bag reindex "$rec" -s mcap || echo "reindex に失敗した（.mcap はそのまま Foxglove で開ける）"
  fi
  ros2 bag info "$rec" 2>/dev/null | sed -n '1,40p' || true
fi
