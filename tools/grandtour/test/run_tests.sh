#!/usr/bin/env bash
# tools/grandtour のテスト（CI の tools_grandtour ジョブ）。numpy・matplotlib・zarr（3.x）・pyproj が要る。
# 1. analysis.py の単体テスト
# 2. 合成ミッションを Hugging Face と同じ形（.tar）で作り、download.py --source-dir で展開する（2 回目は何も落とさない）
# 3. inspect_grandtour.py でレポートを作り、合成したときの真値と比べる
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
tools="$(dirname "$here")"
work="$(mktemp -d)"
# 失敗したら、最後の出力を GitHub Actions の注釈（::error::）にも出す（ログを開かなくても原因が分かるように）
log="$(mktemp)"
exec > >(tee "$log") 2>&1
on_exit() {
  rc=$?
  if [ "$rc" -ne 0 ]; then
    msg="$(tail -n 60 "$log" | sed -e 's/%/%25/g' -e 's/\r/%0D/g' | awk '{printf "%s%%0A", $0}')"
    echo "::error title=tools/grandtour tests failed (exit $rc)::$msg"
  fi
  rm -rf "$work" "$log"
}
trap on_exit EXIT

python3 -m unittest discover -s "$here" -p "test_*.py" -v

python3 "$here/make_fake_missions.py" "$work/src"
python3 "$tools/download.py" --dest "$work/data" --missions ETH-1 ETH-3 SBB-1 --preset lidar --source-dir "$work/src"
second="$(python3 "$tools/download.py" --dest "$work/data" --missions ETH-1 ETH-3 SBB-1 --preset lidar --source-dir "$work/src")"
echo "$second"
test "$(echo "$second" | grep -c ' 0 topics to fetch')" -eq 3
test -f "$work/data/2024-10-01-11-29-55/data/livox_points_undistorted/.zgroup"

python3 "$tools/inspect_grandtour.py" --data-dir "$work/data" --out "$work/report"
python3 "$here/check_report.py" "$work/report"

# 4. ROS 2 bag への変換と、案 A の地図（ETH-1 の地図を、ETH-3 の経路の近くに絞る）
python3 "$tools/grandtour_to_bag.py" "$work/data/2024-10-01-11-29-55" "$work/bags/eth1.mcap" --drop-lidar 0.45:0.5
python3 "$tools/build_map_from_gt.py" "$work/data/2024-10-01-11-29-55" "$work/maps/eth1" --near "$work/data/2024-10-01-12-00-49"
python3 "$here/check_convert.py" "$work/bags/eth1.mcap" "$work/maps/eth1"
# Livox 内蔵の IMU（g 単位）は m/s² に直す
livox_out="$(python3 "$tools/grandtour_to_bag.py" "$work/data/2024-10-01-11-29-55" "$work/bags/eth1_livox_imu.mcap" --imu livox_imu)"
echo "$livox_out" | grep "係数 9.80665"

# 5. ROS 2 があれば（CI の ros2 ジョブ）、bag を rosbag2 で読み、地図をタイル化する
#    GLL_REQUIRE_ROS=1 のときは、ROS 2 か tiled_pcd_map_tiler が無ければ失敗にする（確認が飛ばされないように）
if [ "${GLL_REQUIRE_ROS:-0}" = 1 ] && ! python3 -c "import rosbag2_py" 2>/dev/null; then
  echo "GLL_REQUIRE_ROS=1 but rosbag2_py is not available"; exit 1
fi
if python3 -c "import rosbag2_py" 2>/dev/null; then
  python3 "$here/check_bag_ros2.py" "$work/bags/eth1.mcap"
  tiler="$(command -v tiled_pcd_map_tiler || find /ws/install -type f -name tiled_pcd_map_tiler 2>/dev/null | head -1)"
  if [ -n "$tiler" ]; then
    "$tiler" -i "$work/maps/eth1/map.pcd" -o "$work/maps/eth1/tiles" --tile-size 20 --voxel-size 0.2 --no-covariance
    test -f "$work/maps/eth1/tiles/tile_index.yaml"
    echo "tiled_pcd_map_tiler: tiles written"
  elif [ "${GLL_REQUIRE_ROS:-0}" = 1 ]; then
    echo "GLL_REQUIRE_ROS=1 but tiled_pcd_map_tiler is not found"; exit 1
  fi
fi
echo "tools/grandtour: all tests passed"
