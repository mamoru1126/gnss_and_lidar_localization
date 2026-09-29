#!/usr/bin/env bash
# tools/awsim のテスト（CI の tools_awsim ジョブ）。numpy・PyYAML・pyproj が要る。ROS は要らない。
# 1. drive_core・rigid の単体テスト
# 2. 合成の AWSIM bag（Humble と同じトピック・型）と地図のタイルを作る
# 3. check_lidar_extrinsic.py: ずらした最初の値から、合成したときの取り付け位置に戻るか
# 4. awsim_to_bag.py で変換し、check_awsim.py で中身を確かめる
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
tools="$(dirname "$here")"
work="$(mktemp -d)"
log="$(mktemp)"
exec > >(tee "$log") 2>&1
on_exit() {
  rc=$?
  if [ "$rc" -ne 0 ]; then
    msg="$(tail -n 60 "$log" | sed -e 's/%/%25/g' -e 's/\r/%0D/g' | awk '{printf "%s%%0A", $0}')"
    echo "::error title=tools/awsim tests failed (exit $rc)::$msg"
  fi
  rm -rf "$work" "$log"
}
trap on_exit EXIT

python3 -m unittest discover -s "$here" -p "test_*.py" -v

python3 "$here/make_fake_awsim_bag.py" "$work/fake"

# 取り付け位置（真値 0.9,0,2.0,0,1,88）を、ずらした値（yaw は 88° 違う）から探す
ext="$(python3 "$tools/check_lidar_extrinsic.py" "$work/fake/bag" "$work/fake/tiles" --extrinsic 1.2,0.2,1.7,0,0,0 --scans 20 --points 1500)"
echo "$ext"
python3 - "$ext" <<'EOF'
import sys
line = [l for l in sys.argv[1].splitlines() if "--lidar-extrinsic" in l][0]
v = [float(x) for x in line.split()[-1].split(",")]
truth = [0.9, 0.0, 2.0, 0.0, 1.0, 88.0]
tol = [0.1, 0.1, 0.1, 0.5, 0.5, 0.5]
bad = [(a, b) for a, b, t in zip(v, truth, tol) if abs(a - b) > t]
print("extrinsic found", v, "truth", truth)
sys.exit(1 if bad else 0)
EOF

python3 "$tools/awsim_to_bag.py" "$work/fake/bag" "$work/out/fake.mcap" --lidar-extrinsic 0.9,0,2.0,0,1,88 \
  --gnss-off-time 10:5 --initial-pose 0,0 --drop-lidar 20:2
python3 "$here/check_awsim.py" "$work/out/fake.mcap"
if [ -n "${KEEP_OUT:-}" ]; then
  mkdir -p "$KEEP_OUT" && cp -r "$work/out" "$work/fake" "$KEEP_OUT/"
fi
echo "tools/awsim tests passed"
