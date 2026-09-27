#!/usr/bin/env bash
# 小さな合成の地図で make_bag.py の bag を作り、ROS 2 で読み・再生して確かめる（CI の ros2 ジョブ）。
# colcon build の後、ROS 2 と /ws/install を source した状態で実行する。
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
tools="$(dirname "$here")"
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT

tiler="$(find /ws/install -type f -name tiled_pcd_map_tiler | head -1)"
demo="$(find /ws/install -type f -name tiled_pcd_map_demo | head -1)"
[ -x "$tiler" ] && [ -x "$demo" ] || { echo "tiled_pcd_map tools not found under /ws/install"; exit 1; }

# 合成の地図: 200 m × 120 m の地面と、道に沿った壁・柱（ASCII の PCD）
python3 - "$work/map.pcd" <<'EOF'
import sys
import numpy as np
rng = np.random.default_rng(0)
g = np.mgrid[0:200:0.4, 0:120:0.4].reshape(2, -1).T
ground = np.c_[g, rng.normal(0, 0.02, len(g))]
walls = []
for y in (40.0, 80.0):
    x = np.arange(0, 200, 0.2)
    for z in np.arange(0, 4, 0.3):
        walls.append(np.c_[x, np.full_like(x, y), np.full_like(x, z)])
for cx in range(10, 200, 25):
    t = np.linspace(0, 2 * np.pi, 40)
    for z in np.arange(0, 6, 0.3):
        walls.append(np.c_[cx + 0.5 * np.cos(t), 60 + 12 + 0.5 * np.sin(t), np.full_like(t, z)])
p = np.vstack([ground] + walls)
with open(sys.argv[1], "w") as f:
    f.write("# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
            f"WIDTH {len(p)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(p)}\nDATA ascii\n")
    np.savetxt(f, p, fmt="%.3f")
EOF
printf '10 60\n190 60\n190 70\n20 70\n' > "$work/route.txt"

"$tiler" -i "$work/map.pcd" -o "$work/tiles" --tile-size 20 --voxel-size 0.2
"$demo" "$work/tiles/tile_index.yaml" "$work/route.txt" "$work/frames.json" --speed 5 --record-interval 0.1
python3 "$tools/make_bag.py" "$work/tiles" "$work/frames.json" "$work/demo.mcap"
ros2 bag info "$work/demo.mcap"
python3 "$here/check_bag_ros2.py" "$work/demo.mcap"
