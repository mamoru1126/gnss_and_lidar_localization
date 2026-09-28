#!/usr/bin/env bash
# Dev Container の中で colcon build する。引数は colcon build にそのまま渡す（例: .devcontainer/build.sh --packages-select gll_ros2）。
# ビルドの後、パッケージごとの compile_commands.json を /ws/build/compile_commands.json にまとめる（VS Code の C/C++ 拡張が読む）。
set -eo pipefail
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
cd /ws
colcon build --symlink-install "$@" \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
python3 - <<'PY'
import glob
import json
entries = []
for f in sorted(glob.glob("/ws/build/*/compile_commands.json")):
    with open(f) as fp:
        entries += json.load(fp)
with open("/ws/build/compile_commands.json", "w") as fp:
    json.dump(entries, fp, indent=1)
print(f"/ws/build/compile_commands.json: {len(entries)} entries")
PY
