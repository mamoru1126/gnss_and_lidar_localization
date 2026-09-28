#!/usr/bin/env bash
# Dev Container を作った直後に 1 回だけ実行される（devcontainer.json の postCreateCommand）。
set -eo pipefail
# colcon の出力の名前付きボリュームは、イメージの中の持ち主（UID 1000）で作られる。
# VS Code が ubuntu の UID をホストの利用者に合わせた場合に書き込めるよう、持ち主を今の利用者にする。
sudo chown -R "$(id -u):$(id -g)" /ws/build /ws/install /ws/log
# rosbag や地図の置き場所（GLL_DATA。.gitignore 済み）
mkdir -p "${GLL_DATA:-/ws/src/gnss_and_lidar_localization/data}"
# 最初のビルド（IntelliSense 用の compile_commands.json もできる）
bash "$(dirname "$0")/build.sh"
