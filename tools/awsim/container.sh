#!/usr/bin/env bash
# 記録と走行に使う ROS 2 Humble のコンテナに入る（ホストで。AWSIM を起動してから）。端末ごとに実行してよい。
# 中ではリポジトリがそのまま見え（/ws/src/gnss_and_lidar_localization）、GLL_DATA がリポジトリの data/ を指す。
# 環境変数（ROS_LOCALHOST_ONLY・RMW_IMPLEMENTATION）はコンテナに入っているので、設定は要らない。
#
#   tools/awsim/container.sh              # bash
#   tools/awsim/container.sh <コマンド>    # そのコマンドだけ動かす
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
mkdir -p "$repo/data/awsim"
export HOST_UID="$(id -u)" HOST_GID="$(id -g)"
exec docker compose -f "$repo/docker/compose.yaml" run --rm awsim "${@:-bash}"
