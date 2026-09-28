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
