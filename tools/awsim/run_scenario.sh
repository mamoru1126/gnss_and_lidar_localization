#!/usr/bin/env bash
# 検証シナリオを 1 回で通す（dev コンテナで。ROS 2 と /ws/install を source した状態）:
#   LiDAR の取り付け位置・スタンプのずれ（初回だけ。結果は data に覚えておく）→ 変換 → 推定ノードで再生・録画 → 評価
# ターミナルの出力と結果は tools/awsim/log/<名前>/ にまとめて残る（summary.md が要約）。--push でコミットして push する。
#
#   tools/awsim/run_scenario.sh <シナリオ> [--run nsj_run1] [--name 名前] [--rate 1] [--seed 0]
#                                 [--set KEY=VALUE ...] [--calib] [--no-record] [--push] [-- awsim_to_bag.py に足すオプション ...]
#
# 例: 追跡の照合を VGICP にして、別の名前で残す
#   tools/awsim/run_scenario.sh v_a1 --name v_a1_vgicp --set lidar.registration=vgicp --push
#
# シナリオ（docs/validation_awsim.md 4 章）:
#   v_a0  GNSS + LiDAR（動作確認）
#   v_a1  LiDAR だけ（--no-gnss --initial-pose 0,0）
#   v_a2  LiDAR だけ・初期姿勢をずらして地図の上で初期化（--no-gnss --initial-pose 2,20 --seed N）
#   その他の名前: 変換のオプションは -- の後ろに全部書く
# --set: 推定ノードのパラメータを上書きする（何度でも。例: --set lidar.registration=vgicp --set lidar.vgicp_voxel_size=0.5）
# --calib: LiDAR の取り付け位置とスタンプのずれを求め直す（既定は、前に求めた値があればそれを使う）
# --no-record: 推定の様子を録らない（録った bag は data/awsim/out/<名前>_rec。大きいのでリポジトリには入れない）
set -uo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd "$here/../.." && pwd)"
D="${GLL_DATA:-$repo/data}/awsim"

scenario="" run="nsj_run1" name="" rate=1 seed=0 calib=0 record=1 push=0 extra=() sets=()
usage() { sed -n '2,22p' "$0"; }
while [ $# -gt 0 ]; do
  case "$1" in
    --run) run="$2"; shift 2 ;;
    --name) name="$2"; shift 2 ;;
    --rate) rate="$2"; shift 2 ;;
    --seed) seed="$2"; shift 2 ;;
    --calib) calib=1; shift ;;
    --no-record) record=0; shift ;;
    --push) push=1; shift ;;
    --set) [ $# -ge 2 ] && [[ "$2" == *=* ]] || { echo "--set には KEY=VALUE を渡す"; exit 1; }
           sets+=("$2"); shift 2 ;;
    --) shift; extra=("$@"); break ;;
    -h|--help) usage; exit 0 ;;
    -*) echo "知らないオプション: $1（awsim_to_bag.py のオプションは -- の後ろに書く）"; usage; exit 1 ;;
    *) [ -z "$scenario" ] || { echo "シナリオが 2 つある: $scenario と $1"; usage; exit 1; }
       scenario="$1"; shift ;;
  esac
done
[ -n "$scenario" ] || { usage; exit 1; }
name="${name:-$scenario}"
case "$scenario" in
  v_a0) opts=() ;;
  v_a1) opts=(--no-gnss --initial-pose 0,0) ;;
  v_a2) opts=(--no-gnss --initial-pose 2,20 --seed "$seed") ;;
  *) opts=() ;;
esac
opts+=("${extra[@]}")
[ ${#sets[@]} -gt 0 ] && opts+=(--set "${sets[@]}")

src="$D/$run"
tiles="$D/nsj_tiles"
out="$D/out/$name"
logdir="$repo/tools/awsim/log/$name"
[ -e "$src" ] || { echo "$src が無い（--run で録った bag の名前を指定する）"; exit 1; }
[ -f "$tiles/tile_index.yaml" ] || { echo "$tiles が無い（README の 2 章の tiled_pcd_map_tiler で作る）"; exit 1; }
mkdir -p "$D/out"
rm -rf "$logdir" "$out" "${out}_rec"
mkdir -p "$logdir"
exec > >(tee "$logdir/terminal.log") 2>&1

step() { echo; echo "======== $*"; }
fail() { echo "!!!! $*"; finish 1; }
finish() {
  local rc="$1"
  step "要約"
  python3 "$here/summarize_run.py" "$logdir" --scenario "$scenario" --run "$run" --rc "$rc" -- "${opts[@]}" || true
  if [ "$push" = 1 ]; then
    step "コミットして push"
    # push の成否は、手元と追跡先のコミットが同じになったかで確かめる（認証で失敗しても 0 を返すことがあるため）
    if (cd "$repo" && git add "tools/awsim/log/$name" && git commit -q -m "awsim log: $name" && git push -q) &&
       [ "$(cd "$repo" && git rev-parse HEAD)" = "$(cd "$repo" && git rev-parse '@{u}' 2>/dev/null)" ]; then
      echo "push した: tools/awsim/log/$name"
    else
      echo "push できなかった（コミットはしてある）。ホストで次を実行する:"
      echo "  git push"
    fi
  else
    echo "結果: tools/awsim/log/$name/（summary.md、グラフは report.html）。送るときは --push を付けるか:"
    echo "  git add tools/awsim/log/$name && git commit -m 'awsim log: $name' && git push"
  fi
  exit "$rc"
}

echo "シナリオ $scenario（名前 $name）、録った bag $src、変換のオプション: ${opts[*]:-（なし）}"
echo "commit $(cd "$repo" && git rev-parse --short HEAD 2>/dev/null)$(cd "$repo" && git diff --quiet 2>/dev/null || echo ' +変更あり')、$(date '+%F %T')"

# ---- LiDAR の取り付け位置とスタンプのずれ（前に求めた値を data に覚えておく）
cal="$D/out/${run}_lidar_calib.env"
if [ "$calib" = 1 ] || [ ! -f "$cal" ]; then
  step "LiDAR の取り付け位置と点群のスタンプのずれ（check_lidar_extrinsic.py）"
  python3 "$here/check_lidar_extrinsic.py" "$src" "$tiles" --no-yaw-sweep --time-offset --turning | tee "$logdir/extrinsic.log"
  ext="$(grep -- '--lidar-extrinsic' "$logdir/extrinsic.log" | tail -1 | awk '{print $NF}')"
  tof="$(sed -n 's/.*lidar\.stamp_offset: \([-0-9.]*\).*/\1/p' "$logdir/extrinsic.log" | tail -1)"
  [ -n "$ext" ] || fail "取り付け位置が求まらなかった"
  printf 'EXT=%s\nTOF=%s\n' "$ext" "${tof:-0.0}" > "$cal"
fi
# shellcheck disable=SC1090
. "$cal"
echo "LiDAR: --lidar-extrinsic $EXT --lidar-stamp-offset $TOF（$cal）"

step "変換（awsim_to_bag.py）"
python3 "$here/awsim_to_bag.py" "$src" "$out" --tiles "$tiles" --lidar-extrinsic "$EXT" --lidar-stamp-offset "$TOF" \
  "${opts[@]}" 2>&1 | tee "$logdir/convert.log"
[ "${PIPESTATUS[0]}" = 0 ] || fail "変換に失敗した"

step "推定ノードで再生（replay.sh）"
rec=()
[ "$record" = 1 ] && rec=(--record "${out}_rec")
"$here/replay.sh" "$out" --rate "$rate" "${rec[@]}" 2>&1 | tee "$logdir/replay.log"
cp "${out}_node.log" "$logdir/node.log" 2>/dev/null || true
cp "${out}_rec.record.log" "$logdir/record.log" 2>/dev/null || true
[ -s "${out}_output.csv" ] || fail "推定の出力（${out}_output.csv）が無い"

step "評価（evaluate.py）"
python3 "$here/evaluate.py" "${out}_output.csv" "${out}_groundtruth.csv" --title "$name" --out "$logdir/evaluate.md" >/dev/null
cat "$logdir/evaluate.md"

# 結果のファイル（CSV は gzip で小さくする。録った bag は大きいので入れない）
cp "${out}_params.yaml" "$logdir/params.yaml"
cp "${out}_maps.yaml" "$logdir/maps.yaml" 2>/dev/null || true
gzip -c "${out}_output.csv" > "$logdir/output.csv.gz"
gzip -c "${out}_groundtruth.csv" > "$logdir/groundtruth.csv.gz"
# 真値と推定のグラフ（ブラウザで開く）
python3 "$here/plot_run.py" "$logdir" --title "$name" || echo "グラフを作れなかった（plot_run.py）"
[ "$record" = 1 ] && echo "録った bag: ${out}_rec（リポジトリの data/awsim/out/${name}_rec。Foxglove で開き、frame map_local で見る）"
finish 0
