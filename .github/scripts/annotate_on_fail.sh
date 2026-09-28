#!/usr/bin/env bash
# コマンドを実行し、失敗したら出力の最後の 60 行を GitHub Actions の注釈（::error::）にも出す。
#   .github/scripts/annotate_on_fail.sh "<見出し>" <コマンド> [引数...]
set -o pipefail
title="$1"
shift
log="$(mktemp)"
"$@" 2>&1 | tee "$log"
rc=${PIPESTATUS[0]}
if [ "$rc" -ne 0 ]; then
  msg="$(tail -n 60 "$log" | sed -e 's/%/%25/g' -e 's/\r/%0D/g' | awk '{printf "%s%%0A", $0}')"
  echo "::error title=${title} (exit ${rc})::${msg}"
fi
rm -f "$log"
exit "$rc"
