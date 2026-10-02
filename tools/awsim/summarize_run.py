#!/usr/bin/env python3
"""run_scenario.sh の結果のフォルダ（tools/awsim/log/<名前>/）から summary.md を作る。

  python3 summarize_run.py <フォルダ> [--scenario v_a1] [--run nsj_run1] [--rc 0] [-- 変換のオプション ...]

読むもの: terminal.log の先頭、extrinsic.log、convert.log、node.log、evaluate.md、output.csv.gz と groundtruth.csv.gz。
無いものは飛ばす（途中で失敗したときも、そこまでの要約を作る）。必要なパッケージ: numpy。
"""
import argparse
import collections
import csv
import gzip
import re
import sys
from pathlib import Path

import numpy as np


def read(p):
    """ファイルの中身（色の制御文字は取る）。"""
    return re.sub(r"\x1b\[[0-9;]*m", "", p.read_text(errors="replace")) if p.exists() else ""


def pick(text, keys):
    return [l.rstrip() for l in text.splitlines() if any(k in l for k in keys)]


def node_summary(text):
    """状態の移り変わりと、警告・エラーの種類ごとの数（数字を # にまとめる）。"""
    trans, kinds, first = [], collections.Counter(), {}
    t0 = None
    for line in text.splitlines():
        m = re.search(r"\[(INFO|WARN|ERROR)\] \[(\d+\.\d+)\] \[gll_localizer\]: (.*)", line)
        if not m:
            continue
        lvl, t, msg = m.group(1), float(m.group(2)), m.group(3)
        if "filter initialized" in msg and t0 is None:
            t0 = t
        if msg.startswith("status: "):
            trans.append((t, msg[8:]))
        if lvl in ("WARN", "ERROR"):
            k = f"{lvl} " + re.sub(r"-?\d+(\.\d+)?", "#", msg)
            kinds[k] += 1
            first.setdefault(k, t)
    return trans, kinds, first, t0


def load_csv_gz(p):
    if not p.exists():
        return None
    with gzip.open(p, "rt") as fp:
        return list(csv.DictReader(fp))


def segments(out_rows, gt_rows, width=10.0):
    """width 秒ごとの誤差と、いちばん長かった状態。"""
    t = np.array([float(r["t"]) for r in out_rows])
    st = np.array([r["status"] for r in out_rows])
    x, y, yaw = (np.array([float(r[k]) for r in out_rows]) for k in ("raw_x", "raw_y", "raw_yaw"))
    gt = np.array([float(r["t"]) for r in gt_rows])
    gyaw = np.unwrap([float(r["yaw"]) for r in gt_rows])
    gx = np.interp(t, gt, [float(r["x"]) for r in gt_rows])
    gy = np.interp(t, gt, [float(r["y"]) for r in gt_rows])
    gw = np.interp(t, gt, gyaw)
    h = 0.1
    yr = (np.interp(t + h, gt, gyaw) - np.interp(t - h, gt, gyaw)) / (2 * h)
    dx, dy = x - gx, y - gy
    lat = -np.sin(gw) * dx + np.cos(gw) * dy
    lon = np.cos(gw) * dx + np.sin(gw) * dy
    eyaw = np.degrees((yaw - gw + np.pi) % (2 * np.pi) - np.pi)
    rows = []
    for a in np.arange(t[0], t[-1], width):
        k = (t >= a) & (t < a + width)
        if not k.any():
            continue
        c = collections.Counter(st[k]).most_common(2)
        rows.append((a - t[0], c, np.sqrt(np.mean(lat[k] ** 2)), np.sqrt(np.mean(lon[k] ** 2)),
                     np.sqrt(np.mean(eyaw[k] ** 2)), np.max(np.abs(yr[k]))))
    return rows


def lidar_section(out_rows):
    """出力の CSV の lidar_* 列（直近の照合が行ごとに繰り返し出る）から、照合ごとの結果をまとめる。"""
    seen, scans = set(), []
    for r in out_rows:
        if not r.get("lidar_t") or r["lidar_t"] in seen:
            continue
        seen.add(r["lidar_t"])
        scans.append(r)
    if not scans:
        return []
    L = ["", f"## LiDAR の照合（{len(scans)} 回。出力の CSV から）", "",
         "| 結果 | 回数 | inlier 中央値 | overlap 中央値 | overlap（地図の近く）中央値 |", "|---|---|---|---|---|"]
    by = collections.defaultdict(list)
    for r in scans:
        by[r["lidar_status"]].append(r)
    for k, rs in sorted(by.items(), key=lambda kv: -len(kv[1])):
        med = [np.median([float(r[c]) for r in rs]) for c in ("lidar_inlier", "lidar_overlap", "lidar_overlap_near")]
        L.append(f"| {k} | {len(rs)} | {med[0]:.2f} | {med[1]:.2f} | {med[2]:.2f} |")
    return L


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dir", type=Path)
    ap.add_argument("--scenario", default="")
    ap.add_argument("--run", default="")
    ap.add_argument("--rc", type=int, default=0)
    argv = sys.argv[1:]
    opts = argv[argv.index("--") + 1:] if "--" in argv else []
    a = ap.parse_args(argv[:argv.index("--")] if "--" in argv else argv)
    a.opts = opts
    d = a.dir
    L = [f"# {d.name}", ""]
    head = read(d / "terminal.log").splitlines()[:2]
    L += [f"- {'成功' if a.rc == 0 else '**途中で失敗**（terminal.log を見る）'}",
          f"- シナリオ {a.scenario}、録った bag {a.run}、変換のオプション `{' '.join(a.opts) or '（なし）'}`"]
    L += [f"- {h}" for h in head[1:]]

    ex = read(d / "extrinsic.log")
    lid = pick(read(d / "terminal.log"), ["LiDAR: --lidar-extrinsic"])
    if ex or lid:
        L += ["", "## LiDAR の取り付け位置・スタンプのずれ", "", "```"]
        L += pick(ex, ["最初の値", "見つけた値", "点群のスタンプのずれ", "  -150", "stamp_offset", "注意"]) + lid
        L += ["```"]

    cv = read(d / "convert.log")
    if cv:
        L += ["", "## 変換", "", "```"]
        L += pick(cv, ["真値 ", "最初の停止", "IMU（", "車速と真値", "IMU の yaw", "時刻のずれ", "横向きの速さ", "パラメータ ", "GNSS:", "全体の地図",
                       "注意", "Error", "Traceback"])
        L += ["```"]

    ev = read(d / "evaluate.md")
    if ev:
        L += ["", "## 評価", ""] + ev.splitlines()[2:]

    out_rows, gt_rows = load_csv_gz(d / "output.csv.gz"), load_csv_gz(d / "groundtruth.csv.gz")
    if out_rows and gt_rows:
        L += ["", "## 10 秒ごと（raw の姿勢の誤差。|yaw レート| は真値の最大）", "",
              "| 時刻 [s] | 状態 | 横 RMS [m] | 縦 RMS [m] | yaw RMS [°] | yaw レート [rad/s] |", "|---|---|---|---|---|---|"]
        for t, c, la, lo, ya, yr in segments(out_rows, gt_rows):
            s = " / ".join(f"{k} {n / sum(v for _, v in c) * 100:.0f}%" for k, n in c) if len(c) > 1 else c[0][0]
            L.append(f"| {t:.0f} | {s} | {la:.3f} | {lo:.3f} | {ya:.2f} | {yr:.2f} |")

    if out_rows and out_rows[0].get("lidar_t") is not None:
        L += lidar_section(out_rows)

    nl = read(d / "node.log")
    if nl:
        trans, kinds, first, t0 = node_summary(nl)
        rel = (lambda t: t - t0) if t0 else (lambda t: t)
        ms = [l.split("]: ", 1)[-1] for l in nl.splitlines() if "LiDAR matching summary" in l]
        L += ["", "## 推定ノードのログ", ""] + ([f"照合: `{ms[-1]}`", ""] if ms else []) + [ f"状態の移り変わり {len(trans)} 回（時刻は初期化からの秒）:", "", "```"]
        L += [f"{rel(t):8.1f}  {m}" for t, m in trans[:60]]
        if len(trans) > 60:
            L.append(f"... ほか {len(trans) - 60} 回")
        L += ["```", "", "警告・エラー（数字を # にまとめた種類ごと）:", "", "| 回数 | 最初（s） | 内容 |", "|---|---|---|"]
        for k, n in kinds.most_common(30):
            L.append(f"| {n} | {rel(first[k]):.1f} | `{k[:160]}` |")
    (d / "summary.md").write_text("\n".join(L) + "\n")
    print((d / "summary.md").read_text())


if __name__ == "__main__":
    sys.exit(main())
