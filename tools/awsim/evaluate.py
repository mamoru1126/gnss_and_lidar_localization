#!/usr/bin/env python3
"""推定ノードの出力（debug_csv_path の CSV）を真値と比べる（docs/validation_awsim.md 5 章）。

  python3 evaluate.py <出力.csv> <真値.csv> [--out report.md] [--from 0] [--to 0] \\
      [--max lat_rms=0.10 --max yaw_rms_deg=1.0 ...] [--min lidar_share=0.5 ...]

真値の CSV は awsim_to_bag.py の <out>_groundtruth.csv（t, x, y, z, roll, pitch, yaw。UTM）。
指標は INITIALIZING を除いた行で計算する。--from / --to は bag の最初の出力からの秒で、評価する区間を絞る。
--max / --min に指標の名前と値を与えると、満たさなければ終了コード 1 にする（CI や、条件を変えた比較に使う）。
必要なパッケージ: numpy。
"""
import argparse
import csv
import math
import sys
from pathlib import Path

import numpy as np


def load_csv(path):
    with open(path, newline="") as fp:
        # 推定ノードを止めたときに書きかけた最後の行など、列が足りない行は捨てる
        rows = [r for r in csv.DictReader(fp) if None not in r.values()]
    if not rows:
        sys.exit(f"{path}: 行が無い")
    return rows


def col(rows, k, conv=float):
    return np.array([conv(r[k]) for r in rows])


def wrap(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


def stats(v):
    v = np.abs(np.asarray(v, dtype=float))
    if len(v) == 0:
        return float("nan"), float("nan"), float("nan")
    return float(np.sqrt(np.mean(v ** 2))), float(np.percentile(v, 95)), float(np.max(v))


def evaluate(out_rows, gt_rows, t_from=0.0, t_to=0.0):
    t = col(out_rows, "t")
    t_start = t[0]
    g_t = col(gt_rows, "t")
    g_yaw = np.unwrap(col(gt_rows, "yaw"))
    gx = np.interp(t, g_t, col(gt_rows, "x"), left=np.nan, right=np.nan)
    gy = np.interp(t, g_t, col(gt_rows, "y"), left=np.nan, right=np.nan)
    gyaw = np.interp(t, g_t, g_yaw, left=np.nan, right=np.nan)
    status = np.array([r["status"] for r in out_rows])
    x, y, yaw = col(out_rows, "x"), col(out_rows, "y"), col(out_rows, "yaw")
    vx, vy, vyaw = col(out_rows, "var_x"), col(out_rows, "var_y"), col(out_rows, "var_yaw")
    rel = t - t_start
    use = np.isfinite(gx) & (status != "INITIALIZING") & (rel >= t_from)
    if t_to > 0:
        use &= rel <= t_to
    m = {"duration": float(t[-1] - t[0]), "rows": int(len(t)), "evaluated_rows": int(use.sum())}
    first = np.nonzero(status != "INITIALIZING")[0]
    m["init_time"] = float(t[first[0]] - t_start) if len(first) else float("nan")
    if use.sum() == 0:
        return m, {}
    dx, dy = x - gx, y - gy
    c, s = np.cos(gyaw), np.sin(gyaw)
    lon, lat = c * dx + s * dy, -s * dx + c * dy
    eyaw = wrap(yaw - gyaw)
    m["lat_rms"], m["lat_p95"], m["lat_max"] = stats(lat[use])
    m["lon_rms"], m["lon_p95"], m["lon_max"] = stats(lon[use])
    m["xy_rms"], m["xy_p95"], m["xy_max"] = stats(np.hypot(dx, dy)[use])
    m["yaw_rms_deg"], m["yaw_p95_deg"], m["yaw_max_deg"] = (math.degrees(v) for v in stats(eyaw[use]))
    # 出力の補正ステップ: 出力の増分から、真値の増分を引いたもの（FR-4）
    both = use[1:] & use[:-1]
    step = np.hypot(np.diff(x) - np.diff(gx), np.diff(y) - np.diff(gy))[both]
    m["step_max"] = float(step.max()) if len(step) else float("nan")
    # 共分散の整合性
    with np.errstate(divide="ignore", invalid="ignore"):
        in3 = (np.abs(dx) <= 3 * np.sqrt(vx)) & (np.abs(dy) <= 3 * np.sqrt(vy)) & (np.abs(eyaw) <= 3 * np.sqrt(vyaw))
        nees = dx ** 2 / vx + dy ** 2 / vy + eyaw ** 2 / vyaw
    m["within_3sigma"] = float(np.mean(in3[use]))
    m["nees_mean"] = float(np.nanmean(nees[use]))  # 3 自由度（対角だけ）: 整合していれば 3 前後
    if "raw_cov_xy" in out_rows[0]:
        # 縦（車の前後）と横に分けた整合性（推定値 raw とその共分散で。誤差 / σ の RMS は整合していれば 1 前後）
        rdx, rdy = col(out_rows, "raw_x") - gx, col(out_rows, "raw_y") - gy
        rvx, rvy, rcxy = col(out_rows, "raw_var_x"), col(out_rows, "raw_var_y"), col(out_rows, "raw_cov_xy")
        for name, (a, b) in (("lon", (c, s)), ("lat", (-s, c))):
            e = a * rdx + b * rdy
            with np.errstate(divide="ignore", invalid="ignore"):
                z = e / np.sqrt(a * a * rvx + 2 * a * b * rcxy + b * b * rvy)
            m[f"{name}_within_3sigma"] = float(np.mean(np.abs(z[use]) <= 3))
            m[f"{name}_z_rms"] = float(np.sqrt(np.nanmean(z[use] ** 2)))
    m["lidar_share"] = float(np.mean(np.char.find(status[use].astype(str), "LIDAR") >= 0))
    m["lost_share"] = float(np.mean(status[use] == "LOST"))
    m["dr_distance_max"] = float(np.max(col(out_rows, "dr_distance")[use]))
    per = {}
    for st in sorted(set(status[use])):
        k = use & (status == st)
        per[st] = dict(share=float(k.sum() / use.sum()), lat_rms=stats(lat[k])[0], lon_rms=stats(lon[k])[0],
                       yaw_rms_deg=math.degrees(stats(eyaw[k])[0]))
    return m, per


def report(m, per, title):
    lines = [f"# {title}", "", "| 指標 | 値 |", "|---|---|"]
    fmt = {"duration": "{:.1f} s", "init_time": "{:.1f} s", "within_3sigma": "{:.1%}", "lon_within_3sigma": "{:.1%}", "lat_within_3sigma": "{:.1%}",
           "lon_z_rms": "{:.2f}", "lat_z_rms": "{:.2f}", "lidar_share": "{:.1%}",
           "lost_share": "{:.1%}", "nees_mean": "{:.2f}", "rows": "{}", "evaluated_rows": "{}"}
    for k, v in m.items():
        f = fmt.get(k, "{:.2f}°" if k.endswith("_deg") else "{:.3f} m")
        lines.append(f"| {k} | {f.format(v)} |")
    if per:
        lines += ["", "| 状態 | 時間の割合 | 横 RMS | 縦 RMS | yaw RMS |", "|---|---|---|---|---|"]
        for st, d in per.items():
            lines.append(f"| {st} | {d['share']:.1%} | {d['lat_rms']:.3f} m | {d['lon_rms']:.3f} m | {d['yaw_rms_deg']:.2f}° |")
    return "\n".join(lines) + "\n"


def parse_limits(items):
    out = {}
    for it in items:
        k, v = it.split("=")
        out[k] = float(v)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("output", type=Path)
    ap.add_argument("groundtruth", type=Path)
    ap.add_argument("--out", type=Path, default=None, help="Markdown のレポートを書く")
    ap.add_argument("--title", default="")
    ap.add_argument("--from", dest="t_from", type=float, default=0.0)
    ap.add_argument("--to", dest="t_to", type=float, default=0.0)
    ap.add_argument("--max", nargs="*", default=[], action="extend", metavar="NAME=VALUE")
    ap.add_argument("--min", nargs="*", default=[], action="extend", metavar="NAME=VALUE")
    args = ap.parse_args()
    m, per = evaluate(load_csv(args.output), load_csv(args.groundtruth), args.t_from, args.t_to)
    text = report(m, per, args.title or args.output.name)
    print(text)
    if args.out:
        args.out.write_text(text)
    bad = []
    for k, v in parse_limits(args.max).items():
        if not (m.get(k, float("nan")) <= v):
            bad.append(f"{k} = {m.get(k)} > {v}")
    for k, v in parse_limits(args.min).items():
        if not (m.get(k, float("nan")) >= v):
            bad.append(f"{k} = {m.get(k)} < {v}")
    for b in bad:
        print("NG:", b)
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
