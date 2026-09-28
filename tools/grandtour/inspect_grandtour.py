#!/usr/bin/env python3
"""GrandTour の事前確認（docs/validation_grandtour.md 3.3 節）。落としたミッションを調べ、レポートを作る。

  python3 inspect_grandtour.py --data-dir $GLL_DATA/grandtour --out $GLL_DATA/grandtour/report
  python3 inspect_grandtour.py --data-dir $GLL_DATA/grandtour --missions ETH-1 ETH-3 --out report_eth

出力（--out のフォルダ）:
  report.md          結果の表と判定（これを Claude に渡す）
  site_*.png         場所ごとの軌跡の図
  missions.csv       ミッションごとの数値
  overlap.csv        ミッションの組ごとの経路の重なり
  results.json       上の数値をまとめたもの（機械で読む用）

light の段階（download.py --preset light）のデータで、軌跡の重なり・真値・オドメトリの約束事を調べる。
lidar の段階のデータがあれば、点群と IMU も調べる。必要なパッケージ: numpy、matplotlib、zarr（3.x）、pyproj。
"""
import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import analysis as A  # noqa: E402
import gt_zarr as Z  # noqa: E402
import missions as M  # noqa: E402


def conj(q):
    q = np.array(q, dtype=float, copy=True)
    q[:, :3] *= -1
    return q


def try_read(fn, *args):
    try:
        return fn(*args)
    except KeyError:
        return None


# ------------------------------------------------------------------ 読み込み
def load(mdir, zone):
    root = Z.open_mission(mdir)
    have = Z.topics(root)
    d = dict(root=root, have=have)
    if "navsatfix_cpt7_ie_tc" in have:
        nav = Z.read_navsatfix(root)
        if len(nav["t"]):
            e, n = Z.to_utm(nav["lat"], nav["lon"], zone)
            nav["E"], nav["N"] = e, n
            d["nav"] = nav
    for key, topic in [("tc", "cpt7_ie_tc_odometry"), ("rt", "cpt7_ie_rt_odometry"),
                       ("leg", "anymal_state_odometry"), ("dlio", "dlio_map_odometry")]:
        if topic in have:
            d[key] = Z.read_odometry(root, topic)
    if "prism_position" in have:
        d["prism"] = Z.read_prism(root)
    if "tf" in have:
        d["tf"] = Z.tf_table(root)
    return d


# ------------------------------------------------------------------ ミッションごとの確認
def odom_checks(o):
    """姿勢の解釈（standard / inverted）と、twist の座標系（child / parent）。"""
    if o is None or len(o["t"]) < 10:
        return None
    v = o["v"] if "v" in o and len(o["t_twist"]) == len(o["t"]) and np.allclose(o["t_twist"], o["t"]) else None
    c = A.odom_convention(o["t"], o["p"], o["q"], v)
    c["frame_id"] = o["frame_id"]
    return c


def standardize(o, semantics):
    """オドメトリの姿勢を standard の解釈（機体を親フレームで表したもの）にそろえた (p, q)。"""
    return A.standardize_pose(o["p"], o["q"], semantics)


def _pq_at(o, t, semantics):
    """standard にそろえた姿勢を時刻 t で取る（位置は線形補間、向きは最も近いもの）。"""
    p, q = standardize(o, semantics)
    idx = np.clip(np.searchsorted(o["t"], t), 0, len(o["t"]) - 1)
    pi = A.interp(o["t"], p, t)
    bad = ~np.all(np.isfinite(pi), axis=1)
    pi[bad] = p[idx[bad]]
    return pi, q[idx]


def mission_checks(d, step):
    r = {}
    nav = d.get("nav")
    t_all = [x["t"] for x in (nav, d.get("leg"), d.get("tc")) if x is not None and len(x["t"])]
    r["duration"] = max(t[-1] for t in t_all) - min(t[0] for t in t_all) if t_all else float("nan")
    if nav is not None:
        xy = np.column_stack([nav["E"], nav["N"]])
        r["gnss_n"] = len(nav["t"])
        r["gnss_hz"] = 1.0 / np.median(np.diff(nav["t"])) if len(nav["t"]) > 1 else float("nan")
        gp = A.gaps(nav["t"], 1.0)
        r["gnss_gaps"] = len(gp)
        r["gnss_gap_total"] = sum(g[1] for g in gp)
        r["gnss_span"] = nav["t"][-1] - nav["t"][0]
        if "cov" in nav:
            sh = np.sqrt(np.maximum(nav["cov"][:, 0], nav["cov"][:, 4]))
            r["gnss_std_p50"], r["gnss_std_p95"] = A.pct(sh, 50), A.pct(sh, 95)
        r["length"] = A.path_length(xy)
        r["alt_range"] = float(nav["alt"].max() - nav["alt"].min())
        r["path"], r["head"] = A.resample_path(xy, step)
        r["xy"] = xy
    leg = d.get("leg")
    if leg is not None and "v" in leg:
        sp = np.linalg.norm(leg["v"][:, :2], axis=1)
        r["stationary_start"] = A.stationary_start(leg["t_twist"], sp, leg["w"][:, 2])
        r["speed_p50"], r["speed_p95"] = A.pct(sp, 50), A.pct(sp, 95)
        r["lateral_p95"] = A.pct(np.abs(leg["v"][:, 1]), 95)
    pr = d.get("prism")
    if pr is not None and len(pr["t"]) and r["duration"] == r["duration"]:
        bins = np.unique(np.floor(pr["t"] - pr["t"][0]))
        r["prism_n"] = len(pr["t"])
        r["prism_cover"] = len(bins) / max(r["duration"], 1.0)

    # オドメトリの約束事
    r["conv_leg"] = odom_checks(leg)
    r["conv_tc"] = odom_checks(d.get("tc"))
    r["conv_dlio"] = odom_checks(d.get("dlio"))

    # 静的 TF の解釈: 真値（box_base = cpt7_imu の姿勢）と脚のオドメトリ（base の姿勢）の回転から、
    # base → box_base の向きを求め、tf から計算したものと比べる
    tc = d.get("tc")
    if tc is not None and leg is not None and "tf" in d and r["conv_tc"] and r["conv_leg"]:
        qa = tc["q"] if r["conv_tc"]["semantics"] == "standard" else conj(tc["q"])
        qb = leg["q"] if r["conv_leg"]["semantics"] == "standard" else conj(leg["q"])
        he = A.handeye_rotation(tc["t"], qa, leg["t"], qb)
        if he is not None:
            try:
                R_tf = Z.static_transform(d["tf"], "base", "box_base")[:3, :3]
                r["tf_check"] = dict(n=he["n"], fit_rms_deg=he["rms_deg"],
                                     diff_official_deg=A.angle_between(he["X"], R_tf),
                                     diff_inverse_deg=A.angle_between(he["X"], R_tf.T))
            except KeyError as e:
                r["tf_check"] = dict(error=f"tf に {e} が無い")

    # ie_tc と ie_rt の差（同じフレームのとき）
    rt = d.get("rt")
    if tc is not None and rt is not None and tc["frame_id"] == rt["frame_id"]:
        prt = A.interp(rt["t"], rt["p"][:, :2], tc["t"])
        e = np.linalg.norm(prt - tc["p"][:, :2], axis=1)
        r["rt_vs_tc"] = dict(p50=A.pct(e, 50), p95=A.pct(e, 95), max=float(np.nanmax(e)))

    # 真値とトータルステーションの差（プリズムのレバーアームも同時に求める）
    if pr is not None and tc is not None and len(pr["t"]) > 20 and r["conv_tc"]:
        ok = (pr["t"] >= tc["t"][0]) & (pr["t"] <= tc["t"][-1])
        if ok.sum() > 20:
            p_w, q = _pq_at(tc, pr["t"][ok], r["conv_tc"]["semantics"])
            fit = A.fit_station(pr["p"][ok], p_w, A.quat_to_rot(q))
            r["prism_fit"] = fit
    return r


def lidar_checks(d, stationary):
    """lidar の段階のデータがあれば、点群と IMU を調べる。"""
    root, have = d["root"], d["have"]
    r = {}
    topic = "livox_points_undistorted" if "livox_points_undistorted" in have else (
        "livox_points" if "livox_points" in have else None)
    if topic:
        n = Z.scan_count(root, topic)
        ts = np.asarray(root[topic]["timestamp"][:], dtype=float)
        idx = np.linspace(0, n - 1, min(n, 20)).astype(int)
        counts = [len(Z.read_scan(root, topic, i)["points"]) for i in idx]
        first = Z.read_scan(root, topic, 0)
        r["lidar"] = dict(topic=topic, fields=Z.point_fields(root, topic), scans=n,
                          hz=1.0 / np.median(np.diff(ts)) if n > 1 else float("nan"),
                          points_p50=float(np.median(counts)))
        if "intensity" in first:
            r["lidar"]["intensity_range"] = (float(first["intensity"].min()), float(first["intensity"].max()))
        if "tf" in d:
            try:
                T = Z.static_transform(d["tf"], "base", root[topic].attrs.get("frame_id", "livox_lidar"))
                pb = first["points"] @ T[:3, :3].T + T[:3, 3]
                rr = np.hypot(pb[:, 0], pb[:, 1])
                ring = (rr > 1.0) & (rr < 6.0)
                if ring.sum() > 50:
                    ground = A.pct(pb[ring, 2], 5)
                    r["lidar"]["base_height"] = -ground
                    near = (rr < 0.8) & (pb[:, 2] > ground + 0.05)
                    if near.sum() > 10:
                        r["lidar"]["self_box"] = (pb[near].min(0), pb[near].max(0))
                r["lidar"]["extrinsic"] = T
            except KeyError as e:
                r["lidar"]["tf_error"] = str(e)
    for imu in ("adis_imu", "livox_imu"):
        if imu in have:
            m = Z.read_imu(root, imu)
            t0 = m["t"][0]
            win = m["t"] < t0 + max(min(stationary or 0.0, 2.0), 0.5)
            r[imu] = dict(description=m["description"],
                          hz=1.0 / np.median(np.diff(m["t"])) if len(m["t"]) > 1 else float("nan"),
                          acc_norm=float(np.median(np.linalg.norm(m["acc"][win], axis=1))),
                          gyro_norm=float(np.median(np.linalg.norm(m["gyro"][win], axis=1))),
                          at_rest=bool(stationary and stationary >= 0.5))
    return r


# ------------------------------------------------------------------ 図
def plot_sites(groups, results, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    files = []
    for g in groups:
        g = [c for c in g if "xy" in results[c]]
        if not g:
            continue
        fig, ax = plt.subplots(figsize=(7, 7))
        for c in g:
            xy = results[c]["xy"]
            ax.plot(xy[:, 0], xy[:, 1], lw=1.2, label=c)
            ax.plot(xy[0, 0], xy[0, 1], "o", ms=5, color=ax.lines[-1].get_color())
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_xlabel("UTM E [m]")
        ax.set_ylabel("UTM N [m]")
        ax.ticklabel_format(useOffset=False, style="plain")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
        ax.set_title("site: " + ", ".join(g) + " (o = start)")
        name = "site_" + "_".join(c.replace("Ö", "O").replace("Ä", "A") for c in g) + ".png"
        fig.tight_layout()
        fig.savefig(out / name, dpi=110)
        plt.close(fig)
        files.append(name)
    return files


# ------------------------------------------------------------------ レポート
def f(x, fmt="{:.2f}"):
    if x is None or (isinstance(x, float) and not np.isfinite(x)):
        return "-"
    return fmt.format(x)


def write_report(out, codes, folders, results, overlaps, groups, figs, radius):
    L = ["# GrandTour 事前確認レポート", ""]
    L += ["生成: tools/grandtour/inspect_grandtour.py。このファイルを Claude に渡す。", ""]

    L += ["## 1. ミッションの概要", "",
          "| ミッション | フォルダ | 時間 [s] | 経路長 [m] | GNSS [Hz] | GNSS の欠け（>1 s） | 水平 σ p50 / p95 [m] | 高さの幅 [m] | 最初の静止 [s] | 速さ p50 / p95 [m/s] | 横速度 p95 [m/s] | MS60 の範囲 |",
          "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for c, fo in zip(codes, folders):
        r = results[c]
        L.append(f"| {c} | {fo} | {f(r.get('duration'), '{:.0f}')} | {f(r.get('length'), '{:.0f}')} | "
                 f"{f(r.get('gnss_hz'), '{:.1f}')} | {r.get('gnss_gaps', '-')} 回・{f(r.get('gnss_gap_total'), '{:.0f}')} s | "
                 f"{f(r.get('gnss_std_p50'), '{:.3f}')} / {f(r.get('gnss_std_p95'), '{:.3f}')} | {f(r.get('alt_range'), '{:.1f}')} | "
                 f"{f(r.get('stationary_start'), '{:.1f}')} | {f(r.get('speed_p50'))} / {f(r.get('speed_p95'))} | "
                 f"{f(r.get('lateral_p95'))} | {f(r.get('prism_cover', float('nan')) * 100 if 'prism_cover' in r else None, '{:.0f} %')} |")
    L += ["", "- 最初の静止: 脚のオドメトリの速度が 0.05 m/s・0.05 rad/s 未満の間。`attitude.static_init_time` を決める材料。",
          "- MS60 の範囲: トータルステーションの測定がある秒の割合。", ""]

    L += [f"## 2. ミッションどうしの経路の重なり（{radius:.0f} m 以内）", "",
          "行 = 地図を作るミッション、列 = 照合するミッション。値 = 照合するミッションの経路のうち、地図の経路の近くを通る割合と長さ（同じ向き / 逆向きの長さ）。", ""]
    for g in groups:
        g = [c for c in g if "path" in results[c]]
        if len(g) < 2:
            continue
        L += ["| 地図 \\ 照合 | " + " | ".join(g) + " |", "|---|" + "---|" * len(g)]
        for a in g:
            row = []
            for b in g:
                if a == b:
                    row.append("—")
                else:
                    o = overlaps[(a, b)]
                    row.append(f"{o['frac'] * 100:.0f} %・{o['length']:.0f} m（{o['same']:.0f} / {o['opposite']:.0f}）")
            L.append(f"| {a} | " + " | ".join(row) + " |")
        L.append("")
    ranked = sorted(overlaps.items(), key=lambda kv: -kv[1]["length"])
    if ranked:
        L += ["重なりが長い組（上位 5）:", ""]
        for (a, b), o in ranked[:5]:
            L.append(f"- {a} → {b}: {o['length']:.0f} m（照合側の経路の {o['frac'] * 100:.0f} %。同じ向き {o['same']:.0f} m・逆向き {o['opposite']:.0f} m）")
        L.append("")
    for fn in figs:
        L += [f"![{fn}]({fn})", ""]

    L += ["## 3. データの約束事の確認", "",
          "### 3.1 オドメトリの姿勢と速度", "",
          "姿勢の解釈（standard = 姿勢は機体を親フレームで表したもの。ROS の Odometry と同じ。inverted = その逆）と、速度（twist）の座標系"
          "（child = 機体、parent = 親フレーム）を、位置を微分した速度との差 [m/s] で決める（4 通りのうち最小のもの）。"
          " 速度で決まらないときは、機体座標系で見た移動の向きのそろい方（0〜1）で姿勢の解釈だけを決める。"
          " 歩く向き: 機体の +x からの角度（0° なら前向き、180° ならそのフレームは後ろ向き）。", "",
          "| ミッション | トピック | frame_id | 速度の差 std/child・std/parent・inv/child・inv/parent | そろい方 std / inv | 判定 | 決め手 | 歩く向き [deg] |",
          "|---|---|---|---|---|---|---|---|"]
    for c in codes:
        for key, name in [("conv_leg", "anymal_state_odometry"), ("conv_tc", "cpt7_ie_tc_odometry"), ("conv_dlio", "dlio_map_odometry")]:
            cv = results[c].get(key)
            if not cv:
                continue
            rs = cv["residual"]
            rtxt = "・".join(f(rs.get(k), "{:.3f}") for k in ("standard/child", "standard/parent", "inverted/child", "inverted/parent"))
            L.append(f"| {c} | {name} | {cv['frame_id']} | {rtxt} | {f(cv['score_standard'])} / {f(cv['score_inverted'])} | "
                     f"{cv['semantics']} / {cv['twist_frame'] or '-'} | {'速度' if cv['decided_by'] == 'twist' else '向き'} | "
                     f"{f(cv.get('walk_dir_deg'), '{:.0f}')} |")
    L += ["", "### 3.2 静的 TF の解釈（base → box_base の向き）", "",
          "真値（box_base）と脚のオドメトリ（base）の回転の変化から求めた向きと、tf から計算した向き（GrandTour のサンプルと同じ解釈 / その逆）の差。official が小さければ、サンプルの解釈で正しい。", "",
          "| ミッション | 使った組 | 当てはめの RMS [deg] | official との差 [deg] | 逆との差 [deg] |", "|---|---|---|---|---|"]
    for c in codes:
        tc = results[c].get("tf_check")
        if tc and "error" not in tc:
            L.append(f"| {c} | {tc['n']} | {f(tc['fit_rms_deg'])} | {f(tc['diff_official_deg'])} | {f(tc['diff_inverse_deg'])} |")
        elif tc:
            L.append(f"| {c} | - | - | {tc['error']} | - |")
    L += ["", "### 3.3 真値の精度の目安", "",
          "| ミッション | ie_rt と ie_tc の水平差 p50 / p95 / 最大 [m] | MS60 との差 RMS / p95 / 最大 [m]（点数） | 求めたプリズムの位置（真値の機体座標系）[m] |",
          "|---|---|---|---|"]
    for c in codes:
        r = results[c]
        rt = r.get("rt_vs_tc")
        pf = r.get("prism_fit")
        L.append(f"| {c} | " + (f"{f(rt['p50'])} / {f(rt['p95'])} / {f(rt['max'])}" if rt else "-") + " | "
                 + (f"{f(pf['rms'], '{:.3f}')} / {f(pf['p95'], '{:.3f}')} / {f(pf['max'], '{:.3f}')}（{pf['n']}）" if pf else "-") + " | "
                 + ((np.array2string(pf["lever"], precision=3) if pf["yaw_span_deg"] >= 45 else
                     f"決まらない（向きの変化が {pf['yaw_span_deg']:.0f}° しかない）") if pf else "-") + " |")
    L += ["", "- ie_rt はリアルタイムの PPP 解（精度が低い）。ie_tc は後処理の密結合解（真値に使う）。",
          "- MS60 との差は、トータルステーションの座標を真値に剛体変換で合わせ、プリズムの位置（レバーアーム）も同時に求めた後の残差。", ""]

    if any(k in results[c] for c in codes for k in ("lidar", "adis_imu", "livox_imu")):
        L += ["## 4. 点群と IMU（lidar の段階のデータがあるミッション）", ""]
        for c in codes:
            r = results[c]
            if "lidar" in r:
                li = r["lidar"]
                L += [f"### {c}", "", f"- トピック: `{li['topic']}`、列: {', '.join(li['fields'])}",
                      f"- スキャン数: {li['scans']}、{f(li['hz'], '{:.1f}')} Hz、1 スキャンの点数（中央値）: {li['points_p50']:.0f}"]
                if "intensity_range" in li:
                    L.append(f"- 反射強度の範囲（最初のスキャン）: {li['intensity_range'][0]:.1f}〜{li['intensity_range'][1]:.1f}")
                if "extrinsic" in li:
                    T = li["extrinsic"]
                    L.append(f"- base から見た LiDAR（tf から）: 位置 {np.array2string(T[:3, 3], precision=3)} m、"
                             f"回転行列 {np.array2string(T[:3, :3], precision=3).replace(chr(10), '')}")
                if "base_height" in li:
                    L.append(f"- 地面から base までの高さ（最初のスキャンの推定）: {li['base_height']:.2f} m（`lidar.base_link_height`）")
                if "self_box" in li:
                    lo, hi = li["self_box"]
                    L.append(f"- base の 0.8 m 以内の点の範囲（ロボット自身の点の目安）: min {np.array2string(lo, precision=2)}、max {np.array2string(hi, precision=2)}")
                if "tf_error" in li:
                    L.append(f"- tf の問題: {li['tf_error']}")
                L.append("")
            for imu in ("adis_imu", "livox_imu"):
                if imu in r:
                    m = r[imu]
                    L.append(f"- {c} `{imu}`（{m['description']}）: {f(m['hz'], '{:.0f}')} Hz、"
                             f"{'静止時' if m['at_rest'] else '最初の 0.5 s（静止していない可能性あり）'}の加速度の大きさ {m['acc_norm']:.3f}、"
                             f"角速度の大きさ {m['gyro_norm']:.4f}（加速度が約 9.8 なら m/s²、約 1.0 なら g 単位）")
            L.append("")
    (out / "report.md").write_text("\n".join(L), encoding="utf-8")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data-dir", required=True, type=Path, help="download.py の --dest と同じフォルダ")
    ap.add_argument("--missions", nargs="*", help="略称かフォルダ名（省略すると、落としてあるものの全部）")
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--radius", type=float, default=5.0, help="経路が重なるとみなす距離 [m]")
    ap.add_argument("--step", type=float, default=0.5, help="経路の比較の間隔 [m]")
    ap.add_argument("--utm-zone", type=int, default=32)
    ap.add_argument("--site-link", type=float, default=300.0, help="同じ場所とみなす経路どうしの距離 [m]")
    args = ap.parse_args()

    data = args.data_dir.expanduser()
    if args.missions:
        folders = M.resolve(args.missions)
    else:
        folders = sorted(p.name for p in data.iterdir() if (p / "data").is_dir())
    folders = [fo for fo in folders if (data / fo / "data").is_dir()]
    if not folders:
        sys.exit(f"ミッションが見つからない: {data}")
    out = args.out.expanduser()
    out.mkdir(parents=True, exist_ok=True)

    codes, results = [], {}
    for fo in folders:
        c = M.code_of(fo)
        print(f"reading {c} ({fo}) ...", flush=True)
        d = load(data / fo, args.utm_zone)
        r = mission_checks(d, args.step)
        r.update(lidar_checks(d, r.get("stationary_start")))
        codes.append(c)
        results[c] = r

    paths = {c: results[c]["path"] for c in codes if "path" in results[c]}
    groups = A.group_sites(paths, args.site_link) if paths else []
    overlaps = {}
    for g in groups:
        for a in g:
            for b in g:
                if a != b:
                    overlaps[(a, b)] = A.overlap(paths[a], results[a]["head"], paths[b], results[b]["head"],
                                                 args.radius, args.step)
    figs = plot_sites(groups, results, out)
    write_report(out, codes, folders, results, overlaps, groups, figs, args.radius)

    keys = ["duration", "length", "gnss_n", "gnss_hz", "gnss_gaps", "gnss_gap_total", "gnss_std_p50", "gnss_std_p95",
            "alt_range", "stationary_start", "speed_p50", "speed_p95", "lateral_p95", "prism_n", "prism_cover"]
    with open(out / "missions.csv", "w", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(["code", "folder"] + keys)
        for c, fo in zip(codes, folders):
            w.writerow([c, fo] + [results[c].get(k, "") for k in keys])
    with open(out / "overlap.csv", "w", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(["map", "localize", "frac", "length_m", "same_dir_m", "opposite_dir_m"])
        for (a, b), o in sorted(overlaps.items()):
            w.writerow([a, b, f"{o['frac']:.3f}", f"{o['length']:.1f}", f"{o['same']:.1f}", f"{o['opposite']:.1f}"])
    # 機械で読む用（テストと、Claude の解析用）
    def plain(x):
        if isinstance(x, dict):
            return {k: plain(v) for k, v in x.items() if k not in ("R", "X")}
        if isinstance(x, (list, tuple)):
            return [plain(v) for v in x]
        if isinstance(x, np.ndarray):
            return x.tolist() if x.size <= 16 else None
        if isinstance(x, (np.floating, np.integer, np.bool_)):
            return x.item()
        return x
    skip = {"path", "head", "xy"}
    js = dict(missions={c: plain({k: v for k, v in results[c].items() if k not in skip}) for c in codes},
              overlap={f"{a}->{b}": o for (a, b), o in overlaps.items()},
              sites=groups)
    (out / "results.json").write_text(json.dumps(js, indent=1, ensure_ascii=False, default=str), encoding="utf-8")
    print(f"\nレポート: {out / 'report.md'}")


if __name__ == "__main__":
    main()
