#!/usr/bin/env python3
"""真値と推定値をグラフで比べる HTML を作る（ブラウザで開くだけ。外部のライブラリ・通信なし）。

  python3 plot_run.py <結果のフォルダ> [<結果のフォルダ> ...] [--out report.html] [--raw]
  python3 plot_run.py <真値.csv> <出力.csv>[:名前] [<出力.csv>[:名前] ...] [--out report.html]

結果のフォルダは run_scenario.sh の tools/awsim/log/<名前>/（groundtruth.csv.gz と output.csv.gz）。
フォルダを 2〜3 個並べると、同じ走行の推定どうし（例: gicp と vgicp）を重ねて比べられる（真値は最初のフォルダのもの）。
CSV は awsim_to_bag.py の <out>_groundtruth.csv と、推定ノードの debug_csv_path（.gz でもよい）。

ページの中身:
  - 軌跡: 真値と推定の軌跡を重ねる。ホイールで拡大、ドラッグで移動、ダブルクリックで元に戻す。
    「誤差を拡大」で、推定の位置を 真値 + k ×（推定 − 真値）に置き、cm の違いを見えるようにする
  - 時間変化: 横・縦・yaw の誤差、x・y・yaw の真値と推定。グラフの上にマウスを置くと、その時刻の値と、軌跡の上の位置が出る
  - 状態の帯（LIDAR_AIDED など）
--raw なら出力整形の前の値（raw_x、raw_y、raw_yaw）を使う（既定は出力 x、y、yaw）。必要なパッケージ: numpy。
"""
import argparse
import csv
import gzip
import json
import math
import sys
from pathlib import Path

import numpy as np

STATUS = {"INITIALIZING": 0, "GNSS_AIDED": 1, "GNSS_LIDAR_AIDED": 2, "LIDAR_AIDED": 3, "DEAD_RECKONING": 4,
          "DEGRADED": 5, "LOST": 6}
MAX_RUNS = 3


def read_csv(path):
    p = Path(path)
    op = gzip.open if p.suffix == ".gz" else open
    with op(p, "rt", newline="") as fp:
        return list(csv.DictReader(fp))


def col(rows, k):
    return np.array([float(r[k]) for r in rows])


def resolve(args):
    """引数 → (真値の CSV, [(出力の CSV, 名前), ...])"""
    items = args.inputs
    if all(Path(a.split(":")[0]).is_dir() for a in items):
        gt = None
        runs = []
        for a in items:
            d, _, label = a.partition(":")
            d = Path(d)
            g = next((d / n for n in ("groundtruth.csv.gz", "groundtruth.csv") if (d / n).exists()), None)
            o = next((d / n for n in ("output.csv.gz", "output.csv") if (d / n).exists()), None)
            if o is None:
                sys.exit(f"{d}: output.csv(.gz) が無い")
            gt = gt or g
            runs.append((o, label or d.name))
        if gt is None:
            sys.exit("groundtruth.csv(.gz) が無い")
        return gt, runs
    if len(items) < 2:
        sys.exit("真値の CSV と、推定の出力の CSV を指定する（または結果のフォルダ）")
    runs = []
    for a in items[1:]:
        f, _, label = a.partition(":")
        runs.append((Path(f), label or Path(f).name.split(".")[0]))
    return Path(items[0]), runs


def build(gt_path, runs, raw=False, rate=10.0):
    g = read_csv(gt_path)
    gt_t, gx, gy = col(g, "t"), col(g, "x"), col(g, "y")
    gyaw = np.unwrap(col(g, "yaw"))
    outs = []
    for path, label in runs[:MAX_RUNS]:
        rows = read_csv(path)
        t = col(rows, "t")
        sel = np.nonzero(np.r_[True, np.diff(np.floor(t * rate)) > 0])[0]  # 約 rate Hz に間引く
        rows = [rows[i] for i in sel]
        t = t[sel]
        pre = "raw_" if raw else ""
        x, y, yaw = col(rows, pre + "x"), col(rows, pre + "y"), col(rows, pre + "yaw")
        st = np.array([STATUS.get(r["status"], 0) for r in rows])
        outs.append(dict(label=label, t=t, x=x, y=y, yaw=yaw, st=st))
    t0 = min(o["t"][0] for o in outs)
    t1 = max(o["t"][-1] for o in outs)
    # 真値は推定の区間だけ、同じ間隔で
    gsel = np.nonzero((gt_t >= t0 - 1) & (gt_t <= t1 + 1) & np.r_[True, np.diff(np.floor(gt_t * rate)) > 0])[0]
    ox, oy = float(gx[gsel[0]]), float(gy[gsel[0]])  # 表示の原点（真値の最初の位置）
    data = dict(origin=[ox, oy], t0=t0, status=list(STATUS),
                gt=dict(t=np.round(gt_t[gsel] - t0, 2).tolist(), x=np.round(gx[gsel] - ox, 3).tolist(),
                        y=np.round(gy[gsel] - oy, 3).tolist(), yaw=np.round(np.degrees(gyaw[gsel]), 3).tolist()),
                runs=[])
    for o in outs:
        t = o["t"]
        ok = (t >= gt_t[0]) & (t <= gt_t[-1])
        t = t[ok]
        gxi, gyi, gwi = np.interp(t, gt_t, gx), np.interp(t, gt_t, gy), np.interp(t, gt_t, gyaw)
        dx, dy = o["x"][ok] - gxi, o["y"][ok] - gyi
        lat = -np.sin(gwi) * dx + np.cos(gwi) * dy
        lon = np.cos(gwi) * dx + np.sin(gwi) * dy
        # 推定の yaw を真値の yaw の近くに寄せて（±180° の折り返しを無くして）表示する
        yaw_c = gwi + (o["yaw"][ok] - gwi + np.pi) % (2 * np.pi) - np.pi
        eyaw = np.degrees(yaw_c - gwi)
        st = o["st"][ok]
        use = st != 0
        m = lambda v: float(np.sqrt(np.mean(v[use] ** 2))) if use.any() else float("nan")
        data["runs"].append(dict(
            label=o["label"], t=np.round(t - t0, 2).tolist(),
            x=np.round(o["x"][ok] - ox, 3).tolist(), y=np.round(o["y"][ok] - oy, 3).tolist(),
            yaw=np.round(np.degrees(yaw_c), 3).tolist(),
            lat=np.round(lat, 4).tolist(), lon=np.round(lon, 4).tolist(), eyaw=np.round(eyaw, 3).tolist(),
            st=st.astype(int).tolist(),
            rms=dict(lat=m(lat), lon=m(lon), xy=m(np.hypot(dx, dy)), yaw=m(eyaw))))
    return data


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("inputs", nargs="+")
    ap.add_argument("--out", type=Path, default=None, help="既定は最初のフォルダの report.html（CSV なら ./report.html）")
    ap.add_argument("--title", default="")
    ap.add_argument("--raw", action="store_true", help="出力整形の前の値（raw_*）を使う")
    args = ap.parse_args()
    gt, runs = resolve(args)
    if len(runs) > MAX_RUNS:
        print(f"注意: 比べられるのは {MAX_RUNS} 個まで（残りは無視する）")
    data = build(gt, runs, args.raw)
    out = args.out or ((Path(args.inputs[0].split(":")[0]) / "report.html")
                       if Path(args.inputs[0].split(":")[0]).is_dir() else Path("report.html"))
    title = args.title or " / ".join(r["label"] for r in data["runs"])
    html = TEMPLATE.replace("__TITLE__", title).replace("__FIRST__", data["runs"][0]["label"]).replace("__KIND__", "出力整形の前（raw）" if args.raw else "出力")
    for r in data["runs"]:  # JSON に NaN は書けないので null にする
        r["rms"] = {k: (v if math.isfinite(v) else None) for k, v in r["rms"].items()}
    html = html.replace("__DATA__", json.dumps(data, separators=(",", ":"), allow_nan=False))
    out.write_text(html)
    for r in data["runs"]:
        f = lambda v, d: "—" if v is None else f"{v:.{d}f}"
        print(f"{r['label']}: 横 RMS {f(r['rms']['lat'], 3)} m、縦 RMS {f(r['rms']['lon'], 3)} m、"
              f"yaw RMS {f(r['rms']['yaw'], 2)}°")
    print(f"{out}（ブラウザで開く）")


TEMPLATE = r"""<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>真値と推定の比較</title>
<style>
:root{--bg:#F3F6F5;--surface:#FFFFFF;--ink:#15201F;--muted:#586664;--line:#D3DCDA;--grid:#E4EAE8;
  --gt:#15201F;--r0:#0B6E99;--r1:#B5650A;--r2:#8E4FB5;
  --st-gnss:#E7D9B8;--st-lidar:#D5E6EE;--st-both:#CFE3D4;--st-dr:#E9C77E;--st-lost:#B0412B;--st-init:#E4EAE8}
@media (prefers-color-scheme: dark){:root:not([data-theme="light"]){--bg:#0F1515;--surface:#162020;--ink:#E3ECEA;--muted:#9AAAA7;--line:#2C3A38;--grid:#223030;
  --gt:#E3ECEA;--r0:#2F95C4;--r1:#C98422;--r2:#A77AD6;
  --st-gnss:#4A3E22;--st-lidar:#1D3A46;--st-both:#1F3A2A;--st-dr:#8A6A26;--st-lost:#E68068;--st-init:#223030}}
:root[data-theme="dark"]{--bg:#0F1515;--surface:#162020;--ink:#E3ECEA;--muted:#9AAAA7;--line:#2C3A38;--grid:#223030;
  --gt:#E3ECEA;--r0:#2F95C4;--r1:#C98422;--r2:#A77AD6;
  --st-gnss:#4A3E22;--st-lidar:#1D3A46;--st-both:#1F3A2A;--st-dr:#8A6A26;--st-lost:#E68068;--st-init:#223030}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);font-family:"Noto Sans JP","Hiragino Sans","Yu Gothic",system-ui,sans-serif;font-size:14px;line-height:1.6;padding:20px 16px 48px}
.wrap{max-width:1100px;margin:0 auto;display:flex;flex-direction:column;gap:18px}
h1{font-size:22px;margin:0}
.sub{color:var(--muted);font-size:13px}
.card{background:var(--surface);border:1px solid var(--line);border-radius:10px;padding:12px 14px;display:flex;flex-direction:column;gap:8px}
.head{display:flex;flex-wrap:wrap;gap:6px 16px;align-items:center;justify-content:space-between}
.title{font-weight:600}
.legend{display:flex;flex-wrap:wrap;gap:4px 14px;font-size:12.5px;color:var(--muted)}
.key{display:inline-block;width:18px;height:0;border-top:2.5px solid;vertical-align:4px;margin-right:6px}
.key.gt{border-color:var(--gt);border-top-style:dashed}
.sk{display:inline-block;width:12px;height:9px;border-radius:2px;margin-right:5px}
table{border-collapse:collapse;font-size:13px;font-variant-numeric:tabular-nums}
th,td{padding:4px 10px;border-bottom:1px solid var(--line);text-align:right}
th:first-child,td:first-child{text-align:left}
th{color:var(--muted);font-weight:500;font-size:12px}
.controls{display:flex;flex-wrap:wrap;gap:6px 14px;align-items:center;font-size:13px;color:var(--muted)}
.controls button,.controls select{font:inherit;font-size:12.5px;color:var(--ink);background:var(--surface);border:1px solid var(--line);border-radius:6px;padding:2px 10px;cursor:pointer}
.controls button[aria-pressed="true"]{background:var(--ink);color:var(--surface);border-color:var(--ink)}
#traj{width:100%;height:520px;display:block;border-radius:6px;background:var(--bg);cursor:grab;touch-action:none}
#traj.drag{cursor:grabbing}
.charts{display:flex;flex-direction:column;gap:4px}
.chart{position:relative}
.chart svg{width:100%;height:auto;display:block}
.ax{fill:var(--muted);font-size:11px;font-family:ui-monospace,Menlo,monospace}
.cl{fill:var(--ink);font-size:12px;font-weight:600}
.gl{stroke:var(--grid);stroke-width:1}
.zero{stroke:var(--line);stroke-width:1.2}
.ln{fill:none;stroke-width:1.6;stroke-linejoin:round}
.cross{stroke:var(--muted);stroke-width:1;stroke-dasharray:3 3}
.tip{position:fixed;pointer-events:none;background:var(--surface);border:1px solid var(--line);border-radius:8px;padding:6px 10px;font-size:12.5px;box-shadow:0 4px 14px rgba(0,0,0,.14);white-space:nowrap;z-index:5}
.tip b{font-weight:600}
.tip .m{color:var(--muted)}
.note{font-size:12.5px;color:var(--muted)}
</style>
</head>
<body>
<div class="wrap">
  <header>
    <h1>真値と推定の比較</h1>
    <div class="sub">__TITLE__ ・ 推定は__KIND__の値 ・ 誤差は真値の向きで分けた横（左が正）・縦（前が正）</div>
  </header>

  <div class="card">
    <div class="head"><span class="title">誤差（INITIALIZING を除く RMS）</span></div>
    <div style="overflow-x:auto"><table id="sum"></table></div>
  </div>

  <div class="card">
    <div class="head">
      <span class="title">軌跡</span>
      <span class="legend" id="lg1"></span>
    </div>
    <div class="controls">
      <span>誤差を拡大</span>
      <span id="gain"></span>
      <label><input type="checkbox" id="colst"> 推定の色を状態で塗る</label>
      <span>ホイールで拡大・ドラッグで移動・ダブルクリックで全体</span>
    </div>
    <svg id="traj" role="img" aria-label="真値と推定の軌跡"></svg>
    <div class="legend" id="lgst"></div>
    <div class="note">原点は真値の最初の位置、上が北（UTM の y）。「誤差を拡大」は、推定の位置を 真値 + k ×（推定 − 真値）に描く（k = 1 が実際の位置）。</div>
  </div>

  <div class="card">
    <div class="head">
      <span class="title">時間変化（横軸は最初の出力からの秒）</span>
      <span class="legend" id="lg2"></span>
    </div>
    <div class="controls">
      <span>表示</span>
      <select id="view">
        <option value="err">誤差（横・縦・yaw）</option>
        <option value="val">真値と推定（x・y・yaw）</option>
      </select>
      <span>ドラッグで区間を拡大・ダブルクリックで全体</span>
    </div>
    <div class="charts" id="charts"></div>
    <div class="note">グラフの下の帯は、最初の推定（__FIRST__）の状態。</div>
  </div>
</div>
<div class="tip" id="tip" hidden></div>

<script>
const D = __DATA__;
const NS = "http://www.w3.org/2000/svg";
const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const RC = ["--r0", "--r1", "--r2"];
const ST = D.status;  // 名前の並び
const STC = {INITIALIZING: "--st-init", GNSS_AIDED: "--st-gnss", GNSS_LIDAR_AIDED: "--st-both", LIDAR_AIDED: "--st-lidar",
  DEAD_RECKONING: "--st-dr", DEGRADED: "--st-dr", LOST: "--st-lost"};
const el = (n, a = {}, p) => { const e = document.createElementNS(NS, n); for (const k in a) e.setAttribute(k, a[k]); if (p) p.appendChild(e); return e; };
const fmt = (v, d) => v == null || !isFinite(v) ? "—" : v.toFixed(d);
const tip = document.getElementById("tip");

// ---- 凡例と表
const legend = id => {
  const L = document.getElementById(id);
  L.innerHTML = `<span><i class="key gt"></i>真値</span>` + D.runs.map((r, i) =>
    `<span><i class="key" style="border-color:var(${RC[i]})"></i>${r.label}</span>`).join("");
};
legend("lg1"); legend("lg2");
const used = [...new Set(D.runs.flatMap(r => r.st))].sort();
document.getElementById("lgst").innerHTML = "状態: " + used.map(s =>
  `<span><i class="sk" style="background:var(${STC[ST[s]]})"></i>${ST[s]}</span>`).join(" ");
document.getElementById("sum").innerHTML = "<tr><th>推定</th><th>横 RMS</th><th>縦 RMS</th><th>xy RMS</th><th>yaw RMS</th><th>LIDAR / GNSS で補正</th><th>LOST</th></tr>" +
  D.runs.map((r, i) => {
    const n = r.st.filter(s => s).length;
    const aided = r.st.filter(s => s >= 1 && s <= 3).length, lost = r.st.filter(s => s === 6).length;
    return `<tr><td><i class="key" style="border-color:var(${RC[i]})"></i>${r.label}</td><td>${fmt(r.rms.lat, 3)} m</td><td>${fmt(r.rms.lon, 3)} m</td><td>${fmt(r.rms.xy, 3)} m</td><td>${fmt(r.rms.yaw, 2)}°</td><td>${fmt(100 * aided / n, 1)} %</td><td>${fmt(100 * lost / n, 1)} %</td></tr>`;
  }).join("");

// ---- 時刻から添字（二分探索）
const idxAt = (T, t) => { let lo = 0, hi = T.length - 1; if (t <= T[0]) return 0; if (t >= T[hi]) return hi;
  while (hi - lo > 1) { const m = (lo + hi) >> 1; if (T[m] <= t) lo = m; else hi = m; }
  return t - T[lo] < T[hi] - t ? lo : hi; };
const gtAt = t => { const i = idxAt(D.gt.t, t); return [D.gt.x[i], D.gt.y[i], D.gt.yaw[i]]; };

// ---- 軌跡
const svg = document.getElementById("traj");
let gain = 1, colorByState = false;
const allX = D.gt.x.concat(...D.runs.map(r => r.x)), allY = D.gt.y.concat(...D.runs.map(r => r.y));
const full = () => { const x0 = Math.min(...allX), x1 = Math.max(...allX), y0 = Math.min(...allY), y1 = Math.max(...allY);
  const pad = 0.05 * Math.max(x1 - x0, y1 - y0, 10); return {x: x0 - pad, y: y0 - pad, w: x1 - x0 + 2 * pad, h: y1 - y0 + 2 * pad}; };
let vb = full();
function drawTraj() {
  const W = svg.clientWidth, H = svg.clientHeight;
  // 縦横比を保つ（地図なので）
  const s = Math.min(W / vb.w, H / vb.h);
  const cx = vb.x + vb.w / 2, cy = vb.y + vb.h / 2;
  const P = (x, y) => [(W / 2 + (x - cx) * s).toFixed(1), (H / 2 - (y - cy) * s).toFixed(1)];
  svg.innerHTML = "";
  svg.setAttribute("viewBox", `0 0 ${W} ${H}`);
  // 目盛り（見えている幅に合わせた間隔の格子）
  const span = Math.max(W, H) / s, steps = [0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50, 100, 200, 500];
  const step = steps.find(v => span / v <= 12) || 1000;
  const xs = cx - W / 2 / s, xe = cx + W / 2 / s, ys = cy - H / 2 / s, ye = cy + H / 2 / s;
  for (let x = Math.ceil(xs / step) * step; x <= xe; x += step) { const [px] = P(x, 0); el("line", {x1: px, x2: px, y1: 0, y2: H, class: "gl"}, svg); }
  for (let y = Math.ceil(ys / step) * step; y <= ye; y += step) { const [, py] = P(0, y); el("line", {x1: 0, x2: W, y1: py, y2: py, class: "gl"}, svg); }
  el("text", {x: 8, y: H - 8, class: "ax"}, svg).textContent = `格子 ${step >= 1 ? step + " m" : step * 100 + " cm"}`;
  // 真値
  el("polyline", {points: D.gt.x.map((x, i) => P(x, D.gt.y[i]).join(",")).join(" "), fill: "none",
    stroke: css("--gt"), "stroke-width": 2, "stroke-dasharray": "6 4", opacity: 0.8}, svg);
  // 推定（誤差を gain 倍）
  D.runs.forEach((r, k) => {
    const pts = r.t.map((t, i) => { const g = gtAt(t); return P(g[0] + gain * (r.x[i] - g[0]), g[1] + gain * (r.y[i] - g[1])); });
    if (!colorByState) {
      el("polyline", {points: pts.map(p => p.join(",")).join(" "), fill: "none", stroke: css(RC[k]), "stroke-width": 2, "stroke-linejoin": "round"}, svg);
    } else {
      let i0 = 0;
      for (let i = 1; i <= pts.length; i++) {
        if (i === pts.length || r.st[i] !== r.st[i0]) {
          el("polyline", {points: pts.slice(i0, Math.min(i + 1, pts.length)).map(p => p.join(",")).join(" "), fill: "none",
            stroke: css(STC[ST[r.st[i0]]]), "stroke-width": 4, "stroke-linecap": "round"}, svg);
          i0 = i;
        }
      }
    }
  });
  const g0 = P(D.gt.x[0], D.gt.y[0]);
  el("circle", {cx: g0[0], cy: g0[1], r: 5, fill: css("--surface"), stroke: css("--gt"), "stroke-width": 2}, svg);
  svg._P = P;
  svg._marker = el("g", {visibility: "hidden"}, svg);
}
const markAt = t => {
  if (!svg._P) return;
  const m = svg._marker; m.innerHTML = ""; m.setAttribute("visibility", t == null ? "hidden" : "visible");
  if (t == null) return;
  const g = gtAt(t), pg = svg._P(g[0], g[1]);
  el("circle", {cx: pg[0], cy: pg[1], r: 6, fill: "none", stroke: css("--gt"), "stroke-width": 2}, m);
  D.runs.forEach((r, k) => { const i = idxAt(r.t, t); const p = svg._P(g[0] + gain * (r.x[i] - g[0]), g[1] + gain * (r.y[i] - g[1]));
    el("circle", {cx: p[0], cy: p[1], r: 4.5, fill: css(RC[k]), stroke: css("--surface"), "stroke-width": 2}, m); });
};
// 拡大・移動
svg.addEventListener("wheel", e => { e.preventDefault();
  const r = svg.getBoundingClientRect(), W = r.width, H = r.height, s = Math.min(W / vb.w, H / vb.h);
  const cx = vb.x + vb.w / 2, cy = vb.y + vb.h / 2;
  const mx = cx + (e.clientX - r.left - W / 2) / s, my = cy - (e.clientY - r.top - H / 2) / s;
  const f = Math.exp(e.deltaY * 0.0015);
  vb = {x: mx - (mx - vb.x) * f, y: my - (my - vb.y) * f, w: vb.w * f, h: vb.h * f}; drawTraj(); }, {passive: false});
let drag = null;
svg.addEventListener("pointerdown", e => { drag = {x: e.clientX, y: e.clientY, vb: {...vb}}; svg.setPointerCapture(e.pointerId); svg.classList.add("drag"); });
svg.addEventListener("pointermove", e => { if (!drag) return; const r = svg.getBoundingClientRect(), s = Math.min(r.width / vb.w, r.height / vb.h);
  vb = {...drag.vb, x: drag.vb.x - (e.clientX - drag.x) / s, y: drag.vb.y + (e.clientY - drag.y) / s}; drawTraj(); });
svg.addEventListener("pointerup", () => { drag = null; svg.classList.remove("drag"); });
svg.addEventListener("dblclick", () => { vb = full(); drawTraj(); });
const gains = [1, 10, 100, 1000];
document.getElementById("gain").innerHTML = gains.map(g => `<button type="button" data-g="${g}" aria-pressed="${g === 1}">×${g}</button>`).join(" ");
document.getElementById("gain").addEventListener("click", e => { const b = e.target.closest("button"); if (!b) return;
  gain = +b.dataset.g; document.querySelectorAll("#gain button").forEach(x => x.setAttribute("aria-pressed", x === b)); drawTraj(); });
document.getElementById("colst").addEventListener("change", e => { colorByState = e.target.checked; drawTraj(); });

// ---- 時間変化
const tMax = Math.max(D.gt.t[D.gt.t.length - 1], ...D.runs.map(r => r.t[r.t.length - 1]));
let tr = [0, tMax];
const VIEWS = {
  err: [{k: "lat", name: "横の誤差", unit: "m", zero: true}, {k: "lon", name: "縦の誤差", unit: "m", zero: true},
        {k: "eyaw", name: "yaw の誤差", unit: "°", zero: true}],
  val: [{k: "x", name: "x（東）", unit: "m", gt: "x"}, {k: "y", name: "y（北）", unit: "m", gt: "y"},
        {k: "yaw", name: "yaw", unit: "°", gt: "yaw"}],
};
function drawCharts() {
  const box = document.getElementById("charts"); box.innerHTML = "";
  const specs = VIEWS[document.getElementById("view").value];
  const W = 1000, H = 190, m = {l: 64, r: 12, t: 22, b: 22};
  const pw = W - m.l - m.r, ph = H - m.t - m.b;
  const X = t => m.l + (t - tr[0]) / (tr[1] - tr[0]) * pw;
  const charts = [];
  specs.forEach((sp, ci) => {
    const div = document.createElement("div"); div.className = "chart"; box.appendChild(div);
    const s = el("svg", {viewBox: `0 0 ${W} ${H + (ci === specs.length - 1 ? 26 : 0)}`}, div);
    // 見えている区間の値の範囲
    let lo = Infinity, hi = -Infinity;
    const scan = (T, V) => { for (let i = 0; i < T.length; i++) if (T[i] >= tr[0] && T[i] <= tr[1] && V[i] != null) { lo = Math.min(lo, V[i]); hi = Math.max(hi, V[i]); } };
    D.runs.forEach(r => scan(r.t, r[sp.k]));
    if (sp.gt) scan(D.gt.t, D.gt[sp.gt]);
    if (sp.zero) { const a = Math.max(Math.abs(lo), Math.abs(hi), 1e-3); lo = -a; hi = a; }
    if (!isFinite(lo)) { lo = -1; hi = 1; }
    const pad = (hi - lo) * 0.08 || 1; lo -= pad; hi += pad;
    const Y = v => m.t + ph - (v - lo) / (hi - lo) * ph;
    // 目盛り
    const nice = x => { const p = Math.pow(10, Math.floor(Math.log10(x))); const f = x / p; return (f < 1.5 ? 1 : f < 3 ? 2 : f < 7 ? 5 : 10) * p; };
    const st = nice((hi - lo) / 4);
    for (let v = Math.ceil(lo / st) * st; v <= hi; v += st) {
      el("line", {x1: m.l, x2: W - m.r, y1: Y(v), y2: Y(v), class: Math.abs(v) < st / 1e6 && sp.zero ? "zero" : "gl"}, s);
      el("text", {x: m.l - 6, y: Y(v) + 4, "text-anchor": "end", class: "ax"}, s).textContent = +v.toFixed(6) + "";
    }
    el("text", {x: m.l, y: 14, class: "cl"}, s).textContent = `${sp.name} [${sp.unit}]`;
    // 状態の帯（最初の推定）
    const r0 = D.runs[0];
    for (let i = 0; i < r0.t.length - 1; i++) {
      if (r0.t[i + 1] < tr[0] || r0.t[i] > tr[1]) continue;
      const x0 = Math.max(X(r0.t[i]), m.l), x1 = Math.min(X(r0.t[i + 1]), W - m.r);
      el("rect", {x: x0, y: m.t + ph + 2, width: Math.max(x1 - x0, 0.5), height: 5, fill: css(STC[ST[r0.st[i]]])}, s);
    }
    const clip = `c${ci}`;
    const cp = el("clipPath", {id: clip}, el("defs", {}, s)); el("rect", {x: m.l, y: m.t, width: pw, height: ph}, cp);
    const line = (T, V, color, extra = {}) => {
      let d = "", pen = false;
      const step = Math.max(1, Math.floor(T.filter(t => t >= tr[0] && t <= tr[1]).length / 1500));
      for (let i = 0; i < T.length; i += step) { if (T[i] < tr[0] - 1 || T[i] > tr[1] + 1 || V[i] == null) { pen = false; continue; }
        d += (pen ? "L" : "M") + X(T[i]).toFixed(1) + " " + Y(V[i]).toFixed(1); pen = true; }
      el("path", {d, class: "ln", stroke: color, "clip-path": `url(#${clip})`, ...extra}, s);
    };
    if (sp.gt) line(D.gt.t, D.gt[sp.gt], css("--gt"), {"stroke-dasharray": "6 4", "stroke-width": 2});
    D.runs.forEach((r, k) => line(r.t, r[sp.k], css(RC[k])));
    if (ci === specs.length - 1) {
      const ts = nice((tr[1] - tr[0]) / 8);
      for (let t = Math.ceil(tr[0] / ts) * ts; t <= tr[1]; t += ts)
        el("text", {x: X(t), y: H + 16, "text-anchor": "middle", class: "ax"}, s).textContent = +t.toFixed(3) + " s";
    }
    const cross = el("line", {y1: m.t, y2: m.t + ph, class: "cross", visibility: "hidden"}, s);
    charts.push({s, sp, cross});
  });
  // 重ねた操作: 時刻の表示（すべてのグラフと軌跡に）、ドラッグで区間を拡大
  let sel = null;
  const tOf = (s, e) => { const r = s.getBoundingClientRect(); const x = (e.clientX - r.left) / r.width * 1000; return tr[0] + (x - m.l) / pw * (tr[1] - tr[0]); };
  charts.forEach(({s}) => {
    s.addEventListener("pointermove", e => {
      const t = Math.max(tr[0], Math.min(tr[1], tOf(s, e)));
      charts.forEach(c => { c.cross.setAttribute("x1", X(t)); c.cross.setAttribute("x2", X(t)); c.cross.setAttribute("visibility", "visible"); });
      markAt(t);
      const g = gtAt(t);
      let h = `<b>${t.toFixed(2)} s</b>`;
      D.runs.forEach((r, k) => { const i = idxAt(r.t, t);
        h += `<div><i class="key" style="border-color:var(${RC[k]})"></i>${r.label}: 横 ${fmt(r.lat[i], 3)} m・縦 ${fmt(r.lon[i], 3)} m・yaw ${fmt(r.eyaw[i], 2)}° <span class="m">${ST[r.st[i]]}</span></div>`; });
      h += `<div class="m">真値 x ${g[0].toFixed(2)} m・y ${g[1].toFixed(2)} m・yaw ${g[2].toFixed(1)}°</div>`;
      tip.innerHTML = h; tip.hidden = false;
      const w = tip.offsetWidth; tip.style.left = Math.min(e.clientX + 14, innerWidth - w - 8) + "px"; tip.style.top = (e.clientY + 14) + "px";
      if (sel) { const x0 = Math.min(sel.x, X(t)), x1 = Math.max(sel.x, X(t)); sel.rect.setAttribute("x", x0); sel.rect.setAttribute("width", x1 - x0); }
    });
    s.addEventListener("pointerleave", () => { tip.hidden = true; markAt(null); charts.forEach(c => c.cross.setAttribute("visibility", "hidden")); });
    s.addEventListener("pointerdown", e => { const t = tOf(s, e); sel = {t, x: X(t), rect: el("rect", {x: X(t), y: m.t, width: 0, height: ph, fill: css("--muted"), opacity: 0.15}, s)}; });
    s.addEventListener("pointerup", e => { if (!sel) return; const t = tOf(s, e); const a = Math.min(sel.t, t), b = Math.max(sel.t, t); sel = null;
      if (b - a > (tr[1] - tr[0]) / 200) { tr = [Math.max(0, a), Math.min(tMax, b)]; } drawCharts(); });
    s.addEventListener("dblclick", () => { tr = [0, tMax]; drawCharts(); });
  });
}
document.getElementById("view").addEventListener("change", drawCharts);
addEventListener("resize", drawTraj);
matchMedia("(prefers-color-scheme: dark)").addEventListener("change", () => { drawTraj(); drawCharts(); });
drawTraj(); drawCharts();
</script>
</body>
</html>
"""

if __name__ == "__main__":
    main()
