#!/usr/bin/env python3
"""AWSIM の車を走らせる経路を、lanelet2 の地図（.osm）から作る。進行方向と車線のつながりを守る。

  # 車の今の姿勢から、交通ルールどおりに 1 km ほど走る経路（行き先は乱数。--seed で変わる）
  python3 plan_route.py [lanelet2_map.osm] --start X,Y,YAW_DEG --length 1000 [--seed 0] -o route.txt --svg route.svg

  # 通りたい点を順に回る経路（最短経路。点は近くの車線に寄せる）
  python3 plan_route.py [lanelet2_map.osm] --start X,Y,YAW_DEG --via X1,Y1 X2,Y2 ... -o route.txt --svg route.svg

--start は車の今の姿勢（地図座標と、東から反時計回りの向き [deg]）。awsim_drive.py --print-pose で出せる。
経路は車の位置から始まる。lanelet の中心線をつなぎ、次の lanelet（successor）へは進行方向に沿ってだけ進む。
車線変更は、境界線が破線のときだけ、1 つの lanelet の長さをかけて隣の車線へ移る（--no-lane-change で使わない）。
--svg で、地図の道路（灰）・経路（赤）・始点（緑）・終点（青）の図を書く（ブラウザで開く）。
出力の経路は awsim_drive.py --check で走れるか確かめてから使う。必要なパッケージ: numpy。
"""
import argparse
import heapq
import math
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

LANE_CHANGE_COST = 30.0  # 車線変更 1 回を、この長さ [m] だけ遠回りしたのと同じに扱う


def resample_n(a, n):
    d = np.r_[0, np.cumsum(np.linalg.norm(np.diff(a, axis=0), axis=1))]
    s = np.linspace(0, d[-1], n)
    return np.c_[np.interp(s, d, a[:, 0]), np.interp(s, d, a[:, 1])]


def polyline_len(a):
    return float(np.sum(np.linalg.norm(np.diff(a, axis=0), axis=1)))


class Lanelet:
    def __init__(self, lid, left, right, left_way, right_way, left_nodes, right_nodes, turn):
        self.id = lid
        n = max(2, int(max(polyline_len(left), polyline_len(right)) / 1.0) + 1)
        self.center = (resample_n(left, n) + resample_n(right, n)) / 2
        self.length = polyline_len(self.center)
        self.left_way, self.right_way = left_way, right_way
        self.start_nodes = (left_nodes[0], right_nodes[0])
        self.end_nodes = (left_nodes[-1], right_nodes[-1])
        self.turn = turn
        self.succ = []          # 次の lanelet
        self.change = []        # 車線変更で移れる隣の lanelet（同じ向き、破線）


def load_lanelets(osm):
    root = ET.parse(osm).getroot()
    nodes = {}
    for n in root.iter("node"):
        t = {x.get("k"): x.get("v") for x in n.findall("tag")}
        if "local_x" in t:
            nodes[n.get("id")] = (float(t["local_x"]), float(t["local_y"]))
    ways, way_type = {}, {}
    for w in root.iter("way"):
        ways[w.get("id")] = [nd.get("ref") for nd in w.findall("nd")]
        t = {x.get("k"): x.get("v") for x in w.findall("tag")}
        way_type[w.get("id")] = t.get("subtype", "")
    lls = {}
    for r in root.iter("relation"):
        t = {x.get("k"): x.get("v") for x in r.findall("tag")}
        if t.get("type") != "lanelet" or t.get("subtype") != "road":
            continue
        m = {x.get("role"): x.get("ref") for x in r.findall("member") if x.get("type") == "way"}
        if "left" not in m or "right" not in m:
            continue
        ln, rn = ways[m["left"]], ways[m["right"]]
        left = np.array([nodes[i] for i in ln])
        right = np.array([nodes[i] for i in rn])
        lls[r.get("id")] = Lanelet(r.get("id"), left, right, m["left"], m["right"], ln, rn, t.get("turn_direction", ""))
    # つながり: 前の lanelet の終わりの 2 点（左右の境界の最後のノード）= 次の lanelet の始まりの 2 点
    by_start = {}
    for ll in lls.values():
        by_start.setdefault(ll.start_nodes, []).append(ll.id)
    by_start_pos = []
    for ll in lls.values():
        by_start_pos.append((ll.center[0], ll.id))
    starts = np.array([p for p, _ in by_start_pos])
    start_ids = [i for _, i in by_start_pos]
    for ll in lls.values():
        succ = set(by_start.get(ll.end_nodes, []))
        if True:  # ノードを共有していない所のために、位置と向きでもつなぐ
            d = np.linalg.norm(starts - ll.center[-1], axis=1)
            h0 = heading(ll.center[-2], ll.center[-1])
            for k in np.nonzero(d < 0.5)[0]:
                o = lls[start_ids[k]]
                if abs(wrap(heading(o.center[0], o.center[1]) - h0)) < math.radians(45):
                    succ.add(o.id)
        ll.succ = sorted(succ - {ll.id})
    # 車線変更: 境界の線（way）を共有し、同じ向きで、その線が破線
    by_way = {}
    for ll in lls.values():
        by_way.setdefault(ll.left_way, []).append((ll.id, "left"))
        by_way.setdefault(ll.right_way, []).append((ll.id, "right"))
    for wid, users in by_way.items():
        if way_type.get(wid) != "dashed":
            continue
        for a, sa in users:
            for b, sb in users:
                if a != b and sa != sb:  # a の左の線 = b の右の線（同じ向きの隣の車線）
                    lls[a].change.append(b)
    return lls


def heading(p, q):
    return math.atan2(q[1] - p[1], q[0] - p[0])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def project(ll, p):
    """中心線の上で p に最も近い点の番号と距離。"""
    d = np.linalg.norm(ll.center - p, axis=1)
    k = int(np.argmin(d))
    return k, float(d[k])


def find_lanelet(lls, p, yaw=None, max_dist=3.0):
    best = None
    for ll in lls.values():
        k, d = project(ll, p)
        if d > max_dist:
            continue
        if yaw is not None:
            a, b = ll.center[max(k - 1, 0)], ll.center[min(k + 1, len(ll.center) - 1)]
            if abs(wrap(heading(a, b) - yaw)) > math.radians(45):
                continue
        if best is None or d < best[2]:
            best = (ll.id, k, d)
    return best


def geometry(lls, path, kinds, start_k, end_k=None):
    """lanelet の並びを点列にする。kinds[i] は path[i] から次へ移る方法（"succ" か "change:<隣の id>"）。"""
    pts = []
    for i, lid in enumerate(path):
        c = lls[lid].center
        a = start_k if i == 0 else 0
        b = (end_k + 1) if (end_k is not None and i == len(path) - 1) else len(c)
        seg = c[a:b]
        kind = kinds[i] if i < len(kinds) else "succ"
        if kind.startswith("change:"):
            other = lls[kind.split(":", 1)[1]].center
            o = resample_n(other, len(c))[a:b]
            w = np.linspace(0, 1, len(seg))[:, None]
            w = w * w * (3 - 2 * w)  # なめらかに移る
            seg = (1 - w) * seg + w * o
        if pts and len(seg) and np.linalg.norm(seg[0] - pts[-1]) < 0.3:
            seg = seg[1:]
        pts.extend(seg)
    return np.array(pts)


def edges_from(lls, lid, lane_change):
    for s in lls[lid].succ:
        yield s, "succ", lls[s].length
    if lane_change:
        for b in lls[lid].change:
            for s in lls[b].succ:
                yield s, f"change:{b}", lls[s].length + LANE_CHANGE_COST


def shortest(lls, a, goals, lane_change):
    """a から goals のどれかまで。(lanelet の並び, 移り方の並び)。"""
    dist, prev, pq = {a: 0.0}, {}, [(0.0, a)]
    while pq:
        d, u = heapq.heappop(pq)
        if u in goals and u != a:
            break
        if d > dist[u]:
            continue
        for v, kind, w in edges_from(lls, u, lane_change):
            if d + w < dist.get(v, 1e18):
                dist[v], prev[v] = d + w, (u, kind)
                heapq.heappush(pq, (d + w, v))
    hit = [g for g in goals if g in prev]
    if not hit:
        return None
    g = min(hit, key=lambda x: dist[x])
    path, kinds = [g], []
    while path[-1] != a:
        u, kind = prev[path[-1]]
        path.append(u)
        kinds.append(kind)
    return path[::-1], kinds[::-1]


def live_set(lls, lane_change):
    """いくらでも走り続けられる lanelet（行き止まり = 地図の端 に必ず行き着くものを除く）。"""
    live = set(lls)
    while True:
        dead = {u for u in live if not any(v in live for v, _, _ in edges_from(lls, u, lane_change))}
        if not dead:
            return live
        live -= dead


def random_walk(lls, a, length, rng, lane_change, start_k):
    path, kinds, total, seen = [a], [], lls[a].length - start_k, {a}
    live = live_set(lls, lane_change)
    while total < length:
        cands = list(edges_from(lls, path[-1], lane_change))
        cands = [c for c in cands if c[0] in live] or cands  # 地図の端の行き止まりへは行かない
        cands = [c for c in cands if c[1] == "succ"] or cands  # 車線変更は、ほかに行き先が無いときだけ
        if not cands:
            break
        fresh = [c for c in cands if c[0] not in seen]
        v, kind, _ = (fresh or cands)[rng.integers(len(fresh or cands))]
        path.append(v)
        kinds.append(kind)
        seen.add(v)
        total += lls[v].length
    return path, kinds


def smooth(r, k=3):
    if len(r) < 2 * k + 2:
        return r
    sm = np.array([r[max(0, i - k):i + k + 1].mean(axis=0) for i in range(len(r))])
    sm[0], sm[-1] = r[0], r[-1]
    return sm


def thin(r, step=2.0):
    out = [r[0]]
    for q in r[1:]:
        if np.linalg.norm(q - out[-1]) >= step:
            out.append(q)
    if np.linalg.norm(r[-1] - out[-1]) > 0.3:
        out.append(r[-1])
    return np.array(out)


def write_svg(path, lls, route, start):
    allp = np.vstack([ll.center for ll in lls.values()])
    lo, hi = allp.min(axis=0) - 10, allp.max(axis=0) + 10
    w = hi[0] - lo[0]
    h = hi[1] - lo[1]

    def tr(p):
        return f"{p[0] - lo[0]:.1f},{hi[1] - p[1]:.1f}"

    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w:.0f} {h:.0f}" '
             f'style="background:#111" width="1000">']
    for ll in lls.values():
        parts.append(f'<polyline points="{" ".join(tr(p) for p in ll.center[::2])}" fill="none" '
                     f'stroke="#666" stroke-width="0.8"/>')
    parts.append(f'<polyline points="{" ".join(tr(p) for p in route)}" fill="none" stroke="#f33" stroke-width="2.5"/>')
    parts.append(f'<circle cx="{tr(start).split(",")[0]}" cy="{tr(start).split(",")[1]}" r="6" fill="#3c3"/>')
    parts.append(f'<circle cx="{tr(route[-1]).split(",")[0]}" cy="{tr(route[-1]).split(",")[1]}" r="6" fill="#39f"/>')
    for x in range(int(lo[0] // 100 + 1) * 100, int(hi[0]), 100):
        parts.append(f'<text x="{x - lo[0]:.0f}" y="{h - 4:.0f}" fill="#aaa" font-size="10">{x}</text>')
    for y in range(int(lo[1] // 100 + 1) * 100, int(hi[1]), 100):
        parts.append(f'<text x="2" y="{hi[1] - y:.0f}" fill="#aaa" font-size="10">{y}</text>')
    parts.append("</svg>")
    Path(path).write_text("\n".join(parts))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    default_osm = Path(os.environ.get("GLL_DATA", Path(__file__).resolve().parents[2] / "data")) \
        / "awsim" / "nishishinjuku_autoware_map" / "lanelet2_map.osm"
    ap.add_argument("osm", type=Path, nargs="?", default=default_osm,
                    help=f"lanelet2 の地図（既定: {default_osm}。tools/awsim/setup.sh が置く）")
    ap.add_argument("--start", required=True, help="車の今の姿勢 'X,Y,YAW_DEG'（地図座標、東から反時計回り）")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--length", type=float, help="この長さ [m] ほど走る（行き先は乱数）")
    g.add_argument("--via", nargs="+", metavar="X,Y", help="通る点（順に）")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--no-lane-change", action="store_true")
    ap.add_argument("-o", "--out", type=Path, required=True)
    ap.add_argument("--svg", type=Path, default=None)
    args = ap.parse_args()
    sx, sy, syaw = (float(v) for v in args.start.split(","))
    start = np.array([sx, sy])
    lls = load_lanelets(args.osm)
    lane_change = not args.no_lane_change
    st = find_lanelet(lls, start, math.radians(syaw))
    if st is None:
        sys.exit("車の近く（3 m 以内）に、車の向きと同じ向きの車線が無い（--start の値を確かめる）")
    a, start_k, d0 = st
    if args.length is not None:
        path, kinds = random_walk(lls, a, args.length, np.random.default_rng(args.seed), lane_change, start_k)
        end_k = None
    else:
        path, kinds, cur, cur_k = [a], [], a, start_k
        live = live_set(lls, lane_change)
        end_k = None
        for s in args.via:
            p = np.array([float(v) for v in s.split(",")])
            # 点から 10 m 以内の車線を全部候補にし、進行方向を守って最も近く行けるものを選ぶ
            near = {}
            for ll in lls.values():
                k, d = project(ll, p)
                if d <= 10.0:
                    near[ll.id] = k
            if not near:
                sys.exit(f"{s}: 10 m 以内に車線が無い")
            if cur in near and near[cur] > cur_k:
                end_k = cur_k = near[cur]
                continue
            last = s == args.via[-1]
            # 途中の点では、先へ進める車線（地図の端の行き止まりにならないもの）を先に探す
            r = None if last else shortest(lls, cur, set(near) & live, lane_change)
            if r is None:
                r = shortest(lls, cur, set(near), lane_change)
            if r is None:
                sys.exit(f"{s} へ、進行方向を守って行ける道が無い（前の点から、その向きへは行けない）")
            if not last and r[0][-1] not in live:
                sys.exit(f"{s} の近くの車線は、地図の端で行き止まりになる。その先の点へは行けないので、"
                         "この点を最後にするか、別の点にする（--svg で道の形を見る）")
            path += r[0][1:]
            kinds += r[1]
            cur = path[-1]
            cur_k = end_k = near[cur]
    pts = geometry(lls, path, kinds, start_k, end_k)
    pts = np.vstack([start, pts]) if np.linalg.norm(pts[0] - start) > 0.3 else pts
    route = thin(smooth(pts))
    length = polyline_len(route)
    np.savetxt(args.out, route, fmt="%.2f",
               header=f"plan_route.py: {args.osm.name}, start {args.start}, "
                      + (f"length {args.length:.0f} seed {args.seed}" if args.length else f"via {' '.join(args.via)}"))
    n_change = sum(k.startswith("change") for k in kinds)
    print(f"{args.out}: {length:.0f} m, {len(route)} 点、lanelet {len(path)} 個、車線変更 {n_change} 回、"
          f"約 {length / 1.67 / 60:.1f} 分（6 km/h）。始点は車から {d0:.1f} m")
    if args.svg:
        write_svg(args.svg, lls, route, start)
        print(f"  {args.svg}")


if __name__ == "__main__":
    main()
