#!/usr/bin/env python3
"""lanelet2 の地図（.osm）の道路の中心線に沿って、目標点を順に回る経路を作る（タイル読み込みのデモ用）。

  python3 lanelet_route.py lanelet2_map.osm '[[x0, y0], [x1, y1], ...]' > route.txt

各 lanelet の中心線の点をつなぎ、2.5 m 以内の点どうし（隣の lanelet・車線）もつないだ無向グラフで、
目標点に最も近い点を順に最短経路で結ぶ。進行方向（一方通行）は見ない。座標はノードの local_x / local_y。
出力は 3 m 間隔の「x y」の行（gll_tile_demo の経路ファイル）。numpy と scipy が必要。
"""
import heapq
import json
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import cKDTree


def resample(a, n):
    d = np.r_[0, np.cumsum(np.linalg.norm(np.diff(a, axis=0), axis=1))]
    s = np.linspace(0, d[-1], n)
    return np.c_[np.interp(s, d, a[:, 0]), np.interp(s, d, a[:, 1])]


def centerlines(osm_path, step=2.0):
    root = ET.parse(osm_path).getroot()
    nodes = {}
    for n in root.iter("node"):
        t = {x.get("k"): x.get("v") for x in n.findall("tag")}
        if "local_x" in t:
            nodes[n.get("id")] = (float(t["local_x"]), float(t["local_y"]))
    ways = {w.get("id"): [nd.get("ref") for nd in w.findall("nd")] for w in root.iter("way")}
    for r in root.iter("relation"):
        t = {x.get("k"): x.get("v") for x in r.findall("tag")}
        if t.get("type") != "lanelet" or t.get("subtype") != "road":
            continue
        m = {x.get("role"): x.get("ref") for x in r.findall("member") if x.get("type") == "way"}
        if "left" not in m or "right" not in m:
            continue
        left = np.array([nodes[i] for i in ways[m["left"]]])
        right = np.array([nodes[i] for i in ways[m["right"]]])
        length = max(np.sum(np.linalg.norm(np.diff(left, axis=0), axis=1)), 1e-3)
        n = max(2, int(length / step) + 1)
        yield (resample(left, n) + resample(right, n)) / 2


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    pts, edges = [], []
    for c in centerlines(sys.argv[1]):
        base = len(pts)
        pts.extend(c)
        edges += [(base + i, base + i + 1) for i in range(len(c) - 1)]
    P = np.array(pts)
    tree = cKDTree(P)
    edges += list(tree.query_pairs(2.5))  # 隣の lanelet・車線へのつながり
    adj = [[] for _ in P]
    for i, j in edges:
        w = float(np.linalg.norm(P[i] - P[j]))
        adj[i].append((j, w))
        adj[j].append((i, w))

    def shortest(a, b):
        dist, prev, pq = {a: 0.0}, {}, [(0.0, a)]
        while pq:
            d, u = heapq.heappop(pq)
            if u == b:
                break
            if d > dist[u]:
                continue
            for v, w in adj[u]:
                if d + w < dist.get(v, 1e18):
                    dist[v], prev[v] = d + w, u
                    heapq.heappush(pq, (d + w, v))
        if b not in dist:
            return None
        path = [b]
        while path[-1] != a:
            path.append(prev[path[-1]])
        return path[::-1]

    targets = [tree.query(t)[1] for t in json.loads(sys.argv[2])]
    route = [targets[0]]
    for b in targets[1:]:
        p = shortest(route[-1], b)
        if p is None:
            sys.exit(f"no path to {P[b]}")
        route += p[1:]
    r = P[route]
    k = 5  # 車線の切り替えの段差をならす
    sm = np.array([r[max(0, i - k):i + k + 1].mean(axis=0) for i in range(len(r))])
    sm[0], sm[-1] = r[0], r[-1]
    out = [sm[0]]
    for q in sm[1:]:
        if np.linalg.norm(q - out[-1]) >= 3.0:
            out.append(q)
    out = np.array(out)
    print(f"route length {np.sum(np.linalg.norm(np.diff(out, axis=0), axis=1)):.0f} m, {len(out)} points",
          file=sys.stderr)
    np.savetxt(sys.stdout, out, fmt="%.2f")


if __name__ == "__main__":
    main()
