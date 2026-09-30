"""plan_route.py の単体テスト（小さな合成の lanelet2 の地図で）。

地図: 一方通行（反時計回り）の 100 m × 60 m の四角い環状の道（2 車線、間は破線）と、
東の辺の途中から出て地図の端で終わる行き止まりの道。
"""
import math
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import plan_route as P  # noqa: E402

W = 3.5


class Osm:
    def __init__(self):
        self.nodes, self.ways, self.rels, self.nid = [], [], [], 1

    def node(self, x, y):
        i = self.nid
        self.nid += 1
        self.nodes.append(f'<node id="{i}" lat="0" lon="0"><tag k="local_x" v="{x:.3f}"/><tag k="local_y" v="{y:.3f}"/></node>')
        return i

    def way(self, ids, subtype):
        i = self.nid
        self.nid += 1
        nds = "".join(f'<nd ref="{n}"/>' for n in ids)
        self.ways.append(f'<way id="{i}">{nds}<tag k="type" v="line_thin"/><tag k="subtype" v="{subtype}"/></way>')
        return i

    def lanelet(self, left, right):
        i = self.nid
        self.nid += 1
        self.rels.append(f'<relation id="{i}"><member type="way" role="left" ref="{left}"/>'
                         f'<member type="way" role="right" ref="{right}"/><tag k="type" v="lanelet"/>'
                         f'<tag k="subtype" v="road"/><tag k="one_way" v="yes"/></relation>')
        return i

    def write(self, path):
        Path(path).write_text('<?xml version="1.0"?><osm version="0.6">' + "".join(self.nodes + self.ways + self.rels)
                              + "</osm>")


def make_map(path):
    """反時計回りの環状の道。四角の辺ごとに lanelet（内側の車線と外側の車線）。角は丸めない（辺を細かく分けるだけ）。"""
    o = Osm()
    corners = [(0, 0), (100, 0), (100, 60), (0, 60)]
    # 3 本の境界線（内側の縁、車線の間、外側の縁）を、四角の中心からのずれで作る
    def ring(off):
        c = np.array([50.0, 30.0])
        pts = []
        for i in range(4):
            a, b = np.array(corners[i], float), np.array(corners[(i + 1) % 4], float)
            for t in np.linspace(0, 1, 11)[:-1]:
                p = a + t * (b - a)
                pts.append(p)
        pts = np.array(pts)
        # 角の外向きに off だけずらす（おおよそ。テストには十分）
        d = pts - c
        n = np.c_[np.sign(d[:, 0]) * (np.abs(d[:, 0]) >= 50 - 1e-6), np.sign(d[:, 1]) * (np.abs(d[:, 1]) >= 30 - 1e-6)]
        return pts + off * n
    inner, mid, outer = ring(-W), ring(0.0), ring(W)
    ids = {k: [o.node(*p) for p in r] for k, r in (("in", inner), ("mid", mid), ("out", outer))}
    n = len(inner)
    lanes_in, lanes_out = [], []
    for s in range(4):  # 辺ごと
        idx = [(s * 10 + k) % n for k in range(11)]
        wi = o.way([ids["in"][k] for k in idx], "solid")
        wm = o.way([ids["mid"][k] for k in idx], "dashed")
        wo = o.way([ids["out"][k] for k in idx], "solid")
        # 反時計回りに進むとき、左 = 内側
        lanes_in.append(o.lanelet(wi, wm))
        lanes_out.append(o.lanelet(wm, wo))
    # 東の辺の途中（x = 100 + W、y = 30）から東へ出る行き止まりの道
    left = o.way([o.node(100 + W, 30 + W / 2), o.node(160, 30 + W / 2)], "solid")
    right = o.way([o.node(100 + W, 30 - W / 2), o.node(160, 30 - W / 2)], "solid")
    o.lanelet(left, right)
    o.write(path)


class TestPlanRoute(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.osm = Path(cls.tmp.name) / "map.osm"
        make_map(cls.osm)
        cls.lls = P.load_lanelets(cls.osm)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_graph(self):
        self.assertEqual(sum(1 for ll in self.lls.values() if ll.succ), 8)  # 環状の 8 つだけが次へつながる
        self.assertEqual(sum(1 for ll in self.lls.values() if ll.change), 8)  # 破線をはさんで、互いに移れる
        live = P.live_set(self.lls, True)
        self.assertEqual(len(live), 8)  # 行き止まりの道は入らない

    def test_random_walk_follows_direction(self):
        start = self.lls[P.find_lanelet(self.lls, np.array([20.0, -W / 2]), 0.0)[0]]
        self.assertLess(abs(start.center[0, 1] + W / 2), 0.1)
        rng = np.random.default_rng(0)
        a, k, _ = P.find_lanelet(self.lls, np.array([20.0, -W / 2]), 0.0)
        path, kinds = P.random_walk(self.lls, a, 600, rng, True, k)
        pts = P.geometry(self.lls, path, kinds, k)
        # 反時計回り: 四角の中心から見た角度が増えていく
        ang = np.unwrap(np.arctan2(pts[:, 1] - 30, pts[:, 0] - 50))
        self.assertGreater(ang[-1] - ang[0], 2 * math.pi)
        self.assertTrue(np.all(np.diff(ang) > -0.05))

    def test_cli(self):
        out = Path(self.tmp.name) / "route.txt"
        sys.argv = ["plan_route.py", str(self.osm), "--start", f"20,{-W / 2},0", "--via", "95,40", "50,64",
                    "-o", str(out), "--svg", str(out.with_suffix(".svg"))]
        P.main()
        r = np.loadtxt(out)
        self.assertLess(np.hypot(*(r[0] - [20, -W / 2])), 0.5)
        self.assertLess(np.hypot(*(r[-1] - [50, 60 + W / 2])), 3.0)
        self.assertTrue(out.with_suffix(".svg").read_text().startswith("<svg"))

    def test_off_world(self):
        # 点群が環状の道の所にしか無ければ、東へ出る行き止まりの道（x = 104〜160）は世界の外として使わない
        pts = np.vstack([np.c_[np.linspace(-5, 105, 200), np.full(200, y)] for y in (-5, 0, 60, 65)]
                        + [np.c_[np.full(200, x), np.linspace(-5, 65, 200)] for x in (-5, 0, 100, 105)])
        pcd = Path(self.tmp.name) / "pointcloud_map.pcd"
        pcd.write_text("# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
                       f"WIDTH {len(pts)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(pts)}\nDATA ascii\n"
                       + "\n".join(f"{x:.2f} {y:.2f} 0" for x, y in pts) + "\n")
        off = P.mark_off_world(P.load_lanelets(self.osm), P.read_pcd_xy(pcd), 20.0)
        self.assertEqual(len(off), 1)
        spur = [ll for ll in self.lls.values() if ll.center[:, 0].max() > 150]
        self.assertEqual(off, {spur[0].id})

    def test_wrong_direction_start(self):
        self.assertIsNone(P.find_lanelet(self.lls, np.array([20.0, -W / 2]), math.pi))


if __name__ == "__main__":
    unittest.main()
