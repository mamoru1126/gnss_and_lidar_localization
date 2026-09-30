"""drive_core（経路追従）と rigid（回転）の単体テスト。"""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import rigid as A  # noqa: E402
from drive_core import PurePursuit, clean_route, load_route, simulate  # noqa: E402

REPO = Path(__file__).resolve().parents[3]


class TestDrive(unittest.TestCase):
    def test_square(self):
        route = np.array([[0, 0], [40, 0], [40, 20], [0, 20], [0, 0.5]], dtype=float)
        traj, dev, pp = simulate(route, 0.0, 0.0, 0.0)
        self.assertLess(dev, 1.2)  # 直角の角で内側に入る分
        self.assertLess(np.hypot(*(traj[-1] - route[-1])), 1.0)

    def test_nishishinjuku_route(self):
        # 最初の約 500 m（この経路はその先で U ターンする。車では曲がれないので、走らせる経路は作り直す: README.md）
        route = load_route(REPO / "tools" / "tile_demo" / "route_nishishinjuku.txt")[:120]
        d = route[1] - route[0]
        traj, dev, pp = simulate(route, route[0, 0], route[0, 1], math.atan2(d[1], d[0]), t_max=1000)
        self.assertLess(dev, 0.5)
        self.assertLess(pp.remaining(), 0.5)
        self.assertEqual(pp.tight_turns(), [])
        full = PurePursuit(load_route(REPO / "tools" / "tile_demo" / "route_nishishinjuku.txt"))
        self.assertGreater(len(full.tight_turns()), 0)  # U ターンを見つける

    def test_fast(self):
        # 25 km/h でも、曲がる所で減速して車線（幅 3.5 m）の中に収まり、経路の終わりで止まる
        route = load_route(REPO / "tools" / "tile_demo" / "route_nishishinjuku.txt")[:120]
        d = route[1] - route[0]
        traj, dev, pp = simulate(route, route[0, 0], route[0, 1], math.atan2(d[1], d[0]), t_max=600, speed=25 / 3.6)
        self.assertLess(dev, 1.0)
        self.assertLess(np.hypot(*(traj[-1] - route[-1])), 1.0)
        self.assertLess(pp.v_prof.min(), 25 / 3.6 - 1)  # どこかで減速している
        self.assertLess(len(traj) * 0.05, 471 / (25 / 3.6) * 1.6)

    def test_drag(self):
        # 抵抗のある車でも、積分で 25 km/h に届き、終わりで止まる（比例だけでは 20 km/h ほどで止まっていた）
        route = load_route(REPO / "tools" / "tile_demo" / "route_nishishinjuku.txt")[:120]
        d = route[1] - route[0]
        traj, dev, pp = simulate(route, route[0, 0], route[0, 1], math.atan2(d[1], d[0]), t_max=600,
                                 speed=25 / 3.6, drag=0.2)
        v = np.linalg.norm(np.diff(traj, axis=0), axis=1) / 0.05
        self.assertGreater(v.max() * 3.6, 24.0)
        self.assertLess(v.max() * 3.6, 27.0)
        self.assertLess(np.hypot(*(traj[-1] - route[-1])), 1.0)

    def test_clean_route(self):
        r = clean_route(np.array([[0, 0], [5, 0], [10, 0], [9, 0.1], [10, 0], [10, 0], [15, 0]], dtype=float))
        self.assertTrue(np.allclose(r[[0, -1]], [[0, 0], [15, 0]]))
        d = np.diff(r, axis=0)
        self.assertTrue(np.all(d[:, 0] > 0))  # 折り返しが無い

    def test_far_start(self):
        pp = PurePursuit(np.array([[0, 0], [10, 0]], dtype=float))
        with self.assertRaises(ValueError):
            pp.start(0.0, 20.0)


class TestRigid(unittest.TestCase):
    def test_rpy_roundtrip(self):
        for rpy in [(0.1, -0.2, 2.5), (math.pi, 0.0, math.pi / 2), (0.0, 0.0, -3.0)]:
            R = A.rpy_to_rot(*rpy)
            self.assertTrue(np.allclose(A.rpy_to_rot(*A.rpy_of(R)), R, atol=1e-9))
            self.assertTrue(np.allclose(A.quat_to_rot(A.rot_to_quat(R)), R, atol=1e-9))

    def test_kabsch(self):
        R = A.rpy_to_rot(0.3, -0.4, 1.2)
        src = np.random.default_rng(0).normal(size=(50, 3))
        Rh, _ = A.kabsch_origin(src, src @ R.T)
        self.assertTrue(np.allclose(Rh, R, atol=1e-9))


if __name__ == "__main__":
    unittest.main()
