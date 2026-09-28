"""analysis.py の単体テスト（numpy だけで動く）。python3 -m unittest discover -s tools/grandtour/test"""
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import analysis as A  # noqa: E402


def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def walk(n=2000, dt=0.05, speed=0.6):
    """前に歩きながら曲がる軌跡（歩行の揺れの roll / pitch 付き）。standard の解釈の (t, p, q, v_body)。"""
    t = np.arange(n) * dt
    yaw = 0.3 * np.sin(0.05 * t) + 0.02 * t
    roll = np.deg2rad(2.0) * np.sin(2 * np.pi * 2.0 * t)
    pitch = np.deg2rad(1.5) * np.sin(2 * np.pi * 2.0 * t + 0.7)
    v_world = np.column_stack([speed * np.cos(yaw), speed * np.sin(yaw), np.zeros(n)])
    p = np.cumsum(v_world * dt, axis=0) + np.array([10.0, -5.0, 0.5])
    R = np.stack([rpy(a, b, c) for a, b, c in zip(roll, pitch, yaw)])
    q = np.stack([A.rot_to_quat(r) for r in R])
    v_body = np.einsum("nji,nj->ni", R, v_world)
    return t, p, q, v_body, R


class TestRotation(unittest.TestCase):
    def test_quat_roundtrip(self):
        rng = np.random.default_rng(0)
        for _ in range(50):
            R = rpy(*rng.uniform(-3, 3, 3))
            self.assertTrue(np.allclose(A.quat_to_rot(A.rot_to_quat(R)), R, atol=1e-12))

    def test_rot_log(self):
        R = rpy(0.1, -0.2, 0.3)
        v = A.rot_log(R[None])[0]
        ang = np.linalg.norm(v)
        k = v / ang
        K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        R2 = np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K
        self.assertTrue(np.allclose(R, R2, atol=1e-12))


class TestOdometryConventions(unittest.TestCase):
    def test_semantics_standard(self):
        t, p, q, _, _ = walk()
        s = A.pose_semantics_score(t, p, q)
        self.assertGreater(s["standard"], 0.95)
        self.assertLess(s["inverted"], s["standard"])

    def test_semantics_inverted(self):
        t, p, q, _, R = walk()
        p_inv = -np.einsum("nji,nj->ni", R, p)  # 親フレームを機体で表したもの
        q_inv = q * np.array([-1, -1, -1, 1])
        s = A.pose_semantics_score(t, p_inv, q_inv)
        self.assertGreater(s["inverted"], 0.95)
        self.assertLess(s["standard"], s["inverted"])

    def test_semantics_backward_frame(self):
        # 真値のフレームが機体に対して後ろ向き（yaw 180°）でも、standard を選ぶ
        t, p, q, _, R = walk()
        Rf = R @ rpy(0, 0, np.pi)
        qf = np.stack([A.rot_to_quat(r) for r in Rf])
        s = A.pose_semantics_score(t, p, qf)
        self.assertGreater(s["standard"], 0.95)
        self.assertLess(s["inverted"], s["standard"])
        self.assertGreater(abs(s["standard_dir"]), 170)

    def test_odom_convention_all_cases(self):
        t, p, q, v_body, R = walk()
        v_world = np.einsum("nij,nj->ni", R, v_body)
        p_inv = -np.einsum("nji,nj->ni", R, p)
        q_inv = q * np.array([-1, -1, -1, 1])
        Rb = R @ rpy(0, 0, np.pi)  # 後ろ向きのフレーム
        qb = np.stack([A.rot_to_quat(r) for r in Rb])
        vb = np.einsum("nji,nj->ni", Rb, v_world)
        cases = [((p, q, v_body), ("standard", "child")), ((p, q, v_world), ("standard", "parent")),
                 ((p_inv, q_inv, v_body), ("inverted", "child")), ((p_inv, q_inv, v_world), ("inverted", "parent")),
                 ((p, qb, vb), ("standard", "child"))]
        for (pp, qq, vv), want in cases:
            c = A.odom_convention(t, pp, qq, vv)
            self.assertEqual((c["semantics"], c["twist_frame"]), want)
            self.assertEqual(c["decided_by"], "twist")
        c = A.odom_convention(t, p, q, np.zeros_like(v_body))  # 速度が使えない → 向きのそろい方
        self.assertEqual((c["semantics"], c["decided_by"]), ("standard", "direction"))

    def test_twist_frame(self):
        t, p, q, v_body, R = walk()
        r = A.twist_frame_residual(t, p, q, v_body)
        self.assertLess(r["child"], 0.02)
        self.assertGreater(r["parent"], 0.1)
        v_world = np.einsum("nij,nj->ni", R, v_body)
        r2 = A.twist_frame_residual(t, p, q, v_world)
        self.assertLess(r2["parent"], 0.02)


class TestPaths(unittest.TestCase):
    def test_overlap_same_and_opposite(self):
        s = np.linspace(0, 100, 400)
        a = np.column_stack([s, np.zeros_like(s)])
        b = np.column_stack([s[::-1], np.full_like(s, 1.0)])  # 1 m 横を逆向きに
        pa, ha = A.resample_path(a, 0.5)
        pb, hb = A.resample_path(b, 0.5)
        o = A.overlap(pa, ha, pb, hb, radius=5.0, step=0.5)
        self.assertGreater(o["frac"], 0.99)
        self.assertGreater(o["opposite"], 95)
        self.assertLess(o["same"], 1)
        c = np.column_stack([s, np.full_like(s, 20.0)])  # 20 m 離れた平行な道
        pc, hc = A.resample_path(c, 0.5)
        self.assertEqual(A.overlap(pa, ha, pc, hc, 5.0, 0.5)["frac"], 0.0)

    def test_group_sites(self):
        s = np.linspace(0, 50, 100)
        paths = {"a": np.column_stack([s, 0 * s]), "b": np.column_stack([s, 0 * s + 30]),
                 "c": np.column_stack([s + 5000, 0 * s])}
        groups = sorted(sorted(g) for g in A.group_sites(paths, 300.0))
        self.assertEqual(groups, [["a", "b"], ["c"]])

    def test_stationary_start(self):
        t = np.arange(0, 10, 0.01)
        sp = np.where(t < 3.0, 0.001, 0.5)
        self.assertAlmostEqual(A.stationary_start(t, sp, 0 * t), 3.0, delta=0.02)

    def test_gaps(self):
        t = np.concatenate([np.arange(0, 5, 0.1), np.arange(8, 10, 0.1)])
        g = A.gaps(t, 1.0)
        self.assertEqual(len(g), 1)
        self.assertAlmostEqual(g[0][1], 3.1, delta=1e-6)


class TestFits(unittest.TestCase):
    def test_fit_station(self):
        t, p, q, _, R = walk(n=1500)
        lever = np.array([-0.2, 0.05, 0.6])
        Rs = rpy(0.0, 0.0, np.deg2rad(-50))
        ts = np.array([100.0, 20.0, -3.0])
        prism_world = p + np.einsum("nij,j->ni", R, lever)
        prism_station = (prism_world - ts) @ Rs  # station = Rs^T (world - ts)
        rng = np.random.default_rng(1)
        prism_station += rng.normal(0, 0.002, prism_station.shape)
        fit = A.fit_station(prism_station, p, R)
        self.assertLess(fit["rms"], 0.01)
        self.assertTrue(np.allclose(fit["lever"][:2], lever[:2], atol=0.02))

    def test_handeye(self):
        t, p, q, _, R = walk(n=3000)
        X = rpy(0.02, -0.03, 3.1)  # 機体 b → 機体 a の向き
        C = rpy(0, 0, 0.5)
        Ra = np.einsum("ij,njk,kl->nil", C, R, X)
        qa = np.stack([A.rot_to_quat(r) for r in Ra])
        he = A.handeye_rotation(t, qa, t, q)
        self.assertIsNotNone(he)
        self.assertLess(A.angle_between(he["X"], X), 0.5)


if __name__ == "__main__":
    unittest.main()
