"""evaluate.py の単体テスト（真値に分かっているずれを足した出力で、指標を確かめる）。"""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import evaluate as E  # noqa: E402


def make(n=500, lat_off=0.1, lon_off=0.05, yaw_off=0.01, var=0.0025):
    t = 1.7e9 + np.arange(n) * 0.02
    yaw = 0.3 + 0.001 * np.arange(n)
    x = 381200 + np.cumsum(np.cos(yaw)) * 0.03
    y = 3949800 + np.cumsum(np.sin(yaw)) * 0.03
    gt = [dict(t=t[i], x=x[i], y=y[i], z=0, roll=0, pitch=0, yaw=yaw[i]) for i in range(n)]
    out = []
    for i in range(n):
        c, s = math.cos(yaw[i]), math.sin(yaw[i])
        st = "INITIALIZING" if i < 50 else ("GNSS_LIDAR_AIDED" if i < 300 else "LIDAR_AIDED")
        out.append(dict(t=t[i], x=x[i] + c * lon_off - s * lat_off, y=y[i] + s * lon_off + c * lat_off,
                        yaw=yaw[i] + yaw_off, var_x=var, var_y=var, var_yaw=1e-4, status=st, dr_distance=0.0))
    return [{k: str(v) for k, v in r.items()} for r in out], [{k: str(v) for k, v in r.items()} for r in gt]


class TestEvaluate(unittest.TestCase):
    def test_offsets(self):
        out, gt = make()
        m, per = E.evaluate(out, gt)
        self.assertAlmostEqual(m["lat_rms"], 0.1, places=6)
        self.assertAlmostEqual(m["lon_rms"], 0.05, places=6)
        self.assertAlmostEqual(m["yaw_rms_deg"], math.degrees(0.01), places=6)
        self.assertAlmostEqual(m["init_time"], 1.0, places=3)
        self.assertEqual(m["evaluated_rows"], 450)
        self.assertLess(m["step_max"], 1e-3)  # ずれが向きと一緒に回る分だけ
        self.assertAlmostEqual(m["lidar_share"], 1.0)
        self.assertAlmostEqual(per["LIDAR_AIDED"]["share"], 200 / 450)
        self.assertAlmostEqual(m["within_3sigma"], 1.0)  # 0.1 m < 3 × 0.05 m

    def test_covariance_too_small(self):
        out, gt = make(var=0.0001)  # σ 1 cm に対して 10 cm ずれている
        m, _ = E.evaluate(out, gt)
        self.assertEqual(m["within_3sigma"], 0.0)
        self.assertGreater(m["nees_mean"], 50)


if __name__ == "__main__":
    unittest.main()
