"""経路追従（pure pursuit）と速度の制御。ROS に依存しない（awsim_drive.py とテストから使う）。"""
import math

import numpy as np


def load_route(path, max_turn_deg=120.0):
    """「x y」の行（# はコメント）→ N×2。tools/tile_demo/route_nishishinjuku.txt と同じ形。clean_route を通す。"""
    pts = []
    with open(path) as fp:
        for line in fp:
            line = line.split("#", 1)[0].strip()
            if line:
                x, y = line.split()[:2]
                pts.append((float(x), float(y)))
    if len(pts) < 2:
        raise ValueError(f"{path}: 点が 2 つ以上要る")
    return clean_route(np.array(pts), max_turn_deg)


def clean_route(route, max_turn_deg=120.0):
    """同じ点の重なりと、折り返し（1 点で max_turn_deg より大きく向きが変わる点。lanelet のつなぎ目に出る）を除く。"""
    r = [p for i, p in enumerate(np.asarray(route, dtype=float)) if i == 0 or np.hypot(*(p - route[i - 1])) > 1e-3]
    lim = np.radians(max_turn_deg)
    changed = True
    while changed and len(r) > 2:
        changed = False
        for i in range(1, len(r) - 1):
            a, b = r[i] - r[i - 1], r[i + 1] - r[i]
            na, nb = np.linalg.norm(a), np.linalg.norm(b)
            if na < 1e-3 or nb < 1e-3 or math.acos(np.clip(a @ b / (na * nb), -1, 1)) > lim:
                del r[i]
                changed = True
                break
    return np.array(r)


def resample(route, step=0.5):
    d = np.r_[0, np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))]
    s = np.arange(0, d[-1], step)
    return np.c_[np.interp(s, d, route[:, 0]), np.interp(s, d, route[:, 1])], s


class PurePursuit:
    """経路の点を順に進む追従。今いる場所に最も近い点から始め、後戻りはしない。"""

    def __init__(self, route, wheelbase=2.79, speed=1.67, lookahead_min=3.0, lookahead_gain=1.0,
                 max_steer=math.radians(35), accel_gain=0.8, max_accel=1.0, max_decel=2.0, stop_dist=5.0):
        self.path, self.s = resample(np.asarray(route, dtype=float))
        self.wb, self.v_ref = wheelbase, speed
        self.l_min, self.l_gain = lookahead_min, lookahead_gain
        self.max_steer, self.k_acc = max_steer, accel_gain
        self.max_acc, self.max_dec, self.stop_dist = max_accel, max_decel, stop_dist
        self.idx = None

    def start(self, x, y, max_dist=5.0):
        """今の位置に最も近い経路の点を探す。max_dist より遠ければ ValueError。"""
        d = np.hypot(self.path[:, 0] - x, self.path[:, 1] - y)
        self.idx = int(np.argmin(d))
        if d[self.idx] > max_dist:
            raise ValueError(f"経路が車から {d[self.idx]:.1f} m 離れている（最初の点を車の近くにする）")
        return float(d[self.idx])

    def tight_turns(self, window=2.0):
        """最小回転半径（wheelbase / tan(max_steer)）より急に曲がる所の、経路に沿った距離 [m] の並び。"""
        h = np.unwrap(np.arctan2(*np.diff(self.path, axis=0).T[::-1]))
        n = max(int(window / 0.5), 1)
        if len(h) <= n:
            return []
        curv = np.abs(h[n:] - h[:-n]) / window
        r_min = self.wb / math.tan(self.max_steer)
        idx = np.nonzero(curv > 1.0 / r_min)[0]
        out = []
        for i in idx:
            if not out or self.s[i] - out[-1] > 5.0:
                out.append(float(self.s[i]))
        return out

    def remaining(self):
        return float(self.s[-1] - self.s[self.idx])

    def step(self, x, y, yaw, v):
        """(操舵角 [rad]、目標速度 [m/s]、加速度 [m/s²]、終わったか) を返す。"""
        # 近くの点へ進める（前方の 30 点の中から）
        seg = self.path[self.idx: self.idx + 30]
        k = int(np.argmin(np.hypot(seg[:, 0] - x, seg[:, 1] - y)))
        self.idx += k
        rem = self.remaining()
        done = rem < 0.5
        v_ref = 0.0 if done else min(self.v_ref, math.sqrt(max(2 * 0.5 * max(rem - 1.0, 0.0), 0.0)) + 0.3)
        if rem < self.stop_dist:
            v_ref = min(v_ref, self.v_ref * rem / self.stop_dist)
        ld = self.l_min + self.l_gain * max(v, 0.0)
        j = int(np.searchsorted(self.s, self.s[self.idx] + ld))
        tx, ty = self.path[min(j, len(self.path) - 1)]
        dx, dy = tx - x, ty - y
        ay = -math.sin(yaw) * dx + math.cos(yaw) * dy  # 機体座標系の横
        dist = max(math.hypot(dx, dy), 1e-3)
        steer = math.atan2(2.0 * self.wb * ay, dist * dist)
        steer = max(-self.max_steer, min(self.max_steer, steer))
        acc = max(-self.max_dec, min(self.max_acc, self.k_acc * (v_ref - v)))
        return steer, v_ref, acc, done


def simulate(route, x, y, yaw, dt=0.05, t_max=600.0, **kw):
    """自転車モデルで走らせる（テスト用）。(軌跡 N×2、最大の横ずれ) を返す。"""
    pp = PurePursuit(route, **kw)
    pp.start(x, y)
    v, traj = 0.0, []
    for _ in range(int(t_max / dt)):
        steer, v_ref, acc, done = pp.step(x, y, yaw, v)
        v = max(0.0, v + acc * dt)
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw += v / pp.wb * math.tan(steer) * dt
        traj.append((x, y))
        if done and v < 0.01:
            break
    traj = np.array(traj)
    d = np.min(np.hypot(traj[:, None, 0] - pp.path[None, :, 0], traj[:, None, 1] - pp.path[None, :, 1]), axis=1)
    return traj, float(d.max()), pp
