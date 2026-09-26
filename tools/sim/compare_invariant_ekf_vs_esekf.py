#!/usr/bin/env python3
"""Invariant EKF (SE(2), left-invariant error) と ESEKF (SO(2) x R^2) の比較シミュレーション。

docs/algorithm.md の 6 章で使う図と数値を生成する。
- 車両: 平面運動。ODOM 速度 + ジャイロ（バイアスあり）で予測
- 観測: RTK-FIX の GNSS 位置（シングルアンテナ、レバーアームあり）のみ
- 初期 yaw 誤差を大きくした条件で、yaw の収束と共分散の整合性（NEES）を比べる

使い方:
    python3 tools/sim/compare_invariant_ekf_vs_esekf.py [--runs 200] [--out docs/figures]
"""
import argparse
import os

import numpy as np

J = np.array([[0.0, -1.0], [1.0, 0.0]])


def rot(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s], [s, c]])


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def V(phi):
    if abs(phi) < 1e-9:
        return np.eye(2) + 0.5 * phi * J
    return np.sin(phi) / phi * np.eye(2) + (1 - np.cos(phi)) / phi * J


def se2_exp(xi):
    """xi = (rho_x, rho_y, phi) -> (R, t)"""
    return rot(xi[2]), V(xi[2]) @ xi[:2]


def se2_log(R, t):
    phi = np.arctan2(R[1, 0], R[0, 0])
    return np.r_[np.linalg.solve(V(phi), t), phi]


# ----------------------------------------------------------------------------
# 共通パラメータ
# ----------------------------------------------------------------------------
DT = 0.01            # 予測周期 [s]（IMU 100 Hz）
GNSS_EVERY = 10      # GNSS 10 Hz
T_END = 40.0
SIG_V, SIG_W = 0.05, 0.01          # 入力ノイズ
SIG_B, SIG_S = 1e-4, 1e-4          # ランダムウォーク
SIG_GNSS = 0.02
LEVER = np.array([0.3, 0.2])       # GNSS アンテナのレバーアーム [m]


def true_inputs(t):
    """6 km/h 程度の走行: 直進 → 左旋回 → 直進 → 右旋回 ..."""
    v = 1.5 if t > 1.0 else 1.5 * t  # 最初の 1 秒で加速
    w = 0.25 * np.sin(2 * np.pi * t / 20.0)
    return v, w


# ----------------------------------------------------------------------------
# Invariant EKF（左不変誤差: X = Xhat Exp(xi)）
# ----------------------------------------------------------------------------
class InvariantEKF:
    def __init__(self, p, th, b, s, P):
        self.R, self.p, self.b, self.s, self.P = rot(th), p.copy(), b, s, P.copy()

    @property
    def th(self):
        return np.arctan2(self.R[1, 0], self.R[0, 0])

    def predict(self, v, w):
        dphi = (w - self.b) * DT
        drho = np.array([self.s * v * DT, 0.0])
        Rd, td = se2_exp(np.r_[drho, dphi])
        self.p = self.p + self.R @ td
        self.R = self.R @ Rd
        F = np.eye(5)
        F[:2, :2] = Rd.T
        F[:2, 2] = Rd.T @ J @ td
        F[:2, 4] = [v * DT, 0.0]
        F[2, 3] = -DT
        G = np.zeros((5, 2))
        G[0, 0] = self.s * DT
        G[2, 1] = DT
        Q = G @ np.diag([SIG_V**2, SIG_W**2]) @ G.T + np.diag([0, 0, 0, SIG_B**2 * DT, SIG_S**2 * DT])
        self.P = F @ self.P @ F.T + Q

    def update_gnss(self, y):
        r = self.R.T @ (y - self.p - self.R @ LEVER)
        H = np.zeros((2, 5))
        H[:, :2] = np.eye(2)
        H[:, 2] = J @ LEVER
        Rm = self.R.T @ (SIG_GNSS**2 * np.eye(2)) @ self.R
        S = H @ self.P @ H.T + Rm
        K = self.P @ H.T @ np.linalg.inv(S)
        dx = K @ r
        Rd, td = se2_exp(dx[:3])
        self.p = self.p + self.R @ td
        self.R = self.R @ Rd
        self.b += dx[3]
        self.s += dx[4]
        IKH = np.eye(5) - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ Rm @ K.T

    def error(self, p_true, th_true, b_true, s_true):
        """真値 = 推定 Exp(xi) となる xi（機体座標系）と、バイアス誤差"""
        Rt = rot(th_true)
        xi = se2_log(self.R.T @ Rt, self.R.T @ (p_true - self.p))
        return np.r_[xi, b_true - self.b, s_true - self.s]


# ----------------------------------------------------------------------------
# ESEKF（SO(2) x R^2: p = phat + dp, th = thhat + dth）
# ----------------------------------------------------------------------------
class ESEKF:
    def __init__(self, p, th, b, s, P):
        self.p, self.th_, self.b, self.s, self.P = p.copy(), th, b, s, P.copy()

    @property
    def th(self):
        return self.th_

    def predict(self, v, w):
        tb = self.th_ + 0.5 * (w - self.b) * DT
        c, sn = np.cos(tb), np.sin(tb)
        s = self.s
        F = np.eye(5)
        F[0, 2] = -s * v * DT * sn
        F[1, 2] = s * v * DT * c
        F[0, 3] = 0.5 * s * v * DT**2 * sn
        F[1, 3] = -0.5 * s * v * DT**2 * c
        F[0, 4] = v * DT * c
        F[1, 4] = v * DT * sn
        F[2, 3] = -DT
        G = np.array([[s * DT * c, -0.5 * s * v * DT**2 * sn],
                      [s * DT * sn, 0.5 * s * v * DT**2 * c],
                      [0, DT], [0, 0], [0, 0]])
        Q = G @ np.diag([SIG_V**2, SIG_W**2]) @ G.T + np.diag([0, 0, 0, SIG_B**2 * DT, SIG_S**2 * DT])
        self.p = self.p + s * v * DT * np.array([c, sn])
        self.th_ = wrap(self.th_ + (w - self.b) * DT)
        self.P = F @ self.P @ F.T + Q

    def update_gnss(self, y):
        R = rot(self.th_)
        r = y - self.p - R @ LEVER
        H = np.zeros((2, 5))
        H[:, :2] = np.eye(2)
        H[:, 2] = J @ R @ LEVER
        Rm = SIG_GNSS**2 * np.eye(2)
        S = H @ self.P @ H.T + Rm
        K = self.P @ H.T @ np.linalg.inv(S)
        dx = K @ r
        self.p = self.p + dx[:2]
        self.th_ = wrap(self.th_ + dx[2])
        self.b += dx[3]
        self.s += dx[4]
        IKH = np.eye(5) - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ Rm @ K.T

    def error(self, p_true, th_true, b_true, s_true):
        return np.r_[p_true - self.p, wrap(th_true - self.th_), b_true - self.b, s_true - self.s]


# ----------------------------------------------------------------------------
def run(filter_cls, yaw0_err, rng_seed):
    rng = np.random.default_rng(rng_seed)
    # 真値
    p, th, b, s = np.zeros(2), 0.3, 0.003, 1.02
    # 推定の初期値: 位置は最初の GNSS、yaw は誤差あり
    sig_th0 = np.deg2rad(yaw0_err)
    th0 = th + rng.normal(0, sig_th0)
    p0 = p + rot(th) @ LEVER - rot(th0) @ LEVER + rng.normal(0, SIG_GNSS, 2)
    P0 = np.diag([SIG_GNSS**2 * 4, SIG_GNSS**2 * 4, sig_th0**2, 0.005**2, 0.03**2])
    f = filter_cls(p0, th0, 0.0, 1.0, P0)

    n = int(T_END / DT)
    ts, yaw_err, pos_err, nees = [], [], [], []
    for k in range(1, n + 1):
        t = k * DT
        v, w = true_inputs(t)
        # 真値の運動（中点法）
        tb = th + 0.5 * w * DT
        p = p + s * v * DT * np.array([np.cos(tb), np.sin(tb)])
        th = wrap(th + w * DT)
        b += rng.normal(0, SIG_B * np.sqrt(DT))
        # 観測された入力（ODOM は真の速度 / s、ジャイロはバイアス込み）
        v_m = v + rng.normal(0, SIG_V)
        w_m = w + b + rng.normal(0, SIG_W)
        f.predict(v_m, w_m)
        if k % GNSS_EVERY == 0:
            y = p + rot(th) @ LEVER + rng.normal(0, SIG_GNSS, 2)
            f.update_gnss(y)
        if k % GNSS_EVERY == 0:
            e = f.error(p, th, b, s)
            ts.append(t)
            yaw_err.append(np.rad2deg(wrap(th - f.th)))
            pos_err.append(np.linalg.norm(p - f.p))
            nees.append(float(e[:3] @ np.linalg.solve(f.P[:3, :3], e[:3])))
    return np.array(ts), np.array(yaw_err), np.array(pos_err), np.array(nees)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=200)
    ap.add_argument("--out", default="docs/figures")
    args = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    for name in ["Noto Sans CJK JP", "IPAexGothic", "TakaoGothic"]:
        if any(name in f.name for f in font_manager.fontManager.ttflist):
            plt.rcParams["font.family"] = name
            break

    os.makedirs(args.out, exist_ok=True)
    cases = [10, 30, 60]
    summary = {}
    fig, axes = plt.subplots(2, len(cases), figsize=(13, 6.5), sharex=True)
    for ci, y0 in enumerate(cases):
        for cls, name, color in [(ESEKF, "ESEKF", "#d62728"), (InvariantEKF, "Invariant EKF", "#1f77b4")]:
            Y, N, POS = [], [], []
            for r in range(args.runs):
                ts, ye, pe, ne = run(cls, y0, 1000 + r)
                Y.append(np.abs(ye))
                N.append(ne)
                POS.append(pe)
            Y, N, POS = map(np.array, (Y, N, POS))
            rms_yaw = np.sqrt((Y**2).mean(axis=0))
            anees = N.mean(axis=0)
            axes[0, ci].plot(ts, rms_yaw, color=color, label=name)
            axes[1, ci].plot(ts, anees, color=color, label=name)
            idx5 = np.searchsorted(ts, 5.0)
            idx20 = np.searchsorted(ts, 20.0)
            summary[(y0, name)] = dict(
                yaw_rms_5s=rms_yaw[idx5], yaw_rms_20s=rms_yaw[idx20],
                anees_mean=anees[idx5:].mean(), anees_max=anees[idx5:].max(),
                pos_rms_20s=np.sqrt((POS[:, idx20]**2).mean()))
        axes[0, ci].set_title(f"初期 yaw 誤差 σ = {y0}°")
        axes[0, ci].set_yscale("log")
        axes[0, ci].set_ylabel("yaw 誤差 RMS [deg]")
        axes[0, ci].grid(True, which="both", alpha=0.3)
        axes[1, ci].axhline(3.0, color="k", ls="--", lw=1, label="理想値 3（整合）")
        axes[1, ci].set_yscale("log")
        axes[1, ci].set_ylabel("平均 NEES（姿勢 3 自由度）")
        axes[1, ci].set_xlabel("時間 [s]")
        axes[1, ci].grid(True, which="both", alpha=0.3)
    axes[0, 0].legend()
    axes[1, 0].legend()
    fig.suptitle(f"GNSS 位置のみ（シングルアンテナ）での収束比較（Monte Carlo {args.runs} 回）")
    fig.tight_layout()
    path = os.path.join(args.out, "invariant_ekf_vs_esekf.png")
    fig.savefig(path, dpi=110)
    print("saved", path)
    print("| 初期 yaw σ | 手法 | yaw RMS @5s [deg] | yaw RMS @20s [deg] | 位置 RMS @20s [m] | 平均 NEES（5 s 以降） | 最大 NEES（5 s 以降） |")
    print("|---|---|---|---|---|---|---|")
    for (y0, name), v in summary.items():
        print(f"| {y0}° | {name} | {v['yaw_rms_5s']:.2f} | {v['yaw_rms_20s']:.2f} | {v['pos_rms_20s']:.3f} | {v['anees_mean']:.1f} | {v['anees_max']:.1f} |")


if __name__ == "__main__":
    main()
