"""回転・姿勢・時系列の計算（numpy だけ）。クォータニオンの並びは x, y, z, w。"""
import numpy as np


def quat_to_rot(q):
    q = np.asarray(q, dtype=float)
    single = q.ndim == 1
    q = np.atleast_2d(q)
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    x, y, z, w = q.T
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z)
    R[:, 0, 1] = 2 * (x * y - z * w)
    R[:, 0, 2] = 2 * (x * z + y * w)
    R[:, 1, 0] = 2 * (x * y + z * w)
    R[:, 1, 1] = 1 - 2 * (x * x + z * z)
    R[:, 1, 2] = 2 * (y * z - x * w)
    R[:, 2, 0] = 2 * (x * z - y * w)
    R[:, 2, 1] = 2 * (y * z + x * w)
    R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R[0] if single else R


def rot_to_quat(R):
    R = np.asarray(R, dtype=float)
    tr = np.trace(R)
    if tr > 0:
        s = 2.0 * np.sqrt(tr + 1.0)
        q = [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * np.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k])
        q = [0.0, 0.0, 0.0, 0.0]
        q[i] = 0.25 * s
        q[j] = (R[j, i] + R[i, j]) / s
        q[k] = (R[k, i] + R[i, k]) / s
        q[3] = (R[k, j] - R[j, k]) / s
    q = np.array(q)
    return q / np.linalg.norm(q)


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rpy_to_rot(roll, pitch, yaw):
    """R = Rz(yaw) Ry(pitch) Rx(roll)（gll_ros2 の rpyToMatrix と同じ）。"""
    cr, sr, cp, sp = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rot_z(yaw) @ Ry @ Rx


def rpy_of(R):
    return (float(np.arctan2(R[2, 1], R[2, 2])), float(np.arcsin(np.clip(-R[2, 0], -1, 1))),
            float(np.arctan2(R[1, 0], R[0, 0])))


def yaw_of(R):
    R = np.asarray(R)
    return np.arctan2(R[..., 1, 0], R[..., 0, 0])


def wrap(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


def se3(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def rot_log(R):
    R = np.asarray(R, dtype=float)
    c = np.clip((np.trace(R, axis1=1, axis2=2) - 1) / 2, -1.0, 1.0)
    ang = np.arccos(c)
    v = np.stack([R[:, 2, 1] - R[:, 1, 2], R[:, 0, 2] - R[:, 2, 0], R[:, 1, 0] - R[:, 0, 1]], axis=1)
    s = np.sin(ang)
    k = np.where(s > 1e-9, ang / (2 * np.where(s > 1e-9, s, 1.0)), 0.5)
    return v * k[:, None]


def kabsch_origin(src, dst):
    """dst ≈ R src（並進なし）。"""
    H = np.asarray(src).T @ np.asarray(dst)
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    return Vt.T @ D @ U.T, None


def interp(t_src, x_src, t_dst):
    """列ごとの線形補間。範囲外は NaN。"""
    x_src = np.asarray(x_src, dtype=float)
    one_d = x_src.ndim == 1
    x2 = x_src[:, None] if one_d else x_src
    out = np.empty((len(t_dst), x2.shape[1]))
    for c in range(x2.shape[1]):
        out[:, c] = np.interp(t_dst, t_src, x2[:, c], left=np.nan, right=np.nan)
    return out[:, 0] if one_d else out


def quat_interp(t, q, tq):
    """クォータニオンの列を時刻 tq に補間する（正規化線形補間。範囲外は端の値）。"""
    t = np.asarray(t, dtype=float)
    q = np.asarray(q, dtype=float).copy()
    flip = np.cumsum(np.concatenate([[0], (np.sum(q[1:] * q[:-1], axis=1) < 0).astype(int)])) % 2 == 1
    q[flip] *= -1
    tq = np.clip(np.asarray(tq, dtype=float), t[0], t[-1])
    j = np.clip(np.searchsorted(t, tq), 1, len(t) - 1)
    i = j - 1
    w = ((tq - t[i]) / np.maximum(t[j] - t[i], 1e-12))[:, None]
    out = (1 - w) * q[i] + w * q[j]
    return out / np.linalg.norm(out, axis=1, keepdims=True)


def subsample(t, period):
    t = np.asarray(t)
    if len(t) == 0:
        return np.array([], dtype=int)
    grid = np.arange(t[0], t[-1] + 1e-9, period)
    return np.unique(np.clip(np.searchsorted(t, grid), 0, len(t) - 1))
