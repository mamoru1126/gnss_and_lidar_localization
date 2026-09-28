"""MCAP の読み出しと、CDR（リトルエンディアン）の読み出し（ROS なし・外部ライブラリなし）。

ros2 bag record -s mcap で録った bag（ROS 2 Humble）を読むためのもの。チャンクの圧縮は none と zstd（zstandard が要る）
と lz4（lz4 が要る）に対応する。書き出しは tools/tile_demo/make_bag.py の McapWriter を使う。
"""
import struct
from pathlib import Path

MAGIC = b"\x89MCAP0\r\n"


def _decompress(comp, data, size):
    if comp == "":
        return data
    if comp == "zstd":
        import zstandard
        return zstandard.ZstdDecompressor().decompress(data, max_output_size=size)
    if comp == "lz4":
        import lz4.frame
        return lz4.frame.decompress(data)
    raise ValueError(f"unsupported MCAP chunk compression: {comp}")


def _records(buf, pos, end):
    while pos + 9 <= end:
        op, ln = struct.unpack_from("<BQ", buf, pos)
        yield op, buf[pos + 9: pos + 9 + ln]
        pos += 9 + ln


def _str(body, off):
    n = struct.unpack_from("<I", body, off)[0]
    return body[off + 4: off + 4 + n].decode(), off + 4 + n


def resolve(path):
    """bag のディレクトリ（metadata.yaml と .mcap）か .mcap のファイル → .mcap のファイルの並び。"""
    p = Path(path)
    if p.is_dir():
        files = sorted(p.glob("*.mcap"))
        if not files:
            raise FileNotFoundError(f"{p}: .mcap が無い（ros2 bag record -s mcap で録る）")
        return files
    return [p]


def read_messages(path, topics=None):
    """(topic, 型, log_time [ns], data) を順に返す（ファイルの中の順。時刻順とは限らない）。"""
    for f in resolve(path):
        b = f.read_bytes()
        if b[:8] != MAGIC:
            raise ValueError(f"{f}: not an MCAP file")
        schemas, channels = {}, {}

        def handle(op, body):
            if op == 0x03:
                sid = struct.unpack_from("<H", body, 0)[0]
                name, _ = _str(body, 2)
                schemas[sid] = name
            elif op == 0x04:
                cid, sid = struct.unpack_from("<HH", body, 0)
                topic, _ = _str(body, 4)
                channels[cid] = (topic, sid)
            elif op == 0x05:
                cid, _seq, t, _pt = struct.unpack_from("<HIQQ", body, 0)
                topic, sid = channels[cid]
                if topics is None or topic in topics:
                    return topic, schemas.get(sid, ""), t, body[22:]
            return None

        for op, body in _records(b, 8, len(b) - 8):
            if op == 0x06:
                _s, _e, usize, _crc = struct.unpack_from("<QQQI", body, 0)
                comp, off = _str(body, 28)
                clen = struct.unpack_from("<Q", body, off)[0]
                data = _decompress(comp, body[off + 8: off + 8 + clen], usize)
                for op2, body2 in _records(data, 0, len(data)):
                    r = handle(op2, body2)
                    if r:
                        yield r
            elif op == 0x0F:
                break
            else:
                r = handle(op, body)
                if r:
                    yield r


class Cdr:
    """CDR（リトルエンディアン）の読み出し。"""

    def __init__(self, data):
        if data[:2] not in (b"\x00\x01", b"\x00\x03"):
            raise ValueError("only little-endian CDR is supported")
        self.b, self.p = data, 4

    def _al(self, n):
        self.p += (-(self.p - 4)) % n

    def _un(self, fmt, size):
        self._al(size)
        v = struct.unpack_from("<" + fmt, self.b, self.p)
        self.p += struct.calcsize("<" + fmt)
        return v

    def u8(self):
        v = self.b[self.p]
        self.p += 1
        return v

    def i32(self):
        return self._un("i", 4)[0]

    def u32(self):
        return self._un("I", 4)[0]

    def f32(self, n=1):
        v = self._un(f"{n}f", 4)
        return v if n > 1 else v[0]

    def f64(self, n=1):
        v = self._un(f"{n}d", 8)
        return v if n > 1 else v[0]

    def string(self):
        n = self.u32()
        s = self.b[self.p: self.p + n - 1].decode()
        self.p += n
        return s

    def header(self):
        sec, nsec = self.i32(), self.u32()
        return sec + nsec * 1e-9, self.string()

    def time(self):
        sec, nsec = self.i32(), self.u32()
        return sec + nsec * 1e-9


# ------------------------------------------------------------------ 使う型の読み出し
def pose_stamped(data):
    """geometry_msgs/PoseStamped → (t, frame, (x, y, z), (qx, qy, qz, qw))"""
    c = Cdr(data)
    t, frame = c.header()
    return t, frame, c.f64(3), c.f64(4)


def imu(data):
    """sensor_msgs/Imu → (t, frame, gyro, acc)"""
    c = Cdr(data)
    t, frame = c.header()
    c.f64(4)
    c.f64(9)
    gyro = c.f64(3)
    c.f64(9)
    acc = c.f64(3)
    return t, frame, gyro, acc


def velocity_report(data):
    """autoware_auto_vehicle_msgs/VelocityReport（autoware_vehicle_msgs も同じ並び）
    → (t, frame, longitudinal_velocity, lateral_velocity, heading_rate)"""
    c = Cdr(data)
    t, frame = c.header()
    return (t, frame) + c.f32(3)


def cloud_header(data):
    """sensor_msgs/PointCloud2 のヘッダと列の名前 → (t, frame, [列名], 点の数)"""
    c = Cdr(data)
    t, frame = c.header()
    h, w = c.u32(), c.u32()
    names = []
    for _ in range(c.u32()):
        names.append(c.string())
        c.u32()
        c.u8()
        c.u32()
    return t, frame, names, h * w


def cloud_xyz(data):
    """sensor_msgs/PointCloud2 → (t, frame, xyz（N×3、float32 の x, y, z の列から）)"""
    import numpy as np
    c = Cdr(data)
    t, frame = c.header()
    h, w = c.u32(), c.u32()
    fields = {}
    for _ in range(c.u32()):
        name = c.string()
        off = c.u32()
        dt = c.u8()
        c.u32()
        fields[name] = (off, dt)
    big = c.u8()
    step = c.u32()
    c.u32()
    n = c.u32()
    raw = c.b[c.p: c.p + n]
    if big or any(fields[k][1] != 7 for k in "xyz"):
        raise ValueError("expects little-endian float32 x, y, z")
    arr = np.frombuffer(raw, dtype=np.uint8).reshape(h * w, step)
    xyz = np.stack([arr[:, fields[k][0]: fields[k][0] + 4].copy().view("<f4")[:, 0] for k in "xyz"], axis=1)
    return t, frame, xyz
