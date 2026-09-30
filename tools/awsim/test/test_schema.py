"""MCAP に書くスキーマ（ros2msg）が、依存する型をすべて「MSG: パッケージ/型」の区切りで持つかの単体テスト。
Foxglove は区切りの名前に /msg/ が入っていると型を見つけられない（rosbag2 と同じ形にする）。"""
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE.parents[1] / "tile_demo"))
import awsim_to_bag  # noqa: E402,F401  （型の定義を足す）
import make_bag as MB  # noqa: E402

PRIM = {"bool", "byte", "char", "int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64", "float32",
        "float64", "string", "wstring", "time", "duration"}


def check(full, top):
    """定義の中で使われている型が、すべて区切りの型として定義されているか。見つからない型の並びを返す。"""
    parts = full.split("\n" + "=" * 80 + "\n")
    names = {top.replace("/msg/", "/"): parts[0]}
    for p in parts[1:]:
        head, _, body = p.partition("\n")
        assert head.startswith("MSG: "), head
        names[head[5:]] = body
    missing = []
    for name, body in names.items():
        pkg = name.split("/")[0]
        for line in body.splitlines():
            line = line.split("#")[0].strip()
            if not line or "=" in line:  # 定数
                continue
            typ = line.split()[0].split("[")[0].split("<")[0]
            if typ in PRIM:
                continue
            full_name = typ if "/" in typ else f"{pkg}/{typ}"
            if full_name not in names:
                missing.append((name, full_name))
    return missing


class TestSchema(unittest.TestCase):
    def test_all(self):
        for t in MB.DEPS:
            full = MB.full_definition(t)
            self.assertNotIn("MSG: " + t.split("/")[0] + "/msg/", full)
            self.assertEqual(check(full, t), [], t)


if __name__ == "__main__":
    unittest.main()
