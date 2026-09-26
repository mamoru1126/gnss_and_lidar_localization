#!/usr/bin/env python3
"""gll_tile_demo の記録とタイルから、デモページ（1 つの HTML）を作る（README.md 参照）。

  python3 make_page.py <タイルのディレクトリ> <frames.json> <出力.html>

表示用に、点を 0.2 m の格子に間引き（格子ごとに最も高い点）、高さを明るさにして埋め込む。
"""
import base64
import json
import pathlib
import struct
import sys

import numpy as np
import yaml


def read_tile(path):
    b = pathlib.Path(path).read_bytes()
    if b[:8] != b"GLLTILE1":
        raise ValueError(f"{path}: not a gll tile file")
    n, _flags = struct.unpack("<QI", b[8:20])
    return np.frombuffer(b[20:20 + 12 * n], dtype=np.float32).reshape(-1, 3)


def main():
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    tile_dir, frames_path, out_path = map(pathlib.Path, sys.argv[1:])
    index = yaml.safe_load((tile_dir / "tile_index.yaml").read_text())
    p = np.concatenate([read_tile(tile_dir / t["file"]) for t in index["tiles"]])

    res = 0.2
    cell = np.floor(p[:, :2] / res).astype(np.int64)
    key = cell[:, 0] * 1_000_000 + cell[:, 1]
    order = np.lexsort((p[:, 2], key))
    last = np.r_[key[order][1:] != key[order][:-1], True]
    q = p[order[last]]
    z = q[:, 2]
    bright = np.where(z < 0.3, 70, np.clip(130 + (z - 0.3) * 25, 130, 255)).astype(np.uint8)  # 地面は暗く

    buf = np.empty(len(q), dtype=[("x", "<i2"), ("y", "<i2"), ("b", "u1")])
    buf["x"] = np.round(q[:, 0] / 0.05)
    buf["y"] = np.round(q[:, 1] / 0.05)
    buf["b"] = bright
    data = {"points": base64.b64encode(buf.tobytes()).decode(), "n": int(len(q)),
            **json.loads(frames_path.read_text())}

    tpl = (pathlib.Path(__file__).parent / "template.html").read_text(encoding="utf-8")
    # ブラウザで直接開けるように、文書の宣言と文字コードを付ける
    head = ('<!doctype html>\n<html lang="ja">\n<meta charset="utf-8">\n'
            '<meta name="viewport" content="width=device-width, initial-scale=1">\n')
    page = head + tpl.replace("__DEMO_DATA__", json.dumps(data, separators=(",", ":")))
    out_path.write_text(page, encoding="utf-8")
    print(f"{len(p)} points -> {len(q)} display points, {len(data['frames'])} frames -> {out_path}")


if __name__ == "__main__":
    main()
