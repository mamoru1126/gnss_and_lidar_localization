#!/usr/bin/env python3
"""tiled_pcd_map_demo の記録とタイルから、デモページ（1 つの HTML）を作る（README.md 参照）。

  python3 make_page.py <タイルのディレクトリ> <frames.json> <出力.html>
      [--title 名前] [--heading 見出し] [--note 条件の説明（HTML）]
      [--display-res 0.2] [--follow-view 240]

表示用に、点を display-res [m] の格子に間引き（格子ごとに最も高い点）、タイルごとの地面からの高さを明るさにして埋め込む。
"""
import argparse
import base64
import json
import math
import pathlib
import struct

import numpy as np
import yaml

DEFAULT_NOTE = (
    "タイル 20 m、load 60 m / unload 90 m、先読み 3 s（既定値）。1.5 m/s で南の通路から広場を一周して戻る 200 s の経路。"
    "<code>MapTileManager</code> を同期モードで 0.1 s ごとに更新し、0.5 s ごとに記録した。<br>\n"
    '        点群: <a href="https://github.com/koide3/hdl_localization" target="_blank" rel="noopener">koide3/hdl_localization</a> の '
    "<code>data/map.pcd</code>（BSD-2-Clause）を <code>tiled_pcd_map_tiler</code> でタイル化（表示は 0.2 m に間引き）。"
)


def read_tile(path):
    b = pathlib.Path(path).read_bytes()
    if b[:8] not in (b"TPCMTIL1", b"GLLTILE1"):  # GLLTILE1 は名前を変える前の形式
        raise ValueError(f"{path}: not a gll tile file")
    n, _flags = struct.unpack("<QI", b[8:20])
    return np.frombuffer(b[20:20 + 12 * n], dtype=np.float32).reshape(-1, 3)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("tile_dir", type=pathlib.Path)
    ap.add_argument("frames", type=pathlib.Path)
    ap.add_argument("out", type=pathlib.Path)
    ap.add_argument("--title", default="gll タイル読み込みデモ")
    ap.add_argument("--heading", default="タイル読み込みデモ")
    ap.add_argument("--note", default=DEFAULT_NOTE, help="「条件」の欄に入れる HTML")
    ap.add_argument("--display-res", type=float, default=0.2, help="表示用の間引き [m]")
    ap.add_argument("--follow-view", type=float, default=240.0, help="「自己位置を追う」ときに見せる幅 [m]")
    a = ap.parse_args()

    index = yaml.safe_load((a.tile_dir / "tile_index.yaml").read_text())
    parts = []
    for t in index["tiles"]:
        p = read_tile(a.tile_dir / t["file"]).astype(np.float64)
        if len(p) == 0:
            continue
        ground = np.percentile(p[:, 2], 5)  # タイルの中の地面の高さの目安
        parts.append(np.c_[p, p[:, 2] - ground])
    p = np.concatenate(parts)

    res = a.display_res
    cell = np.floor(p[:, :2] / res).astype(np.int64)
    key = (cell[:, 0] - cell[:, 0].min()) * 10_000_000 + (cell[:, 1] - cell[:, 1].min())
    order = np.lexsort((p[:, 3], key))
    last = np.r_[key[order][1:] != key[order][:-1], True]
    q = p[order[last]]
    h = q[:, 3]
    bright = np.where(h < 0.3, 70, np.clip(130 + (h - 0.3) * 25, 130, 255)).astype(np.uint8)  # 地面は暗く

    # int16 に収めるため、原点をずらして量子化する
    origin = np.floor(q[:, :2].min(axis=0) / 100.0) * 100.0
    extent = float((q[:, :2].max(axis=0) - origin).max())
    quant = max(0.05, math.ceil(extent / 32000 * 100) / 100)
    buf = np.empty(len(q), dtype=[("x", "<i2"), ("y", "<i2"), ("b", "u1")])
    buf["x"] = np.round((q[:, 0] - origin[0]) / quant)
    buf["y"] = np.round((q[:, 1] - origin[1]) / quant)
    buf["b"] = bright
    data = {"points": base64.b64encode(buf.tobytes()).decode(), "n": int(len(q)),
            "origin": [float(origin[0]), float(origin[1])], "quant": quant, "follow_view": a.follow_view,
            **json.loads(a.frames.read_text())}

    tpl = (pathlib.Path(__file__).parent / "template.html").read_text(encoding="utf-8")
    # ブラウザで直接開けるように、文書の宣言と文字コードを付ける
    head = ('<!doctype html>\n<html lang="ja">\n<meta charset="utf-8">\n'
            '<meta name="viewport" content="width=device-width, initial-scale=1">\n')
    page = (tpl.replace("__TITLE__", a.title).replace("__HEADING__", a.heading).replace("__NOTE__", a.note)
            .replace("__DEMO_DATA__", json.dumps(data, separators=(",", ":"))))
    a.out.write_text(head + page, encoding="utf-8")
    print(f"{len(p)} points -> {len(q)} display points ({res} m), {len(data['frames'])} frames, "
          f"{a.out.stat().st_size / 1e6:.1f} MB -> {a.out}")


if __name__ == "__main__":
    main()
