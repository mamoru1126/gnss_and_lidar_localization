# タイル読み込みのデモ

`MapTileManager`（設計書 5.3〜5.5 節）が、自己位置の移動に合わせて点群地図のタイルを読み込み・破棄する様子を、上から見たアニメーションで確かめるためのツール。生成したページは [docs/demo/tile_loading.html](../../docs/demo/tile_loading.html) で、GitHub Pages で [https://sunomamo1126.github.io/gnss_and_lidar_localization/demo/tile_loading.html](https://sunomamo1126.github.io/gnss_and_lidar_localization/demo/tile_loading.html) として公開している（main の `docs/` を公開）。

- 背景は黒、地図の全体を白、照合のターゲットに入っている（読み込み中の）タイルを緑で描く。捨てたタイルは赤枠で一瞬示す。
- 動きは作り物ではなく、実際の `MapTileManager`（既定の設定: タイル 20 m、load 60 m / unload 90 m、先読み 3 s）を同期モードで経路に沿って動かした記録を再生している。

## 作り方

サンプルの地図は [koide3/hdl_localization](https://github.com/koide3/hdl_localization) の `data/map.pcd`（BSD-2-Clause、約 205 万点）。リポジトリには入れていないので、取ってくる。

```bash
# 地図を取ってくる（data/ だけ）
git clone --depth 1 --filter=blob:none --sparse https://github.com/koide3/hdl_localization /tmp/hdl_localization
git -C /tmp/hdl_localization sparse-checkout set data

# タイル化（82 枚）
gll_map_tiler -i /tmp/hdl_localization/data/map.pcd -o /tmp/hdl_tiles --tile-size 20 --voxel-size 0.2

# 経路に沿って MapTileManager を動かし、0.5 s ごとの状態を記録する
gll_tile_demo /tmp/hdl_tiles/tile_index.yaml tools/tile_demo/route_hdl_localization.txt /tmp/frames.json

# ページにする（numpy と PyYAML が必要）
python3 tools/tile_demo/make_page.py /tmp/hdl_tiles /tmp/frames.json docs/demo/tile_loading.html
```

`gll_tile_demo` は `gll_map_tiler` と同じく、コアを `GLL_BUILD_TOOLS=ON`（既定）でビルドするとできる。

## ファイル

| ファイル | 内容 |
|---|---|
| `core/tools/tile_demo_main.cpp` | `gll_tile_demo`。経路（折れ線）に沿って一定の速さ（既定 1.5 m/s）で `MapTileManager::update` を 0.1 s ごとに呼び、自己位置・ターゲットのタイル・読み込み回数を JSON に書く |
| `route_hdl_localization.txt` | hdl_localization の地図用の経路（南の通路から広場を一周して戻る、約 200 s）。1 行に `x y`（地図座標 [m]） |
| `make_page.py` | タイルの点を 0.2 m に間引き、記録と一緒に `template.html` に埋め込んで 1 つの HTML にする |
| `template.html` | ページの本体（Canvas で描画。再生・一時停止・シーク・速度・自己位置の追従） |

ほかの地図で作るときは、その地図に合わせた経路のファイルを用意する（ページの「条件」の説明は hdl_localization 用のままなので、必要なら `template.html` を書き換える）。
