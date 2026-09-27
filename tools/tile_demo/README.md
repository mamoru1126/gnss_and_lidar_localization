# タイル読み込みのデモ

`MapTileManager`（設計書 5.3〜5.5 節）が、自己位置の移動に合わせて点群地図のタイルを読み込み・破棄する様子を、上から見たアニメーションで確かめるためのツール。生成したページは `docs/demo/` にある（地図の出典とライセンスは [docs/demo/README.md](../../docs/demo/README.md)）。小さい地図のページは [docs/demo/tile_loading.html](../../docs/demo/tile_loading.html) で、GitHub Pages で [https://sunomamo1126.github.io/gnss_and_lidar_localization/demo/tile_loading.html](https://sunomamo1126.github.io/gnss_and_lidar_localization/demo/tile_loading.html) として公開している（main の `docs/` を公開）。

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

`gll_tile_demo` は `gll_map_tiler` と同じく、[gll_map](../../map/README.md) を `GLL_MAP_BUILD_TOOLS=ON`（既定）でビルドするとできる（ROS も small_gicp も要らない）。

### 西新宿の地図（docs/demo/nishishinjuku.html）

AWSIM の西新宿の地図（CC BY-NC 4.0）を使う。経路は、地図に付いている lanelet の中心線から `lanelet_route.py` で作った（`route_nishishinjuku.txt`）。

```bash
curl -L -o /tmp/nishishinjuku.zip https://github.com/tier4/AWSIM/releases/download/v1.1.0/nishishinjuku_autoware_map.zip
unzip -q /tmp/nishishinjuku.zip -d /tmp/nishishinjuku
M=/tmp/nishishinjuku/nishishinjuku_autoware_map

gll_map_tiler -i $M/pointcloud_map.pcd -o /tmp/nsj_tiles --tile-size 20 --voxel-size 0.2   # 936 枚

# 経路を作り直す場合（numpy と scipy が必要）
python3 tools/tile_demo/lanelet_route.py $M/lanelet2_map.osm \
    '[[81190,49815],[81260,50120],[81330,50210],[81470,50560],[81700,50640],[81840,50120],[81620,50050],[81560,50400],[81470,50630],[81430,50690]]' \
    > /tmp/route.txt

gll_tile_demo /tmp/nsj_tiles/tile_index.yaml tools/tile_demo/route_nishishinjuku.txt /tmp/nsj_frames.json
python3 tools/tile_demo/make_page.py /tmp/nsj_tiles /tmp/nsj_frames.json docs/demo/nishishinjuku.html \
    --display-res 0.5 --title "西新宿タイル読み込みデモ" --heading "西新宿のタイル読み込み" --note "（条件と出典の HTML）"
```

`make_page.py` の `--note` には、ページの「条件」の欄に出す説明（出典とライセンスを含める）を渡す。

## ファイル

| ファイル | 内容 |
|---|---|
| `map/tools/tile_demo_main.cpp` | `gll_tile_demo`（gll_map のツール）。経路（折れ線）に沿って一定の速さ（既定 1.5 m/s）で `MapTileManager::update` を 0.1 s ごとに呼び、自己位置・領域（MapRegion）のタイル・読み込み回数を JSON に書く |
| `route_nishishinjuku.txt` / `lanelet_route.py` | 西新宿の地図用の経路（約 2.9 km。CC BY-NC 4.0）と、lanelet の中心線から経路を作るスクリプト |
| `route_hdl_localization.txt` | hdl_localization の地図用の経路（南の通路から広場を一周して戻る、約 200 s）。1 行に `x y`（地図座標 [m]） |
| `make_page.py` | タイルの点を間引き（`--display-res`、既定 0.2 m）、記録と一緒に `template.html` に埋め込んで 1 つの HTML にする |
| `template.html` | ページの本体（Canvas で描画。再生・一時停止・シーク・速度・自己位置の追従） |

ほかの地図で作るときは、その地図に合わせた経路のファイルを用意し、`make_page.py` の `--title` / `--heading` / `--note` でページの名前と説明を与える。地図の長い方の軸が横になるように、ページが向きを自動で決める。
