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
tiled_pcd_map_tiler -i /tmp/hdl_localization/data/map.pcd -o /tmp/hdl_tiles --tile-size 20 --voxel-size 0.2

# 経路に沿って MapTileManager を動かし、0.5 s ごとの状態を記録する
tiled_pcd_map_demo /tmp/hdl_tiles/tile_index.yaml tools/tile_demo/route_hdl_localization.txt /tmp/frames.json

# ページにする（numpy と PyYAML が必要）
python3 tools/tile_demo/make_page.py /tmp/hdl_tiles /tmp/frames.json docs/demo/tile_loading.html
```

`tiled_pcd_map_demo` は `tiled_pcd_map_tiler` と同じく、[tiled_pcd_map](../../tiled_pcd_map/README.md) を `TILED_PCD_MAP_BUILD_TOOLS=ON`（既定）でビルドするとできる（ROS も small_gicp も要らない）。

### 西新宿の地図（docs/demo/nishishinjuku.html）

AWSIM の西新宿の地図（CC BY-NC 4.0）を使う。経路は、地図に付いている lanelet の中心線から `lanelet_route.py` で作った（`route_nishishinjuku.txt`）。

```bash
curl -L -o /tmp/nishishinjuku.zip https://github.com/tier4/AWSIM/releases/download/v1.1.0/nishishinjuku_autoware_map.zip
unzip -q /tmp/nishishinjuku.zip -d /tmp/nishishinjuku
M=/tmp/nishishinjuku/nishishinjuku_autoware_map

tiled_pcd_map_tiler -i $M/pointcloud_map.pcd -o /tmp/nsj_tiles --tile-size 20 --voxel-size 0.2   # 936 枚

# 経路を作り直す場合（numpy と scipy が必要）
python3 tools/tile_demo/lanelet_route.py $M/lanelet2_map.osm \
    '[[81190,49815],[81260,50120],[81330,50210],[81470,50560],[81700,50640],[81840,50120],[81620,50050],[81560,50400],[81470,50630],[81430,50690]]' \
    > /tmp/route.txt

tiled_pcd_map_demo /tmp/nsj_tiles/tile_index.yaml tools/tile_demo/route_nishishinjuku.txt /tmp/nsj_frames.json
python3 tools/tile_demo/make_page.py /tmp/nsj_tiles /tmp/nsj_frames.json docs/demo/nishishinjuku.html \
    --display-res 0.5 --title "西新宿タイル読み込みデモ" --heading "西新宿のタイル読み込み" --note "（条件と出典の HTML）"
```

`make_page.py` の `--note` には、ページの「条件」の欄に出す説明（出典とライセンスを含める）を渡す。

## rosbag にして RViz で再生する

同じ記録を ROS 2 の rosbag（MCAP 形式）にして、RViz で再生できる。地図の全体を灰〜白、読み込み中のタイルを緑（点と枠）、捨てたタイルを赤枠で一瞬示し、車の矢印と load / unload の円が自己位置について動く。bag を作るのに ROS は要らない（`make_bag.py` が MCAP と CDR を自分で書く。numpy と PyYAML が必要）。

```bash
# tf を 10 Hz で出すため、0.1 s ごとに記録する
tiled_pcd_map_demo /tmp/nsj_tiles/tile_index.yaml tools/tile_demo/route_nishishinjuku.txt /tmp/nsj_frames_10hz.json \
    --record-interval 0.1
python3 tools/tile_demo/make_bag.py /tmp/nsj_tiles /tmp/nsj_frames_10hz.json /tmp/nishishinjuku_tiles.mcap
# → 1,947 s（約 2.9 km を 1.5 m/s）、約 65 MB

# 再生（ROS 2 Jazzy）。RViz を先に起動しても後に起動してもよい
rviz2 -d tools/tile_demo/tile_demo.rviz
ros2 bag play /tmp/nishishinjuku_tiles.mcap --rate 10   # 10 倍速で約 3 分
```

| トピック | 型 | 中身 |
|---|---|---|
| `/tf` | `tf2_msgs/msg/TFMessage` | `map` → `base_link`（0.1 s ごと）。高さは足元のタイルの地面（点の z の 5% 点）をならしたもの |
| `/tile_demo/map` | `sensor_msgs/msg/PointCloud2` | 地図の全体（`--map-res`、既定 0.5 m に間引き）。`intensity` はタイルの地面からの高さ [m] |
| `/tile_demo/route` | `nav_msgs/msg/Path` | 走る経路 |
| `/tile_demo/vehicle` | `visualization_msgs/msg/MarkerArray` | 車の矢印と、load（60 m）・先読み位置の load・unload（90 m）の円。`base_link` に固定 |
| `/tile_demo/tiles` | `visualization_msgs/msg/MarkerArray` | 読み込み中のタイル（`--tile-res`、既定 0.5 m に間引いた点と枠）。読んだタイルを追加し、捨てたタイルを削除する差分だけを送る |

- 座標は地図の座標のまま（frame は `map`）。bag の時刻は `--start-time`（既定 2026-01-01 00:00 UTC）から始まる。
- `/tile_demo/map`・`route`・`vehicle` は最初に 1 回だけ出し、QoS を transient local にしてある（`ros2 bag play` がこの QoS で出し直すので、後から起動した RViz にも届く）。
- 読み込み中のタイルの点群を変わるたびにまるごと送ると数百 MB になるので、タイルごとのマーカーの追加・削除にしてある（RViz はマーカーを削除されるまで表示し続ける）。
- 西新宿の地図の点を含むので、bag も元の地図と同じく CC BY-NC 4.0（非営利に限る）。リポジトリには入れていない。
- 作った bag は、rosbag2（Jazzy）が使う MCAP の C++ リーダー（v1.3.1）で全メッセージを読めること、埋め込んだメッセージの定義（Jazzy の .msg）どおりに全メッセージを最後のバイトまで解読できること、タイルのマーカーを追加・削除の順に積み上げると記録と全時刻で一致することを確かめてある。ROS を入れた環境での再生はまだ試していない。

## ファイル

| ファイル | 内容 |
|---|---|
| `tiled_pcd_map/tools/tile_demo_main.cpp` | `tiled_pcd_map_demo`（tiled_pcd_map のツール）。経路（折れ線）に沿って一定の速さ（既定 1.5 m/s）で `MapTileManager::update` を 0.1 s ごとに呼び、自己位置・領域（MapRegion）のタイル・読み込み回数を `--record-interval`（既定 0.5 s）ごとに JSON に書く |
| `route_nishishinjuku.txt` / `lanelet_route.py` | 西新宿の地図用の経路（約 2.9 km。CC BY-NC 4.0）と、lanelet の中心線から経路を作るスクリプト |
| `route_hdl_localization.txt` | hdl_localization の地図用の経路（南の通路から広場を一周して戻る、約 200 s）。1 行に `x y`（地図座標 [m]） |
| `make_page.py` | タイルの点を間引き（`--display-res`、既定 0.2 m）、記録と一緒に `template.html` に埋め込んで 1 つの HTML にする |
| `make_bag.py` / `tile_demo.rviz` | 記録とタイルから RViz で再生できる rosbag（MCAP）を作るスクリプトと、その表示の設定 |
| `template.html` | ページの本体（Canvas で描画。再生・一時停止・シーク・速度・自己位置の追従） |

ほかの地図で作るときは、その地図に合わせた経路のファイルを用意し、`make_page.py` の `--title` / `--heading` / `--note` でページの名前と説明を与える。地図の長い方の軸が横になるように、ページが向きを自動で決める。
