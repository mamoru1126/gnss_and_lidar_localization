# tiled_pcd_map: 点群地図のタイル化と動的ロード

大きな点群地図をタイルに分けておき、自己位置の周りのタイルだけを読み込むための C++17 ライブラリ。ROS には依存しない。このリポジトリの自己位置推定（`gll_core`）はこれを使っているが、`tiled_pcd_map` は単体でも使える。たとえば、別の自己位置推定（NDT など）に部分地図を渡したり、地図を扱うツールを作ったりするのに使える。

仕組みの図解は [点群地図の部分ロード](https://sunomamo1126.github.io/gnss_and_lidar_localization/tiled_pcd_map/)（ソース: [docs/tiled_pcd_map/index.html](../docs/tiled_pcd_map/index.html)）。動きはデモページで見られる: [小さい地図](https://sunomamo1126.github.io/gnss_and_lidar_localization/demo/tile_loading.html) / [西新宿の約 1 km 四方の地図](https://sunomamo1126.github.io/gnss_and_lidar_localization/demo/nishishinjuku.html)

## できること

| 機能 | 内容 |
|---|---|
| タイル化 | 統合済みの点群（PCD）を間引き、正方形のタイル（既定 20 m）に分けてファイルに書く（`tiled_pcd_map_tiler`）。GICP 用の点ごとの共分散も付けられる（small_gicp があるとき） |
| 動的ロード | 自己位置を与えるたびに、周り（と進行方向の先読み位置の周り）のタイルを別スレッドで読み込み、遠くなったタイルを捨てる（`MapTileManager`） |
| 領域 | 読み込み中のタイルをまとめた「領域」を作って差し替える（ダブルバッファ）。使う側は `shared_ptr` で受け取るので、差し替えの途中でも安全。領域の中身（KdTree など）は使う側が決められる |
| 複数の地図 | 座標系の違う複数の地図（地図グループ）を、アンカー（地図の 1 点の緯度経度と方位）で UTM にそろえて扱う。どのグループを使うかは現在位置で決める |
| 入出力 | PCD（ascii / binary / binary_compressed）の読み書き、タイルファイル、`tile_index.yaml`、`maps.yaml` |

RAM に載るのは読み込み中のタイルの点だけで、地図全体の点は読まない（地図全体について持つのは、各タイルの範囲と点数の索引だけ）。メモリの使用量は、地図の大きさではなく `load_radius` の範囲の点数で決まる。

## ビルド

必要なもの: C++17、CMake 3.16 以上、Eigen3、GeographicLib、yaml-cpp（テストには GoogleTest）。small_gicp v1.0.1 と OpenMP は任意。

```bash
sudo apt install libeigen3-dev libgeographiclib-dev libyaml-cpp-dev libgtest-dev
cmake -S map -B build_map -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$HOME/.local
cmake --build build_map -j && ctest --test-dir build_map
cmake --install build_map
```

colcon のワークスペースに置けば、`build_type=cmake` のパッケージとしてビルドされる。

| CMake のオプション | 既定 | 内容 |
|---|---|---|
| `TILED_PCD_MAP_USE_SMALL_GICP` | ON | small_gicp と OpenMP が見つかれば、タイル化のときに点ごとの共分散を計算する。見つからなければ、共分散なしでビルドする（CMake のログに `per-point covariances with small_gicp: OFF` と出る） |
| `TILED_PCD_MAP_BUILD_TOOLS` | ON | `tiled_pcd_map_tiler` と `tiled_pcd_map_demo` |
| `TILED_PCD_MAP_BUILD_TESTS` | ON | 単体テスト |

使う側の CMake:

```cmake
find_package(tiled_pcd_map REQUIRED)
target_link_libraries(my_app PRIVATE tiled_pcd_map::tiled_pcd_map)
```

## 使い方

### 1. 地図をタイルにする

```bash
tiled_pcd_map_tiler -i map.pcd -o tiles/ --tile-size 20 --voxel-size 0.2
# → tiles/tiles/<ix>_<iy>.bin と tiles/tile_index.yaml
```

| オプション | 既定 | 内容 |
|---|---|---|
| `-i` | （必須。複数可） | 入力の PCD |
| `-o` | （必須） | 出力先 |
| `--tile-size` | 20 | タイルの一辺 [m] |
| `--voxel-size` | 0.2 | 間引き [m]（ボクセルごとに点を平均する。0 なら間引かない） |
| `--num-neighbors` | 20 | 点ごとの共分散に使う近傍点数 |
| `--threads` | 4 | 共分散の計算の並列数 |
| `--no-covariance` | | 共分散を計算しない（NDT など、共分散の要らない使い方） |

共分散は、タイルに分ける前の点群全体で計算する（タイルの境界で近傍点が欠けないように）。

### 2. 自己位置の周りを読み込む

```cpp
#include <tiled_pcd_map/map_tile_manager.hpp>

namespace tpm = tiled_pcd_map;

// 地図グループ。アンカーを恒等変換にすると、位置（UTM）= 地図座標として扱える
tpm::MapGroup group;
group.id = "site";
group.anchor = tpm::MapAnchor::identity();
group.index = tpm::TileIndex::load("tiles/tile_index.yaml", group.id);

tpm::MapManagerConfig cfg;  // 既定: load 60 m / unload 90 m / 先読み 3 s、別スレッドで読み込む
tpm::MapTileManager maps(cfg, {group}, std::make_shared<tpm::BinaryTileLoader>());

// 自己位置が来るたびに呼ぶ（時刻 [s]、位置、進行方向 [rad]、速さ [m/s]）。見直しが要らなければすぐ戻る
maps.update(t, tpm::Vec2(x, y), yaw, speed);

// 今の領域を受け取る（まだ無ければ nullptr）。既定の領域 TileSetRegion は、読み込んだタイルの点をそのまま持つ
if (auto region = maps.currentRegionAs<tpm::TileSetRegion>()) {
  std::vector<tpm::Vec3f> points = region->points();     // 地図座標
  std::vector<tpm::Vec3> points_utm = region->pointsUtm();  // アンカーで UTM にしたもの
}

// 領域が差し替わるたびに呼ぶ関数（読み込みのスレッドから呼ばれるので、重い処理はしない）
maps.setRegionCallback([](std::shared_ptr<const tpm::MapRegion> r) { /* 別スレッドに知らせるなど */ });
```

動かせる例は [examples/follow_path](examples/follow_path/main.cpp)（インストールした `tiled_pcd_map` を `find_package` で使う）。

### 3. 領域の中身を自分で作る

領域を作る関数（`RegionBuilder`）を渡すと、読み込んだタイルがそろうたびに、それで領域を作る。KdTree やボクセル地図など、使う側の検索構造を作っておくのに使う。作るのは読み込みのスレッドなので、使う側のスレッドは止まらない。

```cpp
struct NdtMap : tpm::MapRegion {
  // ... 使う側の検索構造
};

tpm::RegionBuilder builder = [](const std::string& group, const tpm::MapAnchor& anchor,
                                const std::vector<std::shared_ptr<const tpm::TileData>>& tiles) {
  auto r = std::make_shared<NdtMap>();
  r->group = group;
  r->anchor = anchor;
  for (const auto& t : tiles) {
    r->tiles.push_back(t->id);
    r->num_points += t->points.size();
    // t->points（地図座標）、t->covs（点ごとの共分散の上三角。無ければ空）から検索構造を作る
  }
  return r;
};
tpm::MapTileManager maps(cfg, {group}, std::make_shared<tpm::BinaryTileLoader>(), builder);
auto region = maps.currentRegionAs<NdtMap>();
```

`gll_core` では、`GicpMatcher::buildTarget`（GICP のターゲットと KdTree を作る）をこの関数として渡している（`gll::targetBuilder`）。

### 4. 複数の地図と maps.yaml

座標系の違う複数の地図は、`maps.yaml` にアンカーと一緒に書いて読み込む。位置は UTM で与える。

```cpp
#include <tiled_pcd_map/map_config.hpp>

const tpm::MapSetConfig set = tpm::loadMapSetConfig("maps.yaml");
const std::vector<tpm::MapGroup> groups = tpm::loadMapGroups(set, tpm::UtmProjector(54, true));
tpm::MapTileManager maps(cfg, groups, std::make_shared<tpm::BinaryTileLoader>());
maps.update(t, tpm::Vec2(easting, northing), yaw, speed);
std::string active = maps.activeGroup();  // いま領域を作っているグループ
```

`maps.yaml` の書き方（緯度経度・UTM で直接・`anchor: local`）は、リポジトリの [README](../README.md) と[設計書](../docs/design.md) 4.2 節・5.2 節を参照。

## 動きの決まり

| 項目 | 内容 |
|---|---|
| 見直しの間隔 | `update` は、前回から `update_distance`（既定 1 m）以上動いたか `update_interval`（既定 1 s）以上たったときだけ見直す（`force = true` なら毎回） |
| 読み込むタイル | 現在位置と先読み位置（速さ × `lookahead_time` だけ進行方向に進んだ位置）のどちらかから `load_radius` 以内のタイル。全グループが対象で、次に入るグループのタイルも前もって読む |
| 捨てるタイル | 現在位置から `unload_radius` より遠いタイル（ロードとアンロードを繰り返さないよう、`load_radius` より大きくする） |
| アクティブグループ | 現在位置から `load_radius` 以内のグループのうち最も近いもの。今のグループより `group_switch_margin`（既定 10 m）以上近いグループが現れたときだけ切り替える |
| 領域 | アクティブグループの読み込み済みタイルだけで作る（別のグループの点は混ぜない）。点が `min_target_points`（既定 500）未満なら作らない |
| スレッド | `async = true`（既定）なら読み込みと領域の作成は専用のスレッドで行い、`update` はすぐ戻る。`false` なら `update` の中で行う（テスト向き）。`waitIdle()` で読み込みの完了を待てる |
| 状態 | `stats()` で、アクティブグループ、読み込んだタイル数（グループごと）、領域のタイル数と点数、読み込みの回数と失敗の回数、領域を作った回数が分かる |

## ファイル形式

- **タイルファイル**（リトルエンディアン）: 先頭 8 バイトが `TPCMTIL1`、続いて点数（uint64）、フラグ（uint32。ビット 0 が共分散あり）、点の xyz（float32 × 3 × 点数）、共分散の上三角 xx・xy・xz・yy・yz・zz（float32 × 6 × 点数。フラグが立っているときだけ）
- **tile_index.yaml**: `format: tiled_pcd_map_v1`、`tile_size`、各タイルの `ix`・`iy`・`file`・`num_points`・`bounds_min`・`bounds_max`
- 名前を変える前（`gll_map` の頃）の形式（`GLLTILE1`、`format: gll_tiles_v1`）も読み込める（中身は同じ）

独自の形式で読みたい場合は `ITileLoader` を実装して渡す。

## 中身

| ヘッダ | 内容 |
|---|---|
| `tiled_pcd_map/map_tile_manager.hpp` | `MapTileManager`（動的ロード） |
| `tiled_pcd_map/map_region.hpp` | `MapRegion`、`TileSetRegion`、`RegionBuilder` |
| `tiled_pcd_map/map_manager_config.hpp` | `MapManagerConfig` |
| `tiled_pcd_map/tile.hpp` | `TileId`、`TileMeta`、`TileData`、`TileIndex`、タイルファイルの読み書き、`ITileLoader`、`BinaryTileLoader` |
| `tiled_pcd_map/map_tiler.hpp` | `tileMap`、`writeTiles`、`voxelDownsample` |
| `tiled_pcd_map/pcd_io.hpp` | `readPcd`、`writePcd` |
| `tiled_pcd_map/map_anchor.hpp` | `MapAnchor`（地図座標 ⇔ UTM） |
| `tiled_pcd_map/map_config.hpp` | `loadMapSetConfig`、`loadMapGroups`（maps.yaml） |
| `tiled_pcd_map/math.hpp`、`se2.hpp`、`geodesy.hpp`、`logger.hpp` | 基本の型と角度・`Pose2D`、`SE2`、`UtmProjector`（緯度経度 ⇔ UTM）、`ILogger` |

ヘッダはすべて `tiled_pcd_map/` の下にあり、名前空間は `tiled_pcd_map`（gll には依存しない）。
