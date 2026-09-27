# デモページ

| ページ | 地図 | ライセンス |
|---|---|---|
| [tile_loading.html](https://sunomamo1126.github.io/gnss_and_lidar_localization/demo/tile_loading.html) | [koide3/hdl_localization](https://github.com/koide3/hdl_localization) の `data/map.pcd`（約 200 m × 250 m） | 地図は BSD-2-Clause（Copyright (c) 2019, k.koide） |
| [nishishinjuku.html](https://sunomamo1126.github.io/gnss_and_lidar_localization/demo/nishishinjuku.html) | TIER IV の [AWSIM](https://github.com/autowarefoundation/AWSIM) の西新宿の地図 `nishishinjuku_autoware_map`（約 0.9 km × 1.1 km） | 地図は [CC BY-NC 4.0](https://creativecommons.org/licenses/by-nc/4.0/)（全文: [LICENSE-nishishinjuku.txt](LICENSE-nishishinjuku.txt)） |

どちらもタイル読み込みの様子を再生するページで、作り方は [tools/tile_demo](../../tools/tile_demo/README.md) にある。

## 西新宿の地図について（CC BY-NC 4.0）

`nishishinjuku.html` には、次の素材から作ったデータ（点群・経路）が入っている。このページとそのデータは、元の素材と同じく **CC BY-NC 4.0** で、**非営利の目的に限って**使える。リポジトリのほかの部分の MIT ライセンスは、このデータには及ばない。

- 素材: AWSIM（TIER IV, Inc.、現在は Autoware Foundation が管理）の `nishishinjuku_autoware_map`（`pointcloud_map.pcd`、`lanelet2_map.osm`）。<https://github.com/autowarefoundation/AWSIM> の v1.1.0 のリリースで配布されているもの
- 改変: 点群を `gll_map_tiler` で 0.2 m に間引いて 20 m のタイルに分け、ページでは上から見た 0.5 m の格子にさらに間引いた。経路は lanelet の中心線から作った（`tools/tile_demo/route_nishishinjuku.txt` も同じライセンス）
- ライセンス: [CC BY-NC 4.0](https://creativecommons.org/licenses/by-nc/4.0/)（全文: [LICENSE-nishishinjuku.txt](LICENSE-nishishinjuku.txt)。地図の配布物に同梱されていたもの）
