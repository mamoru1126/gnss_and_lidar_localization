// 地図タイルの管理の設定（設計書 5 章、README.md）。
#pragma once

namespace tiled_pcd_map {

struct MapManagerConfig {
  double load_radius = 60.0;         ///< 現在位置（と先読み位置）からこの距離 [m] 以内のタイルを読み込む
  double unload_radius = 90.0;       ///< 現在位置からこの距離 [m] より遠いタイルを捨てる
  double lookahead_time = 3.0;       ///< 先読み [s]（速さ × この時間だけ進行方向に進んだ位置の周りも読む）
  double update_distance = 1.0;      ///< この距離を動いたら、必要なタイルとアクティブグループを見直す [m]
  double update_interval = 1.0;      ///< 動いていなくても、この間隔で見直す [s]
  double group_switch_margin = 10.0; ///< アクティブグループの切り替えのヒステリシス [m]
  int min_target_points = 500;       ///< 点がこれ未満の領域は出さない（currentRegion() が nullptr になる）
  bool async = true;                 ///< 別スレッドでロードする（false なら update() の中でロードする）
};

}  // namespace tiled_pcd_map
