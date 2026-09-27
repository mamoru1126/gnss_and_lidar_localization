# gnss_and_lidar_localization

RTK-GNSS が FIX する区間では GNSS、それ以外の区間では事前作成した点群地図と LiDAR の照合で、途切れない自己位置（UTM の x, y, yaw）を推定する ROS 2 パッケージ。推定器は SE(2) 上の **Invariant EKF** 1 つで、GNSS・LiDAR・IMU・ODOM を融合する。GNSS 区間が無く、点群地図だけがある現場でも、初期姿勢（または前回保存した位置）から地図上で初期化して動く。

パッケージ名・名前空間などの **gll** は **G**NSS and **L**iDAR **L**ocalization の略（`gll_core`、`gll_ros2`、`gll::`、`gll_map_tiler`、ノード名 `gll_localizer`）。

- 要件定義: [docs/requirements.md](docs/requirements.md)
- 設計書: [docs/design.md](docs/design.md)
- ソフトウェア構成（コンポーネント図・クラス図）: [docs/architecture.md](docs/architecture.md)
- アルゴリズム説明書（Invariant EKF の解説を含む）: [docs/algorithm.md](docs/algorithm.md)
- 図解ページ: [https://sunomamo1126.github.io/gnss_and_lidar_localization/explainer/](https://sunomamo1126.github.io/gnss_and_lidar_localization/explainer/)（ソース: [docs/explainer/index.html](docs/explainer/index.html)）
- 検証計画（i2Nav-Robot）: [docs/validation_i2nav.md](docs/validation_i2nav.md)
- タイル読み込みのデモ: [https://sunomamo1126.github.io/gnss_and_lidar_localization/demo/tile_loading.html](https://sunomamo1126.github.io/gnss_and_lidar_localization/demo/tile_loading.html)（ソース: [docs/demo/tile_loading.html](docs/demo/tile_loading.html)、作り方は [tools/tile_demo](tools/tile_demo/README.md)）

## 実装の状況

| Phase | 内容 | 状態 |
|---|---|---|
| 1 | Docker・CI、コア（SE(2)・Invariant EKF・遅延観測・GNSS・姿勢推定・出力整形・状態監視・デッドレコニング距離の監視・GNSS の再アンカー）、ROS 2 IF（IMU・ODOM・GNSS・diagnostics） | 実装済み |
| 2 | 地図のタイル化ツール（`gll_map_tiler`）、地図タイル管理（非同期ロード・複数の地図グループの切り替え）、small_gicp（GICP）による LiDAR 観測とデスキュー、地図上での初期化、GNSS FIX 中の食い違い判定、LiDAR の再アンカーと再位置推定、ROS 2 IF（PointCloud2・デバッグ出力・前回位置の保存） | 実装済み（合成データのシミュレーションで確認。実データでの調整は Phase 3） |
| 3 | 実データ（i2Nav-Robot）での検証とパラメータ調整、アンカー較正ツール | 未着手 |

## 構成

```
core/                  gll_core: ROS に依存しないコアライブラリ（C++17, Eigen, GeographicLib, small_gicp, yaml-cpp）
                       と地図のタイル化ツール gll_map_tiler
ros2/gll_ros2/         ROS 2 Jazzy のインターフェース（LocalizerNode, launch, パラメータ）
docker/                Dockerfile（dev / runtime）と compose.yaml
tools/sim/             Invariant EKF と ESEKF の比較シミュレーション（Python）
tools/tile_demo/       タイル読み込みのデモページを作るスクリプト
docs/                  設計ドキュメント
```

## ビルドとテスト（Docker）

```bash
export GLL_DATA=/path/to/data          # rosbag や地図を置くディレクトリ（リポジトリの外）
docker compose -f docker/compose.yaml build
docker compose -f docker/compose.yaml run --rm dev

# コンテナ内
cd /ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test && colcon test-result --verbose
```

コアだけなら ROS なしでもビルドできる（`libeigen3-dev libgeographiclib-dev libgtest-dev libyaml-cpp-dev` と、ソースからインストールした [small_gicp](https://github.com/koide3/small_gicp) v1.0.1 が必要。手順は [.github/workflows/ci.yml](.github/workflows/ci.yml) の core ジョブを参照）。

```bash
cmake -S core -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build && ctest --test-dir build --output-on-failure
```

## 起動

```bash
source /ws/install/setup.bash
ros2 launch gll_ros2 localizer.launch.py \
    imu_topic:=/sensing/imu odom_topic:=/sensing/odom gnss_topic:=/sensing/gnss/fix
```

起動直後の数秒（`attitude.static_init_time`、既定 3 秒）は車両を静止させる（IMU の姿勢とバイアスの初期化）。その後、RTK-FIX を得た状態で 1 m ほど走ると yaw が決まり、出力が始まる。

### LiDAR と点群地図を使う

1. **地図をタイルにする**（地図グループごとに 1 回）。入力は外部ツールで作った統合済みの PCD（ascii / binary / binary_compressed）。

   ```bash
   gll_map_tiler -i /data/maps/area_a.pcd -o /data/maps/area_a --tile-size 20 --voxel-size 0.2
   # → /data/maps/area_a/tiles/*.bin と tile_index.yaml
   ```

2. **maps.yaml を書く**。地図グループごとに、タイルの索引とアンカー（地図の 1 点の緯度経度・楕円体高と、地図の x 軸の方位）を与える。UTM で直接与えることもできる。緯度経度が分からない地図だけの現場では `anchor: local` にすると、地図の座標をそのまま `map` として出力する（GNSS の入力は無視する。UTM にアンカーした地図とは混在できない）。

   ```yaml
   utm: {zone: 54, hemisphere: north}      # 任意。gnss.utm_zone と違えば起動を止める
   map_groups:
     - id: area_a
       tile_index: area_a/tile_index.yaml   # maps.yaml からの相対パス
       anchor:
         map_point: [0.0, 0.0, 0.0]         # アンカー点の地図座標 [m]
         latitude: 35.681236
         longitude: 139.767125
         ellipsoid_height: 40.0
         heading_deg: 90.0                  # 地図の x 軸の方位（真北から時計回り）
     - id: area_b
       tile_index: area_b/tile_index.yaml
       anchor: {easting: 386250.0, northing: 3952100.0, ellipsoid_height: 41.0, grid_heading_deg: 12.5}
   ```

   詳細は[設計書](docs/design.md) 4.2 節・5.2 節。

3. **起動する**。LiDAR の取り付け位置（`lidar.extrinsic_xyz` / `extrinsic_rpy_deg`）をパラメータファイルで設定しておく。

   ```bash
   ros2 launch gll_ros2 localizer.launch.py \
       imu_topic:=/sensing/imu odom_topic:=/sensing/odom gnss_topic:=/sensing/gnss/fix \
       points_topic:=/sensing/lidar/points map_config:=/data/maps/maps.yaml
   ```

   点ごとの時刻のフィールド（`time` / `t` / `timestamp` / `time_stamp` / `offset_time`）は自動で判別してデスキューに使う。使ったフィールドは起動後の最初のスキャンでログに出る。Livox Mid-360 は livox_ros_driver2 を `xfer_format: 0`（PointCloud2）で起動する（`timestamp` を使う）。Mid-360 の内蔵 IMU を使う場合は、加速度が g 単位なので `imu.acc_scale: 9.80665` にする。

4. **地図だけの現場での初期化**。静止初期化の後、RViz の「2D Pose Estimate」などで `~/input/initial_pose` に大まかな初期姿勢（数 m・数十度ずれていてよい）を与えると、その周りを探して地図上で初期化する。止めた場所から起動する運用なら、`init.saved_pose_path` に位置を定期保存し、`init.use_saved_pose: true` で次の起動時に使える。見つからないとき（5 回失敗、または 15 s）は、与えた初期姿勢はそのまま使い、保存した位置は捨てて GNSS か初期姿勢を待つ（[設計書](docs/design.md) 3.11 節）。

走行中は、地図の近くでタイルを読み込み（先読みあり）、現在位置に最も近い地図グループで照合する。GNSS の RTK-FIX 中は GNSS を優先し、LiDAR との差を地図グループのアンカーずれとして記録する（ずれが 0.10 m / 0.5° を超えると `/diagnostics` の `gll_localizer: map` が WARN）。LiDAR の照合が続けて棄却されたら、推定値の周りで再位置推定する。

### トピック

| 方向 | トピック | 型 |
|---|---|---|
| 購読 | `~/input/imu` | `sensor_msgs/Imu` |
| 購読 | `~/input/odom` | `nav_msgs/Odometry`（twist だけを使う） |
| 購読 | `~/input/gnss/fix` | `sensor_msgs/NavSatFix`（RTK-FIX のとき `status = 2`、共分散を入れる。設計書 3.6 節） |
| 購読 | `~/input/gnss/velocity` | `geometry_msgs/TwistWithCovarianceStamped`（任意） |
| 購読 | `~/input/initial_pose` | `geometry_msgs/PoseWithCovarianceStamped`（共分散が 0 なら 1 m・30° とみなす） |
| 購読 | `~/input/points` | `sensor_msgs/PointCloud2`（LiDAR。地図を設定したときだけ） |
| 配信 | `~/output/pose` | `geometry_msgs/PoseWithCovarianceStamped`（frame `map` = UTM） |
| 配信 | `~/output/odometry` | `nav_msgs/Odometry` |
| 配信 | `~/output/status` | `diagnostic_msgs/DiagnosticStatus`（`GNSS_AIDED` / `DEGRADED` / `LOST` など。デッドレコニング距離の超過は ERROR） |
| 配信 | `~/debug/raw_pose` | `geometry_msgs/PoseWithCovarianceStamped`（出力整形前） |
| 配信 | `~/debug/lidar_pose` | `geometry_msgs/PoseWithCovarianceStamped`（LiDAR の照合結果と観測共分散） |
| 配信 | `~/debug/map_points` | `sensor_msgs/PointCloud2`（照合に使っている地図。frame `map_local`、transient local） |
| 配信 | `/diagnostics` | `diagnostic_msgs/DiagnosticArray`（1 Hz。`gll_localizer`、`gll_localizer: counters`、`gll_localizer: map`） |
| TF | `map` → `base_link` | |
| TF | `map` → `map_local`（静的） | RViz 用に原点を近くに移したフレーム（`map_local_origin`。指定しなければ最初の地図グループのアンカー付近） |

パラメータは [ros2/gll_ros2/config/localizer.yaml](ros2/gll_ros2/config/localizer.yaml) を参照。

### デッドレコニングの監視

点群地図と GNSS 区間の間には必要十分な距離が設定される、という運用上の前提で動かす（[要件定義](docs/requirements.md) 3 章）。GNSS の RTK-FIX も LiDAR の照合も採用できないまま `monitor.dr_error_distance`（既定 30 m。仮の値）を超えて走ると、`~/output/status` と `/diagnostics` の `gll_localizer` が ERROR になり、次のメッセージが出る（位置の観測を採用すると解除される。停止中は距離が増えない）。

```
DEAD_RECKONING: dead reckoning for 31.2 m without GNSS / LiDAR position (limit 30.0 m)
```

詳細は[設計書](docs/design.md) 3.12 節。

## ライセンス

MIT
