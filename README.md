# gnss_and_lidar_localization

RTK-GNSS が FIX する区間では GNSS、それ以外の区間では事前作成した点群地図と LiDAR の照合で、途切れない自己位置（UTM の x, y, yaw）を推定する ROS 2 パッケージ。推定器は SE(2) 上の **Invariant EKF** 1 つで、GNSS・LiDAR・IMU・ODOM を融合する。

- 要件定義: [docs/requirements.md](docs/requirements.md)
- 設計書: [docs/design.md](docs/design.md)
- ソフトウェア構成（コンポーネント図・クラス図）: [docs/architecture.md](docs/architecture.md)
- アルゴリズム説明書（Invariant EKF の解説を含む）: [docs/algorithm.md](docs/algorithm.md)
- 図解ページ: [docs/explainer/index.html](docs/explainer/index.html)
- 検証計画（i2Nav-Robot）: [docs/validation_i2nav.md](docs/validation_i2nav.md)

## 実装の状況

| Phase | 内容 | 状態 |
|---|---|---|
| 1 | Docker・CI、コア（SE(2)・Invariant EKF・遅延観測・GNSS・姿勢推定・出力整形・状態監視・GNSS の再アンカー）、ROS 2 IF（IMU・ODOM・GNSS） | 実装済み |
| 2 | 地図タイル管理、small_gicp による LiDAR 観測、地図区間での初期化、LiDAR の再アンカーと再位置推定 | 未着手 |
| 3 | 地図グループの切り替え、アンカー較正ツール、実データでの調整 | 未着手 |

## 構成

```
core/                  gll_core: ROS に依存しないコアライブラリ（C++17, Eigen, GeographicLib）
ros2/gll_ros2/         ROS 2 Jazzy のインターフェース（LocalizerNode, launch, パラメータ）
docker/                Dockerfile（dev / runtime）と compose.yaml
tools/sim/             Invariant EKF と ESEKF の比較シミュレーション（Python）
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

コアだけなら ROS なしでもビルドできる（`libeigen3-dev libgeographiclib-dev libgtest-dev` が必要）。

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

### トピック

| 方向 | トピック | 型 |
|---|---|---|
| 購読 | `~/input/imu` | `sensor_msgs/Imu` |
| 購読 | `~/input/odom` | `nav_msgs/Odometry`（twist だけを使う） |
| 購読 | `~/input/gnss/fix` | `sensor_msgs/NavSatFix`（RTK-FIX のとき `status = 2`、共分散を入れる。設計書 3.6 節） |
| 購読 | `~/input/gnss/velocity` | `geometry_msgs/TwistWithCovarianceStamped`（任意） |
| 購読 | `~/input/initial_pose` | `geometry_msgs/PoseWithCovarianceStamped` |
| 配信 | `~/output/pose` | `geometry_msgs/PoseWithCovarianceStamped`（frame `map` = UTM） |
| 配信 | `~/output/odometry` | `nav_msgs/Odometry` |
| 配信 | `~/output/status` | `diagnostic_msgs/DiagnosticStatus`（`GNSS_AIDED` / `DEGRADED` / `LOST` など） |
| 配信 | `~/debug/raw_pose` | `geometry_msgs/PoseWithCovarianceStamped`（出力整形前） |
| 配信 | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` |
| TF | `map` → `base_link` | |

パラメータは [ros2/gll_ros2/config/localizer.yaml](ros2/gll_ros2/config/localizer.yaml) を参照。

## ライセンス

MIT
