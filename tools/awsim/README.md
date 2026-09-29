# AWSIM での検証のスクリプト

[検証計画: AWSIM](../../docs/validation_awsim.md) で使うスクリプト。AWSIM（**v1.3.1**、ROS 2 Humble。tier4.github.io/AWSIM の Quick start demo の `AWSIM_v1.3.1.zip`。v2 系はデモで真値の姿勢が出ないので使わない: 検証計画 1.1 節）で車を走らせて記録した bag を、推定ノード（ROS 2 Jazzy）用の bag に変換し、推定の結果を真値と比べる。

| スクリプト | 動かす場所 | 内容 |
|---|---|---|
| `awsim_drive.py` | AWSIM の PC（Humble） | 決めた経路を 6 km/h で走らせる ROS 2 ノード。`--check` なら ROS なしで経路を確かめるだけ |
| `check_lidar_extrinsic.py` | dev コンテナ | LiDAR の取り付け位置を、地図のタイルと照らして確かめ、合うように直す |
| `awsim_to_bag.py` | dev コンテナ | 変換。GNSS（NavSatFix）を真値から作り、途切れ・LiDAR の欠落・ODOM の劣化・初期姿勢の誤差を入れられる |
| `evaluate.py` | dev コンテナ | 推定ノードの出力の CSV を真値と比べ、指標の表を作る |
| `mcap_io.py`、`rigid.py`、`drive_core.py` | — | MCAP / CDR の読み出し、回転の計算、経路追従（ほかのスクリプトから使う） |

必要なもの: dev コンテナの中なら追加は要らない（numpy・PyYAML・pyproj）。AWSIM の PC では `ros-humble-autoware-auto-msgs`、`ros-humble-rosbag2-storage-mcap`、`python3-numpy`（経路を作るなら `python3-scipy` も）。

## 1. 経路を作って走らせ、記録する（AWSIM の PC）

AWSIM を起動してから（Quick start demo の `ROS_LOCALHOST_ONLY=1`、`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` を、下の全部の端末で同じにする）:

```bash
# 車の位置（地図座標）
ros2 topic echo --once /awsim/ground_truth/vehicle/pose

# 車の位置から始まる経路を、lanelet の中心線から作る（最初の通過点 = 車の位置。M は西新宿の地図のフォルダ）
python3 tools/tile_demo/lanelet_route.py $M/lanelet2_map.osm '[[x0, y0], [x1, y1], [x2, y2]]' > route.txt

# 走れる経路かを確かめる（U ターンなど、曲がれない所があれば出る。終了コード 1）
python3 tools/awsim/awsim_drive.py route.txt --check

# 記録を始めてから、走らせる（5 s 止まってから走り出し、経路の終わりで止まってノードも終わる）
ros2 bag record -s mcap -o nsj_run1 /awsim/ground_truth/vehicle/pose /sensing/imu/tamagawa/imu_raw \
    /vehicle/status/velocity_status /sensing/lidar/top/pointcloud_raw
python3 tools/awsim/awsim_drive.py route.txt --wait 5      # 別の端末
```

- `awsim_drive.py` は `/control/command/control_cmd`・`gear_cmd`・`/vehicle/engage` を出す。車が動かないときは、AWSIM の画面で車の操作が自動（ROS からの指令）になっているか、ほかのノード（Autoware）が同じトピックを出していないかを確かめる。
- 障害物・信号・ほかの車は見ない。ぶつかったら記録をやり直す。
- `--max-distance 300` で、300 m 走ったら止める。
- 記録は圧縮しない（`--compression-mode` を付けない）。zstd で圧縮した bag を読むには `pip install zstandard` が要る。

## 2. 地図・取り付け位置・変換（dev コンテナ）

```bash
D=$GLL_DATA/awsim
tiled_pcd_map_tiler -i $M/pointcloud_map.pcd -o $D/nsj_tiles --tile-size 20 --voxel-size 0.2

# LiDAR の取り付け位置（base_link → velodyne_top）。yaw を 1 周調べてから細かく探す。最後の行の値を次の --lidar-extrinsic に使う
python3 tools/awsim/check_lidar_extrinsic.py $D/nsj_run1 $D/nsj_tiles --extrinsic 0.9,0,2.0,0,0,0

# 変換（出力: v_a0.mcap、v_a0_groundtruth.csv、v_a0_params.yaml、v_a0_maps.yaml）
python3 tools/awsim/awsim_to_bag.py $D/nsj_run1 $D/out/v_a0.mcap --lidar-extrinsic <上の値> --tiles $D/nsj_tiles
```

`awsim_to_bag.py` の主なオプション（全部は `--help`）:

| オプション | 内容 |
|---|---|
| `--gnss-off-time START:DUR ...` | bag の最初から START 秒後から DUR 秒、RTK-FIX を外す（status = −1） |
| `--gnss-off-box X0,Y0,X1,Y1 ...` | アンテナがこの矩形（地図座標）に入っている間、RTK-FIX を外す |
| `--no-gnss` | GNSS を出さない（地図だけ） |
| `--initial-pose ERR_XY,ERR_YAW_DEG` | `/initialpose` を出す（真値からこれだけずらす。向きはランダム、`--seed` で変わる） |
| `--drop-lidar START:DUR ...` | 点群を抜く |
| `--odom-scale`、`--odom-noise` | ODOM の縮尺の誤差と雑音 |
| `--gnss-rate`、`--gnss-sigma`、`--gnss-lever` | 作る GNSS の周期（10 Hz）、水平の σ（0.02 m）、アンテナの位置（0,0,1.5） |
| `--mgrs-origin`、`--utm-zone` | 地図座標の原点の UTM（西新宿は 300000,3900000）と帯（54） |

## 3. 推定ノードで再生して評価する（dev コンテナ）

```bash
ros2 launch gll_ros2 localizer.launch.py params_file:=$D/out/v_a0_params.yaml use_sim_time:=true &
ros2 bag play $D/out/v_a0.mcap --clock 100
kill -INT %1      # 終わったらノードを止める（出力の CSV が閉じられる）
python3 tools/awsim/evaluate.py $D/out/v_a0_output.csv $D/out/v_a0_groundtruth.csv --out $D/out/v_a0.md
```

## テスト

`test/run_tests.sh`（CI の `tools_awsim` ジョブ）は、AWSIM と同じトピック・型の合成の bag（`test/make_fake_awsim_bag.py`）で、経路追従・取り付け位置の探索・変換を確かめる。`test/run_localizer_check.sh`（CI の `ros2` ジョブ）は、変換した bag を ROS 2 Jazzy で読み、推定ノードに通して真値と比べる。

地図（西新宿の地図、CC BY-NC 4.0）から作ったタイル・bag・経路は、リポジトリに入れない。
