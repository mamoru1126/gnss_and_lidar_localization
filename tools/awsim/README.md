# AWSIM での検証のスクリプト

[検証計画: AWSIM](../../docs/validation_awsim.md) で使うスクリプト。AWSIM（**v1.3.1**、ROS 2 Humble。tier4.github.io/AWSIM の Quick start demo の `AWSIM_v1.3.1.zip`。v2 系はデモで真値の姿勢が出ないので使わない: 検証計画 1.1 節）で車を走らせて記録した bag を、推定ノード（ROS 2 Jazzy）用の bag に変換し、推定の結果を真値と比べる。

| スクリプト | 動かす場所 | 内容 |
|---|---|---|
| `plan_route.py` | どこでも（numpy だけ） | 車の今の姿勢から、lanelet2 の地図の進行方向と車線のつながりを守って経路を作る（長さを決めて乱数で、または通る点を並べて）。`--svg` で経路の図 |
| `awsim_drive.py` | AWSIM の PC の Humble のコンテナ | 決めた経路を 6 km/h で走らせる ROS 2 ノード。`--check` なら ROS なしで経路を確かめるだけ |
| `check_lidar_extrinsic.py` | dev コンテナ | LiDAR の取り付け位置を、地図のタイルと照らして確かめ、合うように直す。`--time-offset` で点群のスタンプのずれも求める |
| `awsim_to_bag.py` | dev コンテナ | 変換。GNSS（NavSatFix）を真値から作り、途切れ・LiDAR の欠落・ODOM の劣化・初期姿勢の誤差を入れられる |
| `replay.sh` | dev コンテナ | 変換した bag を推定ノードに通す（起動・再生・停止）。`--record` で推定の様子（地図・スキャン・姿勢）を bag に録る |
| `evaluate.py` | dev コンテナ | 推定ノードの出力の CSV を真値と比べ、指標の表を作る |
| `mcap_io.py`、`rigid.py`、`drive_core.py` | — | MCAP / CDR の読み出し、回転の計算、経路追従（ほかのスクリプトから使う） |

必要なもの: dev コンテナの中なら追加は要らない（numpy・PyYAML・pyproj）。記録と走行は、ROS 2 Humble のコンテナ（`docker/awsim/Dockerfile`。`autoware_msgs` の制御と車両のメッセージ・MCAP の記録・CycloneDDS・numpy・scipy 入り）で行う。ホストに ROS 2 を入れる必要は無い（AWSIM 本体は ROS 2 を中に持っている）。ホストで使うのは Docker と curl・unzip だけ。

| ホストで使うスクリプト | 内容 |
|---|---|
| `setup.sh` | 最初に 1 回。AWSIM v1.3.1 と西新宿の地図を `data/awsim/` に落として展開し、Humble のコンテナを作る |
| `run_awsim.sh` | AWSIM を起動する。通信の環境変数と、CycloneDDS のための `sysctl`・ループバックのマルチキャストをまとめて設定する。既定ではほかの車（NPC）を出さない（`--traffic N` で出す）。`--start X,Y,Z,YAW` で車の最初の位置を変えられる |
| `container.sh` | Humble のコンテナに入る（ホストの利用者の UID で） |
| `record.sh <名前>` | コンテナの中で使う。4 つのトピックを `data/awsim/<名前>/` に MCAP で記録する |

## 1. 経路を作って走らせ、記録する（AWSIM の PC）

データ（AWSIM 本体、地図、記録した bag、経路、変換の出力）は、全部リポジトリの `data/awsim/`（`.gitignore` 済み）に置く。環境変数を設定する必要は無い（コンテナの中では `GLL_DATA` がリポジトリの `data/` を指す）。コマンドはリポジトリの一番上で実行する。

```bash
# 最初に 1 回（ホストで）: AWSIM v1.3.1 と西新宿の地図を data/awsim/ に落とし、Humble のコンテナを作る
tools/awsim/setup.sh

# AWSIM を起動する（ホストで）。ROS_LOCALHOST_ONLY などの環境変数と、CycloneDDS のための sysctl・
# ループバックのマルチキャスト（足りなければ sudo で設定する）を、このスクリプトがまとめて行う
tools/awsim/run_awsim.sh

# Humble のコンテナに入る（ホストで。記録用と走行用に 2 つの端末で。どちらも同じコマンド）
tools/awsim/container.sh
```

コンテナの中で:

```bash
# 真値が出ているか
ros2 topic list | grep ground_truth
# 車の今の姿勢（地図座標 X,Y と向き [deg]）
START=$(python3 tools/awsim/awsim_drive.py --print-pose)

# 車の位置から、交通ルール（一方通行・車線のつながり）どおりに 1.5 km 走る経路を作る（地図は data/awsim/ のもの）
python3 tools/awsim/plan_route.py --start $START --length 1500 --seed 0 -o data/awsim/route.txt --svg data/awsim/route.svg
# 通りたい所があるなら、--length の代わりに --via X1,Y1 X2,Y2 ...（座標は route.svg の目盛りで読む）

# 走れる経路かを確かめる（曲がれない所があれば出る。終了コード 1）
python3 tools/awsim/awsim_drive.py data/awsim/route.txt --check

# 記録を始めてから（1 つ目の端末。data/awsim/nsj_run1/ にできる。Ctrl-C で止める）、
tools/awsim/record.sh nsj_run1
# 走らせる（2 つ目の端末。5 s 止まってから走り出し、経路の終わりで止まって終わる）
python3 tools/awsim/awsim_drive.py data/awsim/route.txt --wait 5
```

- 経路の決め方: まず `--length` で作り、`route.svg`（ブラウザで開く。灰 = 道路、赤 = 経路、緑 = 始点、青 = 終点）を見る。気に入らなければ `--seed` を変えるか、図の目盛りで通りたい交差点の座標を読んで `--via` に並べる。`--via` の点は近く（10 m 以内）の車線に寄せ、進行方向を守った最短の道でつなぐ。lanelet2 の地図は AWSIM の世界（3D の地面）より広いので、点群地図（同じフォルダの `pointcloud_map.pcd`）から 20 m より離れた所を通る車線と、半径 6 m より急に曲がる車線は使わない（世界の外へ出て車が落ちたことがあった）。地図の端で行き止まりになる道の点を途中に置くと、そう表示して止まる。
- `tools/tile_demo/lanelet_route.py` はタイルのデモ用で、進行方向を見ないので、走らせる経路には使わない。
- `awsim_drive.py` は `/control/command/control_cmd`（`autoware_control_msgs/Control`）・`gear_cmd` を、AWSIM と同じ QoS（RELIABLE・TRANSIENT_LOCAL）で出す。車が動かないときは、`requesting incompatible QoS` の警告が出ていないか、AWSIM の画面で車の操作が自動（ROS からの指令）になっているか、ほかのノード（Autoware）が同じトピックを出していないかを確かめる。
- 障害物・信号・ほかの車は見ない。なので `run_awsim.sh` は既定でほかの車を出さない（AWSIM の設定ファイル `data/awsim/awsim_config.json` の `MaxVehicleCount: 0`）。`--traffic 10` のように出すと、ぶつかることがある。
- 速さがありえない値（15 km/h か目標の 2 倍を超える。ぶつかった・地面から落ちた）になったら、ブレーキを出して終わる（終了コード 2）。AWSIM を起動し直す。
- `--max-distance 300` で、300 m 走ったら止める。
- 速さは既定 6 km/h（本システムの想定の最高速度）。`--kmh 25` のように上げられる（`--check` にも同じ値を付けて確かめる）。曲がる所の手前で、横加速度が 1.5 m/s² を超えないように減速し（交差点の右左折はおよそ 10〜15 km/h）、経路の終わりでちょうど止まる。速さは真値の位置の 0.2 s の変化（メッセージの時刻で割る）で測り、比例と積分で追う（AWSIM の `/vehicle/status/velocity_status` は、ぶつかった後などに ±数百 km/h の値を出すことがあったので、表示にだけ使う）（坂や抵抗で目標に届かない分を積分で補う）。1 s ごとに今の速さと目標、残りの距離を出す。止まった後も 2 s ブレーキを出してから終わる。6 km/h より速い記録は本システムの想定の外なので、結果を見るときは分けて扱う。
- `ros2 topic list` に AWSIM のトピックが出ないときは、AWSIM を `tools/awsim/run_awsim.sh` で起動したかを確かめる（別の方法で起動すると、通信の設定がそろわない）。
- 記録した bag や経路は、ホストの自分のファイルになる（コンテナをホストの利用者の UID で動かしている）。
- 記録は圧縮しない（`--compression-mode` を付けない）。zstd で圧縮した bag を読むには `pip install zstandard` が要る。

## 2. 地図・取り付け位置・変換（dev コンテナ）

`D` は `data/awsim` の短縮（リポジトリの一番上で実行する）。

```bash
D=data/awsim
tiled_pcd_map_tiler -i $D/nishishinjuku_autoware_map/pointcloud_map.pcd -o $D/nsj_tiles --tile-size 20 --voxel-size 0.2

# LiDAR の取り付け位置（base_link → velodyne_top）と、点群のスタンプのずれ。
# 既定の最初の値は v1.3.1 の Lexus で合わせた値（0.9,0,2.04,0.9,0,88.2）なので、--no-yaw-sweep で細かく探すだけでよい。
# 「--lidar-extrinsic」と「lidar.stamp_offset」の行の値を次の変換に使う
python3 tools/awsim/check_lidar_extrinsic.py $D/nsj_run1 $D/nsj_tiles --no-yaw-sweep --time-offset --turning

# 変換（出力: v_a0/（rosbag2 の bag: v_a0.mcap と metadata.yaml）、v_a0_groundtruth.csv、v_a0_params.yaml、v_a0_maps.yaml）
python3 tools/awsim/awsim_to_bag.py $D/nsj_run1 $D/out/v_a0 --lidar-extrinsic <上の値> \
    --lidar-stamp-offset <上の値> --tiles $D/nsj_tiles
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
| `--lidar-stamp-offset` | パラメータ `lidar.stamp_offset`（点群のスタンプのずれ） |
| `--full-map-voxel` | 全体の地図（`/map/points`、表示用）を間引く大きさ（1.0 m。0 なら入れない） |
| `--no-fix-lag` | IMU・車速のスタンプのずれ（真値の yaw レートと比べて求める）を直さない |
| `--gnss-rate`、`--gnss-sigma`、`--gnss-lever` | 作る GNSS の周期（10 Hz）、水平の σ（0.02 m）、アンテナの位置（0,0,1.5） |
| `--mgrs-origin`、`--utm-zone` | 地図座標の原点の UTM（西新宿は 300000,3900000）と帯（54） |

## 3. 推定ノードで再生して評価する（dev コンテナ）

```bash
# 推定ノードを起動して bag を再生し、終わったら止める。--record で推定の様子を bag に録る
tools/awsim/replay.sh $D/out/v_a0 --record $D/out/v_a0_rec
python3 tools/awsim/evaluate.py $D/out/v_a0_output.csv $D/out/v_a0_groundtruth.csv --out $D/out/v_a0.md
```

録った bag（`v_a0_rec/`）は Foxglove などで開く。表示の frame は `map_local`（UTM の大きな座標を避けるため、走り始めの位置を
100 m 単位に丸めた点を原点にした frame。`map → map_local` は `/tf_static` にある）。

| トピック | 内容 |
|---|---|
| `/map/points` | 全体の地図（1 m で間引いたもの） |
| `/gll_localizer/debug/map_points` | いま照合に使っている部分の地図（読み込んだタイル） |
| `/gll_localizer/debug/scan_points` | いまのスキャン（前処理の後、0.2 m で間引き）を照合の結果の姿勢で置いたもの。地図に重なっていれば照合が合っている |
| `/gll_localizer/output/pose`、`/gll_localizer/debug/raw_pose` | 推定の出力（滑らかにしたもの・フィルタそのもの） |
| `/gll_localizer/debug/lidar_pose` | 照合の結果の姿勢 |
| `/groundtruth/pose` | 真値 |
| `/tf`、`/tf_static` | `map → base_link`（推定）、`base_link → velodyne_top`、`map → map_local` |
| `/gll_localizer/output/status`、`/diagnostics` | 状態 |

生の点群（`/sensing/lidar/points`）は大きいので録らない。要るときは `--raw-points` を付ける。変換した bag（`v_a0/`）も
そのまま開ける（`/map/points` と `/tf_static` が入っている）。

## テスト

`test/run_tests.sh`（CI の `tools_awsim` ジョブ）は、AWSIM と同じトピック・型の合成の bag（`test/make_fake_awsim_bag.py`）で、経路追従・取り付け位置の探索・変換を確かめる。`test/run_localizer_check.sh`（CI の `ros2` ジョブ）は、変換した bag を ROS 2 Jazzy で読み、推定ノードに通して真値と比べる。

地図（西新宿の地図、CC BY-NC 4.0）から作ったタイル・bag・経路は、リポジトリに入れない。
