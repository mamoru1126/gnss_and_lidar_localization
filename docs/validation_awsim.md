# 検証計画: AWSIM を使った自己位置推定の検証

- 関連文書: [要件定義](./requirements.md) / [設計書](./design.md)（v0.14。9 章「検証計画」） / [アルゴリズム説明書](./algorithm.md) / [tiled_pcd_map](../tiled_pcd_map/README.md) / スクリプトの使い方: [tools/awsim/README.md](../tools/awsim/README.md)
- 状態: ドラフト（v0.6。AWSIM v1.3.1 の制御と車両のメッセージが autoware_msgs（autoware_control_msgs・autoware_vehicle_msgs）で、制御の購読が TRANSIENT_LOCAL だったのに合わせた（最初の記録で車が動かなかった）。v0.5: データを全部リポジトリの data/awsim/ に置き、準備・起動・記録をスクリプトにして、環境変数の設定を要らなくした。v0.4: 経路を、進行方向と車線のつながりを守る plan_route.py で作ることにした。v0.3: 記録と走行を Humble のコンテナで行うことにした。v0.2: v1.3.1 のソースで LiDAR の frame が `velodyne_top` であることを確かめ、取り付け位置の探索に yaw の 1 周の探索を足した。v2 系を使わない理由を 1.1 節に書いた。v0.1: 変換・走行・確認・評価のスクリプトを作り、合成の bag で動作を確かめた。AWSIM での記録はまだ）

本書は、自動運転シミュレータ **AWSIM**（TIER IV。Unity で動く。v1.3.1）の西新宿の地図で車を走らせて記録したデータで、本システムを検証する計画である。確かめるのは、**LiDAR による自己位置推定**（Phase 2）と、**GNSS 区間と地図区間の切り替わり**である。

AWSIM を使う理由:

- 真値（車の姿勢）が誤差なしで得られる。地図（点群と lanelet）も付いていて、地図座標と真値の座標系が一致している。
- 自分で経路と速度（6 km/h）を決めて、何度でも同じ条件で記録できる。GNSS の途切れ・LiDAR の欠落・ODOM の劣化も、変換のときに入れられる。
- 実データの公開データセットは、使える条件（ライセンス、社内の PC からの入手）が合わなかった。

シミュレーションなので、センサの雑音や地図との食い違いは実際より小さい。ここでの目的は、**変換・パラメータ・地図・起動手順が正しくつながること、状態の推移と切り替わりの振る舞い**を確かめることで、精度の最終的な確認は自社の車両のデータで行う（8 章）。

---

## 1. AWSIM と本システムの対応

### 1.1 使う版: v1.3.1（v2 系は使わない）

AWSIM には v1 系（TIER IV。ドキュメントは tier4.github.io/AWSIM）と、Autoware Foundation に移ってからの v2 系（v2.0.1。ドキュメントは autowarefoundation.github.io/AWSIM）がある。**v1.3.1 を使う**（Quick start demo の `AWSIM_v1.3.1.zip`）。v2 系のソース（v2.0.1）を見ると、次の点で本書の手順に合わない。

- **真値の姿勢のトピックが出ない**: v1.3.1 は車両に `PoseSensor`（`/awsim/ground_truth/vehicle/pose`、100 Hz）と `OdometrySensor`（`/awsim/ground_truth/localization/kinematic_state`）が付いている。v2.0.1 は真値を出す部品（`OdometryRos2Publisher`）はあるが、デモのシーンと車両に置かれていない。GNSS（10 Hz）は位置だけで向きが無い。
- ~~制御の型が違う~~: v1.3.1 もすでに `autoware_control_msgs/Control` と `autoware_vehicle_msgs`（新しい Autoware のメッセージ）を使っていた（tier4.github.io のトピックの一覧は古い `autoware_auto_*` のまま。実機の記録で分かった。v0.6）。
- **求める環境が重い**: NVIDIA ドライバ 570 以上、メモリ 32 GB 以上、Vulkan。

v2 系を使う必要が出たら、真値のセンサをシーンに置いたビルド（Unity で開いて作る）と、`awsim_drive.py` の型の切り替えが要る。

### 1.2 トピックと扱い

| 項目 | AWSIM（v1.3.1） | 本システムの想定 | 扱い |
|---|---|---|---|
| 動かす環境 | Ubuntu 22.04、GPU（RTX 2080 Ti 以上、ドライバ 550 推奨）。ROS 2 は AWSIM の中にある | ROS 2 Jazzy（Docker） | **記録と走行は Humble のコンテナ（`docker/awsim/Dockerfile`）で、変換と推定は dev コンテナ（Jazzy）で**行う。bag（MCAP）はリポジトリの `data/awsim/` で渡す（3 章） |
| 車両 | Lexus RX 450h（ホイールベース 2.79 m） | 車輪型、最高 6 km/h | `awsim_drive.py` で、決めた経路を 6 km/h で走らせる（3.2 節） |
| 真値 | `/awsim/ground_truth/vehicle/pose`（`PoseStamped`、100 Hz、base_link の姿勢、地図座標） | — | そのまま真値にする。UTM に直して CSV に書く |
| IMU | `/sensing/imu/tamagawa/imu_raw`（`Imu`、30 Hz） | 6 軸 IMU、100 Hz 以上 | 取り付けの向きを真値との当てはめで求め、base_link の向きに回して出す。**30 Hz と遅い**ので、予測の刻みが粗くなる（6.2 節） |
| ODOM | `/vehicle/status/velocity_status`（`autoware_vehicle_msgs/VelocityReport`、30 Hz。前進速度・横速度・ヨーレート） | `nav_msgs/Odometry`（twist） | `Odometry` に直す。ヨーレートも twist に入れる |
| GNSS | `/sensing/gnss/pose`（`Pose`、1 Hz） | u-blox F9P の RTK-FIX（`NavSatFix`、10 Hz） | **AWSIM の出力は使わず**、真値から `NavSatFix`（10 Hz、σ 2 cm、アンテナの位置 = base_link の上 1.5 m）を作る。途切れは時間か範囲で入れる（status = −1） |
| LiDAR | `/sensing/lidar/top/pointcloud_raw`（`PointCloud2`、10 Hz、frame **`velodyne_top`**。VLP-16 相当） | Livox Mid-360 | 中身はそのまま出す。base_link → `velodyne_top` の位置は**地図と照らして確かめる**（3.4 節）。点ごとの時刻の列があれば `lidar.time_field: auto` で使われ、無ければデスキューしない |
| 地図 | 西新宿の `pointcloud_map.pcd` と `lanelet2_map.osm`（MGRS 54SUE の区画の中の座標） | UTM の地図グループとアンカー | `tiled_pcd_map_tiler` でタイル化。地図座標 = UTM − (300000, 3900000)（54 帯）なので、アンカーは回転なし・縮尺なしで決まる（3.3 節） |
| 時刻 | 各トピックの `header.stamp`（AWSIM の時刻でそろっている） | — | そのまま使う。推定ノードは `use_sim_time` で bag の時刻で動かす |
| ライセンス | AWSIM は Apache 2.0。**西新宿の地図は CC BY-NC 4.0（非営利に限る）** | — | 業務の検証に使ってよいかを確認する（9 章）。地図から作ったもの（タイル、bag、経路）はリポジトリに入れない |

---

## 2. 全体の流れ

```
[AWSIM の PC（Ubuntu 22.04）: AWSIM はホストで、記録と走行は Humble のコンテナで]
  AWSIM を起動 → 車の姿勢を見る → 経路を作る（plan_route.py）→ awsim_drive.py で走らせながら ros2 bag record
                                                                                        │ bag（MCAP）
[dev コンテナ（Jazzy）]                                                                  ▼
  地図: pointcloud_map.pcd → tiled_pcd_map_tiler → タイル
  check_lidar_extrinsic.py（LiDAR の取り付け位置を地図で確かめる）
  awsim_to_bag.py（変換。シナリオごとに GNSS の途切れなどを入れて作り分ける）→ bag・真値・パラメータ・maps.yaml
  ros2 launch gll_ros2 localizer.launch.py … と ros2 bag play --clock → 出力の CSV
  evaluate.py（真値と比べる）→ 指標の表
```

AWSIM と dev コンテナは同じ PC でよい（dev コンテナは Docker なので、Ubuntu 22.04 の上でも Jazzy が動く）。

---

## 3. 手順

コマンドの詳細は [tools/awsim/README.md](../tools/awsim/README.md) にある。

データ（AWSIM 本体、地図、記録した bag、経路、タイル、変換の出力）は、全部リポジトリの `data/awsim/`（`.gitignore` 済み）に置く。どのコンテナでも `GLL_DATA` がリポジトリの `data/`（`/ws/src/gnss_and_lidar_localization/data`）を指すので、環境変数を設定する必要は無い。変換の出力のパラメータに書かれる地図の絶対パスも、dev コンテナと Dev Container で同じになる。コマンドはリポジトリの一番上で実行する。

### 3.1 準備（AWSIM の PC）

1. NVIDIA ドライバを入れる（AWSIM の Quick start demo のとおり。550 推奨）。ホストに ROS 2 は要らない（AWSIM は ROS 2 を中に持っている）。
2. `tools/awsim/setup.sh` を実行する。AWSIM v1.3.1 と西新宿の地図（AWSIM v1.1.0 のリリースの `nishishinjuku_autoware_map.zip`）を `data/awsim/` に落として展開し、記録と走行に使う ROS 2 Humble のコンテナ（`docker/awsim/Dockerfile`。`autoware_msgs` 1.1.0 の制御と車両のメッセージ、MCAP の記録、CycloneDDS、numpy・scipy 入り。ホストのネットワークを使う）を作る。

### 3.2 経路を作って走らせ、記録する（AWSIM の PC）

1. AWSIM を `tools/awsim/run_awsim.sh` で起動する。Quick start demo の通信の設定（`ROS_LOCALHOST_ONLY=1`・`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`、CycloneDDS のための `sysctl` とループバックのマルチキャスト）を、このスクリプトがまとめて行う。
2. `tools/awsim/container.sh` で Humble のコンテナに入る（記録用と走行用に 2 つの端末で）。
3. **車の位置から始まる経路**を `tools/awsim/plan_route.py` で作る。車の今の姿勢（`awsim_drive.py --print-pose`）から、lanelet2 の地図の進行方向と車線のつながり（successor、破線の所だけの車線変更）を守って、長さを決めて乱数で（`--length`）、または通る点を並べて（`--via`）作る。地図の端で行き止まりになる道には入らない。`--svg` の図で経路を確かめる。
4. `python3 tools/awsim/awsim_drive.py data/awsim/route.txt --check` で、ROS なしで経路を走れるかを確かめる（曲がれない急な所があれば出る）。`tools/tile_demo/lanelet_route.py`・`route_nishishinjuku.txt` はタイルのデモ用で、進行方向を見ず U ターンもあるので、走らせる経路には使わない。
5. 記録を始めてから走らせる。最初は **5 s 止まっている**（推定ノードの静止初期化のため）。
   ```bash
   tools/awsim/record.sh nsj_run1                                   # 1 つ目の端末。data/awsim/nsj_run1/。Ctrl-C で止める
   python3 tools/awsim/awsim_drive.py data/awsim/route.txt --wait 5 # 2 つ目の端末。経路の終わりで止まって終わる
   ```

`awsim_drive.py` は真値を見て経路をなぞる（pure pursuit）。障害物・信号・ほかの車は見ないので、NPC の車が出るシーンでは、ぶつかったら記録をやり直す。

### 3.3 地図（dev コンテナ）

```bash
tiled_pcd_map_tiler -i data/awsim/nishishinjuku_autoware_map/pointcloud_map.pcd -o data/awsim/nsj_tiles --tile-size 20 --voxel-size 0.2
```

`maps.yaml` は変換（3.5 節）の `--tiles` で作る。地図座標がそのまま「UTM − MGRS の区画の原点」なので、アンカーは次で決まる（`use_scale_factor: false`。AWSIM の地図は縮尺係数を含まない座標なので、縮尺の補正をしない）。

```yaml
utm: {zone: 54, hemisphere: north}
map_groups:
  - id: awsim
    tile_index: <タイルのフォルダ>/tile_index.yaml
    anchor: {map_point: [x0, y0, z0], easting: x0 + 300000, northing: y0 + 3900000, ellipsoid_height: z0,
             grid_heading_deg: 90.0, use_scale_factor: false, stddev_xy: 0.01, stddev_yaw_deg: 0.05}
```

### 3.4 LiDAR の取り付け位置を確かめる

base_link → `velodyne_top` の位置と向きは、AWSIM の車両の設定による。v1.3.1 の車両（`Lexus RX450h 2015 Sample Sensor`）では、`sensor_kit_base_link` が base_link から (0.9, 0, 2.0) m、yaw 約 −2°・pitch 約 1° にあり、`velodyne_top` はその上に置かれている。Autoware のサンプルのセンサキットでは `velodyne_top` がセンサキットに対して yaw 約 90° 回っているので、点群の座標も同じように回っている可能性がある。そこで探索は、最初に yaw を 1 周（10° ごと）調べてから細かく探す。`check_lidar_extrinsic.py` で、真値の姿勢と取り付け位置で点群を地図に重ね、地図の点がある格子に入る割合を最大にする値を探す。見つけた値と割合を記録し、割合が低い（< 60 %）ときは、地図と真値の座標系が合っているかを先に確かめる。

### 3.5 変換

```bash
python3 tools/awsim/awsim_to_bag.py data/awsim/nsj_run1 data/awsim/out/v_a0.mcap \
    --lidar-extrinsic <3.4 節の値> --tiles data/awsim/nsj_tiles
```

出力は bag（`/sensing/imu`、`/sensing/odom`、`/sensing/gnss/fix`、`/sensing/lidar/points`、`/tf_static`、`/groundtruth/pose`、`/initialpose`）、真値の CSV、パラメータ、`maps.yaml`。IMU の取り付けの向き（当てはめの残差）、最初の停止時間（→ `static_init_time`）も表示されるので記録する。シナリオ（4 章）ごとに、オプションを変えて bag を作り分ける。元の bag は変更しない。

### 3.6 推定ノードで再生して評価する

```bash
ros2 launch gll_ros2 localizer.launch.py params_file:=data/awsim/out/v_a0_params.yaml use_sim_time:=true &
ros2 bag play data/awsim/out/v_a0.mcap --clock 100
# 終わったらノードを止める（Ctrl-C）。出力は v_a0_output.csv
python3 tools/awsim/evaluate.py data/awsim/out/v_a0_output.csv data/awsim/out/v_a0_groundtruth.csv --out data/awsim/out/v_a0.md
```

RViz で見る場合は、`/groundtruth/pose` と推定ノードの出力、`~/debug/map_points`・`~/debug/lidar_pose` を並べる。

---

## 4. 検証シナリオ

| ID | 変換のオプション | 内容 | 主な確認点 |
|---|---|---|---|
| V-A0 | 既定（GNSS あり、LiDAR あり） | 動作確認 | 変換・取り付け位置・IMU の向き・起動手順が正しいこと。GNSS で初期化してから、GNSS と LiDAR の両方を使う状態（`GNSS_LIDAR_AIDED`）で走り切ること。GNSS FIX 中の LiDAR の食い違い判定が誤って出ないこと |
| V-A1 | `--no-gnss --initial-pose 0,0` | 地図だけでの追跡 | 地図区間の精度（横・縦・yaw）、照合の採用率、共分散の整合性（→ `lidar.cov_scale`）、処理時間、タイルの読み込み（ターゲットが空にならないこと）。建物の少ない場所・交差点・長い直線での誤差 |
| V-A2 | `--no-gnss --initial-pose E,Y --seed N`（E = 1〜3 m、Y = 10〜30°、N を変えて 20 回程度） | 地図上での初期化 | 成功率、初期化までの時間、誤った位置で初期化しないこと |
| V-A3 | `--gnss-off-box …` または `--gnss-off-time …` | GNSS 区間と地図区間の切り替わり | 区間の出入りで出力が飛ばないこと（出力の補正ステップ）、状態の推移（`GNSS_AIDED` ↔ `LIDAR_AIDED`）、GNSS に戻ったときの再アンカーの有無、地図の範囲外での `DEAD_RECKONING` |
| V-A4 | `--odom-scale 1.03〜1.05 --odom-noise 0.02` | ODOM を劣化させる | LiDAR の照合で位置が保たれること。`odom_scale` の推定が誤差の方へ動くこと |
| V-A5 | `--no-gnss --drop-lidar 60:10 …`（5〜30 s） | LiDAR を止める | デッドレコニングの誤差の伸び（走った距離に対する横・縦）、`dr_error_distance` での ERROR、再開後の照合または再位置推定での復帰、出力が飛ばないこと |
| V-A6 | V-A3 で、GNSS の範囲を地図の外に置く | 地図の無い区間を GNSS だけで走る | 地図グループの範囲の外に出たときのタイルの扱い、地図に戻ったときの照合 |

V-A3 と V-A6 の範囲は、経路と地図を上から見て決める（西新宿の地図は約 0.9 km × 1.1 km。地図の外は AWSIM の中にも無いので、V-A6 は「GNSS の範囲を広げ、地図の一部のタイルを外して作ったタイルの集合」で代わりにする）。

---

## 5. 評価の方法

### 5.1 真値

AWSIM の真値（`/awsim/ground_truth/vehicle/pose`）を UTM に直したもの（変換の `<out>_groundtruth.csv`）。誤差を含まないので、指標の分解能は推定ノードの出力の時刻の補間（100 Hz の真値の線形補間）で決まる。

### 5.2 指標（`tools/awsim/evaluate.py`）

| 指標 | 内容 | 対応する要件 |
|---|---|---|
| `lat_*`、`lon_*`、`xy_*` | 出力と真値の差を、真値の進行方向（縦）と横に分けた RMS・95 % 値・最大値 | FR-1、FR-3 |
| `yaw_*_deg` | yaw 誤差の RMS・95 % 値・最大値 | FR-3 |
| `step_max` | 出力の周期ごとの増分から真値の増分を引いたものの最大値（出力の補正ステップ） | **FR-4（飛びがないこと）** |
| `within_3sigma`、`nees_mean` | 誤差が報告した 3σ に入る割合、NEES の平均（x、y、yaw の対角だけ。整合していれば 3 前後） | 設計書 3.13 節の判定の前提 |
| `init_time` | 最初の出力から `INITIALIZING` を抜けるまでの時間 | 設計書 3.11 節 |
| `lidar_share`、`lost_share`、状態ごとの表 | 状態の時間の割合と、状態ごとの誤差 | 設計書 3.12・3.13 節 |
| `dr_distance_max` | 位置の観測なしで走った距離の最大値 | FR-8 |

`--from`・`--to` で評価する区間を絞れる（V-A3 の切り替わりの前後、V-A5 の欠落の間など）。`--max`・`--min` で基準を与えると、満たさないとき終了コード 1 を返す（CI の合成データの確認にも使っている）。

照合ごとのイベント（採用・棄却・再アンカー・再位置推定、GICP の品質指標と処理時間）の CSV（`debug_events_path`）は**未実装**。それまでは `~/debug/lidar_pose` と `/diagnostics` を bag に記録して代わりにする。

### 5.3 推定ノードが出すログ（CSV）

`gll_ros2` のパラメータ `debug_csv_path` を指定すると、出力を CSV に書く（変換で作るパラメータには `<out>_output.csv` が入っている）。1 行 = 1 出力周期。

| 列 | 内容 |
|---|---|
| `t` | 時刻（UNIX 秒。有効数字 15 桁） |
| `x, y, yaw` | 出力（出力整形後） |
| `raw_x, raw_y, raw_yaw` | フィルタの推定値 |
| `var_x, var_y, var_yaw` | 出力の共分散の対角（$`\Sigma_w + \mathbf{o}\mathbf{o}^\top`$） |
| `raw_var_x, raw_var_y, raw_var_yaw` | フィルタの共分散の対角 |
| `offset_x, offset_y, offset_yaw` | 出力整形のオフセット |
| `status, recovery_state, active_map_group` | 状態 |
| `roll, pitch, gyro_bias, odom_scale` | 補助推定とバイアス |
| `dr_distance` | 最後に位置の観測を採用してから走った距離 [m]（設計書 3.12 節） |

---

## 6. 合格基準とパラメータ

### 6.1 合格基準（仮）

シミュレーションなので、実データより厳しめの値から始め、初回の結果を見て見直す。

| 項目 | 基準（仮） |
|---|---|
| 水平位置の横方向誤差（地図の上の区間） | RMS < 0.05 m |
| yaw 誤差（地図の上の区間） | RMS < 0.5° |
| 出力の補正ステップ | 1 周期あたり < 0.02 m |
| 共分散の整合性 | 誤差が 3σ 以内に入る割合 > 95 % |
| 地図上での初期化 | 初期姿勢の誤差 3 m・30° 以内で成功率 > 95 %。誤った位置で初期化しない |
| 状態 | 地図の上の通常の走行で `LOST` にならない |
| デッドレコニングの監視 | `dr_distance` が `dr_error_distance` を超えてから 1 出力周期以内に ERROR、照合を採用したら解除 |
| 処理時間 | GICP < 60 ms（p99） |

### 6.2 このデータ用のパラメータ（初期値）

変換が `<out>_params.yaml` に入れる値と、手で見直す値。

| パラメータ | 既定値 | AWSIM 用 | 理由 |
|---|---|---|---|
| `gnss.utm_zone` / `utm_north` | 54 / true | 54 / true | 西新宿（1 章） |
| `gnss.lever_arm` | [0, 0, 0] | [0, 0, 1.5] | 変換で作る GNSS のアンテナの位置（`--gnss-lever`） |
| `imu.rotation_rpy_deg` | [0, 0, 0] | [0, 0, 0] | 変換で base_link の向きに回してある |
| `attitude.static_init_time` | 3.0 s | 最初の停止 − 0.5 s（0.2〜3 s） | 変換が表示・設定する |
| `lidar.extrinsic_xyz` / `extrinsic_rpy_deg` | 0 | 3.4 節で確かめた値 | `--lidar-extrinsic` |
| `lidar.base_link_height` | 0 m | 0 m | AWSIM の base_link は後輪軸の地面の高さ |
| `lidar.cov_scale` | 0.15 | 0.15（まずこのまま） | V-A1 の共分散の整合性で決める |
| `motion.imu_timeout` | 0.1 s | 0.1 s（まずこのまま） | IMU が 30 Hz（33 ms 間隔）なので足りる。記録で間隔が空くときは見直す |
| `map.config_path` | "" | `<out>_maps.yaml` | `--tiles` |
| `debug_csv_path` | "" | `<out>_output.csv` | 評価用 |

---

## 7. 合成データでの確認（CI）

AWSIM の bag と同じトピック・型の小さな合成データ（壁と建物のある 70 m × 50 m の場所を、四角い経路で 1 周する）で、スクリプトと推定ノードのつながりを CI で確かめている。

- `tools_awsim` ジョブ: 経路追従・回転・評価の単体テスト、LiDAR の取り付け位置の探索（ずらした値から、合成したときの値に戻ること）、変換（IMU の向き、GNSS の緯度経度と status、ODOM、点群、パラメータ）。
- `awsim_humble` ジョブ: Humble のコンテナで、`awsim_drive.py` が AWSIM の車の代わり（指令で自転車モデルを動かして真値を出す）を経路の終わりまで走らせること、MCAP で記録できること。
- `ros2` ジョブ: 変換した bag を ROS 2 Jazzy の rosbag2 で読み、推定ノードに通して真値と比べる（GNSS + LiDAR と、地図だけでの初期化と追跡の 2 通り）。

---

## 8. AWSIM で確かめられないこと

次の項目は、自社の車両で記録したデータで検証する（その計画は別に作る）。

- **実際のセンサの性質**: Livox Mid-360 の走査パターンと点の雑音、点ごとの時刻とデスキュー、IMU の雑音・バイアス・取り付けのずれ、車輪のスリップと ODOM の縮尺の変化
- **リアルタイムの RTK-GNSS の振る舞い**: F9P の FIX の出入り、FIX なのに位置が飛ぶ、ドライバの出力の形式と遅れ（変換で作る GNSS は理想的な値）
- **地図と現実の食い違い**: 地図を作った後の環境の変化、地図の歪みとアンカーの誤差（AWSIM では地図と真値の座標系が完全に一致している）
- **`dr_error_distance` の最終的な値**（V-A5 の誤差の伸びは参考値）

---

## 9. 未決事項

1. **西新宿の地図のライセンス**: CC BY-NC 4.0（非営利に限る）。業務の検証に使ってよいかを確認する。使えない場合は、ライセンスの合う別の地図か、AWSIM の上で自分で作った地図（Unity のシーンの点群）を使う。
2. **LiDAR の取り付け位置**: AWSIM の車両の設定から確かめる（3.4 節の探索の結果と比べる）。
3. **IMU の重力**: AWSIM の IMU の加速度に重力が入っているか。変換はどちらでも扱える（入っていなければ足す）が、記録を見て確かめる。
4. **`debug_events_path` の実装**（5.2 節。V-A1 の前に）。
5. **V-A6 の作り方**（地図の一部を外したタイルの集合を作るスクリプト）。
