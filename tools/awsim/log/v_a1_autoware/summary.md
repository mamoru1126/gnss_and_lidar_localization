# v_a1_autoware

- 成功
- シナリオ v_a1、録った bag nsj_run1、変換のオプション `--no-gnss --initial-pose 0,0 --set lidar.overlap_near_distance=1.0 lidar.min_inlier_ratio=0.45 lidar.min_stddev_lon=0.06 relocalize.lost_on_failure=false`
- commit d099dde +変更あり、2026-10-02 05:28:53

## LiDAR の取り付け位置・スタンプのずれ

```
LiDAR: --lidar-extrinsic 0.900,0.000,2.040,0.900,0.100,88.200 --lidar-stamp-offset 0.000（/ws/src/gnss_and_lidar_localization/data/awsim/out/nsj_run1_lidar_calib.env）
```

## 変換

```
真値 17953（60 Hz）、IMU 8977、速度 8977、点群 2992（frame velodyne_top、列 ['x', 'y', 'z', 'intensity', 'channel']、27451 点）、299.2 s
最初の停止 54.8 s → attitude.static_init_time = 3.0 s
IMU（tamagawa/imu_link）→ base_link の向き: rpy [-179.92   -1.91 -179.67] deg、当てはめの残差 加速度 6.759 m/s²・角速度 0.0053 rad/s、重力 あり・向きが逆（AWSIM。直して出す）
車速と真値の比: 前後 +0.999、yaw レート -1.163（yaw レートの符号が逆。直して出す）
IMU の yaw レートと真値の比 +1.000（+1 に近いはず）
時刻のずれ（真値に合わせるためにスタンプに足す値）: IMU -18 ms、車速 -68 ms（yaw レートで見ると +176 ms）（直して出す）
車速の横向きの速さ: yaw レートに比例する分 +1.140 m ×（yaw レート）を取り除く（真値との差の RMS 0.093 → 0.011 m/s）。yaw レートは IMU の値にする
全体の地図 /map/points: 460663 点（1.0 m で間引き、frame map_local）
パラメータ lidar.overlap_near_distance = 1.0
パラメータ lidar.min_inlier_ratio = 0.45
パラメータ lidar.min_stddev_lon = 0.06
パラメータ relocalize.lost_on_failure = False
```

## 評価

| 指標 | 値 |
|---|---|
| duration | 295.4 s |
| rows | 14875 |
| evaluated_rows | 14875 |
| init_time | 0.0 s |
| lat_rms | 0.014 m |
| lat_p95 | 0.034 m |
| lat_max | 0.060 m |
| lon_rms | 0.060 m |
| lon_p95 | 0.126 m |
| lon_max | 0.265 m |
| xy_rms | 0.061 m |
| xy_p95 | 0.126 m |
| xy_max | 0.265 m |
| yaw_rms_deg | 0.06° |
| yaw_p95_deg | 0.11° |
| yaw_max_deg | 0.34° |
| step_max | 0.230 m |
| within_3sigma | 79.4% |
| nees_mean | 7.32 |
| lidar_share | 100.0% |
| lost_share | 0.0% |
| dr_distance_max | 6.025 m |

| 状態 | 時間の割合 | 横 RMS | 縦 RMS | yaw RMS |
|---|---|---|---|---|
| LIDAR_AIDED | 100.0% | 0.014 m | 0.060 m | 0.06° |

## 10 秒ごと（raw の姿勢の誤差。|yaw レート| は真値の最大）

| 時刻 [s] | 状態 | 横 RMS [m] | 縦 RMS [m] | yaw RMS [°] | yaw レート [rad/s] |
|---|---|---|---|---|---|
| 0 | LIDAR_AIDED | 0.001 | 0.001 | 0.05 | 0.00 |
| 10 | LIDAR_AIDED | 0.001 | 0.001 | 0.05 | 0.00 |
| 20 | LIDAR_AIDED | 0.001 | 0.001 | 0.05 | 0.00 |
| 30 | LIDAR_AIDED | 0.001 | 0.001 | 0.05 | 0.00 |
| 40 | LIDAR_AIDED | 0.001 | 0.001 | 0.05 | 0.00 |
| 50 | LIDAR_AIDED | 0.010 | 0.067 | 0.09 | 0.31 |
| 60 | LIDAR_AIDED | 0.008 | 0.084 | 0.06 | 0.08 |
| 70 | LIDAR_AIDED | 0.007 | 0.055 | 0.04 | 0.01 |
| 80 | LIDAR_AIDED | 0.013 | 0.029 | 0.09 | 0.30 |
| 90 | LIDAR_AIDED | 0.030 | 0.063 | 0.05 | 0.29 |
| 100 | LIDAR_AIDED | 0.012 | 0.039 | 0.03 | 0.00 |
| 110 | LIDAR_AIDED | 0.021 | 0.064 | 0.06 | 0.29 |
| 120 | LIDAR_AIDED | 0.016 | 0.102 | 0.07 | 0.29 |
| 130 | LIDAR_AIDED | 0.010 | 0.049 | 0.03 | 0.00 |
| 140 | LIDAR_AIDED | 0.010 | 0.034 | 0.04 | 0.00 |
| 150 | LIDAR_AIDED | 0.008 | 0.033 | 0.03 | 0.02 |
| 160 | LIDAR_AIDED | 0.010 | 0.045 | 0.08 | 0.16 |
| 170 | LIDAR_AIDED | 0.022 | 0.052 | 0.10 | 0.36 |
| 180 | LIDAR_AIDED | 0.013 | 0.071 | 0.07 | 0.17 |
| 190 | LIDAR_AIDED | 0.009 | 0.036 | 0.04 | 0.06 |
| 200 | LIDAR_AIDED | 0.015 | 0.034 | 0.04 | 0.08 |
| 210 | LIDAR_AIDED | 0.037 | 0.093 | 0.11 | 0.36 |
| 220 | LIDAR_AIDED | 0.014 | 0.068 | 0.03 | 0.00 |
| 230 | LIDAR_AIDED | 0.004 | 0.053 | 0.03 | 0.00 |
| 240 | LIDAR_AIDED | 0.008 | 0.092 | 0.03 | 0.00 |
| 250 | LIDAR_AIDED | 0.006 | 0.045 | 0.03 | 0.00 |
| 260 | LIDAR_AIDED | 0.014 | 0.040 | 0.05 | 0.00 |
| 270 | LIDAR_AIDED | 0.014 | 0.051 | 0.02 | 0.00 |
| 280 | LIDAR_AIDED | 0.005 | 0.081 | 0.06 | 0.00 |
| 290 | LIDAR_AIDED | 0.002 | 0.112 | 0.05 | 0.00 |

## LiDAR の照合（2455 回。出力の CSV から）

| 結果 | 回数 | inlier 中央値 | overlap 中央値 | overlap（地図の近く）中央値 |
|---|---|---|---|---|
| ACCEPTED | 2429 | 0.91 | 0.78 | 0.90 |
| NOT_CONVERGED | 25 | 0.83 | 0.61 | 0.84 |
| INITIALIZED | 1 | 1.00 | 0.98 | 0.00 |

## 推定ノードのログ

照合: `LiDAR matching summary (gicp): 2454 scans matched, 2429 accepted, 25 rejected by quality, time mean 15.9 ms / max 43.0 ms`

状態の移り変わり 1 回（時刻は初期化からの秒）:

```
     0.0  INITIALIZING -> LIDAR_AIDED
```

警告・エラー（数字を # にまとめた種類ごと）:

| 回数 | 最初（s） | 内容 |
|---|---|---|
| 1 | -3.4 | `WARN point cloud has no per-point time field: deskew is disabled` |
