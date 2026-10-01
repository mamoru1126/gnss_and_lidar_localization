# v_a1_vgicp

- 成功
- シナリオ v_a1、録った bag nsj_run1、変換のオプション `--no-gnss --initial-pose 0,0 --set lidar.registration=vgicp`
- commit 49d4fac +変更あり、2026-10-01 03:19:47

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
パラメータ lidar.registration = 'vgicp'
```

## 評価

| 指標 | 値 |
|---|---|
| duration | 295.4 s |
| rows | 14876 |
| evaluated_rows | 14876 |
| init_time | 0.0 s |
| lat_rms | 0.035 m |
| lat_p95 | 0.064 m |
| lat_max | 0.235 m |
| lon_rms | 0.073 m |
| lon_p95 | 0.168 m |
| lon_max | 0.293 m |
| xy_rms | 0.081 m |
| xy_p95 | 0.178 m |
| xy_max | 0.351 m |
| yaw_rms_deg | 0.06° |
| yaw_p95_deg | 0.14° |
| yaw_max_deg | 0.34° |
| step_max | 0.224 m |
| within_3sigma | 80.2% |
| nees_mean | 8.26 |
| lidar_share | 93.1% |
| lost_share | 2.9% |
| dr_distance_max | 82.748 m |

| 状態 | 時間の割合 | 横 RMS | 縦 RMS | yaw RMS |
|---|---|---|---|---|
| DEAD_RECKONING | 4.0% | 0.075 m | 0.116 m | 0.10° |
| LIDAR_AIDED | 93.1% | 0.020 m | 0.068 m | 0.06° |
| LOST | 2.9% | 0.148 m | 0.133 m | 0.13° |

## 10 秒ごと（raw の姿勢の誤差。|yaw レート| は真値の最大）

| 時刻 [s] | 状態 | 横 RMS [m] | 縦 RMS [m] | yaw RMS [°] | yaw レート [rad/s] |
|---|---|---|---|---|---|
| 0 | LIDAR_AIDED | 0.001 | 0.002 | 0.05 | 0.00 |
| 10 | LIDAR_AIDED | 0.001 | 0.002 | 0.05 | 0.00 |
| 20 | LIDAR_AIDED | 0.001 | 0.002 | 0.05 | 0.00 |
| 30 | LIDAR_AIDED | 0.001 | 0.002 | 0.05 | 0.00 |
| 40 | LIDAR_AIDED | 0.001 | 0.002 | 0.05 | 0.00 |
| 50 | LIDAR_AIDED | 0.008 | 0.058 | 0.09 | 0.31 |
| 60 | LIDAR_AIDED | 0.008 | 0.086 | 0.06 | 0.09 |
| 70 | LIDAR_AIDED | 0.007 | 0.050 | 0.04 | 0.01 |
| 80 | LIDAR_AIDED | 0.014 | 0.032 | 0.09 | 0.29 |
| 90 | LIDAR_AIDED | 0.035 | 0.059 | 0.05 | 0.30 |
| 100 | LIDAR_AIDED | 0.015 | 0.040 | 0.03 | 0.00 |
| 110 | LIDAR_AIDED | 0.022 | 0.066 | 0.06 | 0.29 |
| 120 | LIDAR_AIDED | 0.016 | 0.110 | 0.07 | 0.29 |
| 130 | LIDAR_AIDED | 0.008 | 0.042 | 0.03 | 0.00 |
| 140 | LIDAR_AIDED | 0.016 | 0.031 | 0.04 | 0.00 |
| 150 | LIDAR_AIDED | 0.010 | 0.031 | 0.02 | 0.02 |
| 160 | LIDAR_AIDED | 0.011 | 0.044 | 0.08 | 0.16 |
| 170 | LIDAR_AIDED | 0.022 | 0.047 | 0.10 | 0.36 |
| 180 | LIDAR_AIDED | 0.014 | 0.060 | 0.07 | 0.17 |
| 190 | LIDAR_AIDED | 0.010 | 0.037 | 0.04 | 0.06 |
| 200 | LIDAR_AIDED | 0.016 | 0.038 | 0.05 | 0.08 |
| 210 | LIDAR_AIDED | 0.043 | 0.102 | 0.13 | 0.36 |
| 220 | DEAD_RECKONING 62% / LOST 38% | 0.135 | 0.166 | 0.13 | 0.00 |
| 230 | LIDAR_AIDED 90% / LOST 10% | 0.071 | 0.156 | 0.06 | 0.00 |
| 240 | LIDAR_AIDED | 0.009 | 0.099 | 0.03 | 0.00 |
| 250 | LIDAR_AIDED | 0.006 | 0.043 | 0.03 | 0.00 |
| 260 | DEAD_RECKONING 56% / LIDAR_AIDED 44% | 0.024 | 0.032 | 0.05 | 0.00 |
| 270 | LIDAR_AIDED 60% / LOST 40% | 0.044 | 0.057 | 0.05 | 0.00 |
| 280 | LIDAR_AIDED | 0.005 | 0.074 | 0.06 | 0.00 |
| 290 | LIDAR_AIDED | 0.001 | 0.161 | 0.05 | 0.00 |

## 推定ノードのログ

照合: `LiDAR matching summary (vgicp): 2483 scans matched, 2248 accepted, 235 rejected by quality, time mean 18.6 ms / max 68.5 ms`

状態の移り変わり 7 回（時刻は初期化からの秒）:

```
     0.0  INITIALIZING -> LIDAR_AIDED
   220.2  LIDAR_AIDED -> DEAD_RECKONING
   226.3  DEAD_RECKONING -> LOST
   231.0  LOST -> LIDAR_AIDED
   264.4  LIDAR_AIDED -> DEAD_RECKONING
   270.3  DEAD_RECKONING -> LOST
   274.2  LOST -> LIDAR_AIDED
```

警告・エラー（数字を # にまとめた種類ごと）:

| 回数 | 最初（s） | 内容 |
|---|---|---|
| 4 | 221.7 | `WARN LiDAR matching failed repeatedly (last: LOW_OVERLAP, inlier #, overlap #): relocalizing around the estimate` |
| 4 | 221.8 | `WARN relocalization failed (low overlap): # hypotheses, overlap # (second #), # ms` |
| 2 | 223.4 | `ERROR dead reckoning for # m without GNSS / LiDAR position (limit # m)` |
| 2 | 224.0 | `WARN LiDAR matching failed repeatedly (last: FEW_INLIERS, inlier #, overlap #): relocalizing around the estimate` |
| 2 | 224.2 | `WARN relocalization failed (low inlier ratio): # hypotheses, overlap # (second #), # ms` |
| 1 | -3.6 | `WARN point cloud has no per-point time field: deskew is disabled` |
