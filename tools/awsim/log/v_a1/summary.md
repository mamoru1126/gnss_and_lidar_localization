# v_a1

- 成功
- シナリオ v_a1、録った bag nsj_run1、変換のオプション `--no-gnss --initial-pose 0,0`
- commit 2b3cd60 +変更あり、2026-09-30 11:05:14

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
全体の地図 /map/points: 460663 点（1.0 m で間引き、frame map_local）
```

## 評価

| 指標 | 値 |
|---|---|
| duration | 295.3 s |
| rows | 14873 |
| evaluated_rows | 14873 |
| init_time | 0.0 s |
| lat_rms | 0.026 m |
| lat_p95 | 0.057 m |
| lat_max | 0.163 m |
| lon_rms | 0.066 m |
| lon_p95 | 0.142 m |
| lon_max | 0.304 m |
| xy_rms | 0.071 m |
| xy_p95 | 0.153 m |
| xy_max | 0.304 m |
| yaw_rms_deg | 0.06° |
| yaw_p95_deg | 0.12° |
| yaw_max_deg | 0.36° |
| step_max | 0.218 m |
| within_3sigma | 77.4% |
| nees_mean | 8.00 |
| lidar_share | 93.0% |
| lost_share | 2.7% |
| dr_distance_max | 81.973 m |

| 状態 | 時間の割合 | 横 RMS | 縦 RMS | yaw RMS |
|---|---|---|---|---|
| DEAD_RECKONING | 4.2% | 0.055 m | 0.093 m | 0.07° |
| LIDAR_AIDED | 93.0% | 0.016 m | 0.063 m | 0.06° |
| LOST | 2.7% | 0.109 m | 0.105 m | 0.10° |

## 10 秒ごと（raw の姿勢の誤差。|yaw レート| は真値の最大）

| 時刻 [s] | 状態 | 横 RMS [m] | 縦 RMS [m] | yaw RMS [°] | yaw レート [rad/s] |
|---|---|---|---|---|---|
| 0 | LIDAR_AIDED | 0.001 | 0.001 | 0.05 | 0.00 |
| 10 | LIDAR_AIDED | 0.001 | 0.001 | 0.04 | 0.00 |
| 20 | LIDAR_AIDED | 0.001 | 0.001 | 0.04 | 0.00 |
| 30 | LIDAR_AIDED | 0.001 | 0.001 | 0.05 | 0.00 |
| 40 | LIDAR_AIDED | 0.001 | 0.001 | 0.05 | 0.00 |
| 50 | LIDAR_AIDED | 0.008 | 0.056 | 0.09 | 0.31 |
| 60 | LIDAR_AIDED | 0.008 | 0.078 | 0.06 | 0.08 |
| 70 | LIDAR_AIDED | 0.007 | 0.053 | 0.04 | 0.01 |
| 80 | LIDAR_AIDED | 0.013 | 0.031 | 0.09 | 0.30 |
| 90 | LIDAR_AIDED | 0.031 | 0.057 | 0.05 | 0.29 |
| 100 | LIDAR_AIDED | 0.012 | 0.036 | 0.03 | 0.00 |
| 110 | LIDAR_AIDED | 0.019 | 0.059 | 0.06 | 0.29 |
| 120 | LIDAR_AIDED | 0.014 | 0.105 | 0.07 | 0.29 |
| 130 | LIDAR_AIDED | 0.009 | 0.042 | 0.04 | 0.00 |
| 140 | LIDAR_AIDED | 0.011 | 0.031 | 0.04 | 0.00 |
| 150 | LIDAR_AIDED | 0.008 | 0.035 | 0.03 | 0.02 |
| 160 | LIDAR_AIDED | 0.011 | 0.045 | 0.08 | 0.16 |
| 170 | LIDAR_AIDED | 0.022 | 0.047 | 0.10 | 0.36 |
| 180 | LIDAR_AIDED | 0.014 | 0.056 | 0.07 | 0.17 |
| 190 | LIDAR_AIDED | 0.009 | 0.035 | 0.04 | 0.06 |
| 200 | LIDAR_AIDED | 0.016 | 0.035 | 0.05 | 0.08 |
| 210 | LIDAR_AIDED | 0.039 | 0.091 | 0.12 | 0.36 |
| 220 | DEAD_RECKONING 64% / LOST 36% | 0.093 | 0.132 | 0.10 | 0.00 |
| 230 | LIDAR_AIDED 91% / LOST 9% | 0.048 | 0.137 | 0.05 | 0.00 |
| 240 | LIDAR_AIDED | 0.008 | 0.113 | 0.03 | 0.00 |
| 250 | LIDAR_AIDED | 0.006 | 0.041 | 0.03 | 0.00 |
| 260 | DEAD_RECKONING 57% / LIDAR_AIDED 43% | 0.025 | 0.032 | 0.05 | 0.00 |
| 270 | LIDAR_AIDED 60% / LOST 40% | 0.044 | 0.056 | 0.05 | 0.00 |
| 280 | LIDAR_AIDED | 0.005 | 0.074 | 0.06 | 0.00 |
| 290 | LIDAR_AIDED | 0.002 | 0.117 | 0.05 | 0.00 |

## 推定ノードのログ

状態の移り変わり 7 回（時刻は初期化からの秒）:

```
     0.0  INITIALIZING -> LIDAR_AIDED
   220.2  LIDAR_AIDED -> DEAD_RECKONING
   226.5  DEAD_RECKONING -> LOST
   230.9  LOST -> LIDAR_AIDED
   264.3  LIDAR_AIDED -> DEAD_RECKONING
   270.6  DEAD_RECKONING -> LOST
   274.4  LOST -> LIDAR_AIDED
```

警告・エラー（数字を # にまとめた種類ごと）:

| 回数 | 最初（s） | 内容 |
|---|---|---|
| 4 | 221.9 | `WARN LiDAR matching failed repeatedly (last: LOW_OVERLAP, inlier #, overlap #): relocalizing around the estimate` |
| 4 | 222.1 | `WARN relocalization failed (low overlap): # hypotheses, overlap # (second #), # ms` |
| 2 | 223.5 | `ERROR dead reckoning for # m without GNSS / LiDAR position (limit # m)` |
| 2 | 224.2 | `WARN LiDAR matching failed repeatedly (last: FEW_INLIERS, inlier #, overlap #): relocalizing around the estimate` |
| 2 | 224.4 | `WARN relocalization failed (low inlier ratio): # hypotheses, overlap # (second #), # ms` |
| 1 | -3.4 | `WARN point cloud has no per-point time field: deskew is disabled` |
