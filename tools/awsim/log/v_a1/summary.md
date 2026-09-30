# v_a1

- 成功
- シナリオ v_a1、録った bag nsj_run1、変換のオプション `--no-gnss --initial-pose 0,0`
- commit 35a59c1 +変更あり、2026-09-30 10:43:43

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
| duration | 295.6 s |
| rows | 14884 |
| evaluated_rows | 14884 |
| init_time | 0.0 s |
| lat_rms | 0.369 m |
| lat_p95 | 0.971 m |
| lat_max | 1.165 m |
| lon_rms | 0.288 m |
| lon_p95 | 0.779 m |
| lon_max | 1.081 m |
| xy_rms | 0.468 m |
| xy_p95 | 1.207 m |
| xy_max | 1.525 m |
| yaw_rms_deg | 0.08° |
| yaw_p95_deg | 0.17° |
| yaw_max_deg | 0.45° |
| step_max | 0.225 m |
| within_3sigma | 82.3% |
| nees_mean | 35.38 |
| lidar_share | 72.9% |
| lost_share | 8.0% |
| dr_distance_max | 81.914 m |

| 状態 | 時間の割合 | 横 RMS | 縦 RMS | yaw RMS |
|---|---|---|---|---|
| DEAD_RECKONING | 4.7% | 0.591 m | 0.162 m | 0.10° |
| DEGRADED | 14.4% | 0.654 m | 0.525 m | 0.13° |
| LIDAR_AIDED | 72.9% | 0.078 m | 0.064 m | 0.06° |
| LOST | 8.0% | 0.819 m | 0.699 m | 0.15° |

## 10 秒ごと（raw の姿勢の誤差。|yaw レート| は真値の最大）

| 時刻 [s] | 状態 | 横 RMS [m] | 縦 RMS [m] | yaw RMS [°] | yaw レート [rad/s] |
|---|---|---|---|---|---|
| 0 | LIDAR_AIDED | 0.001 | 0.001 | 0.05 | 0.00 |
| 10 | LIDAR_AIDED | 0.001 | 0.001 | 0.05 | 0.00 |
| 20 | LIDAR_AIDED | 0.001 | 0.001 | 0.05 | 0.00 |
| 30 | LIDAR_AIDED | 0.001 | 0.001 | 0.05 | 0.00 |
| 40 | LIDAR_AIDED | 0.001 | 0.001 | 0.05 | 0.00 |
| 50 | LIDAR_AIDED 84% / DEAD_RECKONING 16% | 0.300 | 0.045 | 0.07 | 0.31 |
| 60 | DEGRADED 67% / LIDAR_AIDED 33% | 0.042 | 0.127 | 0.11 | 0.11 |
| 70 | LIDAR_AIDED | 0.010 | 0.054 | 0.04 | 0.01 |
| 80 | LIDAR_AIDED | 0.091 | 0.030 | 0.07 | 0.29 |
| 90 | DEGRADED 83% / DEAD_RECKONING 17% | 0.342 | 0.095 | 0.10 | 0.30 |
| 100 | LIDAR_AIDED 56% / DEGRADED 44% | 0.028 | 0.043 | 0.04 | 0.00 |
| 110 | LIDAR_AIDED 88% / DEAD_RECKONING 12% | 0.288 | 0.069 | 0.07 | 0.29 |
| 120 | DEGRADED 69% / LOST 31% | 0.206 | 0.144 | 0.24 | 0.29 |
| 130 | LIDAR_AIDED 77% / DEGRADED 23% | 0.009 | 0.034 | 0.04 | 0.00 |
| 140 | LIDAR_AIDED | 0.011 | 0.034 | 0.04 | 0.00 |
| 150 | LIDAR_AIDED | 0.022 | 0.030 | 0.02 | 0.02 |
| 160 | LIDAR_AIDED | 0.061 | 0.041 | 0.10 | 0.16 |
| 170 | LIDAR_AIDED 63% / LOST 37% | 0.413 | 0.139 | 0.13 | 0.36 |
| 180 | DEGRADED 68% / LOST 32% | 0.072 | 0.061 | 0.13 | 0.17 |
| 190 | LIDAR_AIDED 96% / DEGRADED 4% | 0.029 | 0.035 | 0.04 | 0.06 |
| 200 | LIDAR_AIDED | 0.030 | 0.038 | 0.08 | 0.04 |
| 210 | LOST 69% / LIDAR_AIDED 31% | 0.380 | 0.142 | 0.13 | 0.36 |
| 220 | DEGRADED 65% / LOST 35% | 0.085 | 0.113 | 0.06 | 0.00 |
| 230 | LIDAR_AIDED 88% / LOST 12% | 0.045 | 0.103 | 0.05 | 0.00 |
| 240 | LIDAR_AIDED | 0.008 | 0.096 | 0.03 | 0.00 |
| 250 | LIDAR_AIDED | 0.006 | 0.041 | 0.03 | 0.00 |
| 260 | DEAD_RECKONING 55% / LIDAR_AIDED 45% | 0.023 | 0.042 | 0.05 | 0.00 |
| 270 | LIDAR_AIDED 60% / LOST 40% | 0.046 | 0.049 | 0.05 | 0.00 |
| 280 | LIDAR_AIDED | 0.006 | 0.066 | 0.06 | 0.00 |
| 290 | LIDAR_AIDED | 0.002 | 0.118 | 0.05 | 0.00 |

## 推定ノードのログ

状態の移り変わり 34 回（時刻は初期化からの秒）:

```
     0.0  INITIALIZING -> LIDAR_AIDED
    57.3  LIDAR_AIDED -> DEAD_RECKONING
    58.7  DEAD_RECKONING -> DEGRADED
    66.7  DEGRADED -> LIDAR_AIDED
    90.3  LIDAR_AIDED -> DEAD_RECKONING
    91.9  DEAD_RECKONING -> DEGRADED
    94.9  DEGRADED -> LOST
    95.0  LOST -> DEGRADED
    95.1  DEGRADED -> LOST
    95.1  LOST -> DEGRADED
    95.1  DEGRADED -> LOST
    95.4  LOST -> DEGRADED
    95.4  DEGRADED -> LOST
    95.5  LOST -> DEGRADED
   104.4  DEGRADED -> LIDAR_AIDED
   118.8  LIDAR_AIDED -> DEAD_RECKONING
   120.3  DEAD_RECKONING -> DEGRADED
   120.9  DEGRADED -> LOST
   121.0  LOST -> DEGRADED
   121.0  DEGRADED -> LOST
   124.0  LOST -> DEGRADED
   132.3  DEGRADED -> LIDAR_AIDED
   175.3  LIDAR_AIDED -> DEAD_RECKONING
   176.9  DEAD_RECKONING -> LOST
   183.2  LOST -> DEGRADED
   190.4  DEGRADED -> LIDAR_AIDED
   212.6  LIDAR_AIDED -> DEAD_RECKONING
   214.0  DEAD_RECKONING -> LOST
   219.9  LOST -> DEGRADED
   226.5  DEGRADED -> LOST
   231.1  LOST -> LIDAR_AIDED
   264.5  LIDAR_AIDED -> DEAD_RECKONING
   271.0  DEAD_RECKONING -> LOST
   274.6  LOST -> LIDAR_AIDED
```

警告・エラー（数字を # にまとめた種類ごと）:

| 回数 | 最初（s） | 内容 |
|---|---|---|
| 8 | 176.7 | `WARN LiDAR matching failed repeatedly: relocalizing around the estimate` |
| 5 | 58.7 | `WARN re-anchored to relocalization on awsim (d#=#)` |
| 5 | 58.7 | `WARN relocalized: # hypotheses, overlap # (second #), # ms` |
| 4 | 221.9 | `WARN relocalization failed (low overlap): # hypotheses, overlap # (second #), # ms` |
| 3 | 58.5 | `WARN LiDAR rejected repeatedly: relocalizing around the estimate` |
| 2 | 223.6 | `ERROR dead reckoning for # m without GNSS / LiDAR position (limit # m)` |
| 2 | 224.5 | `WARN relocalization failed (low inlier ratio): # hypotheses, overlap # (second #), # ms` |
| 1 | -3.6 | `WARN point cloud has no per-point time field: deskew is disabled` |
