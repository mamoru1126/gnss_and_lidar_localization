# v_a1_vgicp

- 成功
- シナリオ lidar.registration=vgicp、録った bag nsj_run1、変換のオプション `（なし）`
- commit 30e59b6、2026-10-01 02:52:30

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
GNSS: 2992 件（RTK-FIX 2992、外した 0）、10 Hz、σ 0.02 m
全体の地図 /map/points: 460663 点（1.0 m で間引き、frame map_local）
```

## 評価

| 指標 | 値 |
|---|---|
| duration | 243.4 s |
| rows | 12276 |
| evaluated_rows | 12262 |
| init_time | 0.3 s |
| lat_rms | 0.010 m |
| lat_p95 | 0.020 m |
| lat_max | 0.042 m |
| lon_rms | 0.151 m |
| lon_p95 | 0.220 m |
| lon_max | 0.394 m |
| xy_rms | 0.151 m |
| xy_p95 | 0.221 m |
| xy_max | 0.394 m |
| yaw_rms_deg | 0.10° |
| yaw_p95_deg | 0.23° |
| yaw_max_deg | 0.70° |
| step_max | 0.218 m |
| within_3sigma | 6.3% |
| nees_mean | 232.17 |
| lidar_share | 83.1% |
| lost_share | 0.0% |
| dr_distance_max | 2.773 m |

| 状態 | 時間の割合 | 横 RMS | 縦 RMS | yaw RMS |
|---|---|---|---|---|
| GNSS_AIDED | 16.9% | 0.010 m | 0.158 m | 0.07° |
| GNSS_LIDAR_AIDED | 83.1% | 0.010 m | 0.150 m | 0.11° |

## 10 秒ごと（raw の姿勢の誤差。|yaw レート| は真値の最大）

| 時刻 [s] | 状態 | 横 RMS [m] | 縦 RMS [m] | yaw RMS [°] | yaw レート [rad/s] |
|---|---|---|---|---|---|
| 0 | GNSS_LIDAR_AIDED 97% / INITIALIZING 3% | 0.015 | 0.096 | 0.41 | 0.31 |
| 10 | GNSS_LIDAR_AIDED 99% / GNSS_AIDED 1% | 0.007 | 0.170 | 0.04 | 0.01 |
| 20 | GNSS_LIDAR_AIDED | 0.007 | 0.115 | 0.03 | 0.01 |
| 30 | GNSS_LIDAR_AIDED | 0.007 | 0.134 | 0.16 | 0.30 |
| 40 | GNSS_LIDAR_AIDED | 0.012 | 0.160 | 0.10 | 0.23 |
| 50 | GNSS_LIDAR_AIDED 77% / GNSS_AIDED 23% | 0.009 | 0.188 | 0.06 | 0.00 |
| 60 | GNSS_LIDAR_AIDED | 0.010 | 0.122 | 0.13 | 0.29 |
| 70 | GNSS_LIDAR_AIDED 99% / GNSS_AIDED 1% | 0.009 | 0.167 | 0.05 | 0.19 |
| 80 | GNSS_LIDAR_AIDED 97% / GNSS_AIDED 3% | 0.007 | 0.164 | 0.04 | 0.00 |
| 90 | GNSS_LIDAR_AIDED 75% / GNSS_AIDED 25% | 0.009 | 0.174 | 0.05 | 0.01 |
| 100 | GNSS_LIDAR_AIDED 94% / GNSS_AIDED 6% | 0.010 | 0.160 | 0.05 | 0.02 |
| 110 | GNSS_LIDAR_AIDED 82% / GNSS_AIDED 18% | 0.010 | 0.179 | 0.11 | 0.16 |
| 120 | GNSS_LIDAR_AIDED 89% / GNSS_AIDED 11% | 0.010 | 0.158 | 0.22 | 0.36 |
| 130 | GNSS_LIDAR_AIDED 97% / GNSS_AIDED 3% | 0.010 | 0.172 | 0.11 | 0.17 |
| 140 | GNSS_LIDAR_AIDED 97% / GNSS_AIDED 3% | 0.013 | 0.154 | 0.19 | 0.06 |
| 150 | GNSS_LIDAR_AIDED 94% / GNSS_AIDED 6% | 0.007 | 0.165 | 0.12 | 0.33 |
| 160 | GNSS_LIDAR_AIDED 77% / GNSS_AIDED 23% | 0.015 | 0.144 | 0.15 | 0.36 |
| 170 | GNSS_AIDED 90% / GNSS_LIDAR_AIDED 10% | 0.010 | 0.164 | 0.06 | 0.00 |
| 180 | GNSS_LIDAR_AIDED | 0.011 | 0.171 | 0.05 | 0.00 |
| 190 | GNSS_LIDAR_AIDED 98% / GNSS_AIDED 2% | 0.011 | 0.159 | 0.08 | 0.00 |
| 200 | GNSS_LIDAR_AIDED 99% / GNSS_AIDED 1% | 0.010 | 0.135 | 0.04 | 0.00 |
| 210 | GNSS_AIDED 76% / GNSS_LIDAR_AIDED 24% | 0.011 | 0.173 | 0.07 | 0.00 |
| 220 | GNSS_LIDAR_AIDED 72% / GNSS_AIDED 28% | 0.009 | 0.135 | 0.25 | 0.00 |
| 230 | GNSS_LIDAR_AIDED 63% / GNSS_AIDED 37% | 0.002 | 0.024 | 0.06 | 0.00 |
| 240 | GNSS_AIDED | 0.004 | 0.032 | 0.08 | 0.00 |

## 推定ノードのログ

照合: `LiDAR matching summary (gicp): 2305 scans matched, 1060 accepted, 245 rejected by quality, time mean 21.3 ms / max 44.4 ms`

状態の移り変わり 50 回（時刻は初期化からの秒）:

```
     0.3  INITIALIZING -> GNSS_LIDAR_AIDED
    13.6  GNSS_LIDAR_AIDED -> GNSS_AIDED
    13.7  GNSS_AIDED -> GNSS_LIDAR_AIDED
    50.9  GNSS_LIDAR_AIDED -> GNSS_AIDED
    52.1  GNSS_AIDED -> GNSS_LIDAR_AIDED
    55.3  GNSS_LIDAR_AIDED -> GNSS_AIDED
    55.4  GNSS_AIDED -> GNSS_LIDAR_AIDED
    56.4  GNSS_LIDAR_AIDED -> GNSS_AIDED
    57.4  GNSS_AIDED -> GNSS_LIDAR_AIDED
    77.0  GNSS_LIDAR_AIDED -> GNSS_AIDED
    77.1  GNSS_AIDED -> GNSS_LIDAR_AIDED
    83.0  GNSS_LIDAR_AIDED -> GNSS_AIDED
    83.3  GNSS_AIDED -> GNSS_LIDAR_AIDED
    92.6  GNSS_LIDAR_AIDED -> GNSS_AIDED
    95.1  GNSS_AIDED -> GNSS_LIDAR_AIDED
   106.5  GNSS_LIDAR_AIDED -> GNSS_AIDED
   107.1  GNSS_AIDED -> GNSS_LIDAR_AIDED
   116.2  GNSS_LIDAR_AIDED -> GNSS_AIDED
   116.8  GNSS_AIDED -> GNSS_LIDAR_AIDED
   117.8  GNSS_LIDAR_AIDED -> GNSS_AIDED
   119.0  GNSS_AIDED -> GNSS_LIDAR_AIDED
   127.3  GNSS_LIDAR_AIDED -> GNSS_AIDED
   127.3  GNSS_AIDED -> GNSS_LIDAR_AIDED
   128.3  GNSS_LIDAR_AIDED -> GNSS_AIDED
   129.4  GNSS_AIDED -> GNSS_LIDAR_AIDED
   134.9  GNSS_LIDAR_AIDED -> GNSS_AIDED
   135.2  GNSS_AIDED -> GNSS_LIDAR_AIDED
   141.7  GNSS_LIDAR_AIDED -> GNSS_AIDED
   142.0  GNSS_AIDED -> GNSS_LIDAR_AIDED
   155.7  GNSS_LIDAR_AIDED -> GNSS_AIDED
   156.1  GNSS_AIDED -> GNSS_LIDAR_AIDED
   157.1  GNSS_LIDAR_AIDED -> GNSS_AIDED
   157.3  GNSS_AIDED -> GNSS_LIDAR_AIDED
   166.3  GNSS_LIDAR_AIDED -> GNSS_AIDED
   166.9  GNSS_AIDED -> GNSS_LIDAR_AIDED
   168.3  GNSS_LIDAR_AIDED -> GNSS_AIDED
   179.0  GNSS_AIDED -> GNSS_LIDAR_AIDED
   199.1  GNSS_LIDAR_AIDED -> GNSS_AIDED
   199.3  GNSS_AIDED -> GNSS_LIDAR_AIDED
   200.4  GNSS_LIDAR_AIDED -> GNSS_AIDED
   200.5  GNSS_AIDED -> GNSS_LIDAR_AIDED
   212.4  GNSS_LIDAR_AIDED -> GNSS_AIDED
   222.8  GNSS_AIDED -> GNSS_LIDAR_AIDED
   233.4  GNSS_LIDAR_AIDED -> GNSS_AIDED
   233.6  GNSS_AIDED -> GNSS_LIDAR_AIDED
   234.6  GNSS_LIDAR_AIDED -> GNSS_AIDED
   235.0  GNSS_AIDED -> GNSS_LIDAR_AIDED
   236.0  GNSS_LIDAR_AIDED -> GNSS_AIDED
   236.4  GNSS_AIDED -> GNSS_LIDAR_AIDED
   237.4  GNSS_LIDAR_AIDED -> GNSS_AIDED
```

警告・エラー（数字を # にまとめた種類ごと）:

| 回数 | 最初（s） | 内容 |
|---|---|---|
| 9 | 3.5 | `WARN re-anchored to GNSS (d#=#)` |
| 1 | -55.6 | `WARN point cloud has no per-point time field: deskew is disabled` |
