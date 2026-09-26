# ソフトウェア構成: コンポーネント図・クラス図

- 関連文書: [要件定義](./requirements.md) / [設計書](./design.md)（v0.5）
- 状態: ドラフト（v0.3。設計書 v0.5 の GNSS 入力 `sensor_msgs/NavSatFix` を反映）

本書は、設計書 7 章のアーキテクチャを、実装に入れる粒度のコンポーネント図・クラス図に落としたものである。クラス名・メソッド名は実装時の名前の案で、引数や戻り値の細部は実装時に調整する。

---

## 1. 構成の原則

| 原則 | 内容 |
|---|---|
| 依存の方向 | `ros2`（IF 層） → `core`（ロジック層）の一方向だけ。`core` は ROS のヘッダ・型・時刻・ログ・パラメータを一切参照しない |
| 時刻 | `core` 内の時刻はすべて、データに付いたタイムスタンプ（`double` 秒）で扱う。システム時計は参照しない |
| 外部への口 | `core` が外部に依存する箇所は、インターフェース（`ILogger`、`ITileLoader`、`IScanMatcher`、`IStateEstimator`）で抽象化し、テストで差し替えられるようにする |
| 推定器は状態を持たない | `IStateEstimator` は `FilterState` を受け取って返す純粋関数の集まりにする。状態は `StateHistory` が保持するので、遅延観測の再伝播（replay）が簡単になる |
| 観測はデータ型 | 観測は `std::variant` のデータ型（`Measurement`）で表す。線形化（残差・H・R の計算）は推定器側で行う。Invariant EKF と ESEKF で誤差の定義が異なり、H も変わるため |
| スレッド | `core` が持つスレッドはスキャンマッチング用と地図ロード用の 2 本だけ。フィルタの状態は `Localizer` 内の 1 つの mutex で保護する |

---

## 2. コンポーネント図

### 2.1 全体

```mermaid
flowchart TB
  subgraph EXT["外部"]
    direction LR
    DRV_IMU["IMU ドライバ"]
    DRV_ODOM["ODOM"]
    DRV_GNSS["F9P 独自ドライバ<br/>sensor_msgs/NavSatFix<br/>（+ 任意で速度）"]
    DRV_LIDAR["LiDAR ドライバ"]
    MAPFILES[("統合済み地図<br/>（外部ツールで作成）")]
    TILES[("タイル + tile_index.yaml<br/>+ maps.yaml")]
    DOWN["経路追従など<br/>下流ノード"]
  end

  subgraph ROS2["gll_ros2（IF 層・ROS 2 依存）"]
    NODE["LocalizerNode"]
    CONV["Converters<br/>Imu / Odom / NavSatFix / PointCloud"]
    PARAM["ParameterLoader<br/>→ LocalizerConfig"]
    PUBL["OutputPublisher<br/>pose / odometry / TF / diagnostics"]
    RLOG["RosLogger"]
  end

  subgraph CORE["gll_core（ロジック層・ROS 非依存）"]
    FACADE["Localizer<br/>（ファサード）"]
    EST["estimation"]
    MEAS["measurement"]
    MAPC["map"]
    MATCH["matching"]
    COMMON["common"]
  end

  subgraph TOOLS["tools（オフライン）"]
    TILER["gll_map_tiler"]
    CALIB["gll_anchor_calibrator"]
  end

  DRV_IMU & DRV_ODOM & DRV_GNSS & DRV_LIDAR --> NODE
  NODE --> CONV --> FACADE
  PARAM --> FACADE
  RLOG -. ILogger を実装 .-> COMMON
  FACADE --> PUBL --> DOWN

  FACADE --> EST & MEAS & MAPC & MATCH
  EST & MEAS & MAPC & MATCH --> COMMON
  MAPC -- ITileLoader --> TILES

  MAPFILES --> TILER --> TILES
  TILES --> CALIB
```

### 2.2 コア内部のコンポーネントとインターフェース

箱はパッケージ（中にクラス名）、六角形 `«interface»` はテストや差し替えのために抽象化した境界、点線はインターフェースの実装を表す。クラス単位の関係は 3 章のクラス図を参照。

```mermaid
flowchart TB
  LOC["Localizer<br/>ファサード・排他制御"]

  MEAS["measurement<br/>AttitudeEstimator<br/>MotionInputBuilder<br/>GnssMeasurementBuilder<br/>LidarMeasurementBuilder<br/>StopDetector"]
  EST["estimation<br/>StateHistory / MahalanobisGate<br/>SourceArbiter / RecoveryManager<br/>OutputSmoother / StatusMonitor<br/>Initializer"]
  MATCH["matching<br/>ScanMatchingWorker（スレッド）<br/>ScanPreprocessor"]
  MAPC["map<br/>MapTileManager（スレッド）<br/>MapGroup / TileIndex / MapAnchor"]
  COMMON["common<br/>SE2 / UtmProjector<br/>型定義・設定"]

  ISE{{"«interface»<br/>IStateEstimator"}}
  ISM{{"«interface»<br/>IScanMatcher"}}
  ITL{{"«interface»<br/>ITileLoader"}}
  ILG{{"«interface»<br/>ILogger"}}

  INVEKF["InvEkfSe2"]
  ESEKF["EsEkf2D（比較用）"]
  GICP["GicpMatcher（small_gicp）"]
  BTL["BinaryTileLoader"]
  RLOG["RosLogger（gll_ros2）"]

  LOC --> MEAS
  LOC --> EST
  LOC --> MATCH
  LOC --> MAPC
  LOC --> ILG

  EST --> ISE
  ISE -.-> INVEKF
  ISE -.-> ESEKF
  MATCH --> ISM
  ISM -.-> GICP
  MAPC --> ITL
  ITL -.-> BTL
  ILG -.-> RLOG

  EST -- 再位置推定の依頼 --> MATCH
  MAPC -- ターゲット構築 --> ISM
  MATCH -- 地図ターゲット取得 --> MAPC

  MEAS --> COMMON
  EST --> COMMON
  MAPC --> COMMON
```

### 2.3 コンポーネントの責務

| コンポーネント | 責務 | 主な依存 |
|---|---|---|
| `Localizer` | 外部 API の窓口。センサデータを受け取り、観測を作らせて推定器に渡し、出力を組み立てる。フィルタの状態の排他制御 | 全コンポーネント |
| `measurement` | センサデータを「推定器の入力（`MotionInput`）」と「観測（`Measurement`）」に変換する。採用判定（RTK-FIX、精度、停止など）もここで行う | common |
| `estimation` | 推定アルゴリズム（Invariant EKF）、履歴と再伝播、外れ値ゲート、出力整形、状態監視、初期化。GNSS 優先の判断と LiDAR の食い違い判定（`SourceArbiter`）、再アンカー・再位置推定の状態遷移（`RecoveryManager`） | common |
| `matching` | LiDAR の前処理と位置合わせ。専用スレッドで非同期に実行する | small_gicp, common |
| `map` | 地図グループとアンカー、タイルのロードとアンロード、位置合わせのターゲットの構築とダブルバッファ | common, matching（ターゲット構築） |
| `common` | 型、設定、SE(2) 演算、測地変換、ロガーのインターフェース | Eigen, GeographicLib |
| `gll_ros2` | ROS メッセージ ↔ コアの型の変換、パラメータの読み込み、publish、TF、診断 | rclcpp, gll_core |
| `tools` | 統合済み地図のタイル化（点ごとの共分散の事前計算を含む）、アンカーの較正 | PCL, small_gicp, gll_core |

---

## 3. クラス図

### 3.1 common（型・インターフェース）

```mermaid
classDiagram

  class ImuSample {
    +double t
    +Vector3d gyro
    +Vector3d acc
  }
  class OdomSample {
    +double t
    +double v
    +optional~double~ yaw_rate
  }
  class GnssSample {
    +double t
    +double lat
    +double lon
    +double h
    +Matrix3d cov_enu
    +bool cov_known
    +int raw_status
  }
  class GnssVelocitySample {
    +double t
    +Vector2d vel_en
    +Matrix2d cov
  }
  class LidarScan {
    +double t
    +vector~Vector3f~ points
    +PointTimes point_times
  }
  class GnssFixType {
    <<enumeration>>
    NONE
    SINGLE
    DGPS
    RTK_FLOAT
    RTK_FIX
  }
  class LocalizationStatus {
    <<enumeration>>
    INITIALIZING
    GNSS_AIDED
    LIDAR_AIDED
    GNSS_LIDAR_AIDED
    DEAD_RECKONING
    DEGRADED
    LOST
  }
  class Pose2D {
    +double x
    +double y
    +double yaw
  }
  class Pose3D {
    +Isometry3d T
  }
  class LocalizationOutput {
    +double t
    +Pose2D pose
    +Pose2D raw_pose
    +Matrix3d cov
    +double v
    +double yaw_rate
    +LocalizationStatus status
    +string active_map_group
  }
  class SE2 {
    +Matrix2d R
    +Vector2d t
    +static Exp(Vector3d xi) SE2
    +Log() Vector3d
    +Ad() Matrix3d
    +inverse() SE2
    +compose(SE2 other) SE2
    +act(Vector2d p) Vector2d
    +yaw() double
  }
  class UtmProjector {
    -int zone
    -bool north
    +forward(lat, lon) UtmPoint
    +inverse(E, N) LatLon
  }
  class UtmPoint {
    +double E
    +double N
    +double convergence
    +double scale
  }
  class ILogger {
    <<interface>>
    +debug(msg)*
    +info(msg)*
    +warn(msg)*
    +error(msg)*
  }
  class LocalizerConfig {
    +EstimatorConfig estimator
    +GnssConfig gnss
    +LidarConfig lidar
    +MapConfig map
    +OutputConfig output
    +Extrinsics extrinsics
    +static fromYaml(path) LocalizerConfig
  }

  LocalizationOutput --> Pose2D
  LocalizationOutput --> LocalizationStatus
  UtmProjector ..> UtmPoint
```

`PointTimes` は `std::vector<float>` の別名（点ごとの時刻。LiDAR ドライバが出さない場合は空）。

### 3.2 facade（Localizer）

```mermaid
classDiagram

  class Localizer {
    -LocalizerConfig cfg_
    -mutex filter_mtx_
    -shared_ptr~ILogger~ logger_
    -unique_ptr~IStateEstimator~ estimator_
    -StateHistory history_
    -MahalanobisGate gate_
    -OutputSmoother smoother_
    -StatusMonitor monitor_
    -Initializer initializer_
    -SourceArbiter arbiter_
    -RecoveryManager recovery_
    -AttitudeEstimator attitude_
    -MotionInputBuilder motion_builder_
    -GnssMeasurementBuilder gnss_builder_
    -StopDetector stop_detector_
    -MapTileManager map_manager_
    -ScanMatchingWorker matching_worker_
    +Localizer(cfg, estimator, matcher, tile_loader, logger)
    +addImu(ImuSample)
    +addOdom(OdomSample)
    +addGnss(GnssSample)
    +addGnssVelocity(GnssVelocitySample)
    +addLidarScan(LidarScan)
    +setInitialPose(Pose2D, Matrix3d cov)
    +getOutput() optional~LocalizationOutput~
    +diagnostics() Diagnostics
    -predictTo(double t)
    -applyMeasurement(Measurement m) UpdateResult
    -onMatchingResult(PoseMeasurement m)
    -onRelocalizeResult(RegistrationResult r)
    -handleRejection(Measurement m, UpdateResult r)
  }

  class IStateEstimator {
    <<interface>>
  }
  class StateHistory
  class MahalanobisGate
  class OutputSmoother
  class StatusMonitor
  class Initializer
  class SourceArbiter
  class RecoveryManager
  class AttitudeEstimator
  class MotionInputBuilder
  class GnssMeasurementBuilder
  class StopDetector
  class MapTileManager
  class ScanMatchingWorker
  class ILogger {
    <<interface>>
  }

  Localizer *-- StateHistory
  Localizer *-- MahalanobisGate
  Localizer *-- OutputSmoother
  Localizer *-- StatusMonitor
  Localizer *-- Initializer
  Localizer *-- SourceArbiter
  Localizer *-- RecoveryManager
  Localizer *-- AttitudeEstimator
  Localizer *-- MotionInputBuilder
  Localizer *-- GnssMeasurementBuilder
  Localizer *-- StopDetector
  Localizer *-- MapTileManager
  Localizer *-- ScanMatchingWorker
  Localizer o-- IStateEstimator
  Localizer o-- ILogger
```

### 3.3 estimation（推定）

#### 3.3.1 推定器と観測

```mermaid
classDiagram
  class IStateEstimator {
    <<interface>>
    +predict(FilterState, MotionInput, double dt) FilterState*
    +linearize(FilterState, Measurement) Linearization*
    +correct(FilterState, Linearization) UpdateResult*
    +worldCovariance(FilterState) Matrix3d*
  }
  class InvEkfSe2 {
    -EstimatorConfig cfg_
    +predict(...) FilterState
    +linearize(...) Linearization
    +correct(...) UpdateResult
    +worldCovariance(...) Matrix3d
  }
  class EsEkf2D {
    +predict(...) FilterState
    +linearize(...) Linearization
    +correct(...) UpdateResult
    +worldCovariance(...) Matrix3d
  }
  class FilterState {
    +double t
    +SE2 X
    +double gyro_bias
    +double odom_scale
    +Matrix5d P
  }
  class MotionInput {
    +double t
    +double v
    +double omega
  }
  class Measurement {
    <<variant>>
    GnssPositionMeasurement
    HeadingMeasurement
    PoseMeasurement
    ZeroRateMeasurement
  }
  class GnssPositionMeasurement {
    +double t
    +Vector2d y_utm
    +Matrix2d cov_world
    +Vector2d lever_arm
  }
  class HeadingMeasurement {
    +double t
    +double yaw
    +double var
  }
  class PoseMeasurement {
    +double t
    +SE2 Z
    +Matrix3d cov_body
    +Matrix3d anchor_cov_world
    +string map_group
  }
  class ZeroRateMeasurement {
    +double t
    +double omega_mean
    +double var
  }
  class Linearization {
    +VectorXd r
    +MatrixXd H
    +MatrixXd R
  }
  class UpdateResult {
    +bool accepted
    +bool gate_passed
    +double mahalanobis_d2
    +Vector3d residual_body
    +Vector3d world_delta
  }
  IStateEstimator <|.. InvEkfSe2
  IStateEstimator <|.. EsEkf2D
  IStateEstimator ..> FilterState
  IStateEstimator ..> MotionInput
  IStateEstimator ..> Measurement
  IStateEstimator ..> Linearization
  Measurement ..> GnssPositionMeasurement
  Measurement ..> HeadingMeasurement
  Measurement ..> PoseMeasurement
  Measurement ..> ZeroRateMeasurement
```

#### 3.3.2 履歴・ゲート・優先度・復帰

```mermaid
classDiagram
  class IStateEstimator {
    <<interface>>
  }
  class FilterState
  class StateHistory {
    -deque~Entry~ buf_
    -double length_
    +push(FilterState, MotionInput)
    +latest() FilterState
    +stateAt(t, estimator) optional~FilterState~
    +applyDelayed(m, estimator, gate, policy) UpdateResult
    +applyReanchor(m, inflation, estimator) UpdateResult
    +relativeMotion(t_from, t_to) SE2
    +reset(FilterState)
  }
  class MahalanobisGate {
    -double alpha_
    -int max_consecutive_rejects_
    +check(Linearization, Matrix P) GateResult
  }
  class GatePolicy {
    <<enumeration>>
    REJECT_ON_FAIL
    NEVER_REJECT
    SKIP
  }
  class SourceArbiter {
    -ArbiterConfig cfg_
    -double last_gnss_fix_t_
    -MismatchStatsMap mismatch_
    +onGnssAccepted(double t)
    +isGnssFixActive(double t) bool
    +gatePolicy(m, t) GatePolicy
    +classifyLidar(m, state) LidarDecision
    +mismatchStats(string group) MismatchStats
  }
  class LidarDecision {
    <<enumeration>>
    FUSE_PRIMARY
    FUSE_INFLATED
    REJECT_MISMATCH
  }
  class MismatchStats {
    +size_t count
    +Vector3d mean_world
    +Vector3d stddev_world
  }
  class RecoveryManager {
    -RecoveryConfig cfg_
    -RecoveryState state_
    -deque~Measurement~ gnss_candidates_
    -deque~Measurement~ lidar_candidates_
    -int lidar_reject_count_
    -int relocalize_attempts_
    +onAccepted(Measurement)
    +onRejected(m, result, history) RecoveryAction
    +onRelocalizeResult(result) RecoveryAction
    +onExternalPose()
    +state() RecoveryState
  }
  class RecoveryState {
    <<enumeration>>
    TRACKING
    SUSPECT
    REANCHOR
    RELOCALIZE
    LOST
  }
  class RecoveryAction {
    +Kind kind
    +optional~Measurement~ reanchor_measurement
    +Matrix3d inflation
    +optional~RelocalizeRequest~ relocalize
  }
  SourceArbiter ..> GatePolicy
  SourceArbiter ..> LidarDecision
  SourceArbiter *-- MismatchStats
  RecoveryManager ..> RecoveryState
  RecoveryManager ..> RecoveryAction
  RecoveryManager ..> StateHistory : relativeMotion
  IStateEstimator ..> FilterState
  StateHistory ..> IStateEstimator
  StateHistory ..> MahalanobisGate
  StateHistory *-- FilterState
```

#### 3.3.3 出力整形・状態監視・初期化

```mermaid
classDiagram
  class OutputSmoother {
    -Vector3d offset_
    -double max_rate_xy_
    -double max_rate_yaw_
    +onCorrection(Vector3d world_delta)
    +apply(Pose2D raw, double dt) Pose2D
    +offsetNorm() OffsetNorm
  }
  class StatusMonitor {
    +onAccepted(MeasurementKind, double t)
    +onRejected(MeasurementKind, double t)
    +evaluate(double t, Matrix3d cov, OutputSmoother) LocalizationStatus
  }
  class Initializer {
    -InitPhase phase_
    +onGnss(GnssPositionMeasurement) optional~FilterState~
    +onExternalPose(Pose2D, Matrix3d) FilterState
    +isReady(FilterState) bool
  }
```

補足:

- `MismatchStatsMap` は `std::map<std::string, MismatchStats>`、`OffsetNorm` は（並進 [m], yaw [rad]）の組の別名。
- `IStateEstimator` のメソッドはすべて `const` で、状態を持たない。`FilterState` は値型で、`StateHistory` がリングバッファ（既定 2 s）に保持する。
- `linearize` は `std::visit` で観測の型ごとに分岐する。Invariant EKF 版は設計書 3.5〜3.7 節の式（機体座標系の残差、定数の H）を実装する。
- `GatePolicy` は `SourceArbiter` が観測ごとに決める。RTK-FIX の GNSS は `NEVER_REJECT`（ゲートで落ちたら再アンカーの候補にするだけ）。GNSS FIX 中の LiDAR は、`classifyLidar` による固定閾値の判定を先に行う。
- `RecoveryManager` は設計書 3.13.5 節の状態遷移を持つ。棄却された観測を候補として溜め、`StateHistory::relativeMotion` で最新時刻にそろえて互いに一致するかを判定し、再アンカーや再位置推定を指示する（`RecoveryAction`）。実際の更新は `Localizer` が `StateHistory::applyReanchor` で行う。
- `correct` は注入（$`\hat X \leftarrow \hat X\,\mathrm{Exp}(\delta\xi)`$）と Joseph 形式の共分散更新を行い、出力整形用に世界座標系での移動量 `world_delta` を返す。

### 3.4 measurement（センサデータ → 入力・観測）

```mermaid
classDiagram

  class AttitudeEstimator {
    -Quaterniond q_
    -Vector3d gyro_bias_
    -double kp_
    -double ki_
    +initializeStatic(vector~ImuSample~)
    +update(ImuSample, double v, double v_dot)
    +roll() double
    +pitch() double
    +verticalRate(Vector3d gyro) double
  }
  class MotionInputBuilder {
    -deque~OdomSample~ odom_buf_
    -double odom_hold_max_
    +addOdom(OdomSample)
    +build(ImuSample, AttitudeEstimator) optional~MotionInput~
  }
  class GnssMeasurementBuilder {
    -UtmProjector utm_
    -Vector2d lever_arm_
    -int rtk_fix_status_
    -double fix_since_
    +classify(GnssSample) GnssFixType
    +buildPosition(GnssSample) GnssPositionResult
    +buildHeading(GnssVelocitySample, double yaw_rate, bool rtk_fix_active) optional~HeadingMeasurement~
  }
  class GnssPositionResult {
    +optional~GnssPositionMeasurement~ position
    +GnssFixType fix
    +GnssRejectReason reason
  }
  class LidarMeasurementBuilder {
    +build(RegistrationResult, MapAnchor, double t) optional~PoseMeasurement~
  }
  class StopDetector {
    -double stop_since_
    -RunningMean omega_mean_
    +update(MotionInput) optional~ZeroRateMeasurement~
    +isStopped() bool
  }
  class UtmProjector

  GnssMeasurementBuilder --> UtmProjector
  GnssMeasurementBuilder ..> GnssPositionResult
  MotionInputBuilder ..> AttitudeEstimator
```

### 3.5 matching（スキャンマッチング）

```mermaid
classDiagram

  class IScanMatcher {
    <<interface>>
    +buildTarget(vector~TileData~ tiles, string group) shared_ptr~RegistrationTarget~*
    +align(PreprocessedScan, RegistrationTarget, Pose3D init) RegistrationResult*
  }
  class RegistrationTarget {
    <<abstract>>
    +string map_group
    +set~TileId~ tiles
    +size_t num_points
  }
  class GicpTarget {
    +shared_ptr~small_gicp_PointCloud~ cloud
    +shared_ptr~small_gicp_KdTree~ tree
  }
  class GicpMatcher {
    -GicpConfig cfg_
    +buildTarget(...) shared_ptr~RegistrationTarget~
    +align(...) RegistrationResult
    +alignMultiHypothesis(scan, target, vector~Pose3D~) RegistrationResult
  }
  class RegistrationResult {
    +bool converged
    +int iterations
    +Pose3D T_map_base
    +Matrix6d cov_body
    +double inlier_ratio
    +double error_per_point
  }
  class ScanPreprocessor {
    -Isometry3d T_base_lidar
    -CropConfig crop_
    -double voxel_size_
    +process(LidarScan, optional~RotationDeskew~) PreprocessedScan
  }
  class PreprocessedScan {
    +double t
    +shared_ptr~small_gicp_PointCloud~ cloud
  }
  class ScanMatchingWorker {
    -thread thread_
    -LatestSlot~LidarScan~ pending_
    -IScanMatcher* matcher_
    -ScanPreprocessor preprocessor_
    -MapTileManager* map_manager_
    -PosePredictor predict_pose_
    -ResultCallback on_result_
    +start()
    +stop()
    +submit(LidarScan)
    +submitRelocalize(RelocalizeRequest)
  }

  IScanMatcher <|.. GicpMatcher
  RegistrationTarget <|-- GicpTarget
  IScanMatcher ..> RegistrationTarget
  IScanMatcher ..> RegistrationResult
  ScanMatchingWorker *-- ScanPreprocessor
  ScanMatchingWorker --> IScanMatcher
  ScanPreprocessor ..> PreprocessedScan
```

補足:

- `PosePredictor` は `std::function<Pose3D(double)>`、`ResultCallback` は `std::function<void(PoseMeasurement)>` の別名（図の表記を単純にするための型エイリアス）。
- `ScanMatchingWorker` は最新のスキャンだけを保持する（`LatestSlot`。処理中に届いた古いスキャンは捨てる）。
- 初期値は `predict_pose_`（`Localizer` が `StateHistory` からスキャン時刻の予測姿勢を返すコールバック）で取得する。結果は `on_result_` で `Localizer` に戻し、遅延観測として適用する。
- `GicpMatcher` の中に small_gicp の型を閉じ込める。ほかのクラスは `RegistrationTarget` / `PreprocessedScan` を通してだけ扱う。
- `PreprocessedScan` は現状 small_gicp の点群を持っているので、完全には抽象化できていない。ほかのマッチャに差し替えるときに抽象化する。

### 3.6 map（地図管理）

```mermaid
classDiagram

  class MapAnchor {
    -Vector3d map_point_
    -UtmPoint anchor_utm_
    -double ellipsoid_height_
    -double rotation_
    -double scale_
    -Matrix3d cov_world_
    +static fromConfig(AnchorConfig, UtmProjector) MapAnchor
    +mapToUtm(Pose3D) Pose3D
    +utmToMap(Pose3D) Pose3D
    +covarianceWorld() Matrix3d
  }
  class MapGroup {
    +string id
    +MapAnchor anchor
    +TileIndex index
    +AABB2d utm_bounds
  }
  class TileIndex {
    +double tile_size
    +vector~TileMeta~ tiles
    +static load(path) TileIndex
    +query(AABB2d map_bbox) vector~TileMeta~
  }
  class TileMeta {
    +TileId id
    +string file
    +AABB3d bounds_map
    +AABB2d bounds_utm
    +size_t num_points
  }
  class TileData {
    +TileId id
    +vector~Vector3f~ points
    +vector~Matrix3f~ covs
  }
  class ITileLoader {
    <<interface>>
    +load(TileMeta) TileData*
  }
  class BinaryTileLoader {
    +load(TileMeta) TileData
  }
  class MapTileManager {
    -vector~MapGroup~ groups_
    -UniformGridIndex utm_index_
    -ITileLoader* loader_
    -IScanMatcher* matcher_
    -TileCache cache_
    -atomic_shared_ptr~RegistrationTarget~ front_
    -thread thread_
    +start()
    +stop()
    +updatePose(Pose2D, double v)
    +currentTarget() shared_ptr~const RegistrationTarget~
    +activeGroup() const MapGroup*
    +group(string id) const MapGroup&
  }

  MapGroup *-- MapAnchor
  MapGroup *-- TileIndex
  TileIndex *-- TileMeta
  ITileLoader <|.. BinaryTileLoader
  ITileLoader ..> TileData
  MapTileManager *-- MapGroup
  MapTileManager --> ITileLoader
  MapTileManager ..> IScanMatcher : buildTarget
```

補足:

- `TileCache` は `std::map<TileId, TileData>` の別名。
- `updatePose` は呼び出し側のスレッドで要求中心と必要なタイル集合を計算し、変化があればワーカーに通知するだけにする（すぐに戻る）。
- ワーカーはタイルを読み込み、`IScanMatcher::buildTarget` で新しいターゲットを作る。完成したら `front_` をアトミックに差し替える（ダブルバッファ）。
- 位置合わせ中のスレッドは `shared_ptr` でターゲットを保持しているので、差し替えの影響を受けない。

### 3.7 gll_ros2（IF 層）

```mermaid
classDiagram

  class LocalizerNode {
    <<ROS2 Node>>
    -unique_ptr~Localizer~ localizer_
    -Subscription imu_sub_
    -Subscription odom_sub_
    -Subscription navsatfix_sub_
    -Subscription gnss_vel_sub_
    -Subscription points_sub_
    -Subscription initial_pose_sub_
    -TimerBase output_timer_
    -OutputPublisher publisher_
    -NavSatFixConverter navsatfix_conv_
    +LocalizerNode(NodeOptions)
    -onImu(Imu)
    -onOdom(Odometry)
    -onNavSatFix(NavSatFix)
    -onGnssVelocity(TwistWithCovarianceStamped)
    -onPoints(PointCloud2)
    -onInitialPose(PoseWithCovarianceStamped)
    -onOutputTimer()
  }
  class ParameterLoader {
    +static load(Node) LocalizerConfig
  }
  class ExtrinsicsLoader {
    +static load(tf2_Buffer, frames) Extrinsics
  }
  class RosLogger {
    -rclcpp_Logger logger_
    +debug(msg)
    +info(msg)
    +warn(msg)
    +error(msg)
  }
  class NavSatFixConverter {
    -double stamp_offset_
    +convert(NavSatFix) GnssSample
    +convert(TwistWithCovarianceStamped) GnssVelocitySample
  }
  class MsgConverters {
    <<utility>>
    +toCore(Imu) ImuSample
    +toCore(Odometry) OdomSample
    +toCore(PointCloud2) LidarScan
    +toRos(LocalizationOutput) PoseWithCovarianceStamped
    +toRosOdometry(LocalizationOutput) Odometry
  }
  class OutputPublisher {
    -Publisher pose_pub_
    -Publisher odom_pub_
    -Publisher raw_pose_pub_
    -TransformBroadcaster tf_
    -Updater diag_
    +publish(LocalizationOutput)
    +publishDiagnostics(Diagnostics)
  }
  class ILogger {
    <<interface>>
  }
  class Localizer

  LocalizerNode *-- Localizer
  LocalizerNode *-- OutputPublisher
  LocalizerNode *-- NavSatFixConverter
  LocalizerNode ..> ParameterLoader
  LocalizerNode ..> ExtrinsicsLoader
  LocalizerNode ..> MsgConverters
  ILogger <|.. RosLogger
  LocalizerNode ..> RosLogger : 生成して Localizer に注入
```

ROS 1 版（`gll_ros1`）は、この図の `rclcpp` を `roscpp` に置き換えた同じ構成になる。`Localizer` から下は共通である。

---

## 4. 主要な処理の流れ

### 4.1 IMU 受信（予測）と GNSS 受信（更新）

```mermaid
sequenceDiagram
  participant N as LocalizerNode
  participant L as Localizer
  participant A as AttitudeEstimator
  participant M as MotionInputBuilder
  participant H as StateHistory
  participant E as InvEkfSe2
  participant G as GnssMeasurementBuilder
  participant AR as SourceArbiter
  participant RC as RecoveryManager
  participant S as OutputSmoother

  N->>L: addImu(ImuSample)
  L->>A: update(imu, v, v_dot)
  L->>M: build(imu, attitude)
  M-->>L: MotionInput
  L->>E: predict(latest, input, dt)
  E-->>L: FilterState
  L->>H: push(state, input)

  N->>L: addGnss(GnssSample)
  L->>G: classify(sample) / buildPosition(sample)
  G-->>L: position（RTK-FIX かつ精度・安定待ちを満たすものだけ）
  L->>AR: onGnssAccepted(t) / gatePolicy(m) → NEVER_REJECT
  L->>H: applyDelayed(measurement, estimator, gate, policy)
  H->>E: stateAt(t_z) → linearize → gate → correct
  H->>E: 再伝播（t_z 以降の入力）
  H-->>L: UpdateResult（world_delta, gate_passed）
  alt ゲート不通過（推定値がずれている）
    L->>RC: onRejected(m, result, history)
    RC-->>L: RecoveryAction（3 回一致したら REANCHOR）
    L->>H: applyReanchor(m, inflation)
  end
  L->>S: onCorrection(world_delta)
```

### 4.2 LiDAR 受信（非同期の位置合わせ → 遅延観測）

```mermaid
sequenceDiagram
  participant N as LocalizerNode
  participant L as Localizer
  participant W as ScanMatchingWorker（スレッド）
  participant P as ScanPreprocessor
  participant MT as MapTileManager
  participant G as GicpMatcher
  participant LB as LidarMeasurementBuilder
  participant H as StateHistory
  participant AR as SourceArbiter
  participant RC as RecoveryManager

  N->>L: addLidarScan(scan)
  L->>W: submit(scan)（最新のみ保持してすぐ戻る）
  W->>P: process(scan, deskew)
  W->>MT: currentTarget()
  W->>L: predict_pose(scan.t)（スキャン時刻の予測姿勢）
  L->>H: stateAt(scan.t)
  W->>MT: activeGroup().anchor.utmToMap(pose)
  W->>G: align(scan, target, init)
  G-->>W: RegistrationResult
  W->>LB: build(result, anchor, t)
  LB-->>W: PoseMeasurement
  W->>L: on_result(PoseMeasurement)
  Note over L: mutex 取得
  L->>AR: classifyLidar(m, state)
  alt GNSS FIX 中で食い違い
    AR-->>L: REJECT_MISMATCH（アンカーずれとして記録）
  else FUSE_PRIMARY / FUSE_INFLATED
    L->>H: applyDelayed(...)
    opt ゲート不通過（GNSS 無し）
      L->>RC: onRejected(...)
      RC-->>L: REANCHOR（5 回一致）または RELOCALIZE（2 s 不一致）
    end
  end
```

### 4.3 地図タイルの更新

```mermaid
sequenceDiagram
  participant L as Localizer
  participant MT as MapTileManager
  participant T as ロードワーカー（スレッド）
  participant TL as BinaryTileLoader
  participant G as GicpMatcher

  L->>MT: updatePose(pose, v)（出力周期で呼ぶ）
  MT->>MT: 要求中心 = 位置 + 先読み<br/>必要なタイル集合を計算
  alt タイル集合が変化した
    MT->>T: 通知（すぐ戻る）
    T->>TL: load(不足しているタイル)
    TL-->>T: TileData
    T->>G: buildTarget(ロード済みタイル, group)
    G-->>T: 新しいターゲット（KdTree 構築済み）
    T->>MT: front_ をアトミックに差し替え
    T->>T: アンロード対象をキャッシュから破棄
  end
```

---

## 5. ディレクトリとクラスの対応

| ディレクトリ | クラス / ファイル |
|---|---|
| `core/include/gll/common/` | `types.hpp`（センサデータ・出力の型）、`se2.hpp`（`SE2`）、`geodesy.hpp`（`UtmProjector`）、`logger.hpp`（`ILogger`）、`config.hpp`（`LocalizerConfig`） |
| `core/include/gll/estimation/` | `state_estimator.hpp`（`IStateEstimator`、`FilterState`、`Measurement`）、`inv_ekf_se2.hpp`、`es_ekf_2d.hpp`、`state_history.hpp`、`mahalanobis_gate.hpp`、`output_smoother.hpp`、`status_monitor.hpp`、`initializer.hpp`、`source_arbiter.hpp`、`recovery_manager.hpp` |
| `core/include/gll/measurement/` | `attitude_estimator.hpp`、`motion_input_builder.hpp`、`gnss_measurement_builder.hpp`、`lidar_measurement_builder.hpp`、`stop_detector.hpp` |
| `core/include/gll/matching/` | `scan_matcher.hpp`（`IScanMatcher`、`RegistrationTarget`、`RegistrationResult`）、`gicp_matcher.hpp`、`scan_preprocessor.hpp`、`scan_matching_worker.hpp` |
| `core/include/gll/map/` | `map_anchor.hpp`、`map_group.hpp`（`MapGroup`、`TileIndex`、`TileMeta`）、`tile_loader.hpp`（`ITileLoader`、`BinaryTileLoader`、`TileData`）、`map_tile_manager.hpp` |
| `core/include/gll/` | `localizer.hpp`（`Localizer`） |
| `ros2/gll_ros2/` | `localizer_node.cpp`、`parameter_loader.cpp`、`extrinsics_loader.cpp`、`ros_logger.hpp`、`navsatfix_converter.cpp`、`msg_converters.cpp`、`output_publisher.cpp` |
| `tools/map_tiler/` | `gll_map_tiler`（統合済み地図 → タイル + 点ごとの共分散） |
| `tools/anchor_calibrator/` | `gll_anchor_calibrator` |

## 6. Phase 1 で実装する範囲

設計書 10 章の Phase 1 に対応して、次のクラスを実装する。

- **common**: 全クラス
- **estimation**: `IStateEstimator`、`InvEkfSe2`、`EsEkf2D`（比較用）、`StateHistory`、`MahalanobisGate`、`OutputSmoother`、`StatusMonitor`、`Initializer`（GNSS 区間の初期化のみ）、`SourceArbiter`（GNSS 部分）、`RecoveryManager`（GNSS の再アンカーと `LOST` の判定）
- **measurement**: `AttitudeEstimator`、`MotionInputBuilder`、`GnssMeasurementBuilder`、`StopDetector`
- **facade**: `Localizer`（LiDAR / 地図関連のメンバは空の実装にしておく）
- **gll_ros2**: LiDAR 以外の全クラス

`matching` と `map`、`LidarMeasurementBuilder`、`tools`、`SourceArbiter` / `RecoveryManager` の LiDAR 部分（食い違い判定、LiDAR の再アンカー、再位置推定）は Phase 2 で実装する。
