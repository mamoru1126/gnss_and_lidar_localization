# 設計書: GNSS / LiDAR 統合自己位置推定

- 関連文書: [要件定義](./requirements.md) / [ソフトウェア構成（コンポーネント図・クラス図）](./architecture.md) / [アルゴリズム説明書（Invariant EKF の解説を含む）](./algorithm.md) / [検証計画: i2Nav-Robot](./validation_i2nav.md)
- 状態: ドラフト（v0.11。Phase 2 を実装済み。地図のライブラリ gll_map を切り出した）

| 版 | 変更内容 |
|---|---|
| v0.1 | 初版 |
| v0.2 | 確定したハードウェア前提（u-blox F9P シングルアンテナ、6 軸 IMU、最高速度 6 km/h）を反映。スキャンマッチングを small_gicp に変更。推定器の代替候補として Invariant EKF を追記 |
| v0.3 | 推定器を **Invariant EKF（SE(2)、左不変誤差）に変更**。GNSS 入力を独自ドライバの `ublox_gps/NavPVT` に確定。地図の統合は外部ツールで行い、本システムは統合済みの地図を入力とする前提に変更 |
| v0.4 | **RTK-FIX の GNSS を常に信用する方針**を確定。GNSS / LiDAR の優先度、食い違いの判定、再アンカー、再位置推定の節（3.13）を追加 |
| v0.5 | GNSS 入力を **`sensor_msgs/NavSatFix`** に変更。RTK-FIX の表現・共分散・高さ・時刻について、ドライバとの取り決めを定義。進行方位の観測は、速度トピックがある場合の任意機能に変更 |
| v0.6 | GNSS アンテナのレバーアームを 3 次元（アンテナ高さ $`l_z`$ ≤ 1 m）にし、roll / pitch で水平面に射影して使うよう変更。roll / pitch の誤差を GNSS の観測共分散に加算 |
| v0.7 | 出力の共分散に、出力整形層に残っているオフセットの分（$`\mathbf{o}\mathbf{o}^\top`$）を加えるよう変更（3.10 節） |
| v0.8 | ROS 2 Jazzy と Docker 環境を確定し、ROS 1 対応は後回しに。出力 TF を `map` → `base_link`（`map` = UTM）とし、地図グループの座標系を `pcd_<group_id>` に改名。ODOM を `nav_msgs/Odometry`（twist を使用）に確定し、横方向の速度も予測に使えるようにした |
| v0.9 | 運用上の前提（点群地図と GNSS 区間の間には必要十分な距離を設ける。要件定義 3 章）を反映。**デッドレコニング距離の監視**を追加し、位置の観測なしで一定距離を走ったら diagnostics で ERROR を通知するようにした（3.12 節）。GNSS のメッセージが途切れた後は FIX の安定待ちをやり直す（3.6 節）。複数の地図グループの扱いを見直した: アクティブグループを現在位置で決める、LiDAR の結果は照合に使ったターゲットのグループのアンカーで変換する、タイル ID にグループを含める、z を楕円体高で保持する、グループの重なりを起動時に検出する（3.9 節、5 章） |
| v0.10 | **Phase 2（スキャンマッチング）を実装**し、実装に合わせて更新。**GNSS 区間が無く点群地図だけがある現場**も対象にした: 初期姿勢（外部から与える、または前回保存した位置）の周りで位置合わせして地図上で初期化する（3.11 節）。スキャンマッチングの共分散の式と品質の指標（overlap）を確定し（6.2 節）、多仮説の探索（6.4 節）、再位置推定の探索範囲の下限（3.13.4 節）、タイルと地図設定の形式（5.2 節）、アンカーの UTM での直接指定と `local`（4.2 節）、点群の時刻の扱い（6.1 節）を追記 |
| v0.11 | 地図のタイル化・動的ロードを、単体でも使える ROS 非依存のライブラリ **`gll_map`**（`map/`）に切り出した（7.2 節・7.7 節）。`MapTileManager` は、読み込み中のタイルをまとめた「領域」を、差し替えられる関数（`RegionBuilder`）で作る。`gll_core` はそこに GICP のターゲットを作る関数を渡す。small_gicp は `gll_map` では任意（あればタイル化のときに点ごとの共分散を計算する） |

### 前提ハードウェア・運用条件（v0.2〜v0.3、v0.9 で確定）

| 項目 | 内容 | 設計への主な影響 |
|---|---|---|
| GNSS | u-blox ZED-F9P、**シングルアンテナ** | GNSS から直接ヨーは得られない。初期ヨーも走行中のヨーも、GNSS の位置の系列から Invariant EKF の中で推定する（3.6 節）。ドライバは独自実装で、メッセージ型は **`sensor_msgs/NavSatFix`**（v0.5）。RTK-FIX の表現などはドライバとの取り決めで定める |
| 点群地図 | **統合は外部ツールで行う**。本システムには、地図グループごとに統合済みの点群が入力される | 地図の統合・作成は本システムの範囲外（5.1 節） |
| IMU | **加速度と角速度のみ**（姿勢は出力しない） | roll / pitch を自前で推定する姿勢推定器が必要（3.9 節）。ジャイロバイアスは停止中に推定する |
| 車速 | **最高 6 km/h（約 1.7 m/s）** | タイルの先読みは余裕がある。スキャン中の並進による歪みは小さい（0.1 s で 0.17 m 以下）が、旋回時の回転による歪みは残る（6.1 節）。出力整形のレートは車速に合わせて低めに設定する |
| LiDAR マッチング | **small_gicp** | コア層に置ける（ROS 非依存、Eigen ベース）。タイル単位でのターゲット更新は、ロード済みタイルを結合して再構築する方式にする（6 章） |
| 現場（v0.10） | **GNSS 区間が無く、点群地図だけがある現場もある** | GNSS なしで起動できるよう、地図上での初期化を用意する（3.11 節）。ジャイロバイアスは静止と ZARU、ODOM のスケールは LiDAR の位置観測で推定されるので、GNSS が無くても推定器はそのまま動く |
| 運用（v0.9） | **点群地図と GNSS 区間の間には必要十分な距離が設定される**（要件定義 3 章） | GNSS 区間をはさむ地図グループどうしは重ならないので、照合に使うグループを現在位置だけで決められる（5.5 節）。長いデッドレコニングは前提が崩れた状態とみなし、diagnostics で ERROR を通知する（3.12 節） |

---

## 1. 設計方針のまとめ

| # | 論点 | 決定 | 主な理由 |
|---|---|---|---|
| D-1 | 推定器の構成（要件③） | **単一の推定器で全センサを融合する**（推定器の切り替えは行わない） | 出力の連続性（FR-4）を構造的に確保しやすい。GNSS と LiDAR が同時に有効な区間では両方を重み付きで使える |
| D-2 | 推定アルゴリズム | **Invariant EKF（SE(2) 上の左不変誤差）**（v0.3 で ESEKF から変更） | 計算量は EKF と同等で、ROS に依存しない形で実装しやすい。遷移行列と GNSS の観測行列が推定中の姿勢に依存しないため、シングルアンテナで yaw の誤差が大きくなる場面でも収束と共分散の整合性を保ちやすい。3D 化では SE₂(3) に拡張できる |
| D-3 | 状態の自由度 | **Phase 1 は平面 2D（x, y, yaw）+ バイアス類**。roll / pitch / z はフィルタ外の補助推定で扱う | 出力要件が x, y, yaw のため。3D 化は Phase 4 の拡張として設計上の余地を残す（3.2 節） |
| D-4 | 推定器の座標系 | **状態は常に UTM**（地図座標系ではない） | 地図を切り替えても状態の座標系が変わらないため、地図の切り替えが「観測の座標変換の切り替え」に帰着し、状態は飛ばない |
| D-5 | スキャンマッチング | **small_gicp（GICP）**。`IScanMatcher` インターフェースで抽象化し、ほかの手法にも差し替えられるようにする | ROS 非依存でコア層に置ける。高速で、ヘッセ行列から観測共分散を得られる |
| D-6 | 地図の単位（要件④） | 直接つながる地図群は**1 つの「地図グループ」として統合**し、タイル分割して動的ロードする。GNSS 区間で隔てられた地図は別グループとし、それぞれのアンカーで UTM に固定する | アンカーを地図間で連鎖させることによる誤差の蓄積を避ける |
| D-7 | 地図の読み込みタイミング | **UTM 上の位置と先読み距離から必要なタイルを決め、バックグラウンドでロードしてダブルバッファで差し替える** | 状態が UTM で表されているため、どのグループのタイルが必要かも UTM 上の幾何計算だけで決まる |
| D-10 | GNSS / LiDAR の優先度（v0.4） | **RTK-FIX の GNSS を常に優先して信用する**。GNSS FIX 中の LiDAR は、整合していれば重みを下げて融合し、食い違えば棄却する。推定値がずれたときは再アンカー、LiDAR が失敗したときは再位置推定で復帰する（3.13 節） | 要件（FIX 区間は GNSS）とユーザ方針に合わせるため。モードの切り替えを持たずに、優先度をフィルタの重みと判定の規則だけで表現できる |
| D-8 | 出力の連続性 | ① 観測の Mahalanobis ゲート、② 補正量をレート制限して吸収する出力整形層、③ 地図アンカーの事前較正 の 3 段で担保する | 観測更新で生じる補正ステップを、経路追従に影響しない大きさに抑えるため |
| D-9 | ソフトウェア構成（要件⑥） | **ROS 非依存のコアライブラリ（C++17 + Eigen）** と、薄い ROS 2 インターフェースノードに分離する | ROS 1 への移行は IF 層を差し替えるだけで済む |
| D-12 | 地図だけの現場での起動（v0.10） | 初期姿勢（RViz などから与える、または前回保存した位置）の周りで、複数の初期値から位置合わせして初期化する。初期姿勢をまったく与えない、地図全体からの探索は行わない | 地図全体の探索は重く、似た場所を取り違えやすい。与えた初期姿勢の近くに探索範囲を絞り、一意性を確かめてから採用する |
| D-11 | デッドレコニングの監視（v0.9） | 位置の観測なしで走った距離を積算し、`dr_error_distance` を超えたら diagnostics で ERROR を通知する（3.12 節） | 運用上の前提（地図と GNSS 区間の間の距離）が崩れたことを、共分散のモデルに頼らずに検知するため |

---

## 2. システム構成

```mermaid
flowchart LR
  subgraph Sensors
    IMU[IMU]
    ODOM[ODOM]
    GNSS[GNSS]
    LIDAR[LiDAR]
  end

  subgraph ROS2_IF["ROS 2 IF 層（gll_ros2）"]
    SUB[Subscriber / 型変換]
    PUB[Publisher / TF / Diagnostics]
  end

  subgraph Core["コア層（gll_core, ROS 非依存）"]
    ATT[姿勢推定器<br/>roll / pitch]
    EKF["Invariant EKF<br/>SE(2) + b_g, s"]
    HIST[状態履歴バッファ<br/>遅延観測の再適用]
    GM[GNSS 観測モデル<br/>RTK-FIX 判定・レバーアーム]
    LM[LiDAR 観測モデル<br/>地図座標 → UTM]
    SM[スキャンマッチャ<br/>small_gicp]
    MTM[地図タイルマネージャ<br/>ロード / アンロード<br/>（gll_map）]
    OS[出力整形<br/>補正量のレート制限]
    ST[状態監視<br/>モード判定]
  end

  IMU --> SUB
  ODOM --> SUB
  GNSS --> SUB
  LIDAR --> SUB
  SUB --> ATT --> EKF
  SUB --> EKF
  SUB --> GM --> EKF
  SUB --> SM
  MTM --> SM
  EKF -- 予測姿勢（初期値）--> SM
  SM --> LM --> EKF
  EKF <--> HIST
  EKF -- 位置 --> MTM
  EKF --> OS --> PUB
  EKF --> ST --> PUB
```

データの流れ:

1. IMU（角速度）と ODOM（速度）で**予測**（デッドレコニング）を行う。
2. RTK-FIX の GNSS を**位置観測**として更新する。
3. LiDAR スキャンを、現在の予測姿勢を初期値として地図タイルにマッチングする。得られた地図座標系の姿勢を UTM に変換し、**姿勢観測**として更新する。
4. GNSS と LiDAR の観測は同じフィルタに入るため、両方が有効な区間（地図の縁など）では共分散に応じて重み付き融合される。どちらも無い区間はデッドレコニングでつなぐ。デッドレコニングで走った距離は監視し、長すぎれば diagnostics で ERROR を通知する（3.12 節）。

---

## 3. 推定アルゴリズム

### 3.1 方式の比較と選定

| 方式 | 連続性 | 計算コスト | 遅延観測 | 実装・保守 | 評価 |
|---|---|---|---|---|---|
| (A) GNSS localizer と LiDAR localizer の切り替え | × 切り替え時の差分を別途吸収する仕組みが必要 | ○ | △ | △ 切り替えロジックが肥大化しやすい | 不採用 |
| (B-1) EKF | ○ | ◎ | △（履歴バッファで対応） | ◎ | 2D では (B-2) と実質同等 |
| (B-2) ESEKF | ○ | ◎ | △（履歴バッファで対応） | ◎ | v0.2 までの採用案 |
| (B-3) UKF | ○ | ○ | △ | ○ | 非線形性が弱く、利点が小さい |
| (B-4) ファクターグラフ（iSAM2 / 固定ラグ平滑化） | ◎ | △ | ◎ | △ GTSAM 等への依存が重い | 計算負荷の理由で採用外 |
| **(B-5) Invariant EKF（SE(2)、3D 化時は SE₂(3)）** | ○ | ◎（EKF と同等） | △（履歴バッファで対応） | ○ SE(2) の演算の実装が必要 | **採用（v0.3）** |

**Invariant EKFを選ぶ理由**:

- 本構成は **GNSS がシングルアンテナ**で、起動直後や長いデッドレコニングの後は yaw の誤差が大きくなりうる。通常の EKF / ESEKF は、位置と yaw を別々の誤差として扱う。そのため、遷移行列や GNSS の観測行列が推定中の yaw（$`\hat\theta`$）に依存し、yaw の誤差が大きいと線形化が崩れて、収束の遅れや共分散の過小評価（不整合）が起きやすい。
- Invariant EKF は、姿勢 $`X = (\mathbf{R}(\theta), \mathbf{p}) \in SE(2)`$ の誤差を**群の上で**定義する。本設計の組み合わせ（機体座標系の速度入力による運動と、世界座標系で表される観測）では、次の性質が成り立つ（3.4〜3.7 節）。
  - **遷移行列は入力だけに依存し、推定中の姿勢には依存しない**。
  - **GNSS 位置観測の観測行列は定数になる**。
- その結果、yaw の誤差が大きい状態からでも収束が安定し、共分散の整合性も保ちやすい。計算量は EKF と同じである。
- 注意点: ジャイロバイアス $`b_\omega`$ と ODOM スケール $`s`$ を状態に加えると、厳密な不変性は崩れる（imperfect Invariant EKF）。ただし姿勢の部分の利点はそのまま残る。これは実用上の標準的な構成である。
- 3D 化（Phase 4）では、SE₂(3)（姿勢・速度・位置）上の Invariant EKF に IMU バイアスを加えた構成に拡張する。
- 推定器は `IStateEstimator` インターフェースの背後に置く。比較評価用に ESEKF 版も差し替えられるようにする（10 章 Phase 1 の評価項目）。

### 3.2 状態の自由度と roll / pitch について

要件どおり、出力は x, y, yaw とする。ただし、roll / pitch の可観測性については次の点に注意する。

- **「GNSS・IMU・ODOM では roll / pitch に観測補正がかからない」は、条件によっては成り立たない**。加速度計を使う 3D INS では、ODOM の速度（または車両の非ホロノミック拘束）で速度が拘束されると、比力と重力の関係から roll / pitch は可観測になる（加速度バイアスとは相関が残る）。また、スキャンマッチングは 6 自由度の姿勢を返すので、地図区間では roll / pitch も直接観測できる。
- 一方、Phase 1 の出力には roll / pitch は不要で、加速度計を使う 3D INS は調整項目が増える。そこで Phase 1 では roll / pitch をフィルタの状態に含めず、次の用途に限った**補助推定**（3.9 節）として扱う。
  - IMU の角速度を鉛直軸まわりのヨーレートに射影する（傾斜補正）。
  - ODOM の速度を水平成分に射影する（坂道で v·cos(pitch)）。
  - スキャンマッチングの 6 自由度初期値に使う。
  - GNSS アンテナのレバーアームを水平面に射影する（アンテナ高さの分の水平ずれを補正する。3.6 節）。
- 急な坂が多い、または z / roll / pitch も出力が必要になった場合は、Phase 4 で SE₂(3) の Invariant EKF に拡張する。

### 3.3 状態と誤差の定義

**推定状態**:

```math
\hat{X} = \begin{bmatrix} \mathbf{R}(\hat\theta) & \hat{\mathbf{p}} \\ \mathbf{0}^\top & 1 \end{bmatrix} \in SE(2),\qquad \hat b_\omega,\ \hat s
```

| 記号 | 意味 | 単位 |
|---|---|---|
| $`\hat{\mathbf{p}} = (\hat p_x, \hat p_y)`$ | base_link 原点の UTM 座標（Easting, Northing） | m |
| $`\hat\theta`$ | UTM グリッド座標系での yaw（x 軸 = East から反時計回り） | rad |
| $`\hat b_\omega`$ | ジャイロの鉛直軸バイアス | rad/s |
| $`\hat s`$ | ODOM 速度のスケール係数（名目値 1.0） | - |

**誤差の定義**（左不変誤差。誤差は機体座標系側で定義する）:

```math
X = \hat{X}\,\mathrm{Exp}(\xi),\quad \xi = \begin{bmatrix} \rho_x & \rho_y & \varphi \end{bmatrix}^\top,\qquad b_\omega = \hat b_\omega + \delta b,\quad s = \hat s + \delta s
```

```math
\delta\mathbf{x} = \begin{bmatrix} \rho_x & \rho_y & \varphi & \delta b & \delta s \end{bmatrix}^\top,\qquad \mathbf{P} = \mathrm{Cov}(\delta\mathbf{x}) \in \mathbb{R}^{5\times5}
```

**SE(2) の基本演算**（$`\mathbf{J} = \begin{bmatrix} 0 & -1 \\ 1 & 0 \end{bmatrix}`$）:

```math
\mathrm{Exp}(\rho, \varphi) = \big(\mathbf{R}(\varphi),\ \mathbf{V}(\varphi)\rho\big),\quad
\mathbf{V}(\varphi) = \frac{\sin\varphi}{\varphi}\mathbf{I} + \frac{1-\cos\varphi}{\varphi}\mathbf{J}
```

```math
\mathrm{Ad}_{(\mathbf{R},\mathbf{t})} = \begin{bmatrix} \mathbf{R} & -\mathbf{J}\mathbf{t} \\ \mathbf{0}^\top & 1 \end{bmatrix}
```

$`\mathrm{Log}`$ は $`\mathrm{Exp}`$ の逆写像で、$`\varphi \to 0`$ ではテイラー展開で評価する。これらは `gll/common/se2.hpp` に自前で実装する（数十行程度。単体テストで数値微分と照合する）。外部ライブラリ（manif 等）には依存しない。

$`s`$ の推定は、設定で無効にできるようにする（推定する量が増えると、GNSS と LiDAR が両方無い区間での振る舞いが不安定になりうるため）。

### 3.4 予測（IMU・ODOM 駆動）

入力:

- $`\omega_m`$: 傾斜補正済みのヨーレート（IMU、3.9 節）
- $`v_o`$: 傾斜補正済みの前進速度（ODOM）
- $`v_{\mathrm{lat}}`$: 横方向の速度（ODOM の `twist.linear.y`。通常の車両では 0。v0.8 で追加。スケール係数は前進速度にだけ掛ける）

予測は IMU のタイムスタンプで駆動する（100〜200 Hz）。ODOM の速度は、最新 2 サンプルの線形補間（外挿は最大 `odom_hold_max` 秒まで）で IMU の時刻にそろえる。IMU が途切れた場合は、ODOM のヨーレートで代替する（診断で WARN を出す）。

**推定状態の伝播**（機体座標系の移動量を群の上で積算する。数値積分の誤差が出ない）:

```math
\Delta\varphi = (\omega_m - \hat b_\omega)\Delta t,\quad \Delta\rho = \begin{bmatrix} \hat s\,v_o\,\Delta t \\ v_{\mathrm{lat}}\,\Delta t \end{bmatrix},\quad
\hat X_{k+1} = \hat X_k\,\mathrm{Exp}(\Delta\rho, \Delta\varphi),\quad \hat b_{\omega,k+1} = \hat b_{\omega,k},\ \hat s_{k+1} = \hat s_k
```

**誤差の遷移行列**: $`\mathrm{Exp}(\Delta\rho, \Delta\varphi) = (\mathbf{R}_\Delta, \mathbf{t}_\Delta)`$ とおくと

```math
\mathbf{F} = \begin{bmatrix}
\mathbf{R}_\Delta^\top & \mathbf{R}_\Delta^\top \mathbf{J}\,\mathbf{t}_\Delta & \mathbf{0} & \begin{bmatrix} v_o\Delta t \\ 0 \end{bmatrix} \\
\mathbf{0}^\top & 1 & -\Delta t & 0 \\
\mathbf{0}^\top & 0 & 1 & 0 \\
\mathbf{0}^\top & 0 & 0 & 1
\end{bmatrix}
```

（左上の 3×3 は $`\mathrm{Ad}_{\mathrm{Exp}(\Delta\rho,\Delta\varphi)^{-1}}`$。バイアスの列では、右ヤコビアン $`\mathbf{J}_r(\Delta) \approx \mathbf{I}`$ と近似している。IMU の周期が短いので $`|\Delta\varphi| \ll 1`$ となり、この近似は十分成り立つ）

- **$`\mathbf{F}`$ には推定中の姿勢 $`\hat\theta, \hat{\mathbf{p}}`$ が現れない**（入力 $`v_o, \omega_m`$ とバイアス推定値だけで決まる）。これが ESEKF との本質的な違いである。
- 例: 直進中（$`\mathbf{t}_\Delta = (d, 0)`$）は、yaw の誤差 $`\varphi`$ が機体の横方向の誤差 $`\rho_y`$ に $`d\varphi`$ だけ移る。これは直感とも一致する。

**プロセスノイズ**（入力ノイズ $`\sigma_v, \sigma_\omega`$ と、ランダムウォーク $`\sigma_b, \sigma_s`$）:

```math
\mathbf{G} = \begin{bmatrix}
\hat s\,\Delta t & 0 \\
0 & 0 \\
0 & \Delta t \\
0 & 0 \\
0 & 0
\end{bmatrix},\quad
\mathbf{Q} = \mathbf{G}\,\mathrm{diag}(\sigma_v^2, \sigma_\omega^2)\,\mathbf{G}^\top + \mathrm{diag}(0,0,0,\sigma_b^2\Delta t, \sigma_s^2\Delta t),\qquad
\mathbf{P} \leftarrow \mathbf{F}\mathbf{P}\mathbf{F}^\top + \mathbf{Q}
```

横すべりを許容したい場合は、$`\rho_y`$ に横方向の速度ノイズ $`\sigma_{v,lat}`$ を加える（既定 0.02 m/s。非ホロノミック拘束の不確かさに相当する）。

**停止中**（|v_o| と |ω| が閾値未満の状態が一定時間続いたとき）: ODOM の速度が 0 なので位置は動かない。ゼロ角速度の擬似観測（ZARU）$`z = \bar\omega_m`$（停止中の平均）、$`h = \hat b_\omega`$、$`\mathbf{H} = [0\ 0\ 0\ 1\ 0]`$ で $`b_\omega`$ を推定し、停止中の yaw のドリフトを抑える。

### 3.5 観測更新（共通手順）

すべての観測で、**残差を機体座標系（誤差 $`\xi`$ と同じ座標系）で表す**のが Invariant EKF の要点である。

1. 観測時刻 $`t_z`$ の状態を履歴バッファから取り出す（3.8 節）。
2. 観測ごとの式（3.6、3.7 節）で残差 $`\mathbf{r}`$、観測行列 $`\mathbf{H}`$、観測共分散 $`\mathbf{R}`$ を求める。
3. $`\mathbf{S} = \mathbf{H}\mathbf{P}\mathbf{H}^\top + \mathbf{R}`$ から Mahalanobis 距離 $`d^2 = \mathbf{r}^\top \mathbf{S}^{-1}\mathbf{r}`$ を求める。$`d^2 > \chi^2_{\mathrm{dof}}(1-\alpha)`$ なら棄却する（既定 α = 0.001 → 1 自由度: 10.8、2 自由度: 13.8、3 自由度: 16.3）。
   - **例外**: RTK-FIX の GNSS は棄却しない。GNSS FIX 中の LiDAR は、Mahalanobis 距離ではなく固定閾値で食い違いを判定する。棄却が続いたときの再アンカーと再位置推定も含め、3.13 節に従う。
4. $`\mathbf{K} = \mathbf{P}\mathbf{H}^\top\mathbf{S}^{-1}`$、$`\delta\mathbf{x} = \mathbf{K}\mathbf{r} = (\delta\xi, \delta b, \delta s)`$
5. 注入: $`\hat X \leftarrow \hat X\,\mathrm{Exp}(\delta\xi)`$、$`\hat b_\omega \leftarrow \hat b_\omega + \delta b`$、$`\hat s \leftarrow \hat s + \delta s`$
6. 共分散（Joseph 形式）: $`\mathbf{P} \leftarrow (\mathbf{I}-\mathbf{K}\mathbf{H})\mathbf{P}(\mathbf{I}-\mathbf{K}\mathbf{H})^\top + \mathbf{K}\mathbf{R}\mathbf{K}^\top`$
7. リセット: 厳密には $`\mathbf{P} \leftarrow \mathbf{J}_r(\delta\xi)\,\mathbf{P}\,\mathbf{J}_r(\delta\xi)^\top`$ だが、$`\delta\xi`$ は小さいので単位行列で近似する（関数は分けておき、必要なら後で有効化する）。
8. $`t_z`$ 以降の入力を再適用して、現在時刻まで伝播し直す。
9. 更新前後の出力姿勢の差（世界座標系の $`\Delta x, \Delta y, \Delta\theta`$）を出力整形層に通知する（3.10 節）。

**フィルタの推定値の共分散**（世界座標系の x, y, yaw）: $`\Sigma_w = \mathbf{T}\,\mathbf{P}_{1:3,1:3}\,\mathbf{T}^\top`$、$`\mathbf{T} = \mathrm{blkdiag}(\mathbf{R}(\hat\theta), 1)`$。外に出す共分散は、これに出力整形層のオフセットの分を加えたものになる（3.10 節）。

### 3.6 GNSS 観測（u-blox F9P, `sensor_msgs/NavSatFix`）

v0.5 で、GNSS の入力を標準の `sensor_msgs/NavSatFix` に変更した（ROS 1 にも同じ型があるので、移植も容易になる）。ただし `NavSatFix` には、RTK の FIX / FLOAT を区別する標準の値と、速度・進行方位が無い。そこで、**ドライバとの間で次の取り決め**を置く。

**ドライバとの取り決め**（独自ドライバ側で実装してもらう）:

| 項目 | 取り決め |
|---|---|
| RTK-FIX の表現 | UBX-NAV-PVT の `carrSoln == 2`（fixed）かつ `gnssFixOK` のときだけ、`status.status = STATUS_GBAS_FIX (2)` にする。RTK-FLOAT、DGPS、単独測位は `STATUS_FIX (0)` または `STATUS_SBAS_FIX (1)` にする。コア側で使う値はパラメータ `gnss_rtk_fix_status`（既定 2）で変えられる |
| 共分散 | `position_covariance`（ENU、[m²]）に `hAcc²`（E, N）と `vAcc²`（U）を対角で入れ、`position_covariance_type = COVARIANCE_TYPE_DIAGONAL_KNOWN (2)` にする |
| 高さ | `altitude` は WGS84 の楕円体高にする（ROS の標準の定義どおり。平均海面高ではない） |
| 時刻 | `header.stamp` は、できれば測位時刻（GNSS 時刻をシステム時刻に換算したもの）にする。受信時刻を入れる場合は、パラメータ `gnss_stamp_offset`（既定 0 s。負の値で過去側に補正）で遅延を補正する |
| 速度（任意） | 進行方位の観測を使いたい場合だけ、別トピックで `geometry_msgs/TwistWithCovarianceStamped` を出す（`twist.linear.x` = 東向き速度、`y` = 北向き速度 [m/s]、共分散は `sAcc²`） |

**採用条件**（すべて満たすときだけ RTK-FIX として更新に使う）:

- `status.status == gnss_rtk_fix_status`（既定 2）
- `position_covariance_type` が UNKNOWN 以外で、水平の標準偏差 $`\sqrt{\max(\Sigma_{EE}, \Sigma_{NN})}`$ が `gnss_max_stddev`（既定 0.05 m）以下。取り決めがずれていて RTK-FLOAT が status 2 で届いても、FLOAT は通常 σ が数 cm〜数十 cm なので、ここで落ちる（二重の安全策）
- FIX に遷移してから `gnss_fix_settle_time`（既定 1.0 s）が経過している（FIX 直後の誤 FIX 対策）
  - GNSS のメッセージが `gnss_settle_reset_gap`（既定 3.0 s）以上途切れた場合も、FIX が一度切れたものとみなして安定待ちからやり直す（v0.9）。受信できない区間でドライバが何も出さない場合に、途切れる前の FIX から続いているとみなさないため
- `position_covariance_type == UNKNOWN` の場合は精度を確認できないので、既定では採用しない。`gnss_accept_unknown_covariance: true` にすると、`gnss_default_stddev` を使って採用する（その場合は WARN を出す）

**位置観測**:

- 緯度経度を、サイトで固定した 1 つの UTM ゾーンの $`\mathbf{y} = (E, N)`$ に変換する。ゾーンは設定値で固定し、ゾーン境界をまたいでも切り替えない。
- アンテナのレバーアームを 3 次元で $`\mathbf{l} = (l_x, l_y, l_z)`$（base_link 座標系、`gnss_lever_arm`）とする。**アンテナ高さ $`l_z`$ は未定だが 1 m 以内**の想定（v0.6）。
- **傾きによるアンテナの水平方向のずれ**: 車両が傾くと、高い位置にあるアンテナは水平方向にずれる。$`l_z`$ = 1 m なら、傾き 1° で約 1.7 cm、5° の坂で約 8.7 cm ずれ、RTK の精度（数 cm）から見て無視できない。そこで、姿勢推定器（3.9 節）の roll $`\phi`$ / pitch $`\vartheta`$（観測時刻の値）で、レバーアームを水平面に射影してから使う。

```math
\tilde{\mathbf{l}} = \big[\mathbf{R}_y(\vartheta)\,\mathbf{R}_x(\phi)\,\mathbf{l}\big]_{xy} \;\approx\; \begin{bmatrix} l_x + \vartheta\,l_z \\ l_y - \phi\,l_z \end{bmatrix}
```

  （$`\tilde{\mathbf{l}}`$ は、yaw だけ回した水平な座標系で見たアンテナの位置。実装では近似ではなく左辺の厳密な式で計算する）

- 観測モデルは $`\mathbf{y} = \mathbf{p} + \mathbf{R}(\theta)\tilde{\mathbf{l}} + \mathbf{n}`$ で、これは $`\mathbf{y} = X\,(\tilde{\mathbf{l}}, 1)`$ という**左不変観測**の形になる。そのため、残差を機体座標系で取ると観測行列が定数になる（$`\tilde{\mathbf{l}}`$ は姿勢推定器から与えられる既知の量として扱う）。

```math
\mathbf{r} = \mathbf{R}(\hat\theta)^\top\big(\mathbf{y} - \hat{\mathbf{p}} - \mathbf{R}(\hat\theta)\tilde{\mathbf{l}}\big),\qquad
\mathbf{H} = \begin{bmatrix} \mathbf{I}_2 & \mathbf{J}\tilde{\mathbf{l}} & \mathbf{0} & \mathbf{0} \end{bmatrix} = \begin{bmatrix} 1 & 0 & -\tilde l_y & 0 & 0 \\ 0 & 1 & \ \ \tilde l_x & 0 & 0 \end{bmatrix}
```

```math
\mathbf{R} = \mathbf{R}(\hat\theta)^\top\,\Sigma_{\mathrm{gnss}}\,\mathbf{R}(\hat\theta) + l_z^2\,\mathrm{diag}\big(\sigma_\vartheta^2,\ \sigma_\phi^2\big),\qquad \Sigma_{\mathrm{gnss}} = \max\!\big(\Sigma_{EN},\ \sigma_{\min}^2\mathbf{I}_2\big)
```

- 第 2 項は、roll / pitch の推定誤差（標準偏差 $`\sigma_\phi, \sigma_\vartheta`$、既定 `attitude_stddev` = 0.5°）がアンテナの水平位置の誤差になる分である。$`l_z`$ = 1 m、0.5° なら約 0.9 cm。残差と同じ機体座標系で表されているので、回転せずにそのまま加える。
- $`\Sigma_{EN}`$ は `position_covariance` の東・北の 2×2 ブロック。$`\max`$ は対角要素ごとに下限を取る意味。
- $`\sigma_{\min}`$ = `gnss_min_stddev`（既定 0.02 m）。受信機が報告する精度は楽観的なことが多いので下限を設ける。
- UTM 座標は、グリッドの東・北方向と ENU の東・北方向の違い（子午線収差 γ、数度以内）で共分散を回転させるのが厳密である。ただし等方的な共分散（E と N の分散が同じ）なら影響しないので、無視する。
- 導出: $`X\,\mathrm{Exp}(\xi)\cdot\tilde{\mathbf{l}} \approx \hat X\cdot(\tilde{\mathbf{l}} + \rho + \varphi\mathbf{J}\tilde{\mathbf{l}})`$ より、$`\mathbf{R}(\hat\theta)^\top(\mathbf{y}-\hat{\mathbf{y}}) \approx \rho + \varphi\mathbf{J}\tilde{\mathbf{l}} + \text{noise}`$。
- ESEKF では $`\mathbf{H}`$ が $`\hat\theta`$ に依存していた。Invariant EKF では yaw の推定誤差が大きくても、観測行列そのものは誤らない。

**ヨーの観測**（F9P はシングルアンテナのため、ヘディングを直接は得られない）:

- **位置観測の系列（主）**: 走行中は、$`\mathbf{F}`$ の結合項（yaw 誤差 → 横方向誤差）を通して、位置観測からヨーが推定される。停止中は観測されない（ZARU でドリフトだけを抑える）。`NavSatFix` だけを使う既定の構成では、これが唯一の GNSS 由来のヨーの情報源になる。Invariant EKF はこの形の推定に強いので、実用上は十分と見込む（Phase 1 のシミュレーションで確認する）。
- **進行方位の観測（任意、既定は無効）**: ドライバが速度トピック（上の取り決め）を出す場合だけ、`use_gnss_velocity: true` で有効にする。
  - 条件: RTK-FIX 中で、水平速度 $`|\mathbf{v}| >`$ `cog_min_speed`（既定 0.5 m/s）、|ヨーレート| < `cog_max_yaw_rate`（既定 5 deg/s。レバーアームによる速度成分の影響を避けるため）、かつ前進中。
  - $`z = \mathrm{atan2}(v_N, v_E) + \gamma`$（ENU の角度をグリッドの角度に変換。$`\gamma`$ は子午線収差）
  - $`r = \mathrm{wrap}(z - \hat\theta)`$、$`\mathbf{H} = [0\ 0\ 1\ 0\ 0]`$
  - 分散: $`\sigma_\psi \approx \sigma_v / |\mathbf{v}|`$。最高速の 1.7 m/s でも、$`\sigma_v`$ = 0.05 m/s なら約 1.7° で、**補助的な観測**にとどまる。
  - 後退中は使わない。

### 3.7 LiDAR 観測

スキャンマッチング（6 章）の結果として、地図グループ $`g`$ の座標系での base_link の 6 自由度姿勢 $`\mathbf{T}^{g}_{\mathrm{base}}`$ と、その共分散が得られる。共分散は機体座標系側の摂動に対するもので、small_gicp の定義による。

1. 地図 → UTM 変換 $`\mathbf{T}^{\mathrm{utm}}_{g}`$（4.2 節）で UTM に変換する。$`g`$ は、照合に使ったターゲットのグループである（照合の途中でアクティブグループが切り替わっても、ターゲットのグループのアンカーを使う。5.5 節）。
2. x, y, yaw を取り出して、SE(2) の観測 $`Z = (\mathbf{R}(\psi_z), \mathbf{p}_z)`$ とする。

```math
\mathbf{r} = \mathrm{Log}\big(\hat X^{-1} Z\big) \in \mathbb{R}^3,\qquad \mathbf{H} = \begin{bmatrix} \mathbf{I}_3 & \mathbf{0}_{3\times2} \end{bmatrix}
```

（$`\mathrm{Log}`$ のヤコビアンは $`\mathbf{I}`$ で近似する）

観測共分散（すべて機体座標系で表す）:

```math
\mathbf{R} = \Sigma_{\mathrm{reg}} + \mathbf{T}^\top\Sigma_{\mathrm{anchor}}\mathbf{T} + \Sigma_{\mathrm{floor}},\qquad \mathbf{T} = \mathrm{blkdiag}(\mathbf{R}(\hat\theta), 1)
```

- $`\Sigma_{\mathrm{reg}}`$: スキャンマッチングの共分散（6.2 節の式）から、(x, y, yaw) の成分を取り出したもの。small_gicp の情報行列はもともと機体座標系側の摂動に対するものなので、**座標変換をせずにそのまま使える**。これも左不変誤差を選んだ利点である（roll / pitch が小さいことを前提に、SE(3) → SE(2) の射影として近似する）。
- $`\Sigma_{\mathrm{anchor}}`$: アンカーの不確かさ（世界座標系で与える。maps.yaml の `stddev_xy` / `stddev_yaw_deg`）。
- $`\Sigma_{\mathrm{floor}}`$: 下限値（`min_stddev_xy` 既定 0.02 m、`min_stddev_yaw` 既定 0.2°）。

**注意**: アンカー誤差は時間的に相関するバイアスで、白色雑音ではない。上の式はそれを保守的に近似しているだけである。GNSS と LiDAR が両方有効な区間で、両者の差からアンカー誤差をオンライン推定する拡張（状態に地図グループごとのオフセットを追加する）は Phase 4 の検討事項とする。

**採用条件**（実装: `LidarMeasurementBuilder`）:

- 位置合わせが収束した（反復回数が上限未満）。
- インライア率（`min_inlier_ratio` 既定 0.6）と overlap（`min_overlap` 既定 0.5）が閾値を満たす（6.2 節）。
- 初期値（予測姿勢）からの移動量が `max_jump_xy` / `max_jump_yaw`（既定 1.0 m / 5°）以内。
- GNSS FIX 中は 3.13.2 節の食い違いの判定、それ以外は 3.5 節の Mahalanobis ゲートを通過した。

品質の条件を満たさなかった結果と、ゲートに落ちた結果は、どちらも「LiDAR の棄却」として数え、続けば再位置推定を行う（3.13.4 節）。

### 3.8 遅延観測の扱い

LiDAR 観測は「スキャン時刻 + マッチングの処理時間（数十 ms）」だけ遅れて届き、GNSS にも受信機の遅延がある。そこで、次の**状態履歴バッファ**で遅延観測を扱う。

- 予測ステップごとに $`(t, \mathbf{x}, \mathbf{P}, \mathbf{u})`$ をリングバッファ（既定 2.0 s）に保存する。
- 時刻 $`t_z`$ の観測が届いたら、$`t_z`$ の直前のエントリから $`t_z`$ まで伝播し、そこで更新する。その後、保存しておいた入力 $`\mathbf{u}`$ で現在時刻まで再伝播する。
- 状態が 5 次元と小さいため、2 秒分（IMU 200 Hz で 400 ステップ）を再伝播しても 1 ms 未満で済む見込み。
- バッファより古い観測は破棄し、カウンタを診断に出す。

スキャンマッチングの初期値も、この履歴から得た**スキャン時刻の予測姿勢**を使う。

### 3.9 補助推定: roll / pitch / z

フィルタの状態には含めないが、傾斜補正とスキャンマッチングの初期値に必要な量:

| 量 | GNSS 区間 | 地図区間 |
|---|---|---|
| roll / pitch | 姿勢推定器 `AttitudeEstimator`（下記） | 同左。スキャンマッチングが収束したら、その roll / pitch で補正する（相補的にブレンド） |
| z | GNSS の楕円体高からアンテナ高さ分を引く（$`h - [\mathbf{R}_y(\vartheta)\mathbf{R}_x(\phi)\mathbf{l}]_z`$） | 直前のスキャンマッチング結果の z を、そのグループのアンカーで楕円体高に戻したもの（次の結果までは保持） |

z は常に楕円体高（`map` の z）で保持する（v0.9）。スキャンマッチングの初期値に使うときに、照合に使うターゲットのグループのアンカー（4.2 節）で地図の z に変換する。地図グループの座標系の z のまま持ち越すと、別のグループに入ったときに、初期値の z がアンカーの高さの差だけずれてしまうため。

**姿勢推定器 `AttitudeEstimator`**（IMU が姿勢を出力しないため自前で実装）:

- 方式: Mahony 型の相補フィルタ。ジャイロ 3 軸で姿勢を積分し、加速度計から推定した重力方向で roll / pitch を補正する。ヨーはこの推定器では扱わない（Invariant EKF 側で扱う）。
- 運動加速度の補償: 加速度計の値から、車両の運動による加速度を差し引いてから重力方向を求める。機体座標系で次のように近似する（低速なので十分）。
  ```math
  \mathbf{a}_{\mathrm{lin}} \approx \begin{bmatrix} \dot v_o & v_o\,\omega_z & 0 \end{bmatrix}^\top,\qquad \mathbf{g}_b \approx \mathbf{a}_m - \mathbf{a}_{\mathrm{lin}}
  ```
  （$`\dot v_o`$ は ODOM 速度の差分を平滑化したもの。$`v_o\omega_z`$ は向心加速度）
- 補正ゲインの調整: $`\big|\|\mathbf{g}_b\| - g\big|`$ が大きいとき（段差や衝撃）は補正ゲインを下げる。
- ジャイロバイアス: 起動時に静止状態で `imu_static_init_time`（既定 3 s）の平均から 3 軸のバイアスを求める。以後は停止を検出するたびに更新する（鉛直軸のバイアスは Invariant EKF の $`b_\omega`$ でも推定する）。

傾斜補正:

- ヨーレート: $`\omega_m = [\mathbf{R}_{wb}(\phi, \vartheta)\,\omega_{imu}]_z`$
- 前進速度: $`v_o = v_{odom}\cos\vartheta`$（$`\phi`$: roll、$`\vartheta`$: pitch）

### 3.10 出力の連続性（FR-4）

単一フィルタにすれば推定ソースの切り替えによる不連続は無くなるが、**観測更新そのものによる補正ステップ**は残る。例えば LiDAR が 10 Hz で数 cm ずつ補正すると、出力はその都度数 cm 動く。長いデッドレコニングの後や、地図アンカーに誤差がある場合は、補正ステップが大きくなりうる。これを次の 3 段で抑える。

1. **ゲートと共分散**（3.5 節、3.13 節）: 外れ値は取り込まない。共分散が妥当なら、1 回の補正量はもともと小さい。再アンカーのように推定値を大きく寄せ直す場合も、推定値の変化は次の出力整形層で吸収する。
2. **出力整形層（補正オフセット吸収方式）**:
   - 観測更新で推定姿勢が世界座標系で $`(\Delta x, \Delta y, \Delta\theta)`$ だけ動いたら（3.5 節の手順 9）、出力側のオフセット $`\mathbf{o}`$ からその分を引く。
   - 出力は $`\mathbf{y} = \mathbf{x}_{(x,y,\theta)} + \mathbf{o}`$ とする。
   - $`\mathbf{o}`$ は毎周期、最大 `max_correction_rate_xy`（既定 0.1 m/s）と `max_correction_rate_yaw`（既定 2 deg/s）の速さで 0 に近づける。
   - これにより、出力の変化は「デッドレコニングによる滑らかな移動 + レート制限された補正」だけになる。
   - $`\|\mathbf{o}\|`$ が `offset_error_threshold`（既定 1.0 m / 5 deg）を超えた場合は、黙って飛ばさずに状態を `DEGRADED` にして下流に通知する。
   - フィルタの生の推定値もデバッグ用に別トピックで出す。
   - **出力の共分散**: オフセットが残っている間、出力はフィルタの推定値（その時点での最良推定）から $`\mathbf{o}`$ だけずれている。この既知のずれを、出力の誤差の 2 乗平均に含めて公開する。

     ```math
     \Sigma_{\mathrm{out}} = \Sigma_w + \mathbf{o}\,\mathbf{o}^\top
     ```

     - $`\Sigma_w`$ はフィルタの推定値の共分散（3.5 節）。$`\mathbf{o} = (o_x, o_y, o_\theta)`$ は世界座標系のオフセット。
     - $`\mathbf{o}\mathbf{o}^\top`$ は、オフセットの方向にだけ不確かさを広げる（例: 横方向に 0.3 m のオフセットが残っていれば、フィルタの σ が数 cm でも、出力の横方向の標準偏差は $`\sqrt{\sigma^2 + 0.3^2} \approx`$ 0.3 m になる）。オフセットが吸収されて 0 に戻れば、フィルタの共分散と一致する。
     - これにより、下流（経路追従など）は、再アンカーの直後など「出力がまだ最良推定に追いついていない」間の不確かさを、共分散から正しく読み取れる。
     - `DEGRADED` の判定（オフセットが閾値を超えたとき）は従来どおり。共分散の増加は、それより小さいオフセットのときにも連続的に反映される点が異なる。
   - 補正を遅らせることは、真の誤差の修正を遅らせることと表裏一体である。そのため、レートは車速や経路追従の特性に合わせて調整する（パラメータを大きくすれば素通しになる）。
3. **地図アンカーの事前較正**: GNSS と LiDAR の両方が有効な区間（地図の縁）での両者の差を小さくしておくことが、切り替え時の滑らかさに最も効く。GNSS ログとスキャンマッチング結果の差を最小二乗で推定するアンカー較正ツールを用意する（10 章 Phase 3）。

### 3.11 初期化

| 開始地点 | 手順 |
|---|---|
| GNSS 区間 | ① RTK-FIX を待つ → 位置を初期化する。② 走行して `init_heading_min_distance`（既定 1 m）進んだら、GNSS の変位の向きから粗い yaw を決め、$`\sigma_\varphi`$ = `init_yaw_stddev`（既定 15°）でフィルタを始動する。③ 以後は Invariant EKF が位置観測と進行方位の観測で yaw を詰める。$`\sigma_\varphi`$ < `ready_yaw_stddev`（既定 2°）かつ位置の σ < `ready_pos_stddev`（既定 0.1 m）になったら初期化完了とする。外部から初期姿勢が与えられた場合はそれを優先する。シングルアンテナのため、停止したままでは yaw を決められない。Invariant EKF は yaw の誤差が大きくても線形化が崩れにくいので、粗い yaw から始めても安定して収束する |
| 地図区間 | 外部から初期姿勢を与えるか、前回保存した位置を使う → その周りで、位置の格子 × yaw の初期値から位置合わせを試し、一意に決まった最良の結果で初期化する（下記。v0.10 で実装）。GICP は収束する範囲が狭いので、粗い VGICP（ボクセル 2.0 m）で候補を絞ってから GICP で詰める 2 段階にする（6.4 節） |
| 共通 | 初期化が完了するまでは、出力を `INITIALIZING` として下流に使わせない |

**地図上での初期化**（v0.10。GNSS 区間が無く、点群地図だけがある現場で必要になる）:

1. **初期姿勢**: 次のどちらかで与える。
   - 外部から与える: `~/input/initial_pose`（RViz の「2D Pose Estimate」など）。数 m・数十度ずれていてもよい。共分散が入っていなければ 1 m・30° とみなす。
   - 前回保存した位置: `init.saved_pose_path` に推定した位置を定期的に保存しておき（既定 1 s ごとと終了時）、`init.use_saved_pose: true` なら起動時にそれを初期姿勢にする（σ = 0.5 m・10°）。止めた場所から動かさずに起動する運用を想定する。フィルタがすでに動いていれば使わない。
2. **地図の上か**: 初期姿勢がどれかのタイルから `init_map_distance`（既定 10 m）以内なら、地図上で初期化する（初期化フェーズ `WAIT_MAP_MATCH`。この間は GNSS による初期化も行わない）。地図の外なら、外部から与えた初期姿勢はそのまま使い、保存した位置は使わない（GNSS を待つ）。
3. **探索範囲**: 位置は初期姿勢の共分散の 3σ（`init_min_radius`〜`init_max_radius` = 0.5〜5 m に収める）、yaw は 3σ（10°〜180°）。z は、推定した楕円体高があればそれを、無ければ地図の地面の高さ（周り 3 m の点の z の下から 10 %）に `base_link_height` を足したものを使う。roll / pitch は姿勢推定器の値。
4. **位置合わせ**: 初期姿勢の後の最初のスキャン（姿勢推定器の静止初期化が済んでから）で、6.4 節の多仮説の探索を行う。最良の解が品質の条件を満たし、次点（1 m または 10° 以上離れた解）と区別できれば（overlap の比 ≥ 1.5）、その姿勢で初期化する。初期の共分散は、位置合わせの共分散にアンカーの不確かさを足したもの。
5. **失敗したとき**: 次のスキャンでやり直す。`init_max_attempts`（既定 5）回失敗したら、外部から与えた初期姿勢はそのまま使って初期化し（WARN）、保存した位置は捨てて GNSS か初期姿勢を待つ。LiDAR のデータが来ない、地図のターゲットができないなどで、姿勢推定器の静止初期化の後 `init_timeout`（既定 15 s）たっても決まらないときも同じに扱う（保存した位置のせいで、GNSS による初期化が止まったままにならないように）。
6. **対象外**: 初期姿勢をまったく与えない、地図全体からの探索（大域的な自己位置推定）は行わない（1 章 D-12）。

合成環境のシミュレーションでは、1 m・12° ずれた初期姿勢から、203 通りの候補を試して 0.8 s で初期化できた（9 章）。

### 3.12 状態監視（モード）

推定器は 1 つだが、どの観測が効いているかを状態として公開し、監視・デバッグに使う。

| 状態 | 条件 |
|---|---|
| `INITIALIZING` | 初期化が未完了 |
| `GNSS_AIDED` | 直近 `aid_timeout`（既定 1.0 s）以内に GNSS 観測だけを採用した |
| `LIDAR_AIDED` | 直近 `aid_timeout` 以内に LiDAR 観測だけを採用した |
| `GNSS_LIDAR_AIDED` | 両方を採用した |
| `DEAD_RECKONING` | どちらも採用していない。ただし位置の標準偏差が `dr_max_stddev`（既定 0.3 m）未満 |
| `DEGRADED` | 位置の標準偏差が `dr_max_stddev` 以上、または出力オフセットが閾値を超えた |
| `LOST` | 位置の標準偏差が `lost_stddev`（既定 1.0 m）以上、または棄却が連続した → 再初期化が必要 |

経路追従側は `DEGRADED` で減速、`LOST` で停止する、といった使い方を想定する（振る舞いは下流側の設計で決める）。

#### デッドレコニング距離の監視（v0.9）

運用上の前提（要件定義 3 章）により、地図区間と GNSS 区間の境目でデッドレコニングになる区間は短い。それが想定より長く続いた場合（GNSS が FIX しない、LiDAR の照合が失敗し続けるなど、前提が崩れた場合）を検知して、diagnostics で ERROR を通知する（FR-8）。

- **デッドレコニング距離** $`d_{\mathrm{DR}}`$: 最後に**位置の観測**を採用してから走った距離。予測のたびに次のように積算する（$`v_o`$: 前進速度、$`v_{\mathrm{lat}}`$: 横速度、$`\hat s`$: ODOM スケールの推定値）。

  ```math
  d_{\mathrm{DR}} \leftarrow d_{\mathrm{DR}} + \sqrt{(\hat s\,v_o)^2 + v_{\mathrm{lat}}^2}\;\Delta t
  ```

- **0 に戻すとき**: RTK-FIX の GNSS 位置、LiDAR の姿勢観測（どちらも再アンカーを含む）を採用したとき、およびフィルタの初期化・外部からの初期姿勢の指定のとき。進行方位の観測と ZARU は位置を直さないので、0 に戻さない。
- **判定**: $`d_{\mathrm{DR}}`$ が `dr_error_distance`（既定 30 m。仮の値）を超えたら ERROR にする。位置の観測を採用すると 0 に戻り、ERROR も解除される。初期化が完了する前（`INITIALIZING`）は判定しない。
- **時間ではなく距離で判定する理由**: デッドレコニングの誤差は、主に走った距離に比例して増える（ODOM のスケール誤差、yaw の誤差 × 距離）。停止中は ZARU があるのでほとんど増えない。時間で判定すると、GNSS の無い場所で停車しただけでエラーになってしまう。
- **共分散による判定（`DEGRADED` / `LOST`）との関係**: 共分散は、モデルが正しければ誤差の大きさを表すが、モデル化していない誤差（スリップ、スケールの変化など）には鈍感である。距離による判定は、運用の前提を直接監視する、モデルに依存しない安全策として並べて使う。
- **既定値について**: 30 m は仮の値である。例えば ODOM スケールの誤差が 0.5 % 残っていれば 30 m で縦に 15 cm、yaw の誤差が 0.3° なら横に約 16 cm ずれる。経路追従が許せる誤差と、地図と GNSS 区間の境目でのデッドレコニング区間の最大長から決める。i2Nav-Robot の V1-2（GNSS の間引き）でデッドレコニング中の誤差の伸びを測ってから決める（[検証計画](./validation_i2nav.md)）。

diagnostics（`~/output/status` と、`/diagnostics` の `gll_localizer`）のレベルとメッセージ:

| 条件 | レベル | メッセージ |
|---|---|---|
| `GNSS_AIDED` / `LIDAR_AIDED` / `GNSS_LIDAR_AIDED` / `DEAD_RECKONING` | OK | 状態名 |
| `INITIALIZING` / `DEGRADED` | WARN | 状態名 |
| `LOST` | ERROR | 状態名 |
| $`d_{\mathrm{DR}}`$ が `dr_error_distance` を超えた | **ERROR**（上の 3 行より優先） | `<状態名>: dead reckoning for <距離> m without GNSS / LiDAR position (limit <上限> m)` |

- 例: `DEAD_RECKONING: dead reckoning for 31.2 m without GNSS / LiDAR position (limit 30.0 m)`
- 値（`values`）には `dr_distance_m`（現在の $`d_{\mathrm{DR}}`$）と `dr_error_distance_m`（上限）も入れる。ERROR になったときと解除されたときは、ログ（`RCLCPP_ERROR` / `RCLCPP_INFO`）にも出す。
- 下流（経路追従など）がこの ERROR で停止するか減速するかは、`DEGRADED` / `LOST` と同じく下流側で決める（11 章）。

### 3.13 GNSS / LiDAR の優先度・食い違いの判定・復帰（v0.4）

推定器は 1 つで、GNSS と LiDAR を切り替える「モード」は持たない。そのため、ここでいう「切り替えのロジック」は、次の 3 つからなる。

1. どちらの観測を優先するか
2. 両方が有効なときに食い違ったらどうするか
3. 推定値がずれて観測を受け付けなくなったとき、どう復帰するか

#### 3.13.1 優先度の方針

**RTK-FIX の GNSS は常に信用する**（v0.4 で確定）。ここでいう RTK-FIX は、3.6 節の採用条件（status が RTK-FIX を表す値、水平精度、FIX 後の安定待ち）を満たしたものを指す。

| 状況 | GNSS（RTK-FIX） | LiDAR |
|---|---|---|
| GNSS のみ有効 | 融合する | — |
| 両方有効で、整合している | 融合する（主） | 共分散を `lidar_cov_inflation_under_fix`（既定 4.0）倍に膨らませて融合する。主に yaw の補強（シングルアンテナの弱点を補う） |
| 両方有効で、食い違っている | 融合する（主）。推定値と食い違う場合は再アンカー（3.13.3 節） | **棄却する**。地図グループごとの食い違いとして記録する（3.13.2 節） |
| LiDAR のみ有効 | — | 融合する（主）。ゲートと復帰は 3.13.3 節 |
| どちらも無効 | デッドレコニング。共分散の増加に応じて状態を `DEGRADED` / `LOST` にする。走った距離が `dr_error_distance` を超えたら diagnostics で ERROR を通知する（3.12 節） | |

- 「両方有効」とは、直近 `aid_timeout`（既定 1.0 s）以内に RTK-FIX の GNSS を採用していることを指す。
- **残るリスク**: マルチパスによる誤った FIX も「信用」されるため、推定値がそちらに引っぱられる。ただし出力の変化は出力整形層でレート制限されるので、出力が飛ぶことはない（FR-4）。補正量が閾値を超えれば `DEGRADED` を通知する。このリスクは、方針として受け入れる。

#### 3.13.2 食い違いの判定（GNSS FIX 中の LiDAR）

GNSS FIX 中は、推定値はほぼ GNSS で決まっている。そこで、LiDAR の観測 $`Z`$ と推定値との差を「GNSS と LiDAR の食い違い」とみなす。

```math
\mathbf{e} = \mathrm{Log}\big(\hat X^{-1} Z\big) = (e_x, e_y, e_\psi)
```

- $`\mathbf{e}`$ は機体座標系で表される。
- $`\|(e_x, e_y)\| \le`$ `consistency_xy`（既定 0.15 m）かつ $`|e_\psi| \le`$ `consistency_yaw`（既定 1.0°）なら**整合**とする → 共分散を膨らませて融合する。
- それ以外は**食い違い**とする → LiDAR を棄却する。
- 判定は Mahalanobis 距離ではなく、固定の閾値で行う。GNSS FIX 中は $`\mathbf{P}`$ が小さく、Mahalanobis 距離では数 cm の差でも棄却されてしまうため。

**アンカーずれの記録**: 食い違いの有無にかかわらず、GNSS FIX 中の $`\mathbf{e}`$ を地図グループごとに蓄積する（移動平均、標準偏差、件数）。

- 世界座標系に直した平均が `anchor_mismatch_warn`（既定 0.10 m / 0.5°）を超えたら、診断で WARN「地図グループ X のアンカー較正が必要」を出す。
- 蓄積したデータはログに書き出し、`gll_anchor_calibrator` の入力にする。
- アンカーずれがあると、FIX が外れて LiDAR だけになった瞬間に、推定値と LiDAR の間に差が生じる。この差は 3.13.3 節の再アンカーで LiDAR 側に寄せ、出力整形層で滑らかに吸収する。

#### 3.13.3 ゲートの例外と再アンカー

通常の Mahalanobis ゲート（3.5 節）は、「推定値は正しく、観測が外れている」という前提で観測を落とす。しかし、デッドレコニングが続いた後やアンカーずれがあるときは、**推定値の方がずれている**ことがある。そのまま観測を落とし続けると復帰できなくなる。そこで、次の**再アンカー**の手順を設ける。

**再アンカー**（推定値を観測側に寄せ直す手順）:

1. 候補の観測 $`\{Z_i\}`$ ごとに、観測とその時刻の推定値との差（世界座標系の位置の差と yaw の差）を求める。推定値の方がずれているなら、短い区間ではこの差はほぼ一定になる（推定の相対運動はオドメトリで正確なため）。観測の方が外れているなら、差はばらつく。
2. 差どうしのばらつきが `reanchor_consistency`（既定 0.10 m / 1.0°）以内なら、「観測どうしは互いに一致している（外れているのは推定値の方）」と判断する。
3. 共分散を膨らませる: $`\mathbf{P}_{\xi\xi} \leftarrow \mathbf{P}_{\xi\xi} + \mathrm{diag}(e_x^2, e_y^2, e_\psi^2) + \Sigma_{\mathrm{reanchor}}`$（$`\mathbf{e}`$ は最新の観測と推定値の差）
4. 最新の観測で通常の更新を行う。推定値は観測側へ強く寄る。
5. 棄却のカウンタをリセットする。イベント `REANCHOR(source, |e|)` を診断に出す。
6. 推定値の変化は出力整形層が吸収する（出力は飛ばない）。オフセットが `offset_error_threshold` を超えている間は `DEGRADED` にする。

**観測ごとの扱い**:

| 観測 | ゲートで落ちたとき | 再アンカーの条件 |
|---|---|---|
| GNSS（RTK-FIX） | **棄却しない**（FIX は信用する方針）。推定値がずれていると判断し、再アンカーの候補にする | 連続 `reanchor_confirm_gnss`（既定 3 サンプル = 10 Hz で 0.3 s）が互いに一致したら再アンカーする。単発の外れ値だけを除くための確認で、FIX を疑う目的ではない |
| GNSS の進行方位 | 棄却する（補助的な観測のため） | なし |
| LiDAR（GNSS FIX 中） | 3.13.2 節の固定閾値で判定する（食い違いなら棄却） | なし（GNSS を優先する） |
| LiDAR（GNSS 無し） | Mahalanobis ゲートで棄却し、再アンカーの候補にする | `reanchor_confirm_lidar`（既定 5 スキャン = 0.5 s）の候補が互いに一致したら再アンカーする。候補になるのは品質の条件（6.2 節）を満たした結果だけ |

- **GNSS → LiDAR の引き継ぎ**（FIX が外れた直後）では、アンカーずれがあると LiDAR がゲートで落ちる。上の LiDAR の再アンカーが働き、0.5 秒ほどで LiDAR 側に寄る。
- **LiDAR → GNSS の引き継ぎ**（FIX が戻ったとき）では、FIX が安定待ち（1 s）を経て採用される。推定値とずれていれば、GNSS の再アンカーで 0.3 秒ほどで GNSS 側に寄る。

#### 3.13.4 LiDAR の失敗と再位置推定

GNSS が無い区間で LiDAR の観測が互いに一致しない場合は、推定値ではなく位置合わせの方が失敗している（誤った局所解）と判断する。

1. LiDAR を使わずにデッドレコニングを続ける（共分散は増えていく）。
2. 棄却（ゲートに落ちた結果と、品質の条件を満たさなかった結果の両方を数える）が `relocalize.after_rejects`（既定 20 スキャン = 2 s）続き、かつ GNSS FIX が無い場合は、**再位置推定**を行う。
   - 推定位置の周りに初期値の候補を並べる。半径は、推定値の共分散の 3σ と、位置の観測なしで走った距離（3.12 節）の `radius_per_dr_distance` 倍（既定 5 %）の大きい方を、`min_radius`〜`max_radius`（既定 1〜3 m）に収めたもの。yaw は 3σ を 10°〜30° に収めたもの（v0.10）。フィルタの共分散は、モデル化していない誤差（スリップなど）があると小さすぎることがあるので、走った距離からも下限を決める。
   - 次のスキャンで、各候補から位置合わせを行う（6.4 節の多仮説の探索。粗い VGICP → GICP の 2 段階）。
3. 最良の結果が品質条件を満たし、かつ 2 番目に良い候補（最良から 1 m または 10° 以上離れたもの）より overlap が十分大きい場合（`uniqueness_ratio` 既定 1.5 倍）だけ採用し、再アンカーする。一意に決まらない場合は採用しない（対称な構造での取り違えを防ぐ）。
4. `max_attempts`（既定 3）回失敗したら `LOST` にする。
5. `LOST` の間は、GNSS FIX が無ければ `lost_retry_interval`（既定 5 s）ごとに、最も広い範囲（3 m・30°）で再位置推定を試す（地図だけの現場で自動的に復帰できるように。v0.10）。ほかに、GNSS の採用・再アンカー、外部から与えた初期姿勢、ゲートを通った LiDAR の観測でも復帰する。

合成環境のシミュレーションでは、LiDAR が 25 秒止まって 1.55 m ずれた状態から、LiDAR が戻って 2 秒後に再位置推定（39 候補、0.2 s）で戻った（9 章）。

#### 3.13.5 状態遷移のまとめ

```mermaid
stateDiagram-v2
  [*] --> TRACKING
  TRACKING: TRACKING<br/>通常の融合
  SUSPECT: SUSPECT<br/>棄却が続いている
  REANCHOR: REANCHOR<br/>共分散を膨らませて観測側へ寄せる
  RELOCALIZE: RELOCALIZE<br/>複数の初期値から位置合わせ
  LOST: LOST

  TRACKING --> SUSPECT: ゲートで棄却
  SUSPECT --> TRACKING: 観測を採用
  SUSPECT --> REANCHOR: 棄却された観測どうしが一致<br/>（GNSS 3 回 / LiDAR 5 回）
  REANCHOR --> TRACKING: 更新完了
  SUSPECT --> RELOCALIZE: LiDAR の棄却が 2 s 続き<br/>観測どうしも一致しない（GNSS 無し）
  RELOCALIZE --> REANCHOR: 一意で良好な解
  RELOCALIZE --> LOST: 3 回失敗
  SUSPECT --> LOST: 位置の σ > lost_stddev
  LOST --> REANCHOR: GNSS RTK-FIX を 3 回連続で取得
  LOST --> TRACKING: 外部から初期姿勢を指定
```

- この状態は、3.12 節の状態（どの観測が効いているか）とは**別の軸**である。診断には両方を出す。
- `LocalizationStatus` との対応: `REANCHOR` の後でオフセットを吸収している間は `DEGRADED`、`LOST` は `LOST`、それ以外は 3.12 節の判定に従う。


---

## 4. 座標系

### 4.1 フレーム

| フレーム | 定義 |
|---|---|
| `map` | 固定した UTM ゾーンの (Easting, Northing, 楕円体高)。**推定器の状態と出力はこの座標系**（v0.8 でフレーム名を `utm` から `map` に変更。座標値は UTM そのもの） |
| `pcd_<group_id>` | 地図グループごとの点群地図の座標系。点群とタイルはこの座標系で保存する（出力の `map` と区別するため、`map_` ではなく `pcd_` を付ける） |
| `base_link` | 車両の基準。センサの外部パラメータは base_link 基準で与える |
| `imu`, `lidar`, `gnss_antenna` | 各センサ。base_link からの静的な変換は IF 層が読み込み、コアにパラメータとして渡す |

### 4.2 地図 → UTM 変換（アンカー）

各地図グループについて、次の値を設定ファイルで与える。

- アンカー点の地図座標 $`\mathbf{a}^{g}`$
- アンカー点の緯度・経度・楕円体高
- 地図の x 軸の方位 $`\psi`$（**真北から時計回り**）

変換の手順:

1. 緯度経度 → UTM で $`(E_a, N_a)`$、子午線収差 $`\gamma`$、点縮尺係数 $`k`$ を得る（GeographicLib）。
2. グリッド方位 $`\alpha = \psi - \gamma`$（$`\gamma`$ はグリッド北の真北からの時計回り角）。
3. UTM の x 軸（East）基準の回転角 $`\varphi = \pi/2 - \alpha`$。
4. 水平成分:
   ```math
   \begin{bmatrix} E \\ N \end{bmatrix} = \begin{bmatrix} E_a \\ N_a \end{bmatrix} + k\,\mathbf{R}(\varphi)\left(\mathbf{p}^{g}_{xy} - \mathbf{a}^{g}_{xy}\right)
   ```
   鉛直成分: $`h = h_a + (p^g_z - a^g_z)`$

**縮尺係数 $`k`$ を入れる理由**: UTM のグリッド距離は実距離と最大で約 0.04〜0.1% ずれる（中央子午線付近で 0.9996）。1 km 離れると 0.4〜1 m のずれになる。SLAM で作った地図は実距離なので、グループ全体を 1 つの $`k`$ で縮尺補正する。地図グループの東西の広がりが数 km に及ぶ場合は $`k`$ の変化も無視できなくなるので、グループの分割を検討する（11 章の未決事項）。

この変換は剛体変換ではなく**相似変換**になる。変換はコア内で `MapAnchor` クラスとしてまとめ、スキャンマッチングの初期値計算（UTM → 地図）と結果の変換（地図 → UTM）の両方に同じものを使う。

**アンカーの与え方**（v0.10。maps.yaml の `anchor`。5.2 節）:

| 与え方 | キー | 使いどころ |
|---|---|---|
| 緯度経度 | `latitude`、`longitude`、`ellipsoid_height`、`heading_deg`（真北から時計回り） | 通常。GNSS で測ったアンカー点 |
| UTM で直接 | `easting`、`northing`、`ellipsoid_height`、`grid_heading_deg`（グリッド北から時計回り） | アンカー点の UTM 座標が分かっている場合。子午線収差と縮尺係数は、その点の緯度経度から求める |
| `local` | `anchor: local` | 緯度経度が分からない、地図だけの現場。地図の座標をそのまま `map` として出力する（経路も地図の座標で与える）。座標系が UTM と違うので、GNSS の入力は無視する（購読しない）。UTM にアンカーしたグループとは混在できない（maps.yaml の読み込みでエラー） |

いずれも `map_point`（アンカー点の地図座標）、`stddev_xy`、`stddev_yaw_deg`、`use_scale_factor`（縮尺係数を使うか。既定 true）を指定できる。

### 4.3 数値精度

UTM 座標は $`10^5`$〜$`10^6`$ m のオーダーになる。`float32` では有効桁が足りず、0.1〜0.5 m 程度に丸められる。そこで次のように扱う。

- 点群とタイルは**地図グループの座標系（原点付近）で `float32` のまま保持**する。スキャンマッチングも地図座標系で実行する。
- UTM 座標を扱う状態・観測・出力はすべて `double` にする（`geometry_msgs` も float64）。
- RViz での可視化用に、固定原点の `map_local` フレーム（UTM − 原点オフセット）も TF で出せるようにする（`map` → `map_local` の静的変換）。

---

## 5. 点群地図の管理

### 5.1 地図グループ

- **地図グループ** = 1 つの座標系と 1 つのアンカーを持つ、統合済みの点群地図。
- 直接つながる（間に GNSS 区間を挟まない）地図は、1 つのグループに統合されている必要がある。**統合は外部ツールで行い、本システムは統合済みの地図グループを入力として受け取る**（v0.3）。これが要件の「部分展開」（動的ロード）の前提になる。
- 本システムへの入力は、地図グループごとの「統合済みの点群ファイル（地図座標系）」と「アンカー」（4.2 節）である。
- GNSS 区間で隔てられた地図は別のグループとし、それぞれが GNSS 区間に接する位置でアンカーを持つ。

**地図グループの前提**（v0.9）:

- **異なるグループは水平面（UTM の xy）で重ならない**。運用上の前提（点群地図と GNSS 区間の間には必要十分な距離が設定される。要件定義 3 章）による。起動時に重なりを調べ、重なっていれば WARN を出す（5.5 節）。
- **各グループの地図の z 軸は鉛直（重力の方向）にそろっている**。アンカーによる変換（4.2 節）は、鉛直軸まわりの回転と高さのずれしか扱わないため。そろっていない地図は、外部ツールでそろえてから入力する（11 章の確認事項）。
- 同じ水平位置に高さの違う地図が重なる構成（立体駐車場の階など）は対象外とする。照合に使うグループを水平位置だけで決めているため。

要件の例との対応:

| 例 | グループ構成 |
|---|---|
| 例1: 地図1 ↔ GNSS ↔ 地図2 | グループ A（地図1）、グループ B（地図2）。どちらも GNSS 区間に接していて、それぞれのアンカーで UTM に固定する |
| 例2: GNSS ↔ 地図1 ↔ 地図2 | 地図1 + 地図2 を統合した**グループ A の 1 つだけ**。アンカーは GNSS 区間に接する側に置く。地図2 は地図1 を経由したアンカーの連鎖ではなく、統合済みの地図の一部として扱う |

補足: 統合した地図の内部にも SLAM のドリフトは残る。アンカーから遠い部分ほど UTM とのずれが大きくなりうるのは、連鎖方式と同じである。ただし、次の点で連鎖方式より有利になる。

- アンカーを 1 回だけ使うので、アンカー誤差が掛け算で増幅されない。
- 地図の作成時に GNSS（RTK-FIX 区間）の位置を拘束条件としてポーズグラフに入れれば、グループ内のドリフトそのものを抑えられる（外部ツール側で対応できるなら推奨）。

地図の作成・統合の手順は本書の範囲外とする。

### 5.2 タイル化とメタデータ

オフラインツール `gll_map_tiler` で、外部ツールが出力した統合済みの点群を正方形のタイルに分割する。

- 使い方: `gll_map_tiler -i map.pcd [-i more.pcd ...] -o <出力先> [--tile-size 20] [--voxel-size 0.2] [--num-neighbors 20]`。1 回の実行で 1 つの地図グループを作る。入力の PCD は、DATA が ascii / binary / binary_compressed で、x / y / z が float32 か float64 のもの（PCL には依存しない独自の読み込み。v0.10）
- タイルサイズ: 既定 20 m（地図座標系、xy 平面）
- 前処理: ボクセルダウンサンプリング（既定 0.2 m）。点ごとの共分散は、分割する前の点群全体で近傍 20 点から計算する（6.2 節）
- 出力: `tiles/<ix>_<iy>.bin`（独自のバイナリ形式。下記）と `tile_index.yaml`
- タイルの番号 `<ix>_<iy>` はグループの中でしか一意でないので、コアの中では（グループ ID, ix, iy）の組 `TileId` でタイルを識別する（v0.9）。キャッシュなどで、別のグループの同じ番号のタイルを取り違えないようにするため

地図設定ファイルの例:

```yaml
# maps.yaml（ノードのパラメータ map.config_path で指定する。相対パスはこのファイルからの相対）
utm:                                  # 任意。書いた場合は gnss.utm_zone / utm_north と一致するか確かめる
  zone: 54
  hemisphere: north
map_groups:
  - id: area_a
    tile_index: area_a/tile_index.yaml
    anchor:
      map_point: [0.0, 0.0, 0.0]      # アンカー点の地図座標 [m]
      latitude: 35.681236             # [deg]
      longitude: 139.767125           # [deg]
      ellipsoid_height: 40.0          # [m]
      heading_deg: 90.0               # 地図 x 軸の方位（真北から時計回り）[deg]
      stddev_xy: 0.05                 # アンカーの不確かさ [m]
      stddev_yaw_deg: 0.2
  - id: area_b
    tile_index: area_b/tile_index.yaml
    anchor: {easting: 386250.0, northing: 3952100.0, ellipsoid_height: 41.0, grid_heading_deg: 12.5}
```

```yaml
# area_a/tile_index.yaml（gll_map_tiler が生成）
format: gll_tiles_v1
tile_size: 20
voxel_size: 0.2
num_neighbors: 20
tiles:
  - {ix: 3, iy: -1, file: tiles/3_-1.bin, num_points: 18234, bounds_min: [60.000, -20.000, -5.200], bounds_max: [80.000, 0.000, 12.300]}
```

**タイルファイルの形式**（リトルエンディアン）: 先頭 8 バイトが `GLLTILE1`、続いて点数（uint64）、フラグ（uint32。ビット 0 が共分散あり）、点の xyz（float32 × 3 × 点数）、共分散の上三角 xx・xy・xz・yy・yz・zz（float32 × 6 × 点数）。

起動時に `MapTileManager` が全グループのタイルの四隅を UTM に変換して、タイルの範囲（四角形）を求める。この時点では点群本体は読み込まない。タイルの数は多くても数千枚の想定なので、範囲の判定は全タイルを順に調べる（見直しは `update_distance` 既定 1 m 動くか、`update_interval` 既定 1 s ごと）。

### 5.3 動的ロード / アンロード

**判定周期**: 前回の判定位置から `update_distance`（既定 1 m）移動したとき、または `update_interval`（既定 1 s）たったときのどちらか早い方（v0.10 で `reload_distance` 5 m から変更）。

**要求中心**: 現在位置と、進行方向へ先読みした点（速度 × `lookahead_time`、既定 3 s）の 2 点。

先読みは、タイルを前もって読み込んでおくためだけに使う。ロードの対象は全グループのタイルで、次に入るグループのタイルも、近づいた時点でキャッシュに入る。どのグループで照合するか（アクティブグループ）は、先読みを含まない現在位置で決める（5.5 節、v0.9）。

| 操作 | 条件 |
|---|---|
| ロード要求 | 2 つの要求中心のどちらかから半径 `load_radius`（既定 60 m）以内と交差するタイル |
| アンロード | 現在位置からの距離が `unload_radius`（既定 90 m）を超えたタイル（ヒステリシスでロードとアンロードの繰り返しを防ぐ） |

処理の流れ:

1. ロードは専用のワーカースレッドで非同期に行う（タイルファイルの読み込み → 結合点群と KdTree の再構築）。
2. 準備ができたら、スキャンマッチングのターゲットの**ダブルバッファ**の裏側を差し替え対象として構築する。
3. 反映が完了したら、スキャンマッチングの合間にポインタをアトミックに差し替える。
4. **マッチングの実行中にターゲットが変わることはない**。

`load_radius` は、LiDAR の有効レンジ（前処理のクロップ範囲、既定 50 m）より大きくする。これで、走行中にタイルの読み込みが間に合わない事態を防ぐ。速度が速い場合は `lookahead_time` で調整する。最高 6 km/h では、先読みは 3 s × 1.7 m/s ≈ 5 m 程度で、タイルの入れ替えも数秒〜数十秒に 1 回と少ない。

### 5.4 地図が無い区間

アクティブグループが無い（どのグループのタイルからも `load_radius` より遠い GNSS 区間など。5.5 節）場合は、スキャンマッチングを実行しない（LiDAR 観測なし）。ターゲット上の点数が `min_target_points` 未満の場合も同様。

### 5.5 地図グループの切り替え

状態が UTM で表されているため、「どのグループを使うか」は位置から決まる。v0.9 で、運用上の前提（GNSS 区間をはさむグループどうしは十分に離れていて、重ならない。5.1 節）に合わせて、決め方を次のように単純にした。

- **アクティブグループ**: 現在位置（先読みを含まない）から各グループ $`g`$ のタイルまでの水平距離 $`d_g`$（タイルの中にいれば 0）を求める。$`d_g \le`$ `load_radius` のグループのうち、$`d_g`$ が最小のものをアクティブグループとする。該当が無ければアクティブグループは無し（LiDAR 観測なし。5.4 節）。
- **ヒステリシス**: 今のアクティブグループが上の条件を満たしている間は、ほかのグループの $`d_g`$ が `group_switch_margin`（既定 10 m）以上小さくならない限り切り替えない。GNSS 区間の中ほどで、グループが行ったり来たりしないようにするため。
- **ターゲット**: アクティブグループのタイルだけで作る（座標系が異なる点群を混ぜない）。次のグループのタイルは先読み（5.3 節）で前もってキャッシュに入っているので、切り替えたらすぐにターゲットを作れる。
- **変換に使うアンカー**: 照合の初期値を地図座標に変換するときと、照合の結果を UTM に変換するとき（3.7 節）は、**照合に使ったターゲットのグループ**のアンカーを使う（ターゲットがグループ ID を持つ）。照合の途中でアクティブグループが切り替わっても、別のグループのアンカーで変換してしまわないようにするため。
- **重なりの検出**: 起動時に、グループどうしのタイルの範囲（UTM に変換した四角形）が重なっていないかを調べ、重なっていれば、前提と違うので WARN を出す。重なっている場所では、ヒステリシスによって今のアクティブグループを使い続ける（両方のターゲットを用意して比べることはしない）。
- **切り替えの影響**: 切り替えても状態は UTM のままなので、状態は飛ばない。前提により、グループの間の GNSS 区間で位置が確定するので、前のグループのアンカー誤差は次のグループに持ち越されない。次のグループのアンカーと GNSS の差は、3.13 節の食い違いの判定・再アンカーと、3.10 節の出力整形層で扱う。
- GNSS 区間で RTK-FIX が得られず、前のグループから次のグループまでデッドレコニングでつながった場合は、前提が崩れているので、3.12 節のデッドレコニング距離の監視で ERROR になる（GNSS 区間が `dr_error_distance` より長い場合）。

v0.8 までは「要求範囲（先読みを含む）の中のタイル数が最も多いグループ」をアクティブにしていた。これだと、グループ A の縁を走っている間に、先読みの先にあるグループ B の方がタイル数で勝ってしまい、まだ A の中にいるのに B で照合を始めることがある。また、グループが重なる場合に両方のターゲットで照合して比べる手順を置いていたが、運用上の前提により重ならないので削除した。

シーケンス（例1: 地図1 = グループ A → GNSS → 地図2 = グループ B）:

```mermaid
sequenceDiagram
  participant V as 車両位置（UTM）
  participant MTM as 地図タイルマネージャ
  participant SM as スキャンマッチャ
  participant EKF as Invariant EKF
  participant G as GNSS

  Note over V: グループ A の中を走行（アクティブ = A）
  SM->>EKF: LiDAR 観測（A のアンカーで UTM へ）
  Note over V: A の縁に接近。RTK-FIX を取得
  G->>EKF: GNSS 観測（両方有効 → 重み付き融合）
  Note over V: A の外へ出る（GNSS 区間）
  G->>EKF: GNSS 観測で位置が確定（デッドレコニング距離は 0 に戻る）
  opt GNSS 区間が長く、A からも B からも load_radius より遠い
    MTM->>SM: アクティブ無し（ターゲットが空。LiDAR 観測なし）
  end
  Note over V: B が先読みの範囲に入る
  MTM->>MTM: B のタイルを非同期ロード（キャッシュへ）
  Note over V: B の方が A より group_switch_margin 以上近くなる
  MTM->>SM: アクティブ = B。B のタイルでターゲットを作って差し替え
  SM->>EKF: LiDAR 観測（B のアンカーで UTM へ）＋ GNSS（両方有効）
  Note over V: B の中へ（GNSS FIX が外れる）
  SM->>EKF: LiDAR 観測のみ
```

例2（GNSS → 地図1 + 地図2 の統合グループ A）は、GNSS 区間から A に入る部分だけが上の流れと同じになる。地図1 から地図2 への移動は、同じグループ内でのタイルの入れ替えにすぎない。

---

## 6. スキャンマッチング

### 6.1 前処理

実装: `ScanPreprocessor`（デスキュー → base_link への変換 → クロップ）と `GicpMatcher::prepareSource`（間引き → 点ごとの共分散）。

1. **時刻**: ROS の `PointCloud2` に点ごとの時刻があれば、それを使う（v0.10）。フィールド名は `lidar.time_field`（既定 `auto`: `time` → `t` → `timestamp` → `time_stamp` → `offset_time` の順に探す）。単位は型から決める。
   - 整数型: ヘッダの時刻からの ns（Ouster の `t`、Livox の `offset_time` など）
   - FLOAT32: ヘッダの時刻からの秒（Velodyne の `time` など）
   - FLOAT64: 値の大きさで見分ける。1e15 より大きければ絶対時刻の ns、1e6 より大きければ絶対時刻の秒（Hesai・Robosense の `timestamp` など）、それ以外はヘッダの時刻からの秒。**想定機種の Livox Mid-360**（livox_ros_driver2、`xfer_format: 0` の PointCloud2）は `timestamp` が FLOAT64 の絶対時刻の ns で、前者に当たる（フィールドが 8 バイト境界にない詰めた並びでも読める。単体テストで確認）

   スキャンの代表時刻は、**最後の点の時刻**とする（遅れを小さくするため）。点ごとの時刻が無ければ、`header.stamp + lidar.stamp_offset` を代表時刻にし、デスキューはしない。
2. **歪み補正（デスキュー）**: スキャンの間、機体の速度と角速度が一定とみなし、各点を代表時刻の base_link に移す。時刻 $`\tau`$（代表時刻からの相対時刻。負の値）に測った点 $`\mathbf{p}`$（base_link）は、次の式で移す（SE(3) の指数写像。並進 $`\mathbf{u} = \mathbf{v}\tau`$、回転 $`\mathbf{w} = \omega\tau`$）。

   ```math
   \mathbf{p}' = \mathrm{Exp}(\mathbf{w})\,\mathbf{p} + \mathbf{V}(\mathbf{w})\,\mathbf{u}
   ```

   - $`\omega`$ はバイアスを補正したジャイロの 3 軸、$`\mathbf{v}`$ は ODOM の速度（推定したスケールを掛けたもの）と横速度。v0.8 までの計画（回転だけ）を広げ、並進も補正する。
   - 時刻を 0.1 ms 刻みにまとめて変換を使い回す。合成データでは、1 rad/s で旋回しながら取ったスキャンの歪み（最大 3.8 m）が 2.3 mm になった。
3. base_link 座標系への変換（`lidar.extrinsic_xyz` / `extrinsic_rpy_deg`）→ 距離でのクロップ（LiDAR からの距離 1.0〜50 m）→ 車体の点の除去（`crop_box_*`。既定は無効）
4. ボクセルダウンサンプリング（`source_voxel_size` 既定 0.5 m）→ 近傍 10 点から点ごとの共分散と法線（small_gicp の `estimate_normals_covariances_omp`）

### 6.2 small_gicp による位置合わせ（v0.2 で NDT から変更）

**手法**: GICP（平面どうしの位置合わせ）。ソースとターゲットの両方で点ごとの共分散を使う。small_gicp v1.0.1 のテンプレート（`Registration<GICPFactor, ParallelReductionOMP>`、Levenberg-Marquardt）を使い、small_gicp の型は `GicpMatcher` の中に閉じ込める。

**ターゲット（地図）の構成**: small_gicp には、タイル ID 単位でターゲットの点を削除する仕組みが無い。そこで次のようにする。

1. **点ごとの共分散はオフラインで計算**する。`gll_map_tiler` がグループ全体の点群に対して近傍 20 点から共分散を求め、タイルファイルに保存する。**分割前の点群で計算する**ことで、タイル境界で近傍点が欠けて共分散が劣化するのを防ぐ。
2. タイルファイルは独自のバイナリ形式（5.2 節）とする。コアから PCL への依存を無くすため。
3. アクティブグループのロード済みタイルの集合が変わったら、地図ロードワーカーが**結合した点群と KdTree を再構築**し、ダブルバッファで差し替える（5.3 節）。6 km/h ではタイル集合が変わるのは数秒〜数十秒に 1 回で、再構築は十分に間に合う。

**主なパラメータ**（既定値。実データで調整する）:

| パラメータ | 既定値 |
|---|---|
| `max_correspondence_distance` | 1.0 m |
| 収束判定（回転 / 並進） | 1e-3 rad / 1e-3 m |
| 最大反復回数 | 20 回 |
| スレッド数 | 4（ROS のパラメータの既定。コアの既定は 2） |

**初期値**: 3.8 節のスキャン時刻の予測姿勢（x, y, yaw）を、**照合するターゲットのグループのアンカー**で地図座標系に変換し（5.5 節）、z と roll / pitch を合わせたもの（3.9 節）。

**品質指標**（`RegistrationResult`）:

- `converged` かつ反復回数が上限未満
- インライア率（対応点が見つかったスキャンの点の割合）≥ `min_inlier_ratio`（既定 0.6）
- **overlap** ≥ `min_overlap`（既定 0.5）: 位置合わせ後のスキャンの点のうち、`overlap_distance`（既定 0.3 m）以内に地図の点があるものの割合。**法線が鉛直に近い点（地面・天井）は数えない**（v0.10）。地面はどの水平位置でも重なるので、含めると誤った解でも overlap が高くなるため。水平でない面（壁・柱など）の点が 50 点未満なら、全点で数える
- 初期値からの移動量 ≤ `max_jump_xy` / `max_jump_yaw`（既定 1.0 m / 5°）

**共分散**（v0.10 で確定）:

```math
\Sigma_6 = \kappa\, N_{\mathrm{in}}\, (\mathbf{H} + \epsilon \mathbf{I})^{-1},\qquad \Sigma_{\mathrm{reg}} = [\Sigma_6]_{(x,\,y,\,\psi)}
```

- $`\mathbf{H}`$ は収束点での small_gicp の情報行列（6×6）。**機体座標系側の摂動**（$`\mathbf{T} \leftarrow \mathbf{T}\,\mathrm{Exp}(\delta)`$）に対するもので、$`\delta`$ の並びは回転 3 → 並進 3（v1.0.1 で確認）。このまま 3.7 節の機体座標系の観測共分散に使える（座標変換は要らない）。
- small_gicp の点ごとの共分散は、固有値を (1e-3, 1, 1) にそろえた正規化した値なので、$`\mathbf{H}^{-1}`$ は物理的な誤差の大きさを表さない。また点どうしは強く相関しているので、点数を増やしても精度は $`1/\sqrt{N}`$ では良くならない。そこで**インライア数 $`N_{\mathrm{in}}`$ を掛け戻して点数によらない形にし**、係数 $`\kappa`$ = `cov_scale`（既定 0.15）で実際の精度に合わせる。既定値では、面が十分ある環境で σ が数 cm・0.1° 程度になる。
- 退化した環境（長い廊下など）では、$`\mathbf{H}`$ の情報が小さい方向の分散が大きくなり、その方向の補正は自動的に弱まる。合成データの廊下（x 方向に長い 2 枚の壁）では、x の分散が y の 70 倍になった。
- $`\epsilon`$ = 1e-6 は、情報がまったく無い方向で逆行列が発散しないための値。
- GICP の共分散は実際より小さく見積もられやすいので、3.7 節の $`\Sigma_{\mathrm{floor}}`$ を加える。$`\kappa`$ は実データ（i2Nav-Robot の V2-1）で、誤差と共分散の整合から決める。

**失敗の扱い**: 品質の条件を満たさない結果は観測にしない。その回数は、ゲートで落ちた回数と合わせて数え、続けば再位置推定を行う（3.13.4 節）。

### 6.3 処理時間の目安

- 合成データ（16 ライン、14,000 点のスキャン、地図 36 万点）で、前処理後 4,000 点の GICP が 10 ms（2 スレッド、3 反復）。10 Hz の LiDAR に対して十分。実機の CPU での値は Phase 3 で測る。
- マッチングは専用のワーカースレッドで実行する（`lidar.async`）。処理が追いつかない場合は、最新のスキャンだけを処理する（古いスキャンは捨て、`lidar_dropped` を数える）。
- 多仮説の探索（6.4 節）は 0.2〜0.8 s かかる。その間に届いたスキャンは捨てる（初期化と再位置推定のときだけなので、影響は小さい）。

### 6.4 多仮説の探索（地図上での初期化と再位置推定。v0.10）

実装: `GicpMatcher::search`。初期値の周りに候補を並べ、次の 3 段で解を選ぶ。

1. **候補**: 位置は探索の中心から `position_step`（既定 1 m）間隔の格子で、半径以内の点。yaw は `yaw_step`（既定 20°）間隔で ±探索幅（探索幅が 180° なら全周）。z・roll・pitch は中心の値。
2. **粗い位置合わせ**: 各候補から、ソースを `coarse_source_voxel`（既定 1.0 m）で間引いた点群を、ターゲットのボクセル地図（VGICP、`coarse_voxel_size` 既定 2.0 m）に `coarse_max_iterations`（既定 10）回まで合わせる。候補ごとに並列に処理する。1 m 間隔の overlap（水平でない面の点）で順位を付ける。
3. **詰める**: 上位の候補から、互いに `distinct_xy` / `distinct_yaw`（既定 1 m / 10°）以上離れたものを `refine_candidates`（既定 5）個選び、通常の GICP で詰めて overlap（0.3 m）を求める。
4. **判定**: overlap が最大の解を最良とし、最良から十分離れた解のうち overlap が最大のものを次点とする。最良の解が品質の条件（インライア率、overlap ≥ `min_overlap` 既定 0.6）を満たし、次点の overlap の `uniqueness_ratio`（既定 1.5）倍以上なら採用する。

合成データでの結果（9 章）: 1.2 m・35° ずれた初期値から 147 候補で 3 cm・0.2° 以内、yaw が分からない（全周）場合も 162 候補で見つかった。正方形の部屋の中心（90° ごとに同じ見え方）では、次点との区別がつかず「ambiguous」として採用しなかった。

## 7. ソフトウェアアーキテクチャ

### 7.1 レイヤ構成

実装の粒度のコンポーネント図・クラス図・シーケンス図は [ソフトウェア構成](./architecture.md) を参照。

```mermaid
flowchart TB
  subgraph IF["IF 層（ROS 依存）"]
    R2[gll_ros2<br/>ROS 2 ノード]
    R1[gll_ros1<br/>ROS 1 ノード（将来）]
  end
  subgraph CORE["コア層（ROS 非依存, C++17）"]
    API[Localizer（ファサード）]
    EST[estimation/<br/>InvEkfSe2, StateHistory, OutputSmoother]
    MEAS[measurement/<br/>GnssModel, LidarModel, AttitudeEstimator]
    MAP[map/<br/>MapAnchor, MapTileManager, ITileLoader]
    MATCH[matching/<br/>IScanMatcher, GicpMatcher]
    COMMON[common/<br/>型, ILogger, 設定, 座標変換]
  end
  R2 --> API
  R1 --> API
  API --> EST & MEAS & MAP & MATCH
  EST & MEAS & MAP & MATCH --> COMMON
```

- **依存の向きは IF 層 → コア層の一方向だけ**。コアは ROS のヘッダ、メッセージ型、時刻、ログ、パラメータ API を一切使わない。
- 時刻はすべて、データに付いたタイムスタンプ（`double` 秒）で扱う。コアはシステム時計を参照しない（rosbag の再生やシミュレーションでも同じ動作になる）。
- ログは `ILogger` インターフェースを通して出す（IF 層が `RCLCPP_*` / `ROS_*` に橋渡しする）。
- 設定はプレーンな構造体（`LocalizerConfig`）で受け取る。YAML や ROS パラメータからの読み込みは IF 層の責務とする（コア単体のテスト用に、YAML から読み込むヘルパーは用意する）。

### 7.2 ディレクトリ構成

```
gnss_and_lidar_localization/
├── map/                        # gll_map（v0.11。単体でも使える地図のライブラリ。純粋な CMake。map/README.md）
│   ├── include/gll/
│   │   ├── common/             # math（型・角度）, se2, geodesy（UTM）, logger
│   │   └── map/                # アンカー、maps.yaml、タイルと索引、PCD の読み書き、タイル化、MapTileManager、MapRegion
│   ├── src/
│   ├── tools/                  # gll_map_tiler（統合済みの PCD → タイル + tile_index.yaml）、gll_tile_demo
│   ├── examples/               # find_package(gll_map) で使う例
│   └── test/
├── core/                       # gll_core（純粋な CMake。gll_map を使う。colcon / catkin からも plain CMake でビルド可能）
│   ├── CMakeLists.txt
│   ├── include/gll/
│   │   ├── common/             # types（センサデータ・出力）, config, pose_store
│   │   ├── estimation/         # 推定器（InvEkfSe2 / EsEkf2D）、履歴、ゲート、出力整形、状態監視、初期化、優先度、復帰
│   │   ├── measurement/        # 姿勢推定器、予測入力、停止検出、GNSS / LiDAR の観測の生成
│   │   ├── matching/           # IScanMatcher（MatchTarget は gll_map の MapRegion を継承）、GicpMatcher（small_gicp）、ScanPreprocessor
│   │   └── localizer.hpp       # ファサード
│   ├── src/
│   └── test/                   # GoogleTest（ROS 無しで実行可能。合成環境のレイキャストによる LiDAR の模擬を含む）
├── ros2/
│   └── gll_ros2/               # ament_cmake パッケージ（ノード、launch、パラメータ）
├── ros1/                       # 将来: gll_ros1（catkin）
├── tools/
│   ├── sim/                    # Invariant EKF と ESEKF の比較（Python）
│   └── anchor_calibrator/      # 将来: GNSS ログとスキャンマッチング結果からアンカーを較正
├── docker/
└── docs/
```

### 7.3 コア API（概略）

```cpp
namespace gll {

struct ImuSample   { double t; Eigen::Vector3d gyro; Eigen::Vector3d acc; };   // base_link 座標系
struct OdomSample  { double t; double v; double v_lat; std::optional<double> yaw_rate; };  // twist の x, y, angular.z
struct GnssSample  { double t; double lat, lon, h; Eigen::Matrix3d cov_enu; bool cov_known; int raw_status; };
struct LidarScan   { double t; std::vector<Eigen::Vector3f> points;  // LiDAR 座標系
                     std::vector<float> times; };                    // 点ごとの t からの相対時刻（無ければ空）

struct Pose2D      { double x, y, yaw; };
struct LocalizationOutput {
  double t;
  Pose2D pose;                 // 出力整形後（UTM）
  Pose2D raw_pose;             // フィルタの生の推定値
  Eigen::Matrix3d cov;         // 出力整形後の共分散 Σ_w + o oᵀ（世界座標系の x, y, yaw）
  Eigen::Matrix3d raw_cov;     // フィルタの推定値の共分散 Σ_w
  double v, yaw_rate;
  LocalizationStatus status;
  RecoveryState recovery;
  std::string active_map_group;
  double dr_distance;          // 最後に位置の観測を採用してから走った距離 [m]（3.12 節）
  bool dr_distance_exceeded;   // dr_distance > dr_error_distance（diagnostics で ERROR）
};

class Localizer {
 public:
  enum class InitialPoseSource { EXTERNAL, SAVED };
  Localizer(const LocalizerConfig& cfg, std::unique_ptr<IStateEstimator> estimator = nullptr,
            std::shared_ptr<ILogger> logger = nullptr);

  // LiDAR を使う場合: maps.yaml から作った MapTileManager とスキャンマッチャを渡す
  void setMap(std::shared_ptr<MapTileManager> maps, std::shared_ptr<const IScanMatcher> matcher);

  void addImu(const ImuSample&);
  void addOdom(const OdomSample&);
  void addGnss(const GnssSample&);
  void addLidarScan(LidarScan);               // lidar.async なら専用スレッドに渡してすぐ戻る
  void setInitialPose(double t, const Pose2D&, const Eigen::Matrix3d& cov,
                      InitialPoseSource source = InitialPoseSource::EXTERNAL);  // 地図の上なら周りを探して初期化

  std::optional<LocalizationOutput> getOutput();  // 最新の出力（地図のタイルの見直しもここで行う）
  Diagnostics diagnostics() const;
  std::optional<LidarMatchInfo> lastLidarMatch() const;          // デバッグ出力用
  std::shared_ptr<const MatchTarget> currentMapTarget() const;   // 可視化用
};

}  // namespace gll
```

### 7.4 スレッドモデル

| スレッド | 所有者 | 役割 |
|---|---|---|
| 呼び出し側スレッド | IF 層（ROS のコールバック） | `add*` を呼ぶ。IMU / ODOM / GNSS はその場で予測・更新する（mutex で保護。処理は μs オーダー） |
| マッチングワーカー | コア（`Localizer` の中の `std::thread`。`lidar.async`） | 最新のスキャンだけを保持し、前処理と GICP（初期化・再位置推定では多仮説の探索）を行う。フィルタの mutex は、予測姿勢の取得と観測の適用のときだけ短く取る |
| 地図ロードワーカー | コア（`MapTileManager` の中の `std::thread`。`map.async`） | タイルの読み込み、ターゲット（結合した点群と KdTree）の再構築、ダブルバッファの差し替え |

テストでは `lidar.async` と `map.async` を false にし、同じスレッドで決まった順に処理する（結果が毎回同じになる）。

コアは ROS のタイマーや executor に依存しない。出力の publish 周期（既定 50 Hz）は IF 層のタイマーが決め、`getOutput()` をポーリングする。IMU の受信ごとに publish することもできる。

### 7.5 ROS 2 インターフェース（gll_ros2）

購読:

| トピック（既定） | 型 | 備考 |
|---|---|---|
| `~/input/imu` | `sensor_msgs/Imu` | |
| `~/input/odom` | `nav_msgs/Odometry` | v0.8 で確定。**twist だけを使う**（`twist.linear.x` = 前進速度、`twist.linear.y` = 横速度、`twist.angular.z` = ヨーレート。いずれも `child_frame_id` の機体座標系）。pose（オドメトリ自身が積分した位置・姿勢）は使わない |
| `~/input/gnss/fix` | `sensor_msgs/NavSatFix` | 独自ドライバ。3.6 節の取り決めに従う |
| `~/input/gnss/velocity` | `geometry_msgs/TwistWithCovarianceStamped` | 任意（`use_gnss_velocity: true` のときだけ購読） |
| `~/input/points` | `sensor_msgs/PointCloud2` | `map.config_path` を指定したときだけ購読する。点ごとの時刻のフィールドは自動で判別する（6.1 節） |
| `~/input/initial_pose` | `geometry_msgs/PoseWithCovarianceStamped` | |

**NavSatFix の変換**（IF 層の `NavSatFixConverter`）: `status.status`、`position_covariance(_type)`、`altitude`、`header.stamp` を 3.6 節の取り決めに従ってコアの `GnssSample` に変換する（RTK-FIX の判定そのものはコアの `GnssMeasurementBuilder` で行う）。速度トピックは `GnssVelocitySample` に変換する。

配信:

| トピック（既定） | 型 | 備考 |
|---|---|---|
| `~/output/pose` | `geometry_msgs/PoseWithCovarianceStamped` | frame_id = `map`（UTM）、出力整形後 |
| `~/output/odometry` | `nav_msgs/Odometry` | 速度・ヨーレートを含む |
| `~/output/status` | `diagnostic_msgs/DiagnosticStatus` | 出力周期ごとの状態（3.12 節）。デッドレコニング距離が `dr_error_distance` を超えたら ERROR とメッセージ（v0.9） |
| `~/debug/raw_pose` | `geometry_msgs/PoseWithCovarianceStamped` | フィルタの生の推定値 |
| `~/debug/lidar_pose` | `geometry_msgs/PoseWithCovarianceStamped` | スキャンマッチングの結果（UTM）と観測共分散。品質指標は diagnostics に出す |
| `~/debug/map_points` | `sensor_msgs/PointCloud2` | 照合に使っている地図（ターゲット）。frame_id = `map_local`、ターゲットが変わったときだけ（transient local） |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 1 Hz。`gll_localizer`（`~/output/status` と同じ内容）、`gll_localizer: counters`（入力・採用・棄却の数。LiDAR を含む）、`gll_localizer: map`（アクティブグループ、読み込んだタイル、照合の処理時間と品質。アンカーずれが `anchor_mismatch_warn` を超えたら WARN） |
| TF | | **`map` → `base_link`**（odom フレームは使わない。v0.8）。可視化用に `map` → `map_local` の静的変換も出す（`map_local_origin` を指定しなければ、地図があれば最初のグループのアンカーを 100 m 単位に丸めた点）。フレーム名はパラメータ（`map_frame`、`base_frame`）で変えられる |

### 7.6 ROS 1 への移植（後回し）

v0.8 で、ROS 1 対応の実装は後回しと決めた。コアの ROS 非依存性は保ち、次の方針でいつでも移植できる状態を維持する。


- `gll_ros1` は、`gll_ros2` と同じ構成（購読 → 型変換 → `Localizer` → 配信）を roscpp で書き直すだけで済む。
- 型変換の関数（`fromRosMsg` / `toRosMsg`）はメッセージの定義がほぼ同じなので、ヘッダだけテンプレート化して共有することも検討する。
- コアのビルドは plain CMake なので、catkin からも `find_package(gll_core)` で使える。

### 7.7 外部依存（コア）

v0.11 で、地図の部分を `gll_map` に分けた。`gll_map` の必須の依存は Eigen3・GeographicLib・yaml-cpp だけで、small_gicp は任意（あればタイル化のときに点ごとの共分散を計算する。無ければ共分散なしのタイルを書き、`gll_core` の `GicpMatcher` がターゲットを作るときに計算する）。`gll_core` は `gll_map` と small_gicp・OpenMP を使う。

| ライブラリ | 使うパッケージ | 用途 |
|---|---|---|
| Eigen3 | 両方 | 線形代数 |
| GeographicLib | gll_map | 緯度経度 ↔ UTM、子午線収差、縮尺係数 |
| small_gicp（v1.0.1） | gll_core（gll_map は任意） | スキャンマッチング（GICP / VGICP）、共分散推定、KdTree。タグを固定してソースからビルドする（apt のパッケージは使わない）。GCC 13 と Eigen 3.3 の組み合わせでは、small_gicp の `kdtree.hpp` が `<cstdint>` を必要とするので、使う側で先に含める |
| OpenMP | gll_core（gll_map は任意） | small_gicp の並列化（GCC の libgomp） |
| （PCL） | 使わない | v0.10。PCD の読み込みとボクセルの間引きは `gll_map` の独自実装、PointCloud2 の変換は IF 層の独自実装 |
| yaml-cpp | gll_map | maps.yaml・tile_index.yaml |
| GoogleTest | 両方 | 単体テスト |

---

### 7.8 開発環境（Docker、v0.8）

開発・ビルド・テスト・データ変換は、すべて Docker のコンテナで行う。

| 項目 | 内容 |
|---|---|
| ベースイメージ | `ros:jazzy-ros-base`（Ubuntu 24.04） |
| apt で入れる依存 | Eigen3、GeographicLib、yaml-cpp、GoogleTest、`ros-jazzy-*` の必要パッケージ（`tf2_ros`、各メッセージなど） |
| ソースからビルドする依存 | small_gicp v1.0.1（タグを固定して `git clone` し、静的ライブラリとして CMake でインストール。`SMALL_GICP_VERSION` で変えられる） |
| ステージ | `dev`（ビルド・テスト・デバッグ用。Python の `rosbags` など検証用ツールも入れる）と `runtime`（実行に必要なものだけ） |
| 構成ファイル | `docker/Dockerfile`、`docker/compose.yaml`（ワークスペースとデータディレクトリ `$GLL_DATA` をマウント。RViz 用の X11 転送は任意） |
| VS Code | `.devcontainer/devcontainer.json`（任意。`dev` ステージを使う） |
| CI | GitHub Actions で同じ Dockerfile の `dev` ステージをビルドし、その中で colcon ビルドと単体テストを実行する |

- コア（`gll_core`）は ROS なしでもビルドできる構成を保つ。CI では、ROS なしでのコア単体のビルドとテストも別ジョブで行い、ROS への依存が紛れ込んでいないことを確かめる。`gll_map` は、small_gicp も無い環境で単体でビルド・テストし、インストールしたものを `find_package(gll_map)` で使う例もビルドする（v0.11）。
- i2Nav-Robot の変換・評価スクリプト（[検証計画](./validation_i2nav.md)）も、`dev` ステージのコンテナ内で実行する。

## 8. パラメータ初期値（抜粋）

実装での YAML のキー名は [`localizer.yaml`](../ros2/gll_ros2/config/localizer.yaml) を参照（`gnss.`、`monitor.` などのグループに分けてある。例: `gnss_fix_settle_time` → `gnss.fix_settle_time`、`dr_error_distance` → `monitor.dr_error_distance`）。

| グループ | パラメータ | 既定値 | 備考 |
|---|---|---|---|
| 予測 | `sigma_v` | 0.05 m/s | ODOM 速度のノイズ |
| | `sigma_omega` | 0.01 rad/s | ヨーレートのノイズ |
| | `sigma_bias_rw` | 1e-4 rad/s/√s | ジャイロバイアスのランダムウォーク |
| | `sigma_scale_rw` | 1e-4 /√s | スケールのランダムウォーク（推定を無効にできる） |
| | `odom_hold_max` | 0.1 s | |
| GNSS | `gnss_max_stddev` | 0.05 m | 採用の閾値 |
| | `gnss_min_stddev` | 0.02 m | R の下限 |
| | `gnss_fix_settle_time` | 1.0 s | |
| | `gnss_settle_reset_gap` | 3.0 s | GNSS のメッセージがこれ以上途切れたら、安定待ちをやり直す（v0.9） |
| GNSS | `cog_min_speed` / `cog_max_yaw_rate` | 0.5 m/s / 5 deg/s | 進行方位観測の採用条件 |
| LiDAR | `min_inlier_ratio` / `min_overlap` / `overlap_distance` | 0.6 / 0.5 / 0.3 m | 品質の条件（6.2 節）。実データで調整 |
| | `cov_scale` | 0.15 | 観測共分散の係数 $`\kappa`$（6.2 節）。実データで調整 |
| | `min_stddev_xy` / `min_stddev_yaw` | 0.02 m / 0.2 deg | 観測共分散の下限 |
| | `source_voxel_size` / `source_num_neighbors` | 0.5 m / 10 | スキャンの間引きと共分散 |
| | `min_range` / `max_range` | 1.0 m / 50 m | |
| | `extrinsic_xyz` / `extrinsic_rpy_deg` | 0 | base_link から見た LiDAR |
| | `time_field` / `stamp_offset` | auto / 0 s | 点ごとの時刻（6.1 節） |
| | `max_correspondence_distance` | 1.0 m | |
| | `max_jump_xy` / `max_jump_yaw` | 1.0 m / 5 deg | |
| | `max_iterations` / `num_threads` | 20 / 4 | GICP の反復回数の上限、並列数（OpenMP） |
| | `deskew` / `crop_box_enabled` | true / false | デスキュー、車体の点を除く箱（6.1 節） |
| | `min_source_points` / `min_interval` | 100 / 0 s | 間引いた後の点数の下限、照合の最小間隔 |
| 姿勢推定 | `imu_static_init_time` | 3 s | 起動時のジャイロバイアス推定 |
| | `imu.acc_scale` | 1.0 | 加速度に掛ける係数。Livox Mid-360 の内蔵 IMU（livox_ros_driver2 は g 単位で出す）を使う場合は 9.80665 |
| 予測 | `sigma_v_lat` | 0.02 m/s | 横すべり（非ホロノミック拘束の不確かさ） |
| GNSS | `gnss_rtk_fix_status` | 2（`STATUS_GBAS_FIX`） | RTK-FIX を表す `status.status` の値 |
| | `gnss_stamp_offset` | 0.0 s | `header.stamp` が受信時刻の場合の遅延補正 |
| | `gnss_accept_unknown_covariance` / `gnss_default_stddev` | false / 0.03 m | 共分散が UNKNOWN のときの扱い |
| | `use_gnss_velocity` | false | 進行方位の観測を使うか |
| | `gnss_lever_arm` | [l_x, l_y, l_z]（未定。l_z ≤ 1 m） | base_link から見たアンテナ位置 [m]。TF から取得してもよい |
| 姿勢推定 | `attitude_stddev` | 0.5 deg | roll / pitch の推定誤差の想定値（GNSS 観測共分散への加算に使う） |
| 初期化 | `init_heading_min_distance` | 1.0 m | 粗い yaw を決めるための走行距離 |
| | `init_yaw_stddev` | 15 deg | フィルタ始動時の yaw の σ |
| | `ready_yaw_stddev` / `ready_pos_stddev` | 2 deg / 0.1 m | 初期化完了の条件 |
| ゲート | `gate_alpha` | 0.001 | |
| | `max_consecutive_rejects` | 10 | |
| 出力整形 | `max_correction_rate_xy` | 0.1 m/s | 最高速 1.7 m/s に対して控えめに設定。経路追従の挙動を見て調整する |
| | `max_correction_rate_yaw` | 2 deg/s | |
| | `offset_error_threshold` | 1.0 m / 5 deg | |
| 地図 | `tile_size` | 20 m | ツール側 |
| | `load_radius` / `unload_radius` | 60 m / 90 m | |
| | `lookahead_time` | 3 s | |
| | `update_distance` / `update_interval` | 1 m / 1 s | 必要なタイルとアクティブグループを見直す間隔（v0.10 で `reload_distance` 5 m から変更） |
| | `min_target_points` | 500 | これより点の少ないターゲットでは照合しない |
| | `group_switch_margin` | 10 m | アクティブグループの切り替えのヒステリシス（5.5 節、v0.9） |
| 優先度・食い違い | `lidar_cov_inflation_under_fix` | 4.0 | GNSS FIX 中の LiDAR の共分散の倍率 |
| | `consistency_xy` / `consistency_yaw` | 0.15 m / 1.0 deg | GNSS FIX 中の LiDAR の食い違い判定 |
| | `anchor_mismatch_warn` | 0.10 m / 0.5 deg | アンカー較正の警告 |
| 復帰 | `reanchor_confirm_gnss` / `reanchor_confirm_lidar` | 3 / 5 | 再アンカーに必要な、互いに一致する連続観測数 |
| | `reanchor_consistency` | 0.10 m / 1.0 deg | 観測どうしの一致判定 |
| | `relocalize.after_rejects` | 20 | 再位置推定を始める LiDAR の連続棄却数 |
| | `relocalize.min_radius` / `max_radius` / `max_yaw` | 1 m / 3 m / 30 deg | 再位置推定の探索範囲 |
| | `relocalize.radius_per_dr_distance` | 0.05 | 探索半径の下限を、位置の観測なしで走った距離のこの割合まで広げる |
| | `relocalize.uniqueness_ratio` / `min_overlap` | 1.5 / 0.6 | 最良と次点の overlap の比、採用する overlap の下限 |
| | `relocalize.max_attempts` / `lost_retry_interval` | 3 / 5 s | |
| 地図上の初期化 | `relocalize.init_min_radius` / `init_max_radius` | 0.5 m / 5 m | 初期姿勢の 3σ をこの範囲に収めて探す |
| | `relocalize.init_min_yaw` / `init_max_yaw` | 10 deg / 180 deg | |
| | `relocalize.position_step` / `yaw_step` | 1 m / 20 deg | 候補の間隔 |
| | `relocalize.coarse_voxel_size` / `coarse_source_voxel` | 2.0 m / 1.0 m | 粗い位置合わせ（VGICP） |
| | `relocalize.init_map_distance` / `init_max_attempts` | 10 m / 5 | |
| | `relocalize.init_timeout` | 15 s | 地図上での初期化を待つ時間の上限 |
| | `init.saved_pose_path` / `use_saved_pose` | "" / false | 前回保存した位置から始めるか（3.11 節） |
| | `lidar.base_link_height` | 0 m | 地面から base_link までの高さ（z の初期値） |
| 監視 | `aid_timeout` | 1.0 s | |
| | `dr_max_stddev` / `lost_stddev` | 0.3 m / 1.0 m | |
| | `dr_error_distance` | 30 m（仮） | 位置の観測なしでこの距離を走ったら diagnostics で ERROR（3.12 節、v0.9）。i2Nav の V1-2 の結果で決める |
| 履歴 | `history_length` | 2.0 s | |

---

## 9. 検証計画

公開データセット i2Nav-Robot を使った具体的な検証手順（データの変換、地図の作成、評価の指標、シナリオ）は [検証計画: i2Nav-Robot](./validation_i2nav.md) にまとめた。

| レベル | 内容 | 合格基準（案） |
|---|---|---|
| 単体テスト（コア、ROS 無し） | SE(2) の Exp / Log / Ad の恒等式、Invariant EKF の遷移行列・観測行列を数値微分と比較する。アンカー変換を往復させ（UTM → 地図 → UTM）、測地線の計算と比べる（100 m で 1 cm 以内）。遅延観測の再適用の結果が、時系列順に適用した結果と一致することを確認する。PCD・タイル・maps.yaml の読み書き、デスキュー、GICP、多仮説の探索、タイルマネージャ、再位置推定の状態遷移（v0.10） | 誤差 1e-9 以内（数値の比較） |
| シミュレーション | 合成軌跡とノイズで、GNSS 区間 → デッドレコニング → 地図区間の遷移を再現する。LiDAR は、合成環境（地面・箱・円柱）へのレイキャストで模擬する（16 ライン、回転中の運動も再現。v0.10）。NEES / NIS で共分散の整合性を確認する | NEES が χ² の 95% 区間内 |
| rosbag 再生 | 実走行データで、RTK-FIX 区間の GNSS を意図的に外して LiDAR のみで推定し、GNSS を真値として比較する | 横方向 RMS < 0.10 m（仮） |
| 連続性 | 出力の周期間差分から、速度・ヨーレートに相当する成分を除いた「補正ステップ」の最大値を評価する | 1 周期あたり < 0.02 m（仮） |
| 食い違い・復帰 | シミュレーションで、アンカーずれ（0.1〜0.5 m / 0.5〜2°）、誤った FIX、LiDAR の誤った局所解、長いデッドレコニングを注入し、再アンカー・再位置推定・`LOST` への遷移が 3.13 節どおりに起きることを確認する | 全シナリオで出力の補正ステップが上の基準以内、かつ想定どおりに状態遷移する |
| 地図切り替え | 例1・例2 の経路を想定した走行で、タイルの読み込み遅延や、ターゲットが空になる区間が発生しないことを確認する。例1 では、アクティブグループが現在位置で切り替わり、GNSS 区間の中ほどで行き来しないことも確認する | マッチングのスキップ 0 回、切り替えは GNSS 区間ごとに 1 回 |
| デッドレコニングの監視 | GNSS を止めて走らせ、デッドレコニング距離が `dr_error_distance` を超えた時点で diagnostics が ERROR になり、位置の観測を採用すると解除されることを確認する。停止中は距離が増えないことも確認する（Phase 1 のシミュレーションで実装済み） | 超えてから 1 出力周期以内に ERROR |
| 負荷 | GICP の処理時間、ターゲット再構築の時間、再伝播の時間、メモリ使用量 | GICP < 60 ms（p99） |

**Phase 2 のシミュレーションの結果**（v0.10。`core/test/test_lidar_localization.cpp`。地図と世界が完全に一致する理想的な条件なので、実データではもっと悪くなる）:

| シナリオ | 結果 |
|---|---|
| 地図だけ。1 m・12° ずれた初期姿勢から地図上で初期化し、半径 18 m の円を 1.5 m/s で走る | 初期化 0.8 s（203 候補）。位置誤差 RMS 3 mm（最大 2 cm）、yaw 0.011°、出力の補正ステップ最大 1.1 mm、`LIDAR_AIDED` 100 % |
| 地図1 → GNSS 区間（80 m）→ 地図2（座標系が 40° 回り、アンカーに 10 cm・0.2° の誤差） | アクティブグループの切り替えは 1 回（A → B）。位置誤差 RMS: 地図1 5 mm、GNSS 区間 4 mm、地図2 8 cm（地図2 のアンカーの誤差に合う）。補正ステップ最大 1.3 mm。地図2 のアンカーずれを記録 |
| 地図だけ。LiDAR が 25 s 止まり、その間 ODOM の速度が 5 % 大きく出る | 止まっている間に 1.55 m ずれ、30 m 走った時点で ERROR。LiDAR が戻って 2 s 後に再位置推定で戻り、ERROR も解除。出力は 0.1 m/s で追従し、補正ステップは最大 1.2 mm |
| 地図の近くで初期姿勢を与えたが、LiDAR のデータが来ない | 保存した位置: 静止初期化の後 `init_timeout`（テストでは 5 s）で地図上の初期化をあきらめ、GNSS で初期化（位置誤差 RMS 5 mm）。外部から与えた初期姿勢: 同じ時点で、与えた姿勢のまま初期化 |

---

## 10. 実装フェーズ

| Phase | 内容 |
|---|---|
| 1 | **Docker 環境と CI**、コアの骨格（型、設定、ロガー）、SE(2) 演算、Invariant EKF、状態履歴バッファ、GNSS 観測、補助姿勢推定、出力整形、状態監視（デッドレコニング距離の監視を含む。v0.9）、GNSS の再アンカー / ROS 2 IF（IMU・ODOM・GNSS、diagnostics） / 単体テスト。**GNSS + デッドレコニングで動く状態**。シミュレーションで yaw の初期誤差に対する収束を ESEKF 版と比較し、Invariant EKF の優位を確認する |
| 2（v0.10 で実装済み） | アンカー変換、タイルツール（`gll_map_tiler`）、地図タイルマネージャ（非同期ロード・ダブルバッファ・アクティブグループ・重なりの検出）、GICP の実装と LiDAR 観測、回転と並進の歪み補正、地図区間での初期化（初期姿勢・前回保存した位置）、GNSS FIX 中の LiDAR の食い違い判定と LiDAR の再アンカー、再位置推定、ROS 2 IF（PointCloud2・デバッグ出力・診断） |
| 3 | 実走行データ（i2Nav-Robot）での検証とパラメータ調整（`cov_scale`、品質の閾値、`dr_error_distance` など）、アンカー較正ツール、実機の CPU での処理時間の確認 |
| 4（拡張） | SE₂(3) 上の 3D Invariant EKF + IMU バイアス（z / roll / pitch の出力）、地図アンカーオフセットのオンライン推定、ROS 1 IF |

---

## 11. 未決事項・確認事項

1. ~~GNSS 受信機~~ → u-blox F9P、シングルアンテナ、独自ドライバ、`sensor_msgs/NavSatFix`（v0.5 で確定）。**残り**: 3.6 節の取り決め（status の値、共分散、楕円体高、stamp の意味）にドライバを合わせられるか。速度トピックを出せるか。出力レート。
2. ~~ODOM の形式~~ → `nav_msgs/Odometry`（位置・姿勢・並進速度・旋回速度を含む）。twist だけを使う（v0.8 で確定）。**残り**: 車両が横移動・その場旋回・後退をするか（予測モデルは横速度と後退に対応済み）。
3. ~~IMU~~ → 加速度と角速度のみ（v0.2 で確定）。
4. ~~LiDAR~~ → **Livox Mid-360**、点ごとの時刻あり（v0.10 で確定）。livox_ros_driver2 を `xfer_format: 0`（PointCloud2）で使う（`CustomMsg` には対応しない）。**残り**: 実機のデータで、時刻の判別とデスキューの効果、取り付け位置（`lidar.extrinsic_*`）を確かめる。Mid-360 は鉛直の視野が −7°〜52° と上向きなので、取り付けの高さと傾きで、地面と周りの構造物の見え方が変わる。
5. ~~車速の範囲~~ → 最高 6 km/h（v0.2 で確定）。
6. ~~地図の作り方~~ → 外部ツールで統合済みの地図が入力される（v0.3 で確定）。入力は PCD（ascii / binary / binary_compressed）、アンカーは maps.yaml で与える（v0.10）。
7. **アンカーの高さ**: 楕円体高を与えられるか（地図区間に入るときのスキャンマッチングの z 初期値に使う）。
8. **地図グループの範囲**: 東西の広がりが数 km を超えるグループがあるか（縮尺係数の変化を無視できるか）。
9. **計算機**: CPU のコア数と、ほかに動かす処理の負荷（GICP のスレッド数の配分）。
10. **下流との約束事**: `DEGRADED` / `LOST` のときに経路追従側がどう振る舞うか。
11. **GNSS アンテナの取り付け位置**: base_link から見たアンテナ位置（特に高さ $`l_z`$。1 m 以内の想定）を決める。
12. **地図の鉛直**（v0.9）: 各地図グループの z 軸が鉛直（重力の方向）にそろっているか（5.1 節の前提）。そろっていない場合は、外部ツールでそろえてもらえるか。
13. **`dr_error_distance` の値**（v0.9）: 地図と GNSS 区間の境目でデッドレコニングになる区間の最大長（運用で決まる）と、経路追従が許せる誤差から決める。既定の 30 m は仮の値で、i2Nav-Robot の V1-2 でデッドレコニング中の誤差の伸びを測ってから決める。
14. **デッドレコニングの ERROR のときの下流の振る舞い**（v0.9）: 停止するか、減速して走り続けるか（10 と合わせて決める）。
15. ~~地図だけの現場のアンカー~~ → 緯度経度と方位は分からないので、`anchor: local` で地図の座標をそのまま出力する（v0.10 で確定）。経路も地図の座標で作る。この現場では GNSS を使わない。
16. **`cov_scale` と品質の閾値**（v0.10）: 合成データでしか確かめていない。実データで、誤差と共分散が整合するように決める（i2Nav-Robot の V2-1）。
17. **地図上での初期化の運用**（v0.10）: 起動のたびに RViz などで初期姿勢を与えるか、前回保存した位置を使うか（止めた場所から動かさずに起動できるか）。
