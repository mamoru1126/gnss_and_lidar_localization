// 推定器の状態・入力・観測の型（アルゴリズム説明書 8〜10 章）。
#pragma once

#include "tiled_pcd_map/se2.hpp"
#include "gll/common/types.hpp"

#include <Eigen/Core>

#include <string>
#include <variant>

namespace gll {

/// フィルタの状態。誤差 δx = (ρx, ρy, φ, δb, δs) の共分散 P を持つ。
struct FilterState {
  double t = 0.0;
  SE2 X;               ///< 姿勢（map = UTM）
  double b = 0.0;      ///< ジャイロの鉛直軸バイアス [rad/s]
  double s = 1.0;      ///< ODOM 速度のスケール係数
  Mat5 P = Mat5::Identity();
};

/// 予測の入力。t は区間の終わりの時刻。
struct MotionInput {
  double t = 0.0;
  double v = 0.0;      ///< 傾斜補正済みの前進速度 [m/s]
  double v_lat = 0.0;  ///< 横速度 [m/s]
  double omega = 0.0;  ///< 傾斜補正済みのヨーレート（バイアス未補正）[rad/s]
};

/// GNSS の位置観測（アンテナ位置、UTM）。
struct GnssPositionMeasurement {
  double t = 0.0;
  Vec2 y = Vec2::Zero();                   ///< アンテナの UTM 座標
  Mat2 cov_world = Mat2::Identity();       ///< 世界座標系の共分散
  Vec2 lever_h = Vec2::Zero();             ///< roll / pitch で水平面に射影したレバーアーム
  Mat2 lever_cov_body = Mat2::Zero();      ///< roll / pitch の誤差による追加の共分散（機体座標系）
};

/// yaw の観測（GNSS の進行方位など）。
struct HeadingMeasurement {
  double t = 0.0;
  double yaw = 0.0;
  double var = 1.0;
};

/// 姿勢の観測（LiDAR の位置合わせ結果を UTM に変換したもの。Phase 2）。
struct PoseMeasurement {
  double t = 0.0;
  SE2 Z;
  Mat3 cov_body = Mat3::Identity();
  Mat3 anchor_cov_world = Mat3::Zero();
  std::string map_group;
};

/// 停止中のゼロ角速度観測（ZARU）。
struct ZeroRateMeasurement {
  double t = 0.0;
  double omega_mean = 0.0;
  double var = 1.0;
};

using Measurement =
    std::variant<GnssPositionMeasurement, HeadingMeasurement, PoseMeasurement, ZeroRateMeasurement>;

enum class MeasurementKind { GNSS_POSITION, HEADING, POSE, ZERO_RATE };

inline double measurementTime(const Measurement& m) {
  return std::visit([](const auto& x) { return x.t; }, m);
}

inline MeasurementKind measurementKind(const Measurement& m) {
  switch (m.index()) {
    case 0: return MeasurementKind::GNSS_POSITION;
    case 1: return MeasurementKind::HEADING;
    case 2: return MeasurementKind::POSE;
    default: return MeasurementKind::ZERO_RATE;
  }
}

/// 観測の線形化結果（残差・観測行列・観測共分散）。
struct Linearization {
  Eigen::VectorXd r;
  Eigen::MatrixXd H;
  Eigen::MatrixXd R;
};

/// 外れ値判定に落ちたときの扱い（設計書 3.13 節）。
enum class GatePolicy {
  REJECT_ON_FAIL,     ///< 棄却する
  DEFER_TO_RECOVERY,  ///< 適用せず、再アンカーの候補として RecoveryManager に渡す（RTK-FIX の GNSS）
  SKIP                ///< 判定しない（再アンカーなど）
};

struct UpdateResult {
  bool accepted = false;     ///< 状態に適用したか
  bool gate_passed = false;  ///< 外れ値判定を通ったか
  bool too_old = false;      ///< 履歴より古くて適用できなかったか
  double d2 = 0.0;           ///< Mahalanobis 距離の 2 乗
  int dof = 0;
  Eigen::VectorXd residual;  ///< 残差（推定器の誤差の座標系）
  Vec3 world_delta = Vec3::Zero();  ///< 最新の推定姿勢の、世界座標系での変化量（x, y, yaw）
};

}  // namespace gll
