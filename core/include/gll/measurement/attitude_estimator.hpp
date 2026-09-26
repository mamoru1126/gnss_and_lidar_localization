// roll / pitch の姿勢推定器（Mahony 型の相補フィルタ）。設計書 3.9 節。
// yaw は扱わない（Invariant EKF 側で推定する）。
#pragma once

#include "gll/common/config.hpp"
#include "gll/common/types.hpp"

#include <deque>

namespace gll {

struct Attitude {
  double t = 0.0;
  double roll = 0.0;
  double pitch = 0.0;
};

class AttitudeEstimator {
 public:
  explicit AttitudeEstimator(const AttitudeConfig& cfg = AttitudeConfig()) : cfg_(cfg) {}

  /// 起動時の静止区間のサンプルを溜める。static_init_time 分たまったら初期化して true を返す。
  bool addStaticSample(const ImuSample& s);
  bool initialized() const { return initialized_; }

  /// IMU の 1 サンプルで更新する。v は前進速度、v_dot はその時間微分（運動加速度の補償に使う）。
  void update(const ImuSample& s, double v, double v_dot);

  double roll() const;
  double pitch() const;
  const Vec3& gyroBias() const { return gyro_bias_; }

  /// roll / pitch だけで傾けた回転（yaw = 0）。
  Mat3 tiltRotation() const;

  /// 鉛直軸まわりのヨーレート [R_tilt (ω - b_xy)]_z（z 軸のバイアスは Invariant EKF で扱うので引かない）。
  double verticalRate(const Vec3& gyro) const;

  /// 時刻 t の roll / pitch（履歴から最も近いもの）。
  Attitude attitudeAt(double t) const;

 private:
  AttitudeConfig cfg_;
  bool initialized_ = false;
  Eigen::Quaterniond q_ = Eigen::Quaterniond::Identity();
  Vec3 gyro_bias_ = Vec3::Zero();
  Vec3 integral_ = Vec3::Zero();
  double last_t_ = -1.0;
  // 静止初期化用
  double static_t0_ = -1.0;
  Vec3 acc_sum_ = Vec3::Zero();
  Vec3 gyro_sum_ = Vec3::Zero();
  int static_n_ = 0;
  std::deque<Attitude> history_;
};

}  // namespace gll
