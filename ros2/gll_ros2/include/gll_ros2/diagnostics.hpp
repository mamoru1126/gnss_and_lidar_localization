// 診断メッセージの生成（IF 層。設計書 3.12 節・7.5 節）。
#pragma once

#include <gll/localizer.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <cstdint>
#include <string>

namespace gll_ros2 {

/// 小数点以下 digits 桁の文字列にする。
std::string toFixed(double v, int digits);

/// 推定の状態を診断の重大度にする（INITIALIZING / DEGRADED = WARN、LOST = ERROR、それ以外 = OK）。
uint8_t statusLevel(gll::LocalizationStatus s);

/// 自己位置推定の状態を DiagnosticStatus にする。out が nullptr のときは初期化中（WARN）。
/// 位置の観測（GNSS / LiDAR）なしで dr_error_distance [m] を超えて走ったら ERROR にし、メッセージで知らせる。
diagnostic_msgs::msg::DiagnosticStatus makeLocalizationStatus(const gll::LocalizationOutput* out,
                                                              const gll::Diagnostics& d, double dr_error_distance);

/// 入力数・採用数・棄却理由などのカウンタ（常に OK）。
diagnostic_msgs::msg::DiagnosticStatus makeCounterStatus(const gll::Diagnostics& d);

}  // namespace gll_ros2
