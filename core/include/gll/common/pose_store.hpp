// 最後の自己位置の保存と読み込み（地図だけの現場で、次の起動時の初期姿勢に使う。設計書 3.11 節）。
#pragma once

#include "gll/common/types.hpp"

#include <optional>
#include <string>

namespace gll {

struct SavedPose {
  double t = 0.0;  ///< 保存した時刻（データのタイムスタンプ）[s]
  Pose2D pose;     ///< map（UTM）の x, y, yaw
};

/// 一時ファイルに書いてから置き換える（書き込み中に電源が切れても、前の内容が残るように）。
bool savePose(const std::string& path, const SavedPose& p);

/// 読めなければ nullopt。
std::optional<SavedPose> loadPose(const std::string& path);

}  // namespace gll
