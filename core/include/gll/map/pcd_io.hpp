// PCD ファイルの読み書き（PCL に依存しない最小限の実装。設計書 7.7 節）。
// 読み込み: DATA ascii / binary / binary_compressed、x・y・z が F4 または F8 のもの。NaN の点は捨てる。
#pragma once

#include "gll/common/types.hpp"

#include <string>
#include <vector>

namespace gll {

enum class PcdFormat { ASCII, BINARY, BINARY_COMPRESSED };

/// PCD の x, y, z を読む。失敗したら std::runtime_error を投げる。
std::vector<Vec3f> readPcd(const std::string& path);

/// x, y, z（F4）だけの PCD を書く。BINARY_COMPRESSED は LZF のリテラルだけで書く（圧縮はしないが、正しい形式になる）。
void writePcd(const std::string& path, const std::vector<Vec3f>& points, PcdFormat format = PcdFormat::BINARY);

/// LZF の展開（PCD の binary_compressed 用）。出力の大きさが out_size と一致しなければ std::runtime_error。
std::vector<char> lzfDecompress(const char* in, std::size_t in_size, std::size_t out_size);

}  // namespace gll
