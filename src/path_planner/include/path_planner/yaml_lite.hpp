#pragma once

// 設定ファイル読込用の最小 YAML サブセットパーサ（仕様書 第2.4節: 外部パーサ不使用）。
//
// 対応する構文:
//   - インデントによるブロックマップ `key: value` / `key:` + 子ブロック
//   - ブロックシーケンス `- value`、`- key: value`（マップ要素）
//   - 1行で閉じるフローシーケンス `[1.0, 2.0, [3, 4]]`（入れ子可）
//   - `#` 以降のコメント、"..." / '...' で囲んだスカラー
// 非対応: アンカー/エイリアス、フローマップ `{}`、複数行スカラー、タブインデント。
// 非対応構文は黙って無視せず YamlError を送出する。

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace path_planner
{
class YamlError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

class YamlNode
{
public:
  enum class Type {kNull, kScalar, kMap, kList};

  Type type{Type::kNull};
  std::string scalar;
  std::vector<std::pair<std::string, YamlNode>> map;
  std::vector<YamlNode> list;
  int line{0};

  bool isNull() const {return type == Type::kNull;}
  bool isMap() const {return type == Type::kMap;}
  bool isList() const {return type == Type::kList;}
  bool isScalar() const {return type == Type::kScalar;}

  /// キーが無ければ nullptr。
  const YamlNode * find(const std::string & key) const;
  bool has(const std::string & key) const {return find(key) != nullptr;}
  /// キーが無ければ YamlError。
  const YamlNode & at(const std::string & key) const;

  double asDouble() const;
  int asInt() const;
  bool asBool() const;
  std::string asString() const;
  std::vector<double> asDoubleList() const;
  std::vector<std::string> asStringList() const;
};

YamlNode parseYamlString(const std::string & text);
YamlNode parseYamlFile(const std::string & path);

}  // namespace path_planner
