#include "path_planner/yaml_lite.hpp"

#include <cerrno>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <sstream>

namespace path_planner
{
namespace
{
struct Line
{
  int number{0};
  int indent{0};
  std::string content;
};

std::string trim(const std::string & s)
{
  const auto begin = s.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = s.find_last_not_of(" \t\r\n");
  return s.substr(begin, end - begin + 1);
}

[[noreturn]] void fail(int line, const std::string & message)
{
  throw YamlError("YAML line " + std::to_string(line) + ": " + message);
}

// 引用符外の '#'（行頭または空白直後）以降を除去する。
std::string stripComment(const std::string & s)
{
  char quote = 0;
  for (std::size_t i = 0; i < s.size(); ++i) {
    const char c = s[i];
    if (quote != 0) {
      if (c == quote) {
        quote = 0;
      }
      continue;
    }
    if (c == '"' || c == '\'') {
      quote = c;
    } else if (c == '#' && (i == 0 || s[i - 1] == ' ' || s[i - 1] == '\t')) {
      return s.substr(0, i);
    }
  }
  return s;
}

std::vector<Line> splitLines(const std::string & text)
{
  std::vector<Line> lines;
  std::istringstream stream(text);
  std::string raw;
  int number = 0;
  while (std::getline(stream, raw)) {
    ++number;
    if (!raw.empty() && raw.back() == '\r') {
      raw.pop_back();
    }
    const std::string without_comment = stripComment(raw);
    if (trim(without_comment).empty()) {
      continue;
    }
    int indent = 0;
    while (indent < static_cast<int>(without_comment.size()) &&
      without_comment[static_cast<std::size_t>(indent)] == ' ')
    {
      ++indent;
    }
    if (without_comment[static_cast<std::size_t>(indent)] == '\t') {
      fail(number, "tab indentation is not supported");
    }
    if (trim(without_comment) == "---") {
      continue;
    }
    lines.push_back({number, indent, trim(without_comment)});
  }
  return lines;
}

std::string unquote(const std::string & s, int line)
{
  if (s.size() >= 2 && (s.front() == '"' || s.front() == '\'')) {
    if (s.back() != s.front()) {
      fail(line, "unterminated quoted scalar: " + s);
    }
    return s.substr(1, s.size() - 2);
  }
  return s;
}

YamlNode makeScalar(const std::string & text, int line)
{
  YamlNode node;
  node.line = line;
  const std::string t = trim(text);
  if (t == "~" || t == "null") {
    node.type = YamlNode::Type::kNull;
    return node;
  }
  if (!t.empty() && t.front() == '{') {
    fail(line, "flow mappings are not supported");
  }
  if (!t.empty() && (t.front() == '&' || t.front() == '*' || t.front() == '|' || t.front() == '>')) {
    fail(line, "anchors, aliases and block scalars are not supported");
  }
  node.type = YamlNode::Type::kScalar;
  node.scalar = unquote(t, line);
  return node;
}

// フローシーケンス "[...]" を pos から解析する。
YamlNode parseFlow(const std::string & s, std::size_t & pos, int line)
{
  YamlNode node;
  node.line = line;
  node.type = YamlNode::Type::kList;
  ++pos;  // '['
  std::string token;
  bool token_present = false;
  char quote = 0;
  while (pos < s.size()) {
    const char c = s[pos];
    if (quote != 0) {
      token += c;
      if (c == quote) {
        quote = 0;
      }
      ++pos;
      continue;
    }
    if (c == '"' || c == '\'') {
      quote = c;
      token += c;
      token_present = true;
      ++pos;
    } else if (c == '[') {
      if (!trim(token).empty()) {
        fail(line, "unexpected '[' in flow sequence");
      }
      node.list.push_back(parseFlow(s, pos, line));
      token.clear();
      token_present = false;
      // 入れ子の直後はカンマか ']' を期待する
      while (pos < s.size() && s[pos] == ' ') {
        ++pos;
      }
      if (pos < s.size() && s[pos] == ',') {
        ++pos;
      }
    } else if (c == ',' || c == ']') {
      const std::string t = trim(token);
      if (!t.empty() || token_present) {
        node.list.push_back(makeScalar(t, line));
      } else if (c == ',') {
        fail(line, "empty element in flow sequence");
      }
      token.clear();
      token_present = false;
      ++pos;
      if (c == ']') {
        return node;
      }
    } else {
      token += c;
      if (c != ' ') {
        token_present = true;
      }
      ++pos;
    }
  }
  fail(line, "unterminated flow sequence (multi-line flow is not supported)");
}

YamlNode parseValue(const std::string & text, int line)
{
  const std::string t = trim(text);
  if (!t.empty() && t.front() == '[') {
    std::size_t pos = 0;
    YamlNode node = parseFlow(t, pos, line);
    if (!trim(t.substr(pos)).empty()) {
      fail(line, "trailing characters after flow sequence");
    }
    return node;
  }
  return makeScalar(t, line);
}

// 引用符・括弧の外にある "key:" の区切り位置を返す（無ければ npos）。
std::size_t findKeySeparator(const std::string & s)
{
  char quote = 0;
  int depth = 0;
  for (std::size_t i = 0; i < s.size(); ++i) {
    const char c = s[i];
    if (quote != 0) {
      if (c == quote) {
        quote = 0;
      }
      continue;
    }
    if (c == '"' || c == '\'') {
      quote = c;
    } else if (c == '[') {
      ++depth;
    } else if (c == ']') {
      --depth;
    } else if (c == ':' && depth == 0 && (i + 1 == s.size() || s[i + 1] == ' ')) {
      return i;
    }
  }
  return std::string::npos;
}

bool isListItem(const std::string & content)
{
  return content == "-" || (content.size() >= 2 && content[0] == '-' && content[1] == ' ');
}

class BlockParser
{
public:
  explicit BlockParser(std::vector<Line> lines)
  : lines_(std::move(lines)) {}

  YamlNode parseDocument()
  {
    if (lines_.empty()) {
      return YamlNode{};
    }
    std::size_t idx = 0;
    YamlNode root = parseBlock(idx, lines_[0].indent);
    if (idx != lines_.size()) {
      fail(lines_[idx].number, "unexpected indentation");
    }
    return root;
  }

private:
  std::vector<Line> lines_;

  YamlNode parseBlock(std::size_t & idx, int indent)
  {
    if (isListItem(lines_[idx].content)) {
      return parseList(idx, indent);
    }
    return parseMap(idx, indent);
  }

  YamlNode parseList(std::size_t & idx, int indent)
  {
    YamlNode node;
    node.type = YamlNode::Type::kList;
    node.line = lines_[idx].number;
    while (idx < lines_.size() && lines_[idx].indent == indent && isListItem(lines_[idx].content)) {
      Line & line = lines_[idx];
      const std::string rest = line.content == "-" ? "" : trim(line.content.substr(2));
      if (rest.empty()) {
        ++idx;
        if (idx < lines_.size() && lines_[idx].indent > indent) {
          node.list.push_back(parseBlock(idx, lines_[idx].indent));
        } else {
          YamlNode null_node;
          null_node.line = line.number;
          node.list.push_back(null_node);
        }
      } else if (findKeySeparator(rest) != std::string::npos || isListItem(rest)) {
        // "- key: value" はマップ要素。内容の開始位置を新しいインデントとして扱う。
        const auto offset = line.content.find(rest);
        line.indent = indent + static_cast<int>(offset);
        line.content = rest;
        node.list.push_back(parseBlock(idx, line.indent));
      } else {
        node.list.push_back(parseValue(rest, line.number));
        ++idx;
      }
    }
    if (idx < lines_.size() && lines_[idx].indent > indent) {
      fail(lines_[idx].number, "unexpected indentation in sequence");
    }
    return node;
  }

  YamlNode parseMap(std::size_t & idx, int indent)
  {
    YamlNode node;
    node.type = YamlNode::Type::kMap;
    node.line = lines_[idx].number;
    while (idx < lines_.size() && lines_[idx].indent == indent) {
      const Line line = lines_[idx];
      if (isListItem(line.content)) {
        fail(line.number, "sequence item where a mapping key was expected");
      }
      const std::size_t sep = findKeySeparator(line.content);
      if (sep == std::string::npos) {
        fail(line.number, "expected 'key: value': " + line.content);
      }
      const std::string key = unquote(trim(line.content.substr(0, sep)), line.number);
      const std::string value = trim(line.content.substr(sep + 1));
      for (const auto & entry : node.map) {
        if (entry.first == key) {
          fail(line.number, "duplicate key: " + key);
        }
      }
      ++idx;
      if (!value.empty()) {
        node.map.emplace_back(key, parseValue(value, line.number));
      } else if (idx < lines_.size() && lines_[idx].indent > indent) {
        node.map.emplace_back(key, parseBlock(idx, lines_[idx].indent));
      } else if (idx < lines_.size() && lines_[idx].indent == indent &&
        isListItem(lines_[idx].content))
      {
        node.map.emplace_back(key, parseList(idx, indent));
      } else {
        YamlNode null_node;
        null_node.line = line.number;
        node.map.emplace_back(key, null_node);
      }
    }
    if (idx < lines_.size() && lines_[idx].indent > indent) {
      fail(lines_[idx].number, "unexpected indentation in mapping");
    }
    return node;
  }
};

}  // namespace

const YamlNode * YamlNode::find(const std::string & key) const
{
  if (type != Type::kMap) {
    return nullptr;
  }
  for (const auto & entry : map) {
    if (entry.first == key) {
      return &entry.second;
    }
  }
  return nullptr;
}

const YamlNode & YamlNode::at(const std::string & key) const
{
  const YamlNode * node = find(key);
  if (node == nullptr) {
    throw YamlError("YAML line " + std::to_string(line) + ": missing key '" + key + "'");
  }
  return *node;
}

double YamlNode::asDouble() const
{
  if (type != Type::kScalar) {
    throw YamlError("YAML line " + std::to_string(line) + ": expected a number");
  }
  const std::string s = scalar;
  if (s == ".inf" || s == "inf") {
    return std::numeric_limits<double>::infinity();
  }
  char * end = nullptr;
  errno = 0;
  const double v = std::strtod(s.c_str(), &end);
  if (end == s.c_str() || *end != '\0' || errno == ERANGE) {
    throw YamlError("YAML line " + std::to_string(line) + ": invalid number '" + s + "'");
  }
  return v;
}

int YamlNode::asInt() const
{
  const double v = asDouble();
  if (v != static_cast<double>(static_cast<long long>(v))) {
    throw YamlError("YAML line " + std::to_string(line) + ": expected an integer");
  }
  return static_cast<int>(v);
}

bool YamlNode::asBool() const
{
  if (type == Type::kScalar) {
    if (scalar == "true" || scalar == "True" || scalar == "TRUE") {
      return true;
    }
    if (scalar == "false" || scalar == "False" || scalar == "FALSE") {
      return false;
    }
  }
  throw YamlError("YAML line " + std::to_string(line) + ": expected a boolean");
}

std::string YamlNode::asString() const
{
  if (type != Type::kScalar) {
    throw YamlError("YAML line " + std::to_string(line) + ": expected a string");
  }
  return scalar;
}

std::vector<double> YamlNode::asDoubleList() const
{
  if (type != Type::kList) {
    throw YamlError("YAML line " + std::to_string(line) + ": expected a list of numbers");
  }
  std::vector<double> out;
  out.reserve(list.size());
  for (const auto & item : list) {
    out.push_back(item.asDouble());
  }
  return out;
}

std::vector<std::string> YamlNode::asStringList() const
{
  if (type != Type::kList) {
    throw YamlError("YAML line " + std::to_string(line) + ": expected a list of strings");
  }
  std::vector<std::string> out;
  out.reserve(list.size());
  for (const auto & item : list) {
    out.push_back(item.asString());
  }
  return out;
}

YamlNode parseYamlString(const std::string & text)
{
  BlockParser parser(splitLines(text));
  return parser.parseDocument();
}

YamlNode parseYamlFile(const std::string & path)
{
  std::ifstream file(path);
  if (!file) {
    throw YamlError("cannot open YAML file: " + path);
  }
  std::ostringstream buffer;
  buffer << file.rdbuf();
  return parseYamlString(buffer.str());
}

}  // namespace path_planner
