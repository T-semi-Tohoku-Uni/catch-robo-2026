#include "catchrobo2026_sequence/sequence_check.hpp"
#include "nav_director/route_planner.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using catchrobo2026_sequence::CheckJoints;
using catchrobo2026_sequence::CheckStatus;
using catchrobo2026_sequence::Pose;
using catchrobo2026_sequence::SequenceCheckOptions;
using catchrobo2026_sequence::SequenceCheckResult;
using catchrobo2026_sequence::SequenceConfig;
using catchrobo2026_sequence::Step;
constexpr double kTurn = 6.28318530717958647692;
constexpr double kTolerance = 1e-6;

struct UsageError : std::runtime_error {using std::runtime_error::runtime_error;};
struct Target {std::string kind, name;};
struct Options
{
  std::string config, team{"both"}, initial_pose_name;
  bool syntax_only{false}, all{false}, json{false}, warnings_as_errors{false};
  std::optional<CheckJoints> initial_joints, after_initialization_joints;
  std::optional<Pose> initial_pose;
  std::optional<double> initial_wrist;
  std::vector<Target> targets;
};
struct Entry
{
  std::string team, name;
  std::optional<CheckJoints> initial_joints;
  std::vector<Pose> initial_pending_waypoints;
  std::string pending_resolution;
  SequenceCheckResult result;
};
struct Report
{
  std::string config, mode, initial_message;
  std::optional<CheckJoints> initial_joints, after_initialization_joints;
  CheckStatus status{CheckStatus::FEASIBLE};
  std::vector<Entry> entries;
  std::vector<std::string> notes;
  int exit_code{0};
};

void help()
{
  std::cout << R"(シーケンス設定と経路を、動作指令を出さずに確認します。ROS通信には参加しません。
使い方: ros2 run catchrobo2026_sequence check_sequence_config --config PATH [オプション]

  --syntax-only                    YAML・参照・展開のみ確認
  --team red|blue|both              対象陣営（既定both）
  --initial-joints Q0 Q1 Q2 Q4      初期関節角、4値ともrad
  --initial-pose X Y Z PHI          初期姿勢、XYZはmm、PHIはrad
  --initial-pose-name NAME          poses内の初期姿勢名
  --initial-wrist Q4                姿勢指定時の実第4関節角、rad
  --after-initialization-joints Q0 Q1 Q2 Q4
                                   初期化完了後の明示関節角、rad
  --action ACTION [ACTION ...]      start / initialize / end / pick:0,1 / place:0,0
  --sequence NAME [NAME ...]        名前付きシーケンスを確認
  --all                            各対象を同じ初期状態から独立に確認（既定）
  --json                           JSONを標準出力へ出力
  --warnings-as-errors             warningがあれば終了コード1
  --help                           この説明を表示

--actionと--sequenceは引数順に連続して確認します。失敗後の対象はUNKNOWNとなります。
末尾waypointは次対象の最初のMOVEへ持ち越し、最後まで未消化ならUNKNOWNとなります。
--action start/initialize/endは持越しを破棄します。--sequenceは汎用の連続手順として扱います。
連続PICK/PLACEは同じcontrol_epochでstep_idが増加する実行を想定します。
--allと対象指定は併用できません。独立確認は対象間の連続実行を保証しません。
初期状態を省略した場合は、姿勢を仮定せずUNKNOWNとします。
0と−2πの両方が可能な初期姿勢では--initial-wristが必要です。
終了コード: 0=全対象FEASIBLE、1=INFEASIBLE/設定不正、2=UNKNOWN/引数不正。
JSONのstep_index等は0始まり、通常表示の手順番号は1始まりです。
経路計算上の判定であり、衝突・機構動作・実機停止・実時間での実行を保証しません。
)";
}

double numeric(const std::string & value, const std::string & flag)
{
  std::size_t used = 0;
  double result;
  try {result = std::stod(value, &used);} catch (const std::exception &) {
    throw UsageError(flag + ": 数値が必要です: " + value);
  }
  if (used != value.size() || !std::isfinite(result)) {
    throw UsageError(flag + ": 有限の数値が必要です: " + value);
  }
  return result;
}

Options parse(int argc, char ** argv)
{
  Options options;
  auto take = [&](int & index, const std::string & flag) {
      if (++index >= argc || std::string(argv[index]).rfind("--", 0) == 0) {
        throw UsageError(flag + ": 引数が不足しています");
      }
      return std::string(argv[index]);
    };
  auto joints = [&](int & index, const std::string & flag) {
      CheckJoints values;
      for (auto & value : values) {
        value = static_cast<float>(numeric(take(index, flag), flag));
        if (!std::isfinite(value)) {throw UsageError(flag + ": Float32の範囲を超えています");}
      }
      return values;
    };
  bool initial_seen = false;
  for (int i = 1; i < argc; ++i) {
    const std::string flag = argv[i];
    if (flag == "--config") {options.config = take(i, flag);}
    else if (flag == "--team") {options.team = take(i, flag);}
    else if (flag == "--syntax-only") {options.syntax_only = true;}
    else if (flag == "--all") {options.all = true;}
    else if (flag == "--json") {options.json = true;}
    else if (flag == "--warnings-as-errors") {options.warnings_as_errors = true;}
    else if (flag == "--initial-wrist") {
      if (options.initial_wrist) {throw UsageError("--initial-wristの重複指定です");}
      options.initial_wrist = numeric(take(i, flag), flag);
    } else if (flag == "--after-initialization-joints") {
      if (options.after_initialization_joints) {throw UsageError(flag + "の重複指定です");}
      options.after_initialization_joints = joints(i, flag);
    } else if (flag == "--initial-joints" || flag == "--initial-pose" ||
      flag == "--initial-pose-name")
    {
      if (initial_seen) {throw UsageError("初期状態の指定方式は1つだけ選択してください");}
      initial_seen = true;
      if (flag == "--initial-joints") {options.initial_joints = joints(i, flag);}
      else if (flag == "--initial-pose-name") {options.initial_pose_name = take(i, flag);}
      else {
        Pose pose;
        for (auto & value : pose) {value = numeric(take(i, flag), flag);}
        options.initial_pose = pose;
      }
    } else if (flag == "--action" || flag == "--sequence") {
      const auto begin = options.targets.size();
      while (i + 1 < argc && std::string(argv[i + 1]).rfind("--", 0) != 0) {
        options.targets.push_back({flag == "--action" ? "action" : "sequence", argv[++i]});
      }
      if (begin == options.targets.size()) {throw UsageError(flag + ": 対象を指定してください");}
    } else {throw UsageError("不明な引数です: " + flag);}
  }
  if (options.config.empty()) {throw UsageError("--config PATHを指定してください");}
  if (options.team != "red" && options.team != "blue" && options.team != "both") {
    throw UsageError("--teamはred、blue、bothのいずれかです");
  }
  if (options.all && !options.targets.empty()) {
    throw UsageError("--allと--action/--sequenceは併用できません");
  }
  if (options.initial_wrist && !options.initial_pose && options.initial_pose_name.empty()) {
    throw UsageError("--initial-wristは--initial-poseまたは--initial-pose-nameと併用してください");
  }
  return options;
}

std::string status_name(CheckStatus status)
{
  if (status == CheckStatus::FEASIBLE) {return "FEASIBLE";}
  if (status == CheckStatus::INFEASIBLE) {return "INFEASIBLE";}
  return "UNKNOWN";
}

std::string quote(const std::string & value)
{
  std::ostringstream output;
  output << '"';
  for (const unsigned char c : value) {
    switch (c) {
      case '"': output << "\\\""; break;
      case '\\': output << "\\\\"; break;
      case '\b': output << "\\b"; break;
      case '\f': output << "\\f"; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default:
        if (c < 0x20) {
          output << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c) << std::dec;
        } else {output << c;}
    }
  }
  output << '"';
  return output.str();
}

void write_number(double value)
{
  const auto precision = std::cout.precision();
  if (std::isfinite(value)) {std::cout << std::setprecision(17) << value;}
  else {std::cout << "null";}
  std::cout.precision(precision);
}

void write_joints(const std::optional<CheckJoints> & joints)
{
  if (!joints) {std::cout << "null"; return;}
  std::cout << '[';
  for (std::size_t i = 0; i < joints->size(); ++i) {
    if (i != 0) {std::cout << ',';}
    write_number((*joints)[i]);
  }
  std::cout << ']';
}

void write_poses(const std::vector<Pose> & poses)
{
  std::cout << '[';
  for (std::size_t i = 0; i < poses.size(); ++i) {
    if (i != 0) {std::cout << ',';}
    std::cout << '[';
    for (std::size_t axis = 0; axis < poses[i].size(); ++axis) {
      if (axis != 0) {std::cout << ',';}
      write_number(poses[i][axis]);
    }
    std::cout << ']';
  }
  std::cout << ']';
}

void write_json(const Report & report)
{
  std::cout << "{\"status\":" << quote(status_name(report.status))
            << ",\"mode\":" << quote(report.mode) << ",\"config\":" << quote(report.config)
            << ",\"initial_joints\":";
  write_joints(report.initial_joints);
  std::cout << ",\"initial_message\":" << quote(report.initial_message)
            << ",\"after_initialization_joints\":";
  write_joints(report.after_initialization_joints);
  std::cout << ",\"results\":[";
  for (std::size_t i = 0; i < report.entries.size(); ++i) {
    if (i != 0) {std::cout << ',';}
    const auto & entry = report.entries[i];
    const auto & result = entry.result;
    std::cout << "{\"team\":" << quote(entry.team) << ",\"name\":" << quote(entry.name)
              << ",\"status\":" << quote(status_name(result.status))
              << ",\"message\":" << quote(result.message) << ",\"initial_joints\":";
    write_joints(entry.initial_joints);
    std::cout << ",\"final_joints\":";
    write_joints(result.final_joints);
    std::cout << ",\"initial_pending_waypoints\":";
    write_poses(entry.initial_pending_waypoints);
    std::cout << ",\"pending_waypoints\":";
    write_poses(result.pending_waypoints);
    std::cout << ",\"consumed_pending_waypoints\":" << result.consumed_pending_waypoints
              << ",\"discarded_pending_waypoints\":" << result.discarded_pending_waypoints
              << ",\"pending_resolution\":" << quote(entry.pending_resolution);
    std::cout << ",\"phi_travel\":";
    write_number(result.phi_travel);
    std::cout << ",\"max_wrist_step\":";
    write_number(result.max_wrist_step);
    std::cout << ",\"route_count\":" << result.route_count
              << ",\"sample_count\":" << result.sample_count << ",\"diagnostics\":[";
    for (std::size_t j = 0; j < result.diagnostics.size(); ++j) {
      if (j != 0) {std::cout << ',';}
      const auto & item = result.diagnostics[j];
      std::cout << "{\"severity\":" << quote(item.severity) << ",\"code\":" << quote(item.code)
                << ",\"message\":" << quote(item.message) << ",\"step_index\":" << item.step_index
                << ",\"route_index\":" << item.route_index
                << ",\"sample_index\":" << item.sample_index << '}';
    }
    std::cout << "]}";
  }
  std::cout << "],\"notes\":[";
  for (std::size_t i = 0; i < report.notes.size(); ++i) {
    if (i != 0) {std::cout << ',';}
    std::cout << quote(report.notes[i]);
  }
  std::cout << "],\"exit_code\":" << report.exit_code << "}\n";
}

void write_text(const Report & report)
{
  std::cout << status_name(report.status) << ": " << report.config << '\n';
  std::cout << "初期関節角 [q0, q1, q2, q4] rad: ";
  write_joints(report.initial_joints);
  std::cout << "  " << report.initial_message << '\n';
  if (report.after_initialization_joints) {
    std::cout << "初期化後の明示関節角 rad: ";
    write_joints(report.after_initialization_joints);
    std::cout << '\n';
  }
  for (const auto & note : report.notes) {std::cout << note << '\n';}
  for (const auto & entry : report.entries) {
    const auto & result = entry.result;
    std::cout << '[' << entry.team << "] " << entry.name << ": " << status_name(result.status)
              << " — " << result.message << '\n';
    std::cout << "  検査済み区間のphi総移動量=" << result.phi_travel << " rad、第4関節最大指令差="
              << result.max_wrist_step << " rad、経路=" << result.route_count
              << "、標本=" << result.sample_count << '\n';
    if (!result.pending_waypoints.empty()) {
      std::cout << "  この対象終了時の持越しwaypoint=" << result.pending_waypoints.size();
      if (!entry.pending_resolution.empty()) {std::cout << "、" << entry.pending_resolution;}
      else {std::cout << "（後続経路は未検査）";}
      std::cout << '\n';
    }
    if (result.consumed_pending_waypoints || result.discarded_pending_waypoints) {
      std::cout << "  先行waypoint: 経路へ連結=" << result.consumed_pending_waypoints
                << "、境界で破棄=" << result.discarded_pending_waypoints << '\n';
    }
    for (const auto & item : result.diagnostics) {
      std::cout << "  " << item.severity << ' ' << item.code;
      if (item.code != "INITIAL_STATE_UNKNOWN" && item.code != "SKIPPED_AFTER_FAILURE") {
        std::cout << " 手順=" << item.step_index + 1;
      }
      if (item.code == "wrist_wrap_jump" || item.code == "unreachable_sample" ||
        item.code == "invalid_group_sample")
      {
        std::cout << " 経路=" << item.route_index + 1 << " 標本=" << item.sample_index + 1;
      }
      std::cout << ": " << item.message << '\n';
    }
    if (result.final_joints) {
      std::cout << "  最終関節角 rad: ";
      write_joints(result.final_joints);
      std::cout << '\n';
    }
  }
}

void validate_joints(const CheckJoints & joints)
{
  if (joints[3] < -kTurn - kTolerance || joints[3] > kTolerance) {
    throw std::runtime_error("初期第4関節角は[-2π, 0]radの範囲が必要です");
  }
  nav_director::RoutePlanner planner;
  const auto state = planner.stateFromJoints(joints);
  if (!std::isfinite(state.pose.x) || !std::isfinite(state.pose.y) ||
    !std::isfinite(state.pose.z) || !std::isfinite(state.pose.phi))
  {
    throw std::runtime_error("初期関節角から有限の姿勢を計算できません");
  }
}

void resolve_initial(const Options & options, const SequenceConfig & config, Report & report)
{
  if (options.after_initialization_joints) {
    validate_joints(*options.after_initialization_joints);
    report.after_initialization_joints = options.after_initialization_joints;
  }
  if (options.initial_joints) {
    validate_joints(*options.initial_joints);
    report.initial_joints = options.initial_joints;
    report.initial_message = "--initial-jointsで指定した状態です";
    return;
  }
  if (!options.initial_pose && options.initial_pose_name.empty()) {
    report.initial_message = "初期状態未指定。姿勢を仮定しません";
    return;
  }
  const auto pose = options.initial_pose ? *options.initial_pose :
    config.named_pose(options.initial_pose_name);
  nav_director::RoutePlanner planner;
  CheckJoints joints;
  if (!planner.solveIK({pose[0], pose[1], pose[2], pose[3]}, joints)) {
    throw std::runtime_error("指定した初期姿勢は逆運動学で到達不能です");
  }
  const double raw = pose[3] - joints[0];
  if (options.initial_wrist) {
    const double wrist = *options.initial_wrist;
    if (wrist < -kTurn - kTolerance || wrist > kTolerance ||
      std::abs(std::remainder(wrist - raw, kTurn)) > 1e-5)
    {
      throw std::runtime_error("--initial-wristが初期姿勢のphiまたは[-2π, 0]radと一致しません");
    }
    joints[3] = static_cast<float>(wrist);
  } else {
    double normalized = std::fmod(raw, kTurn);
    if (normalized > 0.0) {normalized -= kTurn;}
    if (std::abs(normalized) <= kTolerance || std::abs(normalized + kTurn) <= kTolerance) {
      report.initial_message = "初期第4関節角は0または−2πの両方が可能です。"
        "--initial-wristで実際の角度を指定してください";
      return;
    }
    joints[3] = static_cast<float>(normalized);
  }
  validate_joints(joints);
  report.initial_joints = joints;
  report.initial_message = options.initial_pose_name.empty() ?
    "指定した初期姿勢から算出した関節角です" :
    "poses." + options.initial_pose_name + "から算出した関節角です";
}

std::vector<Step> compile_target(
  const SequenceConfig & config, const std::string & team, const Target & target)
{
  if (target.kind == "sequence") {return config.compile_named(target.name);}
  if (target.name == "start") {return config.compile_start();}
  if (target.name == "initialize") {return config.compile_initialization();}
  if (target.name == "end") {return config.compile_end();}
  const auto colon = target.name.find(':');
  const auto comma = target.name.find(',');
  if (colon == std::string::npos || comma == std::string::npos || comma <= colon + 1 ||
    (target.name.substr(0, colon) != "pick" && target.name.substr(0, colon) != "place"))
  {
    throw UsageError("不正なaction指定です: " + target.name);
  }
  auto index = [&](std::size_t begin, std::size_t length) {
      const auto value = target.name.substr(begin, length);
      std::size_t used = 0;
      int result;
      try {result = std::stoi(value, &used);} catch (const std::exception &) {
        throw UsageError("actionの位置番号が不正です: " + target.name);
      }
      if (used != value.size()) {throw UsageError("actionの位置番号が不正です: " + target.name);}
      return result;
    };
  const auto kind = target.name.substr(0, colon);
  const int first = index(colon + 1, comma - colon - 1);
  const int second = index(comma + 1, std::string::npos);
  if (first < 0 || first > 3 ||
    (kind == "pick" && (second < 1 || second > 4)) ||
    (kind == "place" && (second < 0 || second > 1)))
  {
    throw UsageError("actionの位置番号がUI範囲外です: " + target.name);
  }
  return config.compile(team, kind, first, second);
}

bool deferred_only(const SequenceCheckResult & result)
{
  return result.status == CheckStatus::UNKNOWN && result.final_joints &&
         !result.pending_waypoints.empty() &&
         std::none_of(result.diagnostics.begin(), result.diagnostics.end(),
           [](const auto & item) {return item.severity == "unknown" &&
                    item.code != "pending_waypoints";});
}

Report check(const Options & options)
{
  Report report;
  report.config = options.config;
  report.mode = options.syntax_only ? "syntax" : options.targets.empty() ? "independent" : "chain";
  report.notes.push_back("経路計算上の判定です。衝突・機構動作・実機停止・実時間は保証しません。");
  const auto config = SequenceConfig::load(options.config);
  if (options.syntax_only) {
    report.initial_message = "文法・参照・展開の確認のみ。経路は計算していません";
    report.notes.push_back("設定の読込・全参照検証に成功しました。動作の実行可否は未判定です。");
    return report;
  }
  resolve_initial(options, config, report);
  const bool independent = options.targets.empty();
  if (independent) {
    report.notes.push_back("各対象を同じ初期状態から独立に確認します。対象間の連続実行は保証しません。");
  } else {
    report.notes.push_back("指定した対象を引数順に連続して確認します。各陣営は同じ初期状態から始めます。");
    report.notes.push_back("末尾waypointを持ち越します。連続PICK/PLACEは同epoch・step_id増加を想定し、"
      "START/INITIALIZE/ENDでは持越しを破棄します。");
  }
  const auto teams = options.team == "both" ? std::vector<std::string>{"red", "blue"} :
    std::vector<std::string>{options.team};
  SequenceCheckOptions engine_options;
  engine_options.after_initialization_joints = report.after_initialization_joints;
  bool warnings = false;
  for (const auto & team : teams) {
    auto targets = options.targets;
    if (independent) {
      targets = {{"action", "start"}, {"action", "initialize"}, {"action", "end"}};
      for (const auto & binding : config.configured_bindings(team)) {
        targets.push_back({"action", binding.kind + ":" + std::to_string(binding.index1) + "," +
          std::to_string(binding.index2)});
      }
    }
    auto current = report.initial_joints;
    std::vector<Pose> pending;
    std::vector<std::size_t> deferred_entries;
    bool blocked = false;
    for (const auto & target : targets) {
      Entry entry;
      entry.team = team;
      entry.name = target.kind == "sequence" ? "sequence:" + target.name : target.name;
      entry.initial_joints = independent ? report.initial_joints : current;
      entry.initial_pending_waypoints = independent ? std::vector<Pose>{} : pending;
      // Validate every requested name even when an earlier target cannot finish.
      const auto steps = compile_target(config, team, target);
      if (blocked && !independent) {
        entry.result.status = CheckStatus::UNKNOWN;
        entry.result.message = "先行対象が完了できないため、後続の開始状態を確定できません";
        entry.result.diagnostics.push_back({"info", "SKIPPED_AFTER_FAILURE", entry.result.message});
      } else if (!entry.initial_joints) {
        entry.result.status = CheckStatus::UNKNOWN;
        entry.result.message = report.initial_message;
        entry.result.diagnostics.push_back({"info", "INITIAL_STATE_UNKNOWN", entry.result.message});
      } else {
        engine_options.pending_waypoints = entry.initial_pending_waypoints;
        engine_options.clear_pending_waypoints = target.kind == "action" &&
          (target.name == "start" || target.name == "initialize" || target.name == "end");
        entry.result = catchrobo2026_sequence::check_sequence_steps(
          steps, entry.initial_joints, engine_options);
      }
      for (const auto & diagnostic : entry.result.diagnostics) {
        warnings = warnings || diagnostic.severity == "warning" || diagnostic.severity == "WARNING";
      }
      if (!independent) {
        if (entry.result.consumed_pending_waypoints || entry.result.discarded_pending_waypoints) {
          const std::string resolution = entry.result.consumed_pending_waypoints ?
            "後続 " + entry.name + " の経路で検査済み" :
            "後続 " + entry.name + " の境界で破棄（経由点移動は未実行）";
          for (const auto previous : deferred_entries) {
            auto & earlier = report.entries[previous];
            earlier.pending_resolution = resolution;
            earlier.result.status = CheckStatus::FEASIBLE;
            earlier.result.message = resolution;
            for (auto & diagnostic : earlier.result.diagnostics) {
              if (diagnostic.code == "pending_waypoints") {
                diagnostic.severity = "info";
                diagnostic.code = "pending_waypoints_resolved";
                diagnostic.message = resolution;
              }
            }
          }
          deferred_entries.clear();
        }
        const bool deferred = deferred_only(entry.result);
        if ((entry.result.status != CheckStatus::FEASIBLE && !deferred) ||
          !entry.result.final_joints)
        {
          blocked = true;
          current.reset();
          pending.clear();
        } else {
          current = entry.result.final_joints;
          pending = entry.result.pending_waypoints;
          if (deferred) {deferred_entries.push_back(report.entries.size());}
        }
      }
      report.entries.push_back(std::move(entry));
    }
  }
  for (const auto & entry : report.entries) {
    if (entry.result.status == CheckStatus::INFEASIBLE) {report.status = CheckStatus::INFEASIBLE;}
    else if (entry.result.status == CheckStatus::UNKNOWN && report.status == CheckStatus::FEASIBLE) {
      report.status = CheckStatus::UNKNOWN;
    }
  }
  report.exit_code = report.status == CheckStatus::INFEASIBLE ? 1 :
    report.status == CheckStatus::UNKNOWN ? 2 : 0;
  if (options.warnings_as_errors && warnings) {
    report.exit_code = 1;
    report.notes.push_back("--warnings-as-errorsによりwarningを終了コード1として扱いました。");
  }
  return report;
}
}  // namespace

int main(int argc, char ** argv)
{
  bool json = false;
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--help") {help(); return 0;}
    if (std::string(argv[i]) == "--json") {json = true;}
  }
  Report report;
  try {
    const auto options = parse(argc, argv);
    report.config = options.config;
    report = check(options);
  } catch (const UsageError & error) {
    report.status = CheckStatus::UNKNOWN;
    report.mode = "usage_error";
    report.exit_code = 2;
    report.initial_message = error.what();
  } catch (const std::exception & error) {
    report.status = CheckStatus::INFEASIBLE;
    report.mode = "configuration_error";
    report.exit_code = 1;
    report.initial_message = error.what();
  }
  if (json) {write_json(report);} else {write_text(report);}
  return report.exit_code;
}
