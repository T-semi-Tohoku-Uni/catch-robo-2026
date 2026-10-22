#pragma once

// 経路後処理: shortcut と C2 区分3次スプライン平滑化（仕様書 第8節）。
//
// ParameterizedPath は s in [0, 1] で q(s), dq/ds, d2q/ds2 を評価でき、
// 全軸で共通の経路パラメータ s を持つ。

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "path_planner/collision_checker.hpp"
#include "path_planner/types.hpp"

namespace path_planner
{
/// s in [0, 1] でパラメータ化された C2 連続区分3次多項式経路。
/// 区間 k (s in [knots[k], knots[k+1]]) で u = s - knots[k] とおくと
///   q_j(s) = a[k][j] + b[k][j]*u + c[k][j]*u^2 + d[k][j]*u^3
class ParameterizedPath
{
public:
  struct SegmentCoeffs
  {
    std::array<double, kDof> a{};
    std::array<double, kDof> b{};
    std::array<double, kDof> c{};
    std::array<double, kDof> d{};
  };

  ParameterizedPath() = default;
  ParameterizedPath(std::vector<double> knots, std::vector<SegmentCoeffs> segments);

  /// 静止経路（始点 == 終点）を生成する。
  static ParameterizedPath makeStationary(const State & q);
  /// 折れ線 waypoints を共通の弦長パラメータ s in [0, 1] で自然3次スプライン補間する。
  static ParameterizedPath fromWaypoints(const Path & waypoints);

  bool empty() const {return segments_.empty();}
  bool isStationary(double tol = 1e-12) const;

  State position(double s) const;
  State firstDerivative(double s) const;
  State secondDerivative(double s) const;

  const std::vector<double> & knots() const {return knots_;}
  const std::vector<SegmentCoeffs> & segments() const {return segments_;}

private:
  std::vector<double> knots_;
  std::vector<SegmentCoeffs> segments_;

  std::size_t findSegment(double s, double & u) const;
};

struct PostProcessConfig
{
  std::size_t shortcut_iterations{100};
  double max_path_deviation{0.01};
  double collision_check_resolution{0.01};
  std::size_t max_smoothing_retries{8};
  State distance_weights{2.0, 1.0, 1.0, 1.0};
};

struct PostProcessResult
{
  Path shortcut_path;
  ParameterizedPath spline;
  FailureCode failure{FailureCode::kNone};
  std::string message;
  double max_deviation{0.0};
  std::size_t smoothing_retries_used{0};

  explicit operator bool() const {return failure == FailureCode::kNone && !spline.empty();}
};

class PathPostProcessor
{
public:
  PathPostProcessor(
    PostProcessConfig config,
    std::shared_ptr<const StateValidator> validator);

  /// 折れ線経路に shortcut を適用する（始終点を保存し、全辺の妥当性を保証する）。
  Path shortcut(const Path & raw_path, std::uint32_t seed) const;

  /// shortcut + C2 スプライン補間 + 最大偏差・衝突・関節範囲の検証と保守的再試行。
  PostProcessResult process(const Path & raw_path, std::uint32_t seed) const;

  const PostProcessConfig & config() const {return config_;}

private:
  PostProcessConfig config_;
  std::shared_ptr<const StateValidator> validator_;

  double maxDeviationFromPolyline(
    const ParameterizedPath & spline,
    const Path & polyline) const;
  bool isSplineCollisionFree(const ParameterizedPath & spline) const;
};

}  // namespace path_planner
