#pragma once

// 幾何経路探索の共通インターフェース（仕様書 第7.1節）。

#include <cstddef>
#include <cstdint>
#include <string>

#include "path_planner/types.hpp"

namespace path_planner
{
struct PlanResult
{
  Path path;
  FailureCode failure{FailureCode::kNone};
  std::string message;
  std::size_t iterations{0};

  explicit operator bool() const {return failure == FailureCode::kNone && !path.empty();}
};

class GeometricPlanner
{
public:
  virtual ~GeometricPlanner() = default;
  virtual PlanResult plan(
    const State & start, const State & goal,
    std::uint32_t seed) const = 0;
};

}  // namespace path_planner
