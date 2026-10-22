// 経路後処理（shortcut および C2 スプライン平滑化）の単体テスト（仕様書 第8, 16.1節）。

#include <cmath>
#include <iostream>
#include <memory>

#include "path_planner/collision_checker.hpp"
#include "path_planner/path_post_processor.hpp"
#include "path_planner/robot_model.hpp"
#include "path_planner/trajectory_pipeline.hpp"

namespace
{
int g_failures = 0;

void check(bool cond, const char * expr, int line)
{
  if (!cond) {
    std::cerr << "[FAIL] line " << line << ": " << expr << std::endl;
    ++g_failures;
  }
}

#define EXPECT_TRUE(expr) check(static_cast<bool>(expr), #expr, __LINE__)
#define EXPECT_FALSE(expr) check(!static_cast<bool>(expr), "!(" #expr ")", __LINE__)
#define EXPECT_NEAR(a, b, tol) check(std::abs((a) - (b)) <= (tol), #a " ~= " #b, __LINE__)
#define EXPECT_EQ(a, b) check((a) == (b), #a " == " #b, __LINE__)

using namespace path_planner;

// カスタム StateValidator: 第0関節と第1関節の空間で中心 (0.5, 0.5) 半径 0.25 の円領域を禁止する。
class CircleObstacleValidator : public StateValidator
{
public:
  bool isStateValid(const State & q) const override
  {
    if (!isFiniteState(q)) {
      return false;
    }
    for (double v : q) {
      if (std::abs(v) > 2.5) {
        return false;
      }
    }
    const double dx = q[0] - 0.5;
    const double dy = q[1] - 0.5;
    return std::sqrt(dx * dx + dy * dy) >= 0.25;
  }

  bool isMotionValid(const State & a, const State & b) const override
  {
    if (!isStateValid(a) || !isStateValid(b)) {
      return false;
    }
    constexpr int kSteps = 100;
    for (int i = 1; i < kSteps; ++i) {
      const double r = static_cast<double>(i) / static_cast<double>(kSteps);
      State q(kDof);
      for (std::size_t j = 0; j < kDof; ++j) {
        q[j] = a[j] + r * (b[j] - a[j]);
      }
      if (!isStateValid(q)) {
        return false;
      }
    }
    return true;
  }

  double clearance(const State & q) const override
  {
    const double dx = q[0] - 0.5;
    const double dy = q[1] - 0.5;
    return std::sqrt(dx * dx + dy * dy) - 0.25;
  }
};

void testShortcutShorteningAndObstacleRejection()
{
  auto validator = std::make_shared<CircleObstacleValidator>();
  PostProcessConfig cfg;
  cfg.shortcut_iterations = 150;
  cfg.max_path_deviation = 0.01;
  cfg.collision_check_resolution = 0.01;
  PathPostProcessor pp(cfg, validator);

  // 1. 障害物がない領域での冗長なジグザグ経路は 2 点に短縮され、始終点が保存される。
  const Path zigzag{
    {-0.8, -0.8, 0.0, 0.0},
    {-0.6, -0.2, 0.1, -0.1},
    {-0.4, -0.7, -0.1, 0.1},
    {-0.2, -0.3, 0.0, 0.0}};
  const Path shortened = pp.shortcut(zigzag, 42);
  EXPECT_EQ(shortened.size(), 2u);
  for (std::size_t j = 0; j < kDof; ++j) {
    EXPECT_NEAR(shortened.front()[j], zigzag.front()[j], 1e-12);
    EXPECT_NEAR(shortened.back()[j], zigzag.back()[j], 1e-12);
  }

  // 2. 直線ショートカットが (0.5, 0.5) の障害物を横切る経路では、衝突ショートカットが拒否される。
  const Path detour{
    {0.0, 0.0, 0.0, 0.0},
    {0.0, 1.0, 0.0, 0.0},
    {1.0, 1.0, 0.0, 0.0}};
  EXPECT_FALSE(validator->isMotionValid(detour.front(), detour.back()));
  const Path kept = pp.shortcut(detour, 42);
  EXPECT_TRUE(kept.size() >= 3u);
  for (std::size_t i = 0; i + 1 < kept.size(); ++i) {
    EXPECT_TRUE(validator->isMotionValid(kept[i], kept[i + 1]));
  }
}

void testSplineC2ContinuityAndMaxDeviationAndRecheck()
{
  auto validator = std::make_shared<CircleObstacleValidator>();
  PostProcessConfig cfg;
  cfg.shortcut_iterations = 80;
  cfg.max_path_deviation = 0.015;
  cfg.collision_check_resolution = 0.01;
  cfg.max_smoothing_retries = 6;
  PathPostProcessor pp(cfg, validator);

  // 障害物を迂回する急な折れ線。初回スプラインは角で大きく膨らむが、
  // 節点細分化の再試行により max_path_deviation 以内かつ非衝突な C2 スプラインが得られる。
  const Path sharp_detour{
    {0.0, 0.0, 0.0, 0.0},
    {0.0, 1.0, 0.2, -0.1},
    {1.0, 1.0, 0.4, 0.1}};
  const PostProcessResult res = pp.process(sharp_detour, 99);
  EXPECT_TRUE(res);
  EXPECT_TRUE(res.max_deviation <= cfg.max_path_deviation);

  // C0, C1, C2 連続性を全内部節点で確認する。
  const auto & knots = res.spline.knots();
  for (std::size_t k = 1; k + 1 < knots.size(); ++k) {
    const double s_left = knots[k] - 1e-9;
    const double s_right = knots[k] + 1e-9;
    const State q_l = res.spline.position(s_left);
    const State q_r = res.spline.position(s_right);
    const State dq_l = res.spline.firstDerivative(s_left);
    const State dq_r = res.spline.firstDerivative(s_right);
    const State d2q_l = res.spline.secondDerivative(s_left);
    const State d2q_r = res.spline.secondDerivative(s_right);
    for (std::size_t j = 0; j < kDof; ++j) {
      EXPECT_NEAR(q_l[j], q_r[j], 1e-5);
      EXPECT_NEAR(dq_l[j], dq_r[j], 1e-5);
      EXPECT_NEAR(d2q_l[j], d2q_r[j], 1e-4);
    }
  }

  // スプライン全点が非衝突であること
  for (int i = 0; i <= 200; ++i) {
    const double s = static_cast<double>(i) / 200.0;
    EXPECT_TRUE(validator->isStateValid(res.spline.position(s)));
  }
}
}  // namespace

int main()
{
  testShortcutShorteningAndObstacleRejection();
  testSplineC2ContinuityAndMaxDeviationAndRecheck();

  if (g_failures == 0) {
    std::cout << "[PASS] test_path_post_processor" << std::endl;
    return 0;
  }
  std::cerr << "[FAIL] test_path_post_processor failures=" << g_failures << std::endl;
  return 1;
}
