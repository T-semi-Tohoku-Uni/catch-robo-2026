// トルク制約付き TOPP-RA の単体テスト（仕様書 第11, 16.1節）。
// 速度律速、加速度律速、第1軸トルク律速、非実行可能経路（静的重力トルク超過）、適応格子を検証する。

#include <cmath>
#include <iostream>
#include <memory>

#include "path_planner/dynamics_model.hpp"
#include "path_planner/path_post_processor.hpp"
#include "path_planner/toppra_parameterizer.hpp"

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

std::shared_ptr<const RneaDynamicsModel> makeTestDynamics(
  double theta1_peak_torque,
  double base_yaw_inertia_zz = 0.40)
{
  RobotDynamicsConfig cfg;
  cfg.source = "toppra_test";
  cfg.measured_at = "2026-10-20";
  cfg.safety_factor = 1.0;
  cfg.gravity = {0.0, 0.0, -9.80665};
  for (std::size_t i = 0; i < kUrdfJointCount; ++i) {
    cfg.links[i].mass = 0.4;
    cfg.links[i].center_of_mass = {0.0, 0.0, 0.1};
    cfg.links[i].inertia(0, 0) = 0.01;
    cfg.links[i].inertia(1, 1) = 0.01;
    cfg.links[i].inertia(2, 2) = 0.01;
  }
  cfg.links[0].inertia(2, 2) = base_yaw_inertia_zz;
  cfg.theta1.continuous_torque = theta1_peak_torque;
  cfg.theta1.peak_torque = theta1_peak_torque;
  cfg.theta1.gear_ratio = 1.0;
  cfg.theta1.efficiency = 1.0;
  cfg.theta1.curve_in_motor_side = false;
  cfg.theta1.positive_speed_torque_curve = {
    {0.0, theta1_peak_torque},
    {3.0, theta1_peak_torque * 0.8},
    {6.0, theta1_peak_torque * 0.5}};
  cfg.theta1.negative_speed_torque_curve = cfg.theta1.positive_speed_torque_curve;
  return std::make_shared<RneaDynamicsModel>(cfg);
}

void testVelocityAndAccelerationAndTorqueLimiting()
{
  // 第1軸を 0 -> 1.0 rad へ動かす直線経路
  const ParameterizedPath path = ParameterizedPath::fromWaypoints({
    {0.0, 0.2, 0.2, 0.0},
    {1.0, 0.2, 0.2, 0.0}});

  auto wide_dyn = makeTestDynamics(50.0, 0.35);

  // 1. 速度律速ケース: 加速度・トルク上限は十分大きく、最大速度を 0.5 rad/s に制限
  ToppraConfig vel_cfg;
  vel_cfg.grid_points = 100;
  vel_cfg.max_velocity = {0.5, 2.0, 2.0, 2.0};
  vel_cfg.max_acceleration = {5.0, 5.0, 5.0, 5.0};
  ToppraParameterizer vel_topp(vel_cfg, wide_dyn);
  const ToppraResult vel_res = vel_topp.parameterize(path);
  EXPECT_TRUE(vel_res);
  EXPECT_TRUE(vel_res.duration > 2.0);  // L / v_max = 1.0 / 0.5 = 2.0s + 加減速時間
  EXPECT_NEAR(vel_res.minimum_velocity_margin, 0.0, 1e-3);

  // 2. 加速度律速ケース: 速度上限は大きく、最大加速度を 0.5 rad/s^2 に制限
  //    三角速度プロファイルの理論最短時間は T = 2 * sqrt(L / a_max) = 2 * sqrt(2) ~= 2.828s
  ToppraConfig acc_cfg = vel_cfg;
  acc_cfg.max_velocity = {5.0, 5.0, 5.0, 5.0};
  acc_cfg.max_acceleration = {0.5, 5.0, 5.0, 5.0};
  ToppraParameterizer acc_topp(acc_cfg, wide_dyn);
  const ToppraResult acc_res = acc_topp.parameterize(path);
  EXPECT_TRUE(acc_res);
  EXPECT_NEAR(acc_res.duration, 2.0 * std::sqrt(1.0 / 0.5), 0.05);
  EXPECT_NEAR(acc_res.minimum_acceleration_margin, 0.0, 1e-3);

  // 3. 第1軸トルク律速ケース: 運動学上限は広く、第1軸許容トルクを 0.15 Nm に絞る
  auto tight_dyn = makeTestDynamics(0.15, 0.35);
  ToppraConfig tau_cfg = acc_cfg;
  tau_cfg.max_velocity = {5.0, 5.0, 5.0, 5.0};
  tau_cfg.max_acceleration = {5.0, 5.0, 5.0, 5.0};
  ToppraParameterizer tau_topp(tau_cfg, tight_dyn);
  const ToppraResult tau_res = tau_topp.parameterize(path);
  EXPECT_TRUE(tau_res);
  EXPECT_TRUE(tau_res.duration > acc_res.duration);
  EXPECT_TRUE(tau_res.theta1_peak_torque <= 0.15 + 1e-6);
  EXPECT_TRUE(tau_res.minimum_torque_margin >= -1e-6);
  EXPECT_TRUE(tau_res.bottleneck.find("type=torque") != std::string::npos);
}

void testInfeasibleStaticTorquePathAndAdaptiveGrid()
{
  // 第1軸に傾いた重力場を模擬し、静止状態でも重力トルクが上限を超える非実行可能ケースを検証する。
  RobotDynamicsConfig cfg = makeTestDynamics(0.20, 0.30)->config();
  cfg.gravity = {0.0, -9.80665, 0.0};  // 横向き重力により base_yaw に定常重力トルクが発生
  cfg.links[1].mass = 3.0;
  cfg.links[1].center_of_mass = {0.0, 0.0, 0.3};
  auto infeasible_dyn = std::make_shared<RneaDynamicsModel>(cfg);

  const ParameterizedPath path = ParameterizedPath::fromWaypoints({
    {0.0, 1.2, 1.2, 0.0},
    {0.5, 1.2, 1.2, 0.0}});

  ToppraConfig topp_cfg;
  topp_cfg.grid_points = 80;
  topp_cfg.max_velocity = {2.0, 2.0, 2.0, 2.0};
  topp_cfg.max_acceleration = {4.0, 4.0, 4.0, 4.0};
  ToppraParameterizer topp(topp_cfg, infeasible_dyn);
  const ToppraResult res = topp.parameterize(path);
  EXPECT_FALSE(res);
  EXPECT_EQ(res.failure, FailureCode::kTimeParameterizationInfeasible);

  // 適応格子: 急カーブを持つ経路では topp_grid_points より多くの格子点へ自動細分化される。
  const ParameterizedPath curved = ParameterizedPath::fromWaypoints({
    {0.0, 0.1, 0.1, 0.0},
    {0.4, 1.2, 1.2, 0.2},
    {0.8, 0.1, 0.1, 0.0}});
  topp_cfg.grid_points = 50;
  topp_cfg.adaptive_grid_max_points = 200;
  ToppraParameterizer adaptive_topp(topp_cfg, makeTestDynamics(10.0));
  const auto grid = adaptive_topp.buildAdaptiveGrid(curved);
  EXPECT_TRUE(grid.size() > 50u && grid.size() <= 200u);
}
}  // namespace

int main()
{
  testVelocityAndAccelerationAndTorqueLimiting();
  testInfeasibleStaticTorquePathAndAdaptiveGrid();

  if (g_failures == 0) {
    std::cout << "[PASS] test_toppra_parameterizer" << std::endl;
    return 0;
  }
  std::cerr << "[FAIL] test_toppra_parameterizer failures=" << g_failures << std::endl;
  return 1;
}
