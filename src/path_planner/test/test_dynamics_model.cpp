// 動力学モデル（RNEA・解析解比較・ペイロード・速度―トルク曲線）の単体テスト（仕様書 第10, 11.2, 16.1節）。

#include <cmath>
#include <iostream>

#include "path_planner/dynamics_model.hpp"

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

using namespace path_planner;

RobotDynamicsConfig makeBaseTestDynamics()
{
  RobotDynamicsConfig cfg;
  cfg.source = "unit_test";
  cfg.measured_at = "2026-10-20";
  cfg.safety_factor = 0.8;
  cfg.gravity = {0.0, 0.0, -9.80665};
  for (auto & link : cfg.links) {
    link.mass = 1e-9;
    link.center_of_mass = {0.0, 0.0, 0.0};
    link.inertia = Matrix3::zero();
  }
  cfg.theta1.continuous_torque = 4.0;
  cfg.theta1.peak_torque = 8.0;
  cfg.theta1.gear_ratio = 10.0;
  cfg.theta1.efficiency = 0.9;
  cfg.theta1.curve_in_motor_side = false;
  cfg.theta1.positive_speed_torque_curve = {{0.0, 8.0}, {1.0, 6.0}, {2.0, 2.0}};
  cfg.theta1.negative_speed_torque_curve = {{0.0, 7.0}, {1.0, 5.0}, {2.0, 1.5}};
  return cfg;
}

void testOneAndTwoLinkAnalyticalComparison()
{
  RobotDynamicsConfig cfg = makeBaseTestDynamics();
  const double g = 9.80665;
  const double l1 = 0.480;
  const double m1 = 2.5;
  const double lc1 = 0.240;
  const double i1_yy = 0.045;
  const double m2 = 1.8;
  const double lc2 = 0.200;
  const double i2_yy = 0.025;

  // link[1] = upper_arm_link, link[2] = forearm_link
  cfg.links[1].mass = m1;
  cfg.links[1].center_of_mass = {0.0, 0.0, lc1};
  cfg.links[1].inertia(1, 1) = i1_yy;

  cfg.links[2].mass = m2;
  cfg.links[2].center_of_mass = {0.0, 0.0, lc2};
  cfg.links[2].inertia(1, 1) = i2_yy;

  RneaDynamicsModel model(cfg);

  const double th2 = 0.4;
  const double th3 = 1.1;
  const double dth2 = -0.5;
  const double dth3 = 0.8;
  const double ddth2 = 1.2;
  const double ddth3 = -0.7;

  const State q{0.0, th2, th3, 0.0};
  const State qd{0.0, dth2, dth3, 0.0};
  const State qdd{0.0, ddth2, ddth3, 0.0};
  const State tau = model.inverseDynamics(q, qd, qdd);

  // 2リンク平行リンク機構の厳密なラグランジュ運動方程式による解析解
  const double m22 = i1_yy + m1 * lc1 * lc1 + m2 * l1 * l1;
  const double m33 = i2_yy + m2 * lc2 * lc2;
  const double m23 = m2 * l1 * lc2 * std::cos(th2 - th3);
  const double c2 = m2 * l1 * lc2 * std::sin(th2 - th3) * (dth3 * dth3);
  const double c3 = -m2 * l1 * lc2 * std::sin(th2 - th3) * (dth2 * dth2);
  const double g2 = -(m1 * lc1 + m2 * l1) * g * std::sin(th2);
  const double g3 = -m2 * g * lc2 * std::sin(th3);

  const double expected_tau2 = m22 * ddth2 + m23 * ddth3 + c2 + g2;
  const double expected_tau3 = m23 * ddth2 + m33 * ddth3 + c3 + g3;

  EXPECT_NEAR(tau[1], expected_tau2, 1e-7);
  EXPECT_NEAR(tau[2], expected_tau3, 1e-7);
  EXPECT_NEAR(model.effectiveInertia(1, q), m22, 1e-7);
  EXPECT_NEAR(model.effectiveInertia(2, q), m33, 1e-7);
}

void testPayloadAndTheta1EffectiveInertia()
{
  RobotDynamicsConfig cfg = makeBaseTestDynamics();
  for (std::size_t i = 0; i < kUrdfJointCount; ++i) {
    cfg.links[i].mass = 0.5;
    cfg.links[i].center_of_mass = {0.0, 0.0, 0.1};
    cfg.links[i].inertia(0, 0) = 0.005;
    cfg.links[i].inertia(1, 1) = 0.005;
    cfg.links[i].inertia(2, 2) = 0.005;
  }
  RneaDynamicsModel no_payload_model(cfg);

  // 第1軸(鉛直旋回軸)の有効慣性 M(0,0) は、アームを畳んだ姿勢より水平に伸ばした姿勢で大きくなる（仕様書 第19節）。
  const State q_folded{0.0, 0.05, 0.05, 0.0};
  const State q_extended{0.0, kPi * 0.5, kPi * 0.5, 0.0};
  const double i_folded = no_payload_model.effectiveInertia(0, q_folded);
  const double i_extended = no_payload_model.effectiveInertia(0, q_extended);
  EXPECT_TRUE(i_extended > i_folded * 2.0);

  // ペイロードを追加すると、伸展姿勢での重力保持トルクと第1軸慣性がともに増加し、バージョンが進む。
  Payload heavy;
  heavy.mass = 1.5;
  heavy.center_of_mass = {0.0, 0.0, -0.05};
  heavy.inertia = {0.01, 0.0, 0.0, 0.01, 0.0, 0.01};
  RneaDynamicsModel loaded_model = no_payload_model.withPayload(heavy);
  EXPECT_TRUE(loaded_model.version() == no_payload_model.version() + 1);

  const double i_ext_loaded = loaded_model.effectiveInertia(0, q_extended);
  EXPECT_TRUE(i_ext_loaded > i_extended);

  const State zero(kDof, 0.0);
  const State tau_unloaded = no_payload_model.inverseDynamics(q_extended, zero, zero);
  const State tau_loaded = loaded_model.inverseDynamics(q_extended, zero, zero);
  EXPECT_TRUE(std::abs(tau_loaded[2]) > std::abs(tau_unloaded[2]));
}

void testSpeedTorqueCurveInterpolationAndDirection()
{
  RobotDynamicsConfig cfg = makeBaseTestDynamics();
  cfg.safety_factor = 0.8;
  // positive: (0, 8), (1, 6), (2, 2) -> safety_factor 0.8 により (0, 6.4), (1, 4.8), (2, 1.6)
  // negative: (0, 7), (1, 5), (2, 1.5) -> safety_factor 0.8 により (0, 5.6), (1, 4.0), (2, 1.2)
  RneaDynamicsModel model(cfg);

  // 節点 (speed = 0.0)
  const TorqueBounds b0 = model.torqueBounds(0, 0.0);
  EXPECT_TRUE(b0.feasible);
  EXPECT_NEAR(b0.max_torque, 6.4, 1e-9);
  EXPECT_NEAR(b0.min_torque, -5.6, 1e-9);

  // 正転補間 (speed = +0.5 -> rated pos=7.0, neg=6.0 -> allowed +5.6, -4.8)
  const TorqueBounds b_pos = model.torqueBounds(0, 0.5);
  EXPECT_TRUE(b_pos.feasible);
  EXPECT_NEAR(b_pos.max_torque, 5.6, 1e-9);
  EXPECT_NEAR(b_pos.min_torque, -4.8, 1e-9);

  // 逆転補間 (speed = -1.5 -> rated pos=4.0, neg=3.25 -> allowed +3.2, -2.6)
  const TorqueBounds b_neg = model.torqueBounds(0, -1.5);
  EXPECT_TRUE(b_neg.feasible);
  EXPECT_NEAR(b_neg.max_torque, 3.2, 1e-9);
  EXPECT_NEAR(b_neg.min_torque, -2.6, 1e-9);

  // 範囲外速度 (|speed| = 2.5 > 2.0) は外挿せず feasible = false（仕様書 第11.2節）
  const TorqueBounds b_out_pos = model.torqueBounds(0, 2.5);
  const TorqueBounds b_out_neg = model.torqueBounds(0, -2.1);
  EXPECT_FALSE(b_out_pos.feasible);
  EXPECT_FALSE(b_out_neg.feasible);
}
}  // namespace

int main()
{
  testOneAndTwoLinkAnalyticalComparison();
  testPayloadAndTheta1EffectiveInertia();
  testSpeedTorqueCurveInterpolationAndDirection();

  if (g_failures == 0) {
    std::cout << "[PASS] test_dynamics_model" << std::endl;
    return 0;
  }
  std::cerr << "[FAIL] test_dynamics_model failures=" << g_failures << std::endl;
  return 1;
}
