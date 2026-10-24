// jerk 制限・最終検証器・RMS トルク計算の単体テスト（仕様書 第12, 16.1節）。

#include <cmath>
#include <iostream>
#include <limits>
#include <memory>

#include "path_planner/collision_checker.hpp"
#include "path_planner/dynamics_model.hpp"
#include "path_planner/jerk_limiter.hpp"
#include "path_planner/path_post_processor.hpp"
#include "path_planner/toppra_parameterizer.hpp"
#include "path_planner/trajectory_pipeline.hpp"
#include "path_planner/trajectory_validator.hpp"

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
  double peak_torque = 12.0,
  double cont_torque = 6.0)
{
  RobotDynamicsConfig cfg;
  cfg.source = "validator_test";
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
  cfg.links[0].inertia(2, 2) = 0.25;
  cfg.theta1.continuous_torque = cont_torque;
  cfg.theta1.peak_torque = peak_torque;
  cfg.theta1.gear_ratio = 1.0;
  cfg.theta1.efficiency = 1.0;
  cfg.theta1.curve_in_motor_side = false;
  cfg.theta1.positive_speed_torque_curve = {
    {0.0, peak_torque}, {2.0, peak_torque * 0.8}, {5.0, peak_torque * 0.5}};
  cfg.theta1.negative_speed_torque_curve = cfg.theta1.positive_speed_torque_curve;
  return std::make_shared<RneaDynamicsModel>(cfg);
}

void testRmsTorqueAnalyticalComparison()
{
  // 1. 定トルク (tau_1(t) = 3.0 Nm for t in [0, 2.0]) -> RMS = 3.0 Nm
  TimedTrajectory constant_traj;
  constant_traj.points = {
    {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {3.0, 0, 0, 0}, 0.0},
    {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {3.0, 0, 0, 0}, 1.0},
    {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {3.0, 0, 0, 0}, 2.0}};
  EXPECT_NEAR(computeTheta1RmsTorque(constant_traj), 3.0, 1e-12);

  // 2. 休止時間込み周期 (動作 2s で 3.0 Nm、休止 2s で 0.0 Nm、周期 4s) -> RMS = 3.0 / sqrt(2)
  EXPECT_NEAR(computeTheta1RmsTorque(constant_traj, 4.0, 0.0), 3.0 / std::sqrt(2.0), 1e-12);

  // 3. 区分一定トルク: [0, 1] で 4.0 Nm, [1, 3] で 1.0 Nm (急峻ステップを微小遷移で表現)
  TimedTrajectory piecewise_traj;
  piecewise_traj.points = {
    {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {4.0, 0, 0, 0}, 0.0},
    {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {4.0, 0, 0, 0}, 1.0},
    {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {1.0, 0, 0, 0}, 1.0 + 1e-9},
    {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {1.0, 0, 0, 0}, 3.0}};
  const double expected_pw = std::sqrt((16.0 * 1.0 + 1.0 * 2.0) / 3.0);
  EXPECT_NEAR(computeTheta1RmsTorque(piecewise_traj), expected_pw, 1e-6);
}

void testJerkLimiterAndValidatorEachFailureCode()
{
  auto dyn = makeTestDynamics(10.0, 5.0);
  auto scene = std::make_shared<CollisionScene>();
  KinematicChain chain = defaultKinematicChain();
  auto checker = std::make_shared<CollisionChecker>(
    chain, defaultRobotCollisionModel(chain), scene, 0.002);
  JointBounds bounds{{-2.5, -1.5, -1.5, -2.5}, {2.5, 1.5, 1.5, 2.5}};
  auto state_val = std::make_shared<ConfigurationValidator>(bounds, checker, 0.02);

  const State start{0.0, 0.2, 0.3, 0.0};
  const State goal{0.6, 0.4, 0.5, 0.1};
  const ParameterizedPath path = ParameterizedPath::fromWaypoints({start, goal});

  ToppraConfig topp_cfg;
  topp_cfg.grid_points = 60;
  topp_cfg.max_velocity = {1.5, 1.5, 1.5, 1.5};
  topp_cfg.max_acceleration = {3.0, 3.0, 3.0, 3.0};
  ToppraParameterizer topp(topp_cfg, dyn);
  const ToppraResult topp_res = topp.parameterize(path);
  EXPECT_TRUE(topp_res);

  JerkLimiterConfig jerk_cfg;
  jerk_cfg.max_velocity = topp_cfg.max_velocity;
  jerk_cfg.max_acceleration = topp_cfg.max_acceleration;
  jerk_cfg.max_jerk = {15.0, 15.0, 15.0, 15.0};
  jerk_cfg.sample_dt = 0.005;
  JerkLimiter limiter(jerk_cfg, dyn);
  const JerkLimitResult jerk_res = limiter.limit(path, topp_res.profile);
  EXPECT_TRUE(jerk_res);

  ValidatorConfig val_cfg;
  val_cfg.bounds = bounds;
  val_cfg.max_velocity = jerk_cfg.max_velocity;
  val_cfg.max_acceleration = jerk_cfg.max_acceleration;
  val_cfg.max_jerk = jerk_cfg.max_jerk;
  val_cfg.validation_dt = 0.005;
  TrajectoryValidator validator(val_cfg, state_val, dyn);

  // 正常軌道は kNone で合格する
  const ValidationReport ok_rep = validator.validate(jerk_res.trajectory, start, goal);
  EXPECT_TRUE(ok_rep);
  EXPECT_EQ(ok_rep.failure, FailureCode::kNone);

  // ---- 各制約を1つずつ違反させ、正しい FailureCode を返すことを確認する（仕様書 第16.1節） ----

  // 1. kInvalidInput: 点数が 1 点のみ
  {
    TimedTrajectory bad = jerk_res.trajectory;
    bad.points.resize(1);
    EXPECT_EQ(validator.validate(bad, start, goal).failure, FailureCode::kInvalidInput);
  }

  // 2. kMissingDynamics: 動力学モデル無しで require_dynamics = true
  {
    TrajectoryValidator no_dyn_val(val_cfg, state_val, nullptr);
    EXPECT_EQ(
      no_dyn_val.validate(jerk_res.trajectory, start, goal).failure,
      FailureCode::kMissingDynamics);
  }

  // 3. kNumericalError: 非単調時刻および NaN
  {
    TimedTrajectory bad_time = jerk_res.trajectory;
    bad_time.points[2].time = bad_time.points[1].time;
    EXPECT_EQ(validator.validate(bad_time, start, goal).failure, FailureCode::kNumericalError);

    TimedTrajectory bad_nan = jerk_res.trajectory;
    bad_nan.points[1].velocity[0] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(validator.validate(bad_nan, start, goal).failure, FailureCode::kNumericalError);
  }

  // 4. kJointLimitViolation: 速度上限超過 / jerk 上限超過 / 終端不一致
  {
    TimedTrajectory bad_vel = jerk_res.trajectory;
    bad_vel.points[3].velocity[0] = val_cfg.max_velocity[0] + 0.5;
    EXPECT_EQ(validator.validate(bad_vel, start, goal).failure, FailureCode::kJointLimitViolation);

    TimedTrajectory bad_jerk = jerk_res.trajectory;
    bad_jerk.points[3].jerk[0] = val_cfg.max_jerk[0] + 5.0;
    EXPECT_EQ(validator.validate(bad_jerk, start, goal).failure, FailureCode::kJointLimitViolation);

    const State wrong_goal{0.9, 0.4, 0.5, 0.1};
    EXPECT_EQ(
      validator.validate(jerk_res.trajectory, start, wrong_goal).failure,
      FailureCode::kJointLimitViolation);
  }

  // 5. kCollisionDetected: 経路途中に障害物を置いた場合
  {
    auto obs_scene = std::make_shared<CollisionScene>();
    const auto mid_pose = forwardKinematics(
      chain, toUrdfJointPositions(jerk_res.trajectory.points[jerk_res.trajectory.points.size() / 2].position))[5];
    Obstacle obs;
    obs.name = "mid_obstacle";
    obs.shape.type = ShapeType::kSphere;
    obs.shape.radius = 0.10;
    obs.shape.pose = mid_pose;
    obs_scene->obstacles.push_back(obs);
    auto obs_checker = std::make_shared<CollisionChecker>(
      chain, defaultRobotCollisionModel(chain), obs_scene, 0.002);
    auto obs_state_val = std::make_shared<ConfigurationValidator>(bounds, obs_checker, 0.02);
    TrajectoryValidator obs_validator(val_cfg, obs_state_val, dyn);
    EXPECT_EQ(
      obs_validator.validate(jerk_res.trajectory, start, goal).failure,
      FailureCode::kCollisionDetected);
  }

  // 6. kTorqueLimitViolation: 瞬時許容トルク上限を超過させた場合
  {
    auto low_peak_dyn = makeTestDynamics(0.01, 5.0);
    TrajectoryValidator low_peak_val(val_cfg, state_val, low_peak_dyn);
    const ValidationReport rep = low_peak_val.validate(jerk_res.trajectory, start, goal);
    EXPECT_EQ(rep.failure, FailureCode::kTorqueLimitViolation);
    EXPECT_TRUE(rep.message.find("torque limit violated") != std::string::npos);
  }

  // 7. kThermalLimitViolation: 瞬時トルクは許容内だが連続定格(RMS)を超過させた場合
  {
    auto low_rms_dyn = makeTestDynamics(10.0, 0.001);
    TrajectoryValidator low_rms_val(val_cfg, state_val, low_rms_dyn);
    const ValidationReport rep = low_rms_val.validate(jerk_res.trajectory, start, goal);
    EXPECT_EQ(rep.failure, FailureCode::kThermalLimitViolation);
  }
}
class FiniteStateValidator : public StateValidator {
public:
  bool isStateValid(const State &q) const override {return isFiniteState(q);}
  bool isMotionValid(const State &a,const State &b) const override {return isFiniteState(a)&&isFiniteState(b);}
  double clearance(const State &) const override {return 1;}
};
void testPositionSamplesCannotForgeDerivatives() {
  ValidatorConfig cfg;cfg.require_dynamics=false;
  auto state=std::make_shared<FiniteStateValidator>();
  TimedTrajectory forged;
  for(double position:{0.0,.1,0.0}) {
    DynamicState p;p.position={position,0,0,0};p.velocity=p.acceleration=p.jerk=State(kDof,0);
    p.time=forged.points.size()*.1;forged.points.push_back(p);
  }
  const auto acceleration=TrajectoryValidator(cfg,state).validate(forged,forged.points.front().position,forged.points.back().position);
  EXPECT_FALSE(acceleration);EXPECT_TRUE(acceleration.message.find("Position slopes")!=std::string::npos);
  // Boundary ghost samples must reject rapid two-point commands with forged zero rates.
  forged.points.clear();
  for(double position:{0.0,.1}) {
    DynamicState p;p.position={position,0,0,0};p.velocity=p.acceleration=p.jerk=State(kDof,0);
    p.time=forged.points.size()*.05;forged.points.push_back(p);
  }
  cfg.max_velocity=State(kDof,2);cfg.max_acceleration=State(kDof,5);cfg.max_jerk=State(kDof,100);
  EXPECT_FALSE(TrajectoryValidator(cfg,state).validate(forged,forged.points.front().position,forged.points.back().position));
  cfg.max_acceleration=State(kDof,100);cfg.max_jerk=State(kDof,100);
  EXPECT_FALSE(TrajectoryValidator(cfg,state).validate(forged,forged.points.front().position,forged.points.back().position));
  cfg.max_acceleration=State(kDof,5);
  forged.points.back().time=2.0;
  EXPECT_TRUE(TrajectoryValidator(cfg,state).validate(forged,forged.points.front().position,forged.points.back().position));
  cfg.max_acceleration=State(kDof,100);cfg.max_jerk=State(kDof,1);forged.points.clear();
  for(double position:{0.0,.01,.02,.05}) {
    DynamicState p;p.position={position,0,0,0};p.velocity=p.acceleration=p.jerk=State(kDof,0);
    p.time=forged.points.size()*.1;forged.points.push_back(p);
  }
  const auto jerk=TrajectoryValidator(cfg,state).validate(forged,forged.points.front().position,forged.points.back().position);
  EXPECT_FALSE(jerk);EXPECT_TRUE(jerk.message.find("Position slopes violate jerk")!=std::string::npos);
}

}  // namespace

int main()
{
  testPositionSamplesCannotForgeDerivatives();
  testRmsTorqueAnalyticalComparison();
  testJerkLimiterAndValidatorEachFailureCode();

  if (g_failures == 0) {
    std::cout << "[PASS] test_trajectory_validator" << std::endl;
    return 0;
  }
  std::cerr << "[FAIL] test_trajectory_validator failures=" << g_failures << std::endl;
  return 1;
}
