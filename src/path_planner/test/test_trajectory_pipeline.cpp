// 軌道生成パイプラインおよび ROS 2 アダプターの統合テスト（仕様書 第16.2, 17節）。
//
// 統合テスト 1～8:
//  1. 同一始終点で安全な静止軌道を返す。
//  2. 第1軸を大きく動かす候補より、アームを畳んで旋回する候補を選ぶ。
//  3. 第1軸上限を下げると軌道時間が増え、ピークトルクが上限内に残る。
//  4. 静的重力トルクだけで上限超過する姿勢を、低速化で誤って成功させない。
//  5. jerk処理前は合格、処理後にトルク超過するケースを最終検証で拒否する。
//  6. ペイロード変更後に旧軌道が無効化される。
//  7. 衝突シーン変更中に生成された結果が配信されない。
//  8. 動力学値未設定で hardware_output_enabled=true なら起動失敗する。

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "path_planner/dynamics_model.hpp"
#include "path_planner/trajectory_node.hpp"
#include "path_planner/trajectory_pipeline.hpp"
#include "path_planner/yaml_lite.hpp"

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

TrajectoryPipelineConfig makeTestPipelineConfig()
{
  TrajectoryPipelineConfig cfg;
  cfg.hardware_output_enabled = false;
  cfg.candidate_count = 4;
  cfg.base_seed = 42;
  cfg.planning_time_limit = 2.0;
  cfg.distance_weights = {2.0, 1.0, 1.0, 1.0};
  cfg.joint_bounds = {{-2.5, -1.6, -1.6, -2.5}, {2.5, 1.6, 1.6, 2.5}};
  cfg.shortcut_iterations = 80;
  cfg.max_path_deviation = 0.02;
  cfg.collision_check_resolution = 0.02;
  cfg.collision_margin = 0.002;
  cfg.topp_grid_points = 60;
  cfg.adaptive_grid_max_points = 200;
  cfg.validation_dt = 0.005;
  cfg.max_velocity = {1.5, 1.5, 1.5, 1.5};
  cfg.max_acceleration = {3.0, 3.0, 3.0, 3.0};
  cfg.max_jerk = {25.0, 25.0, 25.0, 25.0};
  return cfg;
}

RobotDynamicsConfig makeTestDynamicsConfig(double theta1_peak = 8.0, double theta1_cont = 5.0)
{
  RobotDynamicsConfig dyn;
  dyn.source = "integration_test";
  dyn.measured_at = "2026-10-20";
  dyn.safety_factor = 0.9;
  dyn.gravity = {0.0, 0.0, -9.80665};
  for (std::size_t i = 0; i < kUrdfJointCount; ++i) {
    dyn.links[i].mass = 0.6;
    dyn.links[i].center_of_mass = {0.0, 0.0, 0.15};
    dyn.links[i].inertia(0, 0) = 0.015;
    dyn.links[i].inertia(1, 1) = 0.015;
    dyn.links[i].inertia(2, 2) = 0.015;
  }
  dyn.links[0].inertia(2, 2) = 0.25;
  dyn.theta1.continuous_torque = theta1_cont;
  dyn.theta1.peak_torque = theta1_peak;
  dyn.theta1.gear_ratio = 1.0;
  dyn.theta1.efficiency = 1.0;
  dyn.theta1.curve_in_motor_side = false;
  dyn.theta1.positive_speed_torque_curve = {
    {0.0, theta1_peak}, {2.0, theta1_peak * 0.85}, {5.0, theta1_peak * 0.5}};
  dyn.theta1.negative_speed_torque_curve = dyn.theta1.positive_speed_torque_curve;
  return dyn;
}

// 統合テスト 1: 同一始終点で安全な静止軌道を返す + seed 固定再現性
void testStationaryAndSeedReproducibility()
{
  TrajectoryPipeline pipeline(makeTestPipelineConfig(), makeTestDynamicsConfig());
  const State q{0.2, 0.3, 0.4, -0.1};
  const PipelineResult r_stat = pipeline.execute(q, q, 42);
  EXPECT_TRUE(r_stat);
  EXPECT_EQ(r_stat.failure, FailureCode::kNone);
  EXPECT_TRUE(r_stat.duration > 0.0);
  for (const auto & pt : r_stat.trajectory.points) {
    for (std::size_t j = 0; j < kDof; ++j) {
      EXPECT_NEAR(pt.position[j], q[j], 1e-12);
      EXPECT_NEAR(pt.velocity[j], 0.0, 1e-12);
      EXPECT_NEAR(pt.acceleration[j], 0.0, 1e-12);
    }
  }

  // seed 固定時の完全再現性（仕様書 第17節）
  const State goal{0.7, 0.5, 0.6, 0.2};
  const PipelineResult r1 = pipeline.execute(q, goal, 100);
  const PipelineResult r2 = pipeline.execute(q, goal, 100);
  EXPECT_TRUE(r1);
  EXPECT_TRUE(r2);
  EXPECT_EQ(r1.selected_candidate, r2.selected_candidate);
  EXPECT_NEAR(r1.duration, r2.duration, 1e-12);
  EXPECT_EQ(r1.trajectory.points.size(), r2.trajectory.points.size());
}

// 統合テスト 2: 第1軸を大きく動かす候補（または伸ばしたまま旋回する候補）より、アームを畳んで旋回する候補を選ぶ
void testFoldedTurnCandidateSelection()
{
  TrajectoryPipelineConfig cfg = makeTestPipelineConfig();
  cfg.shortcut_iterations = 0;  // 候補の幾何形状をそのまま比較する
  cfg.cost_weights.path_length = 0.2;
  cfg.cost_weights.theta1_motion = 1.0;
  cfg.cost_weights.theta1_inertia = 8.0;
  cfg.cost_weights.theta1_torque = 8.0;
  TrajectoryPipeline pipeline(cfg, makeTestDynamicsConfig());

  const State start{-0.8, 1.1, 1.1, 0.0};
  const State goal{0.8, 1.1, 1.1, 0.0};

  // 候補0: アームを水平近くに伸ばしたまま第1軸を大きく往復・旋回する経路
  const Path extended_swing{
    start,
    {-1.1, 1.2, 1.2, 0.0},
    {0.0, 1.2, 1.2, 0.0},
    goal};

  // 候補1: まずアームを鉛直近くへ畳み、慣性 M(0,0) が小さい状態で第1軸を旋回し、終点で伸ばす経路
  const Path folded_turn{
    start,
    {-0.8, 0.15, 0.15, 0.0},
    {0.8, 0.15, 0.15, 0.0},
    goal};

  const PipelineResult res =
    pipeline.executeFromCandidates({extended_swing, folded_turn}, start, goal, 42);
  EXPECT_TRUE(res);
  EXPECT_EQ(res.selected_candidate, 1u);
}

// 統合テスト 3: 第1軸上限を下げると軌道時間が増え、ピークトルクが上限内に残る
void testLoweringTheta1TorqueLimitIncreasesDuration()
{
  const TrajectoryPipelineConfig cfg = makeTestPipelineConfig();
  TrajectoryPipeline high_pipe(cfg, makeTestDynamicsConfig(6.0, 5.0));
  TrajectoryPipeline low_pipe(cfg, makeTestDynamicsConfig(0.25, 0.25));

  const State start{0.0, 0.3, 0.3, 0.0};
  const State goal{0.9, 0.3, 0.3, 0.0};

  const PipelineResult r_high = high_pipe.execute(start, goal, 42);
  const PipelineResult r_low = low_pipe.execute(start, goal, 42);
  EXPECT_TRUE(r_high);
  EXPECT_TRUE(r_low);
  EXPECT_TRUE(r_low.duration > r_high.duration);
  // safety_factor = 0.9 なので low_pipe の許容上限は 0.25 * 0.9 = 0.225 Nm
  EXPECT_TRUE(r_low.theta1_peak_torque <= 0.225 + 1e-6);
  EXPECT_TRUE(r_low.minimum_torque_margin >= -1e-6);
}

// 統合テスト 4: 静的重力トルクだけで上限超過する姿勢を、低速化で誤って成功させない
void testStaticGravityTorqueOverloadRejected()
{
  RobotDynamicsConfig dyn = makeTestDynamicsConfig(0.20, 0.20);
  dyn.gravity = {0.0, -9.80665, 0.0};  // 横向き重力成分により第1軸に静的重力トルクが発生
  dyn.links[1].mass = 3.0;
  dyn.links[1].center_of_mass = {0.0, 0.0, 0.3};

  TrajectoryPipeline pipeline(makeTestPipelineConfig(), dyn);
  const State start{0.0, 1.1, 1.1, 0.0};
  const State goal{0.4, 1.1, 1.1, 0.0};
  const PipelineResult res = pipeline.execute(start, goal, 42);
  EXPECT_FALSE(res);
  EXPECT_EQ(res.failure, FailureCode::kTimeParameterizationInfeasible);
}

// 統合テスト 5: jerk 処理前は合格、処理後にトルク超過するケースを最終検証で拒否する
void testPostJerkTorqueViolationRejectedByFinalValidator()
{
  TrajectoryPipelineConfig cfg = makeTestPipelineConfig();
  // TOPP-RA と JerkLimiter ではトルクチェックを無効化し、最終検証器だけでトルク違反を捕捉させる。
  cfg.enforce_torque_in_toppra = false;
  cfg.enforce_torque_in_jerk_limiter = false;
  TrajectoryPipeline pipeline(cfg, makeTestDynamicsConfig(0.05, 5.0));

  const State start{0.0, 0.3, 0.3, 0.0};
  const State goal{0.8, 0.3, 0.3, 0.0};
  const PipelineResult res = pipeline.execute(start, goal, 42);
  EXPECT_FALSE(res);
  EXPECT_EQ(res.failure, FailureCode::kTorqueLimitViolation);
  EXPECT_EQ(res.failure_stage, "final_validation");
}

// 統合テスト 6 & 7: ペイロード変更後の旧軌道無効化、および衝突シーン変更中の結果破棄・非配信
void testPayloadInvalidationAndSceneChangeDiscard()
{
  auto pipeline = std::make_shared<TrajectoryPipeline>(
    makeTestPipelineConfig(), makeTestDynamicsConfig());
  TrajectoryNode node(pipeline);

  const State start{0.0, 0.2, 0.3, 0.0};
  const State goal{0.5, 0.4, 0.5, 0.1};

  const PipelineResult valid_res = pipeline->execute(start, goal, 42);
  EXPECT_TRUE(valid_res);
  EXPECT_TRUE(pipeline->isTrajectoryValidForExecution(valid_res, start));

  // 統合テスト 6: ペイロード変更後に旧軌道が無効化される
  Payload new_payload;
  new_payload.mass = 0.8;
  new_payload.center_of_mass = {0.0, 0.0, -0.04};
  new_payload.inertia = {0.002, 0.0, 0.0, 0.002, 0.0, 0.002};
  pipeline->updatePayload(new_payload);

  std::string reason;
  EXPECT_FALSE(pipeline->isTrajectoryValidForExecution(valid_res, start, &reason));
  EXPECT_TRUE(reason.find("payload") != std::string::npos);

  // 正常なサービス呼び出しで一度配信されたことを確認する
  GenerateTrajectoryRequest req;
  req.start_joint_names = {"theta2", "theta1", "theta4", "theta3"};  // 順序入れ替えもテスト
  req.start_positions = {start[1], start[0], start[3], start[2]};
  req.goal_joint_names = {"theta1", "theta2", "theta3", "theta4"};
  req.goal_positions = goal;
  req.seed = 42;
  const GenerateTrajectoryResponse ok_resp = node.handleGenerateTrajectory(req);
  EXPECT_TRUE(ok_resp.success);
  EXPECT_TRUE(node.hasPublishedTimedTrajectory());
  EXPECT_EQ(node.publishedTimedTrajectoryCount(), 1u);
  for (const auto & pt : ok_resp.trajectory.points) {
    EXPECT_EQ(pt.positions.size(), kDof);
    EXPECT_EQ(pt.velocities.size(), kDof);
    EXPECT_EQ(pt.accelerations.size(), kDof);
  }

  // 統合テスト 7: 衝突シーン変更によりゴールが塞がれた場合、生成結果は配信されず旧軌道もクリアされる
  const auto goal_pose =
    forwardKinematics(defaultKinematicChain(), toUrdfJointPositions(goal))[5];
  Obstacle blocker;
  blocker.name = "goal_blocker";
  blocker.shape.type = ShapeType::kSphere;
  blocker.shape.radius = 0.20;
  blocker.shape.pose = goal_pose;
  pipeline->updateCollisionScene({blocker});

  // 旧軌道はシーン更新により無効化されていること
  EXPECT_FALSE(pipeline->isTrajectoryValidForExecution(valid_res, start, &reason));

  const GenerateTrajectoryResponse blocked_resp = node.handleGenerateTrajectory(req);
  EXPECT_FALSE(blocked_resp.success);
  EXPECT_FALSE(node.hasPublishedTimedTrajectory());
  EXPECT_EQ(node.publishedTimedTrajectoryCount(), 1u);  // 新たな配信は行われない
}

// 統合テスト 8: 動力学値未設定で hardware_output_enabled=true なら起動失敗する
void testStartupFailureWhenHardwareEnabledWithUnsetDynamics()
{
  const YamlNode pipe_yaml = parseYamlFile("config/trajectory_pipeline.yaml");
  const YamlNode dyn_yaml = parseYamlFile("config/robot_dynamics.yaml");

  RobotDynamicsConfig unset_dyn = loadRobotDynamicsConfig(dyn_yaml);
  TrajectoryPipelineConfig unset_pipe = loadTrajectoryPipelineConfig(pipe_yaml, unset_dyn.chain);

  // hardware_output_enabled=false の既定値では validation_only で安全に起動できる
  TrajectoryPipeline safe_pipe(unset_pipe, unset_dyn);
  EXPECT_EQ(safe_pipe.operationMode(), OperationMode::kValidationOnly);
  EXPECT_FALSE(safe_pipe.isHardwareOutputAllowed());

  // hardware_output_enabled=true かつ未設定値がある場合は例外で起動失敗する
  unset_pipe.hardware_output_enabled = true;
  bool threw_on_unset_kin = false;
  try {
    TrajectoryPipeline bad_pipe(unset_pipe, unset_dyn);
  } catch (const std::runtime_error &) {
    threw_on_unset_kin = true;
  }
  EXPECT_TRUE(threw_on_unset_kin);

  // 運動学上限だけ設定しても、robot_dynamics.yaml が UNSET ならやはり起動失敗する
  TrajectoryPipelineConfig kin_ready = makeTestPipelineConfig();
  kin_ready.hardware_output_enabled = true;
  bool threw_on_unset_dyn = false;
  try {
    TrajectoryPipeline bad_pipe2(kin_ready, unset_dyn);
  } catch (const std::runtime_error &) {
    threw_on_unset_dyn = true;
  }
  EXPECT_TRUE(threw_on_unset_dyn);

  // 両方とも実測値が揃っていれば kHardwareReady で起動成功する
  TrajectoryPipeline hw_ready(kin_ready, makeTestDynamicsConfig());
  EXPECT_EQ(hw_ready.operationMode(), OperationMode::kHardwareReady);
  EXPECT_TRUE(hw_ready.isHardwareOutputAllowed());
}
}  // namespace

int main()
{
  testStationaryAndSeedReproducibility();
  testFoldedTurnCandidateSelection();
  testLoweringTheta1TorqueLimitIncreasesDuration();
  testStaticGravityTorqueOverloadRejected();
  testPostJerkTorqueViolationRejectedByFinalValidator();
  testPayloadInvalidationAndSceneChangeDiscard();
  testStartupFailureWhenHardwareEnabledWithUnsetDynamics();

  if (g_failures == 0) {
    std::cout << "[PASS] test_trajectory_pipeline" << std::endl;
    return 0;
  }
  std::cerr << "[FAIL] test_trajectory_pipeline failures=" << g_failures << std::endl;
  return 1;
}
