#include "path_planner/trajectory_pipeline.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "path_planner/yaml_lite.hpp"

namespace path_planner
{
namespace
{
using Clock = std::chrono::steady_clock;

double elapsedSeconds(Clock::time_point start)
{
  return std::chrono::duration<double>(Clock::now() - start).count();
}

const YamlNode * findNestedOrDotted(
  const YamlNode & node,
  const std::string & prefix,
  const std::string & subkey)
{
  if (const YamlNode * dotted = node.find(prefix + "." + subkey)) {
    return dotted;
  }
  if (const YamlNode * group = node.find(prefix)) {
    return group->find(subkey);
  }
  return nullptr;
}
}  // namespace

const char * toString(OperationMode mode)
{
  switch (mode) {
    case OperationMode::kHardwareReady: return "hardware_ready";
    case OperationMode::kSimulationOnly: return "simulation_only";
    case OperationMode::kValidationOnly: return "validation_only";
  }
  return "validation_only";
}

bool TrajectoryPipelineConfig::hasValidKinematicLimits(std::string * reason) const
{
  const auto fail = [&](const std::string & msg) {
      if (reason != nullptr) {
        *reason = msg;
      }
      return false;
    };
  if (max_velocity.size() != kDof || max_acceleration.size() != kDof || max_jerk.size() != kDof) {
    return fail("max_velocity/max_acceleration/max_jerk must have 4 elements");
  }
  for (std::size_t j = 0; j < kDof; ++j) {
    if (!std::isfinite(max_velocity[j]) || max_velocity[j] <= 0.0) {
      return fail("max_velocity[" + std::to_string(j) + "] is unset or <= 0");
    }
    if (!std::isfinite(max_acceleration[j]) || max_acceleration[j] <= 0.0) {
      return fail("max_acceleration[" + std::to_string(j) + "] is unset or <= 0");
    }
    if (!std::isfinite(max_jerk[j]) || max_jerk[j] <= 0.0) {
      return fail("max_jerk[" + std::to_string(j) + "] is unset or <= 0");
    }
  }
  return true;
}

RobotCollisionModel defaultRobotCollisionModel(const KinematicChain & chain)
{
  RobotCollisionModel model;

  RobotShape base_yaw;
  base_yaw.name = "base_yaw";
  base_yaw.link = "base_yaw_link";
  base_yaw.link_index = findLinkIndex(chain, base_yaw.link);
  base_yaw.shape.type = ShapeType::kCapsule;
  base_yaw.shape.radius = 0.114;
  base_yaw.shape.length = 0.090;
  base_yaw.shape.pose = makeTransform({-0.020, 0.0, 0.045}, {0.0, 0.0, 0.0});

  RobotShape upper_arm;
  upper_arm.name = "upper_arm";
  upper_arm.link = "upper_arm_link";
  upper_arm.link_index = findLinkIndex(chain, upper_arm.link);
  upper_arm.shape.type = ShapeType::kBox;
  upper_arm.shape.half_extents = {0.191 * 0.5, 0.071 * 0.5, 0.480 * 0.5};
  upper_arm.shape.pose = makeTransform({0.0, 0.0, 0.240}, {0.0, 0.0, 0.0});

  RobotShape forearm;
  forearm.name = "forearm";
  forearm.link = "forearm_link";
  forearm.link_index = findLinkIndex(chain, forearm.link);
  forearm.shape.type = ShapeType::kBox;
  forearm.shape.half_extents = {0.07025 * 0.5, 0.071 * 0.5, 0.480 * 0.5};
  forearm.shape.pose = makeTransform({0.0, 0.0, 0.240}, {0.0, 0.0, 0.0});

  RobotShape wrist;
  wrist.name = "wrist";
  wrist.link = "wrist_link";
  wrist.link_index = findLinkIndex(chain, wrist.link);
  wrist.shape.type = ShapeType::kBox;
  wrist.shape.half_extents = {0.040 * 0.5, 0.050 * 0.5, 0.050 * 0.5};
  wrist.shape.pose = makeTransform({0.020, 0.0, 0.0}, {0.0, 0.0, 0.0});

  RobotShape flange;
  flange.name = "flange";
  flange.link = "flange_link";
  flange.link_index = findLinkIndex(chain, flange.link);
  flange.shape.type = ShapeType::kBox;
  flange.shape.half_extents = {0.027 * 0.5, 0.492 * 0.5, 0.124 * 0.5};
  flange.shape.pose = makeTransform({0.0, 0.0, -0.062}, {0.0, 0.0, kPi * 0.5});

  model.shapes = {base_yaw, upper_arm, forearm, wrist, flange};
  // 自己衝突検査対象ペア: upper_arm と flange（隣接リンクは常に接触し得るため除外）
  model.self_collision_pairs = {{1, 4}};
  return model;
}

TrajectoryPipelineConfig loadTrajectoryPipelineConfig(
  const YamlNode & root,
  const KinematicChain & chain)
{
  const YamlNode * section = &root;
  if (root.has("trajectory_pipeline")) {
    section = &root.at("trajectory_pipeline");
  }
  if (section->has("ros__parameters")) {
    section = &section->at("ros__parameters");
  }
  const YamlNode & node = *section;

  TrajectoryPipelineConfig cfg;
  cfg.collision_model = defaultRobotCollisionModel(chain);

  if (node.has("hardware_output_enabled")) {
    cfg.hardware_output_enabled = node.at("hardware_output_enabled").asBool();
  }
  if (node.has("candidate_count")) {
    cfg.candidate_count = static_cast<std::size_t>(node.at("candidate_count").asInt());
  }
  if (node.has("base_seed")) {
    cfg.base_seed = static_cast<std::uint32_t>(node.at("base_seed").asInt());
  }
  if (node.has("planning_time_limit")) {
    cfg.planning_time_limit = node.at("planning_time_limit").asDouble();
  }
  if (node.has("distance_weights")) {
    cfg.distance_weights = node.at("distance_weights").asDoubleList();
  }
  if (node.has("lower_bounds")) {
    cfg.joint_bounds.lower = node.at("lower_bounds").asDoubleList();
  }
  if (node.has("upper_bounds")) {
    cfg.joint_bounds.upper = node.at("upper_bounds").asDoubleList();
  }
  if (node.has("rrt_step_size")) {
    cfg.rrt_step_size = node.at("rrt_step_size").asDouble();
  }
  if (node.has("rrt_goal_bias")) {
    cfg.rrt_goal_bias = node.at("rrt_goal_bias").asDouble();
  }
  if (node.has("rrt_max_iterations")) {
    cfg.rrt_max_iterations = static_cast<std::size_t>(node.at("rrt_max_iterations").asInt());
  }
  if (node.has("shortcut_iterations")) {
    cfg.shortcut_iterations = static_cast<std::size_t>(node.at("shortcut_iterations").asInt());
  }
  if (node.has("max_path_deviation")) {
    cfg.max_path_deviation = node.at("max_path_deviation").asDouble();
  }
  if (node.has("collision_check_resolution")) {
    cfg.collision_check_resolution = node.at("collision_check_resolution").asDouble();
  }
  if (node.has("collision_margin")) {
    cfg.collision_margin = node.at("collision_margin").asDouble();
  }

  if (const YamlNode * w = findNestedOrDotted(node, "cost_weights", "path_length")) {
    cfg.cost_weights.path_length = w->asDouble();
  }
  if (const YamlNode * w = findNestedOrDotted(node, "cost_weights", "theta1_motion")) {
    cfg.cost_weights.theta1_motion = w->asDouble();
  }
  if (const YamlNode * w = findNestedOrDotted(node, "cost_weights", "theta1_inertia")) {
    cfg.cost_weights.theta1_inertia = w->asDouble();
  }
  if (const YamlNode * w = findNestedOrDotted(node, "cost_weights", "theta1_torque")) {
    cfg.cost_weights.theta1_torque = w->asDouble();
  }
  if (const YamlNode * w = findNestedOrDotted(node, "cost_weights", "clearance")) {
    cfg.cost_weights.clearance = w->asDouble();
  }

  if (node.has("topp_grid_points")) {
    cfg.topp_grid_points = static_cast<std::size_t>(node.at("topp_grid_points").asInt());
  }
  if (node.has("adaptive_grid_max_points")) {
    cfg.adaptive_grid_max_points =
      static_cast<std::size_t>(node.at("adaptive_grid_max_points").asInt());
  }
  if (node.has("validation_dt")) {
    cfg.validation_dt = node.at("validation_dt").asDouble();
  }
  if (node.has("cycle_period")) {
    cfg.cycle_period = node.at("cycle_period").asDouble();
  }
  if (node.has("start_velocity_zero")) {
    cfg.start_velocity_zero = node.at("start_velocity_zero").asBool();
  }
  if (node.has("goal_velocity_zero")) {
    cfg.goal_velocity_zero = node.at("goal_velocity_zero").asBool();
  }
  if (node.has("max_velocity")) {
    cfg.max_velocity = node.at("max_velocity").asDoubleList();
  }
  if (node.has("max_acceleration")) {
    cfg.max_acceleration = node.at("max_acceleration").asDoubleList();
  }
  if (node.has("max_jerk")) {
    cfg.max_jerk = node.at("max_jerk").asDoubleList();
  }
  if (node.has("joint_names")) {
    cfg.joint_names = node.at("joint_names").asStringList();
  }
  if (node.has("collision_model")) {
    cfg.collision_model = loadRobotCollisionModel(node.at("collision_model"), chain);
  }
  return cfg;
}

TrajectoryPipeline::TrajectoryPipeline(
  TrajectoryPipelineConfig pipeline_config,
  RobotDynamicsConfig dynamics_config,
  std::shared_ptr<const CollisionScene> initial_scene)
: config_(std::move(pipeline_config))
{
  if (config_.candidate_count == 0) {
    throw std::invalid_argument("candidate_count must be >= 1");
  }
  if (config_.joint_names.size() != kDof || config_.joint_names[0] != "theta1") {
    throw std::invalid_argument("joint_names must have 4 elements with joint_names[0] == 'theta1'");
  }
  if (config_.collision_model.shapes.empty()) {
    config_.collision_model = defaultRobotCollisionModel(dynamics_config.chain);
  }

  std::string kin_reason;
  const bool kin_ok = config_.hasValidKinematicLimits(&kin_reason);
  std::string dyn_reason;
  const bool hw_dyn_ok = dynamics_config.isCompleteForHardware(&dyn_reason);
  const bool sim_dyn_ok = dynamics_config.hasValidDynamics(&dyn_reason);

  // hardware_output_enabled=true かつ必須値欠落時は起動を失敗させる（仕様書 第3.2, 16.2節 8項）。
  if (config_.hardware_output_enabled) {
    if (!kin_ok) {
      throw std::runtime_error(
              "cannot start with hardware_output_enabled=true: " + kin_reason);
    }
    if (!hw_dyn_ok) {
      throw std::runtime_error(
              "cannot start with hardware_output_enabled=true: " + dyn_reason);
    }
    mode_ = OperationMode::kHardwareReady;
  } else if (kin_ok && sim_dyn_ok) {
    mode_ = OperationMode::kSimulationOnly;
  } else {
    mode_ = OperationMode::kValidationOnly;
  }

  dynamics_ = std::make_shared<const RneaDynamicsModel>(std::move(dynamics_config), 1);
  if (initial_scene) {
    scene_ = std::move(initial_scene);
  } else {
    auto empty_scene = std::make_shared<CollisionScene>();
    empty_scene->version = 1;
    scene_ = empty_scene;
  }
}

void TrajectoryPipeline::updatePayload(const Payload & payload)
{
  std::lock_guard<std::mutex> lock(mutex_);
  dynamics_ = std::make_shared<const RneaDynamicsModel>(dynamics_->withPayload(payload));
}

void TrajectoryPipeline::updateCollisionScene(std::vector<Obstacle> obstacles)
{
  for (const auto & o : obstacles) {
    validateShape(o.shape, o.name);
  }
  std::lock_guard<std::mutex> lock(mutex_);
  auto next = std::make_shared<CollisionScene>();
  next->obstacles = std::move(obstacles);
  next->version = scene_->version + 1;
  scene_ = std::move(next);
}

std::uint64_t TrajectoryPipeline::currentDynamicsVersion() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return dynamics_->version();
}

std::uint64_t TrajectoryPipeline::currentSceneVersion() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return scene_->version;
}

RobotDynamicsConfig TrajectoryPipeline::dynamicsConfig() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return dynamics_->config();
}

std::shared_ptr<const CollisionScene> TrajectoryPipeline::collisionScene() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return scene_;
}

bool TrajectoryPipeline::isTrajectoryValidForExecution(
  const PipelineResult & result,
  const State & current_position,
  std::string * reason) const
{
  const auto fail = [&](const std::string & msg) {
      if (reason != nullptr) {
        *reason = msg;
      }
      return false;
    };
  if (!result || result.trajectory.points.empty()) {
    return fail("pipeline result is not valid");
  }
  std::uint64_t dyn_ver = 0;
  std::uint64_t scn_ver = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    dyn_ver = dynamics_->version();
    scn_ver = scene_->version;
  }
  if (result.dynamics_version != dyn_ver) {
    return fail("trajectory invalidated by payload/dynamics update");
  }
  if (result.scene_version != scn_ver) {
    return fail("trajectory invalidated by collision scene update");
  }
  if (!isFiniteState(current_position)) {
    return fail("current joint state is invalid");
  }
  const State & start_pos = result.trajectory.points.front().position;
  for (std::size_t j = 0; j < kDof; ++j) {
    if (std::abs(current_position[j] - start_pos[j]) > config_.start_state_tolerance) {
      return fail("current joint position deviates from trajectory start");
    }
  }
  return true;
}

PipelineResult TrajectoryPipeline::execute(
  const State & start,
  const State & goal,
  std::uint32_t seed,
  const std::atomic<bool> * cancel_requested) const
{
  PipelineResult result;
  std::shared_ptr<const RneaDynamicsModel> dyn_snapshot;
  std::shared_ptr<const CollisionScene> scene_snapshot;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    dyn_snapshot = dynamics_;
    scene_snapshot = scene_;
  }
  result.dynamics_version = dyn_snapshot->version();
  result.scene_version = scene_snapshot->version;

  std::string kin_reason;
  if (!config_.hasValidKinematicLimits(&kin_reason)) {
    result.failure = FailureCode::kMissingDynamics;
    result.failure_stage = "config_check";
    result.message = "missing kinematic limits: " + kin_reason;
    return result;
  }
  std::string dyn_reason;
  if (!dyn_snapshot->config().hasValidDynamics(&dyn_reason)) {
    result.failure = FailureCode::kMissingDynamics;
    result.failure_stage = "config_check";
    result.message = "missing dynamics parameters: " + dyn_reason;
    return result;
  }

  auto checker = std::make_shared<CollisionChecker>(
    dyn_snapshot->config().chain, config_.collision_model, scene_snapshot, config_.collision_margin);
  auto state_validator = std::make_shared<ConfigurationValidator>(
    config_.joint_bounds, checker, config_.collision_check_resolution);

  if (!isFiniteState(start) || !isFiniteState(goal)) {
    result.failure = FailureCode::kInvalidInput;
    result.failure_stage = "input_validation";
    result.message = "start or goal is not a finite 4D state";
    return result;
  }
  if (!config_.joint_bounds.contains(start) || !config_.joint_bounds.contains(goal)) {
    result.failure = FailureCode::kJointLimitViolation;
    result.failure_stage = "input_validation";
    result.message = "start or goal violates joint position bounds";
    return result;
  }
  if (!state_validator->isStateValid(start) || !state_validator->isStateValid(goal)) {
    result.failure = FailureCode::kCollisionDetected;
    result.failure_stage = "input_validation";
    result.message = "start or goal is in collision";
    return result;
  }

  const std::uint32_t base_seed = (seed == 0) ? config_.base_seed : seed;

  RrtConnectConfig rrt_cfg;
  rrt_cfg.bounds = config_.joint_bounds;
  rrt_cfg.distance_weights = config_.distance_weights;
  rrt_cfg.step_size = config_.rrt_step_size;
  rrt_cfg.goal_bias = config_.rrt_goal_bias;
  rrt_cfg.max_iterations = config_.rrt_max_iterations;
  RrtConnect planner(rrt_cfg, state_validator);

  PostProcessConfig pp_cfg;
  pp_cfg.shortcut_iterations = config_.shortcut_iterations;
  pp_cfg.max_path_deviation = config_.max_path_deviation;
  pp_cfg.collision_check_resolution = config_.collision_check_resolution;
  pp_cfg.distance_weights = config_.distance_weights;
  PathPostProcessor post_processor(pp_cfg, state_validator);

  std::vector<std::pair<std::string, double>> stage_durations;
  const auto t_plan_start = Clock::now();
  std::vector<Path> raw_candidates;
  std::vector<ParameterizedPath> splines;
  FailureCode last_plan_failure = FailureCode::kPlanningFailed;
  std::string last_plan_msg = "no valid candidate path found";

  // 同一始終点の場合は 1 候補で十分
  const bool is_same_endpoint = planner.weightedDistance(start, goal) <= 1e-12;
  const std::size_t target_candidates = is_same_endpoint ? 1 : config_.candidate_count;

  for (std::size_t c = 0; c < target_candidates; ++c) {
    if (cancel_requested != nullptr && cancel_requested->load()) {
      result.failure = FailureCode::kPlanningFailed;
      result.failure_stage = "planning";
      result.message = "planning cancelled by newer request";
      return result;
    }
    if (c > 0 && elapsedSeconds(t_plan_start) >= config_.planning_time_limit) {
      break;
    }
    const std::uint32_t cand_seed = base_seed + static_cast<std::uint32_t>(c);
    const PlanResult plan_res = planner.plan(start, goal, cand_seed);
    if (!plan_res) {
      last_plan_failure = plan_res.failure;
      last_plan_msg = plan_res.message;
      continue;
    }
    const PostProcessResult pp_res = post_processor.process(plan_res.path, cand_seed);
    if (!pp_res) {
      last_plan_failure = pp_res.failure;
      last_plan_msg = pp_res.message;
      continue;
    }
    raw_candidates.push_back(pp_res.shortcut_path);
    splines.push_back(pp_res.spline);
  }
  stage_durations.emplace_back("planning_and_smoothing", elapsedSeconds(t_plan_start));

  if (splines.empty()) {
    result.failure = last_plan_failure;
    result.failure_stage =
      (last_plan_failure == FailureCode::kSmoothingFailed) ? "smoothing" : "planning";
    result.message = last_plan_msg;
    result.stage_durations = std::move(stage_durations);
    return result;
  }

  return parameterizeAndValidate(
    splines, raw_candidates, start, goal,
    dyn_snapshot, scene_snapshot, state_validator,
    cancel_requested, std::move(stage_durations));
}

PipelineResult TrajectoryPipeline::executeFromCandidates(
  const std::vector<Path> & candidate_paths,
  const State & start,
  const State & goal,
  std::uint32_t seed) const
{
  PipelineResult result;
  std::shared_ptr<const RneaDynamicsModel> dyn_snapshot;
  std::shared_ptr<const CollisionScene> scene_snapshot;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    dyn_snapshot = dynamics_;
    scene_snapshot = scene_;
  }
  result.dynamics_version = dyn_snapshot->version();
  result.scene_version = scene_snapshot->version;

  std::string kin_reason;
  if (!config_.hasValidKinematicLimits(&kin_reason)) {
    result.failure = FailureCode::kMissingDynamics;
    result.failure_stage = "config_check";
    result.message = "missing kinematic limits: " + kin_reason;
    return result;
  }
  std::string dyn_reason;
  if (!dyn_snapshot->config().hasValidDynamics(&dyn_reason)) {
    result.failure = FailureCode::kMissingDynamics;
    result.failure_stage = "config_check";
    result.message = "missing dynamics parameters: " + dyn_reason;
    return result;
  }
  if (candidate_paths.empty()) {
    result.failure = FailureCode::kInvalidInput;
    result.failure_stage = "input_validation";
    result.message = "candidate_paths is empty";
    return result;
  }

  auto checker = std::make_shared<CollisionChecker>(
    dyn_snapshot->config().chain, config_.collision_model, scene_snapshot, config_.collision_margin);
  auto state_validator = std::make_shared<ConfigurationValidator>(
    config_.joint_bounds, checker, config_.collision_check_resolution);

  PostProcessConfig pp_cfg;
  pp_cfg.shortcut_iterations = config_.shortcut_iterations;
  pp_cfg.max_path_deviation = config_.max_path_deviation;
  pp_cfg.collision_check_resolution = config_.collision_check_resolution;
  pp_cfg.distance_weights = config_.distance_weights;
  PathPostProcessor post_processor(pp_cfg, state_validator);

  const auto t_pp = Clock::now();
  std::vector<Path> raw_candidates;
  std::vector<ParameterizedPath> splines;
  for (std::size_t i = 0; i < candidate_paths.size(); ++i) {
    const PostProcessResult pp_res =
      post_processor.process(candidate_paths[i], seed + static_cast<std::uint32_t>(i));
    if (pp_res) {
      raw_candidates.push_back(pp_res.shortcut_path);
      splines.push_back(pp_res.spline);
    }
  }
  std::vector<std::pair<std::string, double>> stage_durations;
  stage_durations.emplace_back("post_process", elapsedSeconds(t_pp));

  if (splines.empty()) {
    result.failure = FailureCode::kSmoothingFailed;
    result.failure_stage = "smoothing";
    result.message = "all provided candidates failed post-processing";
    result.stage_durations = std::move(stage_durations);
    return result;
  }

  return parameterizeAndValidate(
    splines, raw_candidates, start, goal,
    dyn_snapshot, scene_snapshot, state_validator,
    nullptr, std::move(stage_durations));
}

PipelineResult TrajectoryPipeline::parameterizeAndValidate(
  const std::vector<ParameterizedPath> & splines,
  const std::vector<Path> & raw_candidates,
  const State & start,
  const State & goal,
  const std::shared_ptr<const RneaDynamicsModel> & dyn_snapshot,
  const std::shared_ptr<const CollisionScene> & scene_snapshot,
  const std::shared_ptr<const StateValidator> & state_validator,
  const std::atomic<bool> * cancel_requested,
  std::vector<std::pair<std::string, double>> stage_durations) const
{
  PipelineResult result;
  result.dynamics_version = dyn_snapshot->version();
  result.scene_version = scene_snapshot->version;
  result.candidate_count = splines.size();

  // 1. 第1軸負荷を考慮した候補評価・ソート
  const auto t_cost = Clock::now();
  TorqueAwareCost cost_evaluator(
    config_.cost_weights, config_.distance_weights,
    dyn_snapshot, state_validator, config_.cost_normalization);
  std::vector<CandidateScore> scores;
  cost_evaluator.selectBest(splines, &scores);

  std::vector<std::size_t> order(splines.size());
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(
    order.begin(), order.end(),
    [&](std::size_t a, std::size_t b) {return scores[a].total_cost < scores[b].total_cost;});
  stage_durations.emplace_back("cost_evaluation", elapsedSeconds(t_cost));

  // 2. TOPP-RA -> JerkLimiter -> TrajectoryValidator
  ToppraConfig topp_cfg;
  topp_cfg.grid_points = config_.topp_grid_points;
  topp_cfg.adaptive_grid_max_points = config_.adaptive_grid_max_points;
  topp_cfg.start_velocity_zero = config_.start_velocity_zero;
  topp_cfg.goal_velocity_zero = config_.goal_velocity_zero;
  topp_cfg.max_velocity = config_.max_velocity;
  topp_cfg.max_acceleration = config_.max_acceleration;
  topp_cfg.enable_torque_constraints = config_.enforce_torque_in_toppra;
  ToppraParameterizer toppra(topp_cfg, dyn_snapshot, config_.joint_names);

  JerkLimiterConfig jerk_cfg;
  jerk_cfg.max_velocity = config_.max_velocity;
  jerk_cfg.max_acceleration = config_.max_acceleration;
  jerk_cfg.max_jerk = config_.max_jerk;
  jerk_cfg.start_velocity_zero = config_.start_velocity_zero;
  jerk_cfg.goal_velocity_zero = config_.goal_velocity_zero;
  jerk_cfg.enforce_torque_limits = config_.enforce_torque_in_jerk_limiter;
  jerk_cfg.sample_dt = config_.validation_dt;
  JerkLimiter jerk_limiter(jerk_cfg, dyn_snapshot, config_.joint_names);

  ValidatorConfig val_cfg;
  val_cfg.bounds = config_.joint_bounds;
  val_cfg.max_velocity = config_.max_velocity;
  val_cfg.max_acceleration = config_.max_acceleration;
  val_cfg.max_jerk = config_.max_jerk;
  val_cfg.validation_dt = config_.validation_dt;
  val_cfg.cycle_period = config_.cycle_period;
  val_cfg.start_velocity_zero = config_.start_velocity_zero;
  val_cfg.goal_velocity_zero = config_.goal_velocity_zero;
  val_cfg.require_dynamics = true;
  TrajectoryValidator validator(val_cfg, state_validator, dyn_snapshot);

  // 最良スコア候補を選択して時間パラメータ化・最終検証を行う（仕様書 第9節）。
  const std::size_t best_idx = order.front();
  result.selected_candidate = best_idx;
  result.geometric_path = raw_candidates[best_idx];
  result.candidate_paths = raw_candidates;

  if (cancel_requested != nullptr && cancel_requested->load()) {
    result.failure = FailureCode::kPlanningFailed;
    result.failure_stage = "cancelled";
    result.message = "cancelled before time parameterization";
    result.stage_durations = std::move(stage_durations);
    return result;
  }

  const auto t_topp = Clock::now();
  const ToppraResult topp_res = toppra.parameterize(splines[best_idx]);
  stage_durations.emplace_back("toppra", elapsedSeconds(t_topp));
  result.bottleneck = topp_res.bottleneck;

  if (!topp_res) {
    result.failure = topp_res.failure;
    result.failure_stage = "toppra";
    result.message = topp_res.message;
    result.minimum_torque_margin = topp_res.minimum_torque_margin;
    result.stage_durations = std::move(stage_durations);
    return result;
  }

  const auto t_jerk = Clock::now();
  const JerkLimitResult jerk_res = jerk_limiter.limit(splines[best_idx], topp_res.profile);
  stage_durations.emplace_back("jerk_limiter", elapsedSeconds(t_jerk));
  if (!jerk_res) {
    result.failure = jerk_res.failure;
    result.failure_stage = "jerk_limiter";
    result.message = jerk_res.message;
    result.theta1_peak_torque = jerk_res.theta1_peak_torque;
    result.theta1_rms_torque = jerk_res.theta1_rms_torque;
    result.minimum_torque_margin = jerk_res.minimum_torque_margin;
    result.stage_durations = std::move(stage_durations);
    return result;
  }

  const auto t_val = Clock::now();
  const ValidationReport val_rep = validator.validate(jerk_res.trajectory, start, goal);
  stage_durations.emplace_back("validation", elapsedSeconds(t_val));

  result.duration = val_rep.duration;
  result.theta1_peak_torque = val_rep.theta1_peak_torque;
  result.theta1_rms_torque = val_rep.theta1_rms_torque;
  result.minimum_torque_margin = val_rep.minimum_torque_margin;
  result.theta1_rms_margin = val_rep.theta1_rms_margin;

  if (!val_rep) {
    result.failure = val_rep.failure;
    result.failure_stage = "final_validation";
    result.message = val_rep.message;
    result.stage_durations = std::move(stage_durations);
    return result;
  }

  // 計画中にシーンまたはペイロードが変化していないか最終確認する（仕様書 第13.2, 16.2節 6, 7項）。
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (dynamics_->version() != dyn_snapshot->version()) {
      result.failure = FailureCode::kMissingDynamics;
      result.failure_stage = "concurrency_guard";
      result.message = "payload/dynamics changed during trajectory generation; discarding result";
      result.stage_durations = std::move(stage_durations);
      return result;
    }
    if (scene_->version != scene_snapshot->version) {
      result.failure = FailureCode::kCollisionDetected;
      result.failure_stage = "concurrency_guard";
      result.message = "collision scene changed during trajectory generation; discarding result";
      result.stage_durations = std::move(stage_durations);
      return result;
    }
  }

  result.trajectory = jerk_res.trajectory;
  result.torque_verified = true;
  result.failure = FailureCode::kNone;
  result.message = "OK";
  result.stage_durations = std::move(stage_durations);
  return result;
}

}  // namespace path_planner
