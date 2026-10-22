// RRT-Connect および自作衝突判定の単体テスト（仕様書 第7, 16.1節）。

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "path_planner/collision_checker.hpp"
#include "path_planner/robot_model.hpp"
#include "path_planner/rrt_connect.hpp"
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

void testPrimitiveCollisionDistances()
{
  // 1. 球―球
  Shape s1{ShapeType::kSphere, 0.1, 0.0, {}, makeTransform({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0})};
  Shape s2{ShapeType::kSphere, 0.2, 0.0, {}, makeTransform({0.5, 0.0, 0.0}, {0.0, 0.0, 0.0})};
  EXPECT_NEAR(shapeDistance(s1, s2), 0.2, 1e-9);

  // 2. 球―カプセル
  Shape cap{ShapeType::kCapsule, 0.05, 0.4, {}, makeTransform({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0})};
  Shape sp{ShapeType::kSphere, 0.05, 0.0, {}, makeTransform({0.2, 0.0, 0.1}, {0.0, 0.0, 0.0})};
  EXPECT_NEAR(shapeDistance(sp, cap), 0.10, 1e-9);

  // 3. カプセル―カプセル
  Shape cap2{ShapeType::kCapsule, 0.05, 0.4, {}, makeTransform({0.25, 0.0, 0.0}, {0.0, 0.0, 0.0})};
  EXPECT_NEAR(shapeDistance(cap, cap2), 0.15, 1e-9);

  // 4. 球―直方体(OBB)
  Shape box{ShapeType::kBox, 0.0, 0.0, {0.1, 0.1, 0.1},
    makeTransform({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0})};
  Shape sp_near{ShapeType::kSphere, 0.05, 0.0, {},
    makeTransform({0.25, 0.0, 0.0}, {0.0, 0.0, 0.0})};
  EXPECT_NEAR(shapeDistance(sp_near, box), 0.10, 1e-9);

  // 5. カプセル―直方体(OBB)
  Shape cap_near{ShapeType::kCapsule, 0.05, 0.2, {},
    makeTransform({0.25, 0.0, 0.0}, {0.0, 0.0, 0.0})};
  EXPECT_NEAR(shapeDistance(cap_near, box), 0.10, 1e-6);

  // 6. 直方体―直方体(SAT)
  Shape box2{ShapeType::kBox, 0.0, 0.0, {0.1, 0.1, 0.1},
    makeTransform({0.35, 0.0, 0.0}, {0.0, 0.0, 0.0})};
  EXPECT_NEAR(shapeDistance(box, box2), 0.15, 1e-9);
  Shape box_overlap{ShapeType::kBox, 0.0, 0.0, {0.1, 0.1, 0.1},
    makeTransform({0.10, 0.0, 0.0}, {0.0, 0.0, 0.0})};
  EXPECT_TRUE(shapeDistance(box, box_overlap) < 0.0);
}

void testStraightLineAndSeedDeterminism()
{
  auto scene = std::make_shared<CollisionScene>();
  KinematicChain chain = defaultKinematicChain();
  auto checker = std::make_shared<CollisionChecker>(
    chain, defaultRobotCollisionModel(chain), scene, 0.002);
  JointBounds bounds{{-2.5, -1.5, -1.5, -2.5}, {2.5, 1.5, 1.5, 2.5}};
  auto validator = std::make_shared<ConfigurationValidator>(bounds, checker, 0.02);

  RrtConnectConfig cfg;
  cfg.bounds = bounds;
  cfg.distance_weights = {2.0, 1.0, 1.0, 1.0};
  cfg.step_size = 0.15;
  cfg.max_iterations = 2000;
  RrtConnect planner(cfg, validator);

  const State start{0.0, 0.2, 0.4, 0.0};
  const State goal{0.5, 0.4, 0.6, 0.2};
  const PlanResult r1 = planner.plan(start, goal, 123);
  const PlanResult r2 = planner.plan(start, goal, 123);
  EXPECT_TRUE(r1);
  EXPECT_TRUE(r2);
  EXPECT_EQ(r1.path.size(), r2.path.size());
  for (std::size_t i = 0; i < r1.path.size(); ++i) {
    for (std::size_t j = 0; j < kDof; ++j) {
      EXPECT_NEAR(r1.path[i][j], r2.path[i][j], 1e-12);
    }
  }
  // 始終点の一致と全辺の妥当性
  for (std::size_t j = 0; j < kDof; ++j) {
    EXPECT_NEAR(r1.path.front()[j], start[j], 1e-12);
    EXPECT_NEAR(r1.path.back()[j], goal[j], 1e-12);
  }
  for (std::size_t i = 0; i + 1 < r1.path.size(); ++i) {
    EXPECT_TRUE(validator->isMotionValid(r1.path[i], r1.path[i + 1]));
  }
}

void testObstacleDetourAndUnreachableAndBounds()
{
  KinematicChain chain = defaultKinematicChain();
  auto scene = std::make_shared<CollisionScene>();
  // 直線経路の途中にある球障害物を配置し、迂回経路が生成されることを確認する。
  const State mid{0.0, 0.6, 0.6, 0.0};
  const auto mid_frames = forwardKinematics(chain, toUrdfJointPositions(mid));
  Obstacle obs;
  obs.name = "center_blocker";
  obs.shape.type = ShapeType::kSphere;
  obs.shape.radius = 0.08;
  obs.shape.pose = mid_frames[5];  // flange_link 原点付近を塞ぐ
  scene->obstacles.push_back(obs);

  auto checker = std::make_shared<CollisionChecker>(
    chain, defaultRobotCollisionModel(chain), scene, 0.002);
  JointBounds bounds{{-2.5, -1.8, -1.8, -2.5}, {2.5, 1.8, 1.8, 2.5}};
  auto validator = std::make_shared<ConfigurationValidator>(bounds, checker, 0.02);

  const State start{-0.6, 0.2, 0.3, 0.0};
  const State goal{0.6, 0.2, 0.3, 0.0};
  EXPECT_TRUE(validator->isStateValid(start));
  EXPECT_TRUE(validator->isStateValid(goal));

  RrtConnectConfig cfg;
  cfg.bounds = bounds;
  cfg.distance_weights = {2.0, 1.0, 1.0, 1.0};
  cfg.step_size = 0.15;
  cfg.max_iterations = 3000;
  RrtConnect planner(cfg, validator);

  const PlanResult detour = planner.plan(start, goal, 77);
  EXPECT_TRUE(detour);
  for (std::size_t i = 0; i + 1 < detour.path.size(); ++i) {
    EXPECT_TRUE(validator->isMotionValid(detour.path[i], detour.path[i + 1]));
  }

  // 関節境界外の始点は即座に kJointLimitViolation
  const State out_of_bounds{3.0, 0.0, 0.0, 0.0};
  const PlanResult r_bounds = planner.plan(out_of_bounds, goal, 77);
  EXPECT_FALSE(r_bounds);
  EXPECT_EQ(r_bounds.failure, FailureCode::kJointLimitViolation);

  // 到達不能: 始点・終点は非衝突だが、theta1 = 0 付近を直方体壁で完全に分断した場合
  auto blocked_scene = std::make_shared<CollisionScene>();
  Obstacle dividing_wall;
  dividing_wall.name = "dividing_wall";
  dividing_wall.shape.type = ShapeType::kBox;
  dividing_wall.shape.half_extents = {0.03, 0.45, 1.0};
  // theta1 = 0 のとき upper_arm/forearm が通過する X = 0.675, Y > 0.10 に壁を置く
  dividing_wall.shape.pose = makeTransform({0.675, 0.55, 0.50}, {0.0, 0.0, 0.0});
  blocked_scene->obstacles = {dividing_wall};

  JointBounds tight_bounds{{-0.9, 0.5, 0.5, -0.1}, {0.9, 0.7, 0.7, 0.1}};
  const State s_unreach{-0.8, 0.6, 0.6, 0.0};
  const State g_unreach{0.8, 0.6, 0.6, 0.0};
  auto blocked_checker = std::make_shared<CollisionChecker>(
    chain, defaultRobotCollisionModel(chain), blocked_scene, 0.002);
  auto blocked_validator = std::make_shared<ConfigurationValidator>(
    tight_bounds, blocked_checker, 0.02);
  EXPECT_TRUE(blocked_validator->isStateValid(s_unreach));
  EXPECT_TRUE(blocked_validator->isStateValid(g_unreach));

  cfg.bounds = tight_bounds;
  cfg.max_iterations = 200;
  RrtConnect blocked_planner(cfg, blocked_validator);
  const PlanResult r_unreach = blocked_planner.plan(s_unreach, g_unreach, 42);
  EXPECT_FALSE(r_unreach);
  EXPECT_EQ(r_unreach.failure, FailureCode::kPlanningFailed);
}
}  // namespace

int main()
{
  testPrimitiveCollisionDistances();
  testStraightLineAndSeedDeterminism();
  testObstacleDetourAndUnreachableAndBounds();

  if (g_failures == 0) {
    std::cout << "[PASS] test_rrt_connect" << std::endl;
    return 0;
  }
  std::cerr << "[FAIL] test_rrt_connect failures=" << g_failures << std::endl;
  return 1;
}
