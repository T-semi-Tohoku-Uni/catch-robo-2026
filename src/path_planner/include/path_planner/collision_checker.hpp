#pragma once

// 自作衝突判定（仕様書 第7.4節）。
// ロボットと障害物を球・カプセル・直方体(OBB)で近似し、順運動学でワールド座標へ変換して
// 距離/交差を判定する。連続衝突判定ではないため collision_margin を加えた保守的判定とする。

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "path_planner/math3d.hpp"
#include "path_planner/robot_model.hpp"
#include "path_planner/types.hpp"

namespace path_planner
{
class YamlNode;

enum class ShapeType {kSphere, kCapsule, kBox};

/// プリミティブ形状。pose は所属座標系（リンクまたはワールド）での形状中心姿勢。
///  - 球: radius
///  - カプセル: 局所 z 軸方向の線分（長さ length、中心が原点）+ 半径 radius
///  - 直方体: half_extents（局所 x, y, z の半寸法）
struct Shape
{
  ShapeType type{ShapeType::kSphere};
  double radius{0.0};
  double length{0.0};
  Vector3 half_extents{};
  Transform pose{};
};

/// 不正寸法なら std::invalid_argument（無視しない）。
void validateShape(const Shape & shape, const std::string & name);

/// 形状をワールド姿勢 frame の下へ変換する。
Shape transformShape(const Shape & shape, const Transform & frame);

/// 2形状間の距離。交差時は 0 以下（貫通量の目安）。
/// 直方体―直方体は分離軸判定による距離の下界（保守側）を返す。
double shapeDistance(const Shape & a, const Shape & b);

struct RobotShape
{
  std::string name;
  std::string link;
  int link_index{-1};
  Shape shape;              // リンク座標系での形状
  bool check_environment{true};
};

struct Obstacle
{
  std::string name;
  Shape shape;              // ワールド座標系での形状
};

/// ロボット形状と検査対象ペア（自己衝突は検査するペアを明示）。
struct RobotCollisionModel
{
  std::vector<RobotShape> shapes;
  std::vector<std::pair<std::size_t, std::size_t>> self_collision_pairs;
};

/// 計画中は不変として扱うシーンのスナップショット（仕様書 第7.2節）。
struct CollisionScene
{
  std::vector<Obstacle> obstacles;
  std::uint64_t version{0};
};

/// YAML の `collision_model` ノードからロボット形状と自己衝突ペアを読む。
RobotCollisionModel loadRobotCollisionModel(const YamlNode & node, const KinematicChain & chain);
/// YAML の `obstacles` リストを読む。
std::vector<Obstacle> loadObstacles(const YamlNode & node);

struct CollisionReport
{
  bool colliding{false};
  double min_distance{0.0};
  std::string pair;         // 最小距離（または衝突）となった組
};

class CollisionChecker
{
public:
  CollisionChecker(
    KinematicChain chain, RobotCollisionModel model,
    std::shared_ptr<const CollisionScene> scene, double margin);

  /// theta（4要素）で衝突（距離 < margin）していれば true。
  bool isColliding(const State & theta) const;
  /// 最小距離と対応ペアを返す（クリアランス評価用）。
  CollisionReport evaluate(const State & theta) const;

  const KinematicChain & chain() const {return chain_;}
  double margin() const {return margin_;}
  std::uint64_t sceneVersion() const {return scene_->version;}

private:
  KinematicChain chain_;
  RobotCollisionModel model_;
  std::shared_ptr<const CollisionScene> scene_;
  double margin_;

  std::vector<Shape> worldShapes(const State & theta) const;
};

/// 状態・動作妥当性判定のインターフェース（RRT・後処理・検証で共用）。
class StateValidator
{
public:
  virtual ~StateValidator() = default;
  virtual bool isStateValid(const State & q) const = 0;
  /// 両端を含め、各関節差が resolution 以下になるよう線形補間した全姿勢を検査する。
  virtual bool isMotionValid(const State & a, const State & b) const = 0;
  /// 最小クリアランス (m)。
  virtual double clearance(const State & q) const = 0;
};

/// 関節範囲 + 自作衝突判定による妥当性判定。
class ConfigurationValidator : public StateValidator
{
public:
  ConfigurationValidator(
    JointBounds bounds, std::shared_ptr<const CollisionChecker> checker, double resolution);

  bool isStateValid(const State & q) const override;
  bool isMotionValid(const State & a, const State & b) const override;
  double clearance(const State & q) const override;

  const JointBounds & bounds() const {return bounds_;}
  double resolution() const {return resolution_;}

private:
  JointBounds bounds_;
  std::shared_ptr<const CollisionChecker> checker_;
  double resolution_;
};

}  // namespace path_planner
