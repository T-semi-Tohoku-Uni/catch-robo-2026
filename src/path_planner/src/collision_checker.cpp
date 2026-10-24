#include "path_planner/collision_checker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "path_planner/yaml_lite.hpp"

namespace path_planner
{
namespace
{
constexpr double kEps = 1e-12;

double clamp01(double v) {return std::min(1.0, std::max(0.0, v));}

struct Segment
{
  Vector3 p;
  Vector3 q;
};

Segment capsuleSegment(const Shape & capsule)
{
  const Vector3 axis = capsule.pose.rotation.column(2);
  const Vector3 half = axis * (capsule.length * 0.5);
  return {capsule.pose.translation - half, capsule.pose.translation + half};
}

double pointSegmentDistance(const Vector3 & point, const Segment & seg)
{
  const Vector3 d = seg.q - seg.p;
  const double len_sq = dot(d, d);
  double t = 0.0;
  if (len_sq > kEps) {
    t = clamp01(dot(point - seg.p, d) / len_sq);
  }
  return norm(point - (seg.p + d * t));
}

// Ericson, Real-Time Collision Detection 5.1.9 の線分間最近接距離。
double segmentSegmentDistance(const Segment & s1, const Segment & s2)
{
  const Vector3 d1 = s1.q - s1.p;
  const Vector3 d2 = s2.q - s2.p;
  const Vector3 r = s1.p - s2.p;
  const double a = dot(d1, d1);
  const double e = dot(d2, d2);
  const double f = dot(d2, r);
  double s = 0.0;
  double t = 0.0;
  if (a <= kEps && e <= kEps) {
    return norm(r);
  }
  if (a <= kEps) {
    t = clamp01(f / e);
  } else {
    const double c = dot(d1, r);
    if (e <= kEps) {
      s = clamp01(-c / a);
    } else {
      const double b = dot(d1, d2);
      const double denom = a * e - b * b;
      s = denom > kEps ? clamp01((b * f - c * e) / denom) : 0.0;
      t = (b * s + f) / e;
      if (t < 0.0) {
        t = 0.0;
        s = clamp01(-c / a);
      } else if (t > 1.0) {
        t = 1.0;
        s = clamp01((b - c) / a);
      }
    }
  }
  return norm((s1.p + d1 * s) - (s2.p + d2 * t));
}

// 点と OBB の符号付き距離（内部は負）。
double pointBoxSignedDistance(const Vector3 & point, const Shape & box)
{
  const Vector3 local = transpose(box.pose.rotation) * (point - box.pose.translation);
  const Vector3 q{
    std::abs(local.x) - box.half_extents.x,
    std::abs(local.y) - box.half_extents.y,
    std::abs(local.z) - box.half_extents.z};
  const Vector3 outside{std::max(q.x, 0.0), std::max(q.y, 0.0), std::max(q.z, 0.0)};
  const double inside = std::min(std::max(q.x, std::max(q.y, q.z)), 0.0);
  return norm(outside) + inside;
}

// 線分と OBB の最近接（符号付き）距離。符号付き距離関数は凸なので黄金分割で最小化する。
double segmentBoxSignedDistance(const Segment & seg, const Shape & box)
{
  const auto f = [&](double t) {
      return pointBoxSignedDistance(seg.p + (seg.q - seg.p) * t, box);
    };
  constexpr double kInvPhi = 0.6180339887498949;
  double lo = 0.0;
  double hi = 1.0;
  double x1 = hi - kInvPhi * (hi - lo);
  double x2 = lo + kInvPhi * (hi - lo);
  double f1 = f(x1);
  double f2 = f(x2);
  for (int i = 0; i < 80; ++i) {
    if (f1 < f2) {
      hi = x2;
      x2 = x1;
      f2 = f1;
      x1 = hi - kInvPhi * (hi - lo);
      f1 = f(x1);
    } else {
      lo = x1;
      x1 = x2;
      f1 = f2;
      x2 = lo + kInvPhi * (hi - lo);
      f2 = f(x2);
    }
  }
  return std::min({f1, f2, f(0.0), f(1.0)});
}

// OBB 同士の分離軸判定。返り値は全15軸での最大分離量（交差時は負）。
// 分離量は真の距離の下界なので、クリアランス判定は保守側になる。
double boxBoxSeparation(const Shape & a, const Shape & b)
{
  const Vector3 diff = b.pose.translation - a.pose.translation;
  Vector3 axes[15];
  int count = 0;
  for (int i = 0; i < 3; ++i) {
    axes[count++] = a.pose.rotation.column(i);
  }
  for (int i = 0; i < 3; ++i) {
    axes[count++] = b.pose.rotation.column(i);
  }
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      axes[count++] = cross(a.pose.rotation.column(i), b.pose.rotation.column(j));
    }
  }
  double best = -std::numeric_limits<double>::infinity();
  for (int k = 0; k < count; ++k) {
    const double n = norm(axes[k]);
    if (n < 1e-9) {
      continue;
    }
    const Vector3 axis = axes[k] / n;
    double ra = 0.0;
    double rb = 0.0;
    for (int i = 0; i < 3; ++i) {
      ra += a.half_extents[i] * std::abs(dot(a.pose.rotation.column(i), axis));
      rb += b.half_extents[i] * std::abs(dot(b.pose.rotation.column(i), axis));
    }
    best = std::max(best, std::abs(dot(diff, axis)) - ra - rb);
  }
  return best;
}

Shape loadShape(const YamlNode & node, const std::string & name)
{
  Shape shape;
  const std::string type = node.at("type").asString();
  Vector3 xyz{};
  Vector3 rpy{};
  if (node.has("xyz")) {
    const auto v = node.at("xyz").asDoubleList();
    if (v.size() != 3) {
      throw std::invalid_argument(name + ".xyz must have 3 elements");
    }
    xyz = {v[0], v[1], v[2]};
  }
  if (node.has("rpy")) {
    const auto v = node.at("rpy").asDoubleList();
    if (v.size() != 3) {
      throw std::invalid_argument(name + ".rpy must have 3 elements");
    }
    rpy = {v[0], v[1], v[2]};
  }
  shape.pose = makeTransform(xyz, rpy);
  if (type == "sphere") {
    shape.type = ShapeType::kSphere;
    shape.radius = node.at("radius").asDouble();
  } else if (type == "capsule") {
    shape.type = ShapeType::kCapsule;
    shape.radius = node.at("radius").asDouble();
    shape.length = node.at("length").asDouble();
  } else if (type == "box") {
    shape.type = ShapeType::kBox;
    const auto size = node.at("size").asDoubleList();
    if (size.size() != 3) {
      throw std::invalid_argument(name + ".size must have 3 elements");
    }
    shape.half_extents = {size[0] * 0.5, size[1] * 0.5, size[2] * 0.5};
  } else {
    throw std::invalid_argument("unsupported shape type '" + type + "' for " + name);
  }
  validateShape(shape, name);
  return shape;
}
}  // namespace

void validateShape(const Shape & shape, const std::string & name)
{
  const auto positive = [](double v) {return std::isfinite(v) && v > 0.0;};
  switch (shape.type) {
    case ShapeType::kSphere:
      if (!positive(shape.radius)) {
        throw std::invalid_argument("sphere '" + name + "' requires radius > 0");
      }
      break;
    case ShapeType::kCapsule:
      if (!positive(shape.radius) || !std::isfinite(shape.length) || shape.length < 0.0) {
        throw std::invalid_argument("capsule '" + name + "' requires radius > 0, length >= 0");
      }
      break;
    case ShapeType::kBox:
      if (!positive(shape.half_extents.x) || !positive(shape.half_extents.y) ||
        !positive(shape.half_extents.z))
      {
        throw std::invalid_argument("box '" + name + "' requires positive size");
      }
      break;
    default:
      throw std::invalid_argument("unsupported shape type for '" + name + "'");
  }
  if (!isFinite(shape.pose.translation)) {
    throw std::invalid_argument("shape '" + name + "' pose must be finite");
  }
}

Shape transformShape(const Shape & shape, const Transform & frame)
{
  Shape out = shape;
  out.pose = frame * shape.pose;
  return out;
}

double shapeDistance(const Shape & a, const Shape & b)
{
  // 型の組を正規化する（a.type <= b.type）。
  if (static_cast<int>(a.type) > static_cast<int>(b.type)) {
    return shapeDistance(b, a);
  }
  if (a.type == ShapeType::kSphere && b.type == ShapeType::kSphere) {
    return norm(a.pose.translation - b.pose.translation) - a.radius - b.radius;
  }
  if (a.type == ShapeType::kSphere && b.type == ShapeType::kCapsule) {
    return pointSegmentDistance(a.pose.translation, capsuleSegment(b)) - a.radius - b.radius;
  }
  if (a.type == ShapeType::kSphere && b.type == ShapeType::kBox) {
    return pointBoxSignedDistance(a.pose.translation, b) - a.radius;
  }
  if (a.type == ShapeType::kCapsule && b.type == ShapeType::kCapsule) {
    return segmentSegmentDistance(capsuleSegment(a), capsuleSegment(b)) - a.radius - b.radius;
  }
  if (a.type == ShapeType::kCapsule && b.type == ShapeType::kBox) {
    return segmentBoxSignedDistance(capsuleSegment(a), b) - a.radius;
  }
  if (a.type == ShapeType::kBox && b.type == ShapeType::kBox) {
    return boxBoxSeparation(a, b);
  }
  throw std::invalid_argument("unsupported shape pair");
}

RobotCollisionModel loadRobotCollisionModel(const YamlNode & node, const KinematicChain & chain)
{
  RobotCollisionModel model;
  const YamlNode & shapes = node.at("shapes");
  if (!shapes.isList() || shapes.list.empty()) {
    throw std::invalid_argument("collision_model.shapes must be a non-empty list");
  }
  for (const auto & item : shapes.list) {
    RobotShape rs;
    rs.name = item.at("name").asString();
    rs.link = item.at("link").asString();
    rs.link_index = findLinkIndex(chain, rs.link);
    if (rs.link_index < 0) {
      throw std::invalid_argument("collision shape '" + rs.name + "' refers to unknown link '" +
              rs.link + "'");
    }
    rs.shape = loadShape(item, rs.name);
    if (item.has("check_environment")) {
      rs.check_environment = item.at("check_environment").asBool();
    }
    for (const auto & existing : model.shapes) {
      if (existing.name == rs.name) {
        throw std::invalid_argument("duplicate collision shape name '" + rs.name + "'");
      }
    }
    model.shapes.push_back(rs);
  }
  const auto index_of = [&](const std::string & name) {
      for (std::size_t i = 0; i < model.shapes.size(); ++i) {
        if (model.shapes[i].name == name) {
          return i;
        }
      }
      throw std::invalid_argument("self_collision_pairs refers to unknown shape '" + name + "'");
    };
  if (const YamlNode * pairs = node.find("self_collision_pairs")) {
    if (!pairs->isNull()) {
      if (!pairs->isList()) {
        throw std::invalid_argument("self_collision_pairs must be a list of [a, b]");
      }
      for (const auto & pair : pairs->list) {
        const auto names = pair.asStringList();
        if (names.size() != 2 || names[0] == names[1]) {
          throw std::invalid_argument("each self collision pair must name two different shapes");
        }
        model.self_collision_pairs.emplace_back(index_of(names[0]), index_of(names[1]));
      }
    }
  }
  return model;
}

std::vector<Obstacle> loadObstacles(const YamlNode & node)
{
  std::vector<Obstacle> obstacles;
  if (node.isNull()) {
    return obstacles;
  }
  if (!node.isList()) {
    throw std::invalid_argument("obstacles must be a list");
  }
  for (const auto & item : node.list) {
    Obstacle o;
    o.name = item.at("name").asString();
    o.shape = loadShape(item, o.name);
    obstacles.push_back(o);
  }
  return obstacles;
}

CollisionChecker::CollisionChecker(
  KinematicChain chain, RobotCollisionModel model,
  std::shared_ptr<const CollisionScene> scene, double margin)
: chain_(std::move(chain)), model_(std::move(model)), scene_(std::move(scene)), margin_(margin)
{
  validateKinematicChain(chain_);
  if (!scene_) {
    throw std::invalid_argument("collision scene must not be null");
  }
  if (!std::isfinite(margin_) || margin_ < 0.0) {
    throw std::invalid_argument("collision_margin must be >= 0");
  }
  for (const auto & rs : model_.shapes) {
    if (rs.link_index < 0 || rs.link_index >= static_cast<int>(kLinkCount)) {
      throw std::invalid_argument("collision shape '" + rs.name + "' has invalid link index");
    }
    validateShape(rs.shape, rs.name);
  }
  for (const auto & pair : model_.self_collision_pairs) {
    if (pair.first >= model_.shapes.size() || pair.second >= model_.shapes.size()) {
      throw std::invalid_argument("self collision pair index out of range");
    }
  }
  for (const auto & o : scene_->obstacles) {
    validateShape(o.shape, o.name);
  }
}

std::vector<Shape> CollisionChecker::worldShapes(const State & theta) const
{
  const auto frames = forwardKinematics(chain_, toUrdfJointPositions(theta));
  std::vector<Shape> out;
  out.reserve(model_.shapes.size());
  for (const auto & rs : model_.shapes) {
    out.push_back(transformShape(rs.shape, frames[static_cast<std::size_t>(rs.link_index)]));
  }
  return out;
}

CollisionReport CollisionChecker::evaluate(const State & theta) const
{
  CollisionReport report;
  report.min_distance = std::numeric_limits<double>::infinity();
  if (!isFiniteState(theta)) {
    report.colliding = true;
    report.min_distance = -std::numeric_limits<double>::infinity();
    report.pair = "non-finite state";
    return report;
  }
  const std::vector<Shape> shapes = worldShapes(theta);
  const auto consider = [&](double d, const std::string & a, const std::string & b) {
      if (d < report.min_distance) {
        report.min_distance = d;
        report.pair = a + "-" + b;
      }
    };
  for (const auto & pair : model_.self_collision_pairs) {
    consider(
      shapeDistance(shapes[pair.first], shapes[pair.second]),
      model_.shapes[pair.first].name, model_.shapes[pair.second].name);
  }
  for (std::size_t i = 0; i < shapes.size(); ++i) {
    if (!model_.shapes[i].check_environment) {
      continue;
    }
    for (const auto & o : scene_->obstacles) {
      consider(shapeDistance(shapes[i], o.shape), model_.shapes[i].name, o.name);
    }
  }
  report.colliding = report.min_distance < margin_;
  return report;
}

bool CollisionChecker::isColliding(const State & theta) const
{
  if (!isFiniteState(theta)) {
    return true;
  }
  const std::vector<Shape> shapes = worldShapes(theta);
  for (const auto & pair : model_.self_collision_pairs) {
    if (shapeDistance(shapes[pair.first], shapes[pair.second]) < margin_) {
      return true;
    }
  }
  for (std::size_t i = 0; i < shapes.size(); ++i) {
    if (!model_.shapes[i].check_environment) {
      continue;
    }
    for (const auto & o : scene_->obstacles) {
      if (shapeDistance(shapes[i], o.shape) < margin_) {
        return true;
      }
    }
  }
  return false;
}

ConfigurationValidator::ConfigurationValidator(
  JointBounds bounds, std::shared_ptr<const CollisionChecker> checker, double resolution)
: bounds_(std::move(bounds)), checker_(std::move(checker)), resolution_(resolution)
{
  if (bounds_.lower.size() != kDof || bounds_.upper.size() != kDof) {
    throw std::invalid_argument("joint bounds must have 4 elements");
  }
  for (std::size_t i = 0; i < kDof; ++i) {
    if (!std::isfinite(bounds_.lower[i]) || !std::isfinite(bounds_.upper[i]) ||
      bounds_.lower[i] > bounds_.upper[i])
    {
      throw std::invalid_argument("joint bounds must satisfy lower <= upper");
    }
  }
  if (!checker_) {
    throw std::invalid_argument("collision checker must not be null");
  }
  if (!std::isfinite(resolution_) || resolution_ <= 0.0) {
    throw std::invalid_argument("collision_check_resolution must be > 0");
  }
}

bool ConfigurationValidator::isStateValid(const State & q) const
{
  return bounds_.contains(q) && !checker_->isColliding(q);
}

bool ConfigurationValidator::isMotionValid(const State & a, const State & b) const
{
  if (!isStateValid(a) || !isStateValid(b)) {
    return false;
  }
  double max_diff = 0.0;
  for (std::size_t i = 0; i < kDof; ++i) {
    max_diff = std::max(max_diff, std::abs(b[i] - a[i]));
  }
  const auto steps = static_cast<std::size_t>(std::ceil(max_diff / resolution_));
  State q(kDof);
  for (std::size_t k = 1; k < steps; ++k) {
    const double ratio = static_cast<double>(k) / static_cast<double>(steps);
    for (std::size_t i = 0; i < kDof; ++i) {
      q[i] = a[i] + ratio * (b[i] - a[i]);
    }
    if (!isStateValid(q)) {
      return false;
    }
  }
  return true;
}

double ConfigurationValidator::clearance(const State & q) const
{
  return checker_->evaluate(q).min_distance;
}

}  // namespace path_planner
