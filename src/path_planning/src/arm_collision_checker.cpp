#include "path_planning/arm_collision_checker.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/cylinder.h>
#include <fcl/geometry/shape/sphere.h>
#include <fcl/narrowphase/collision.h>
#include <urdf_parser/urdf_parser.h>

namespace path_planning {
namespace {
const std::array<std::string, 5> joint_names{
  "base_yaw_joint", "shoulder_joint", "elbow_joint", "wrist_level_joint", "flange_yaw_joint"};
const std::array<std::string, 5> body_names{
  "base_yaw_link", "upper_arm_link", "forearm_link", "wrist_link", "flange_link"};

LinkPair ordered(LinkPair pair) {
  if (pair.second < pair.first) { std::swap(pair.first, pair.second); }
  return pair;
}
fcl::Transform3d transform(const urdf::Pose & pose) {
  fcl::Transform3d result = fcl::Transform3d::Identity();
  Eigen::Quaterniond rotation(pose.rotation.w, pose.rotation.x,
    pose.rotation.y, pose.rotation.z);
  if (!rotation.coeffs().allFinite() || rotation.norm() < 1e-12) {
    throw std::invalid_argument("Invalid URDF rotation");
  }
  result.linear() = rotation.normalized().toRotationMatrix();
  result.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  if (!result.matrix().allFinite()) { throw std::invalid_argument("Invalid URDF origin"); }
  return result;
}
void positive(double value) {
  if (!std::isfinite(value) || value <= 0) {
    throw std::invalid_argument("Collision dimensions must be finite and positive");
  }
}
std::shared_ptr<fcl::CollisionGeometryd> geometry(const urdf::Geometry & shape) {
  switch (shape.type) {
    case urdf::Geometry::BOX: {
      const auto & box = static_cast<const urdf::Box &>(shape);
      positive(box.dim.x); positive(box.dim.y); positive(box.dim.z);
      return std::make_shared<fcl::Boxd>(box.dim.x, box.dim.y, box.dim.z);
    }
    case urdf::Geometry::CYLINDER: {
      const auto & cylinder = static_cast<const urdf::Cylinder &>(shape);
      positive(cylinder.radius); positive(cylinder.length);
      return std::make_shared<fcl::Cylinderd>(cylinder.radius, cylinder.length);
    }
    case urdf::Geometry::SPHERE: {
      const auto & sphere = static_cast<const urdf::Sphere &>(shape);
      positive(sphere.radius);
      return std::make_shared<fcl::Sphered>(sphere.radius);
    }
    default:
      throw std::invalid_argument("Unsupported URDF collision geometry (mesh is not supported)");
  }
}
}  // namespace

struct ArmCollisionChecker::Impl {
  struct Link {
    std::string name;
    std::size_t parent;
    int angle_index;
    fcl::Transform3d origin;
    Eigen::Vector3d axis;
    bool limited{false};
    double lower{0}, upper{0};
  };
  struct Body {
    std::size_t link;
    fcl::Transform3d origin;
    std::shared_ptr<fcl::CollisionGeometryd> shape;
  };
  std::vector<Link> links;
  std::vector<Body> bodies;
  std::vector<std::pair<std::size_t, std::size_t>> pairs;
  std::set<std::string> movable_joints;

  std::vector<fcl::Transform3d> frames(const State & q) const;

  void append(const urdf::LinkConstSharedPtr & link, std::size_t parent) {
    Link entry{link->name, parent, -1, fcl::Transform3d::Identity(), Eigen::Vector3d::UnitZ()};
    if (link->parent_joint) {
      const auto & joint = *link->parent_joint;
      entry.origin = transform(joint.parent_to_joint_origin_transform);
      if (joint.mimic) { throw std::invalid_argument("URDF mimic joints are not supported"); }
      if (joint.type != urdf::Joint::FIXED) {
        auto name = std::find(joint_names.begin(), joint_names.end(), joint.name);
        if (name == joint_names.end() ||
          (joint.type != urdf::Joint::CONTINUOUS && joint.type != urdf::Joint::REVOLUTE)) {
          throw std::invalid_argument("Unsupported movable joint: " + joint.name);
        }
        entry.angle_index = static_cast<int>(std::distance(joint_names.begin(), name));
        entry.axis = Eigen::Vector3d(joint.axis.x, joint.axis.y, joint.axis.z);
        if (!entry.axis.allFinite() || entry.axis.norm() < 1e-12) {
          throw std::invalid_argument("Invalid joint axis: " + joint.name);
        }
        entry.axis.normalize();
        if (joint.type == urdf::Joint::REVOLUTE) {
          if (!joint.limits || !std::isfinite(joint.limits->lower) ||
            !std::isfinite(joint.limits->upper) || joint.limits->lower > joint.limits->upper) {
            throw std::invalid_argument("Invalid revolute limits: " + joint.name);
          }
          entry.limited = true;
          entry.lower = joint.limits->lower; entry.upper = joint.limits->upper;
        }
        movable_joints.insert(joint.name);
      }
    }
    const auto index = links.size();
    links.push_back(entry);
    for (const auto & collision : link->collision_array) {
      if (!collision || !collision->geometry) {
        throw std::invalid_argument("Missing collision geometry on " + link->name);
      }
      bodies.push_back({index, transform(collision->origin), geometry(*collision->geometry)});
    }
    for (const auto & child : link->child_links) { append(child, index); }
  }
};

std::vector<LinkPair> ArmCollisionChecker::defaultCheckedPairs() {
  // Check only the hand against the upper arm by default.
  return {{"flange_link", "upper_arm_link"}};
}

ArmCollisionChecker::ArmCollisionChecker(const std::string & xml,
  const std::vector<LinkPair> & checked_pairs) : impl_(std::make_unique<Impl>()) {
  auto model = urdf::parseURDF(xml);
  if (!model || !model->getRoot()) { throw std::invalid_argument("Cannot parse robot_description URDF"); }
  impl_->append(model->getRoot(), 0);
  if (impl_->movable_joints.size() != joint_names.size()) {
    throw std::invalid_argument("URDF must contain all five CatchRobo arm joints");
  }
  for (const auto & name : body_names) {
    auto link = model->getLink(name);
    if (!link || link->collision_array.empty()) {
      throw std::invalid_argument("Required arm collision geometry is missing: " + name);
    }
  }
  std::set<LinkPair> checked;
  for (const auto & pair : checked_pairs) {
    if (pair.first == pair.second || !model->getLink(pair.first) || !model->getLink(pair.second)) {
      throw std::invalid_argument("Unknown or identical links in checked collision pair");
    }
    if (model->getLink(pair.first)->collision_array.empty() ||
      model->getLink(pair.second)->collision_array.empty()) {
      throw std::invalid_argument("Checked collision pair must have geometry on both links");
    }
    checked.insert(ordered(pair));
  }
  for (std::size_t i = 0; i < impl_->bodies.size(); ++i) {
    for (std::size_t j = i + 1; j < impl_->bodies.size(); ++j) {
      auto first = impl_->bodies[i].link, second = impl_->bodies[j].link;
      if (first != second && checked.count(ordered(
        {impl_->links[first].name, impl_->links[second].name})) != 0) {
        impl_->pairs.emplace_back(i, j);
      }
    }
  }
}
ArmCollisionChecker::~ArmCollisionChecker() = default;

std::vector<fcl::Transform3d> ArmCollisionChecker::Impl::frames(const State & q) const {
  if (q.size() != 4 || !std::all_of(q.begin(), q.end(), [](double v) { return std::isfinite(v); })) {
    throw std::invalid_argument("Expected four finite absolute joint angles");
  }
  const std::array<double, 5> angles{q[0], q[1], q[2]-q[1], -q[2],
    2*q[0]+q[3]-std::acos(-1.0)/2};
  if (!std::all_of(angles.begin(), angles.end(), [](double v) { return std::isfinite(v); })) {
    throw std::invalid_argument("Converted joint angles overflow");
  }
  std::vector<fcl::Transform3d> frames;
  frames.reserve(links.size());
  for (const auto & link : links) {
    auto relative = link.origin;
    if (link.angle_index >= 0) {
      const auto angle = angles[link.angle_index];
      if (link.limited && (angle < link.lower || angle > link.upper)) {
        throw std::invalid_argument("URDF joint limit exceeded at " + link.name);
      }
      relative.rotate(Eigen::AngleAxisd(angle, link.axis));
    }
    frames.push_back(frames.empty() ? relative : frames[link.parent] * relative);
  }
  return frames;
}

std::array<double, 3> ArmCollisionChecker::linkPosition(const State & q,
  const std::string & name) const {
  const auto frames = impl_->frames(q);
  for (std::size_t i = 0; i < impl_->links.size(); ++i) {
    if (impl_->links[i].name == name) {
      const auto position = frames[i].translation();
      return {position.x(), position.y(), position.z()};
    }
  }
  throw std::invalid_argument("Unknown link: " + name);
}

CollisionCheckResult ArmCollisionChecker::check(const State & q) const {
  std::vector<fcl::Transform3d> frames;
  try { frames = impl_->frames(q); }
  catch (const std::invalid_argument & error) { return {false, {}, error.what()}; }
  std::vector<fcl::Transform3d> poses;
  poses.reserve(impl_->bodies.size());
  for (const auto & body : impl_->bodies) { poses.push_back(frames[body.link] * body.origin); }
  for (const auto & pair : impl_->pairs) {
    const auto i = pair.first, j = pair.second;
    fcl::CollisionRequestd request;
    fcl::CollisionResultd result;
    fcl::collide(impl_->bodies[i].shape.get(), poses[i],
      impl_->bodies[j].shape.get(), poses[j], request, result);
    if (result.isCollision()) {
      LinkPair names{impl_->links[impl_->bodies[i].link].name,
        impl_->links[impl_->bodies[j].link].name};
      return {false, names, "Self collision: " + names.first + " <-> " + names.second};
    }
  }
  return {true, {}, "Collision free"};
}

bool ArmCollisionChecker::isStateValid(const State & angles) const { return check(angles).valid; }
}  // namespace path_planning
