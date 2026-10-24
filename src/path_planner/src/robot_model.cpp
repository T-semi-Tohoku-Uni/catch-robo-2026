#include "path_planner/robot_model.hpp"

#include <cmath>
#include <set>
#include <stdexcept>

#include "path_planner/yaml_lite.hpp"

namespace path_planner
{
namespace
{
Vector3 toVector3(const YamlNode & node, const std::string & what)
{
  const std::vector<double> v = node.asDoubleList();
  if (v.size() != 3) {
    throw std::invalid_argument(what + " must have 3 elements");
  }
  return {v[0], v[1], v[2]};
}

void requireTheta(const State & theta)
{
  if (theta.size() != kDof) {
    throw std::invalid_argument("state must have 4 elements [theta1..theta4]");
  }
}
}  // namespace

const char * toString(FailureCode code)
{
  switch (code) {
    case FailureCode::kNone: return "kNone";
    case FailureCode::kInvalidInput: return "kInvalidInput";
    case FailureCode::kMissingDynamics: return "kMissingDynamics";
    case FailureCode::kPlanningFailed: return "kPlanningFailed";
    case FailureCode::kSmoothingFailed: return "kSmoothingFailed";
    case FailureCode::kTimeParameterizationInfeasible: return "kTimeParameterizationInfeasible";
    case FailureCode::kJerkLimitInfeasible: return "kJerkLimitInfeasible";
    case FailureCode::kCollisionDetected: return "kCollisionDetected";
    case FailureCode::kJointLimitViolation: return "kJointLimitViolation";
    case FailureCode::kTorqueLimitViolation: return "kTorqueLimitViolation";
    case FailureCode::kThermalLimitViolation: return "kThermalLimitViolation";
    case FailureCode::kNumericalError: return "kNumericalError";
  }
  return "kUnknown";
}

bool isFiniteState(const State & q)
{
  if (q.size() != kDof) {
    return false;
  }
  for (double v : q) {
    if (!std::isfinite(v)) {
      return false;
    }
  }
  return true;
}

const CouplingMatrix & couplingMatrix()
{
  // q_u = [theta1, theta2, theta3 - theta2, -theta3, 2*theta1 + theta4 - pi/2]
  static const CouplingMatrix kMatrix{{
    {{1.0, 0.0, 0.0, 0.0}},
    {{0.0, 1.0, 0.0, 0.0}},
    {{0.0, -1.0, 1.0, 0.0}},
    {{0.0, 0.0, -1.0, 0.0}},
    {{2.0, 0.0, 0.0, 1.0}}}};
  return kMatrix;
}

const UrdfJointArray & couplingOffset()
{
  static const UrdfJointArray kOffset{0.0, 0.0, 0.0, 0.0, -kPi / 2.0};
  return kOffset;
}

UrdfJointArray toUrdfJointRates(const State & theta_rate)
{
  requireTheta(theta_rate);
  const CouplingMatrix & j = couplingMatrix();
  UrdfJointArray out{};
  for (std::size_t r = 0; r < kUrdfJointCount; ++r) {
    double s = 0.0;
    for (std::size_t c = 0; c < kDof; ++c) {
      s += j[r][c] * theta_rate[c];
    }
    out[r] = s;
  }
  return out;
}

UrdfJointArray toUrdfJointPositions(const State & theta)
{
  UrdfJointArray out = toUrdfJointRates(theta);
  const UrdfJointArray & offset = couplingOffset();
  for (std::size_t r = 0; r < kUrdfJointCount; ++r) {
    out[r] += offset[r];
  }
  return out;
}

KinematicChain defaultKinematicChain()
{
  KinematicChain chain;
  chain.world_to_base = makeTransform({0.675, -0.190, 0.0}, {0.0, 0.0, 0.0});
  chain.joints[0] = {"base_yaw_joint", "base_yaw_link", {0.0, 0.0, 0.0}, {0.0, 0.0, kPi / 2.0},
    {0.0, 0.0, -1.0}};
  chain.joints[1] = {"shoulder_joint", "upper_arm_link", {-0.040, 0.0, 0.090}, {0.0, 0.0, 0.0},
    {0.0, 1.0, 0.0}};
  chain.joints[2] = {"elbow_joint", "forearm_link", {0.0, 0.0, 0.480}, {0.0, 0.0, 0.0},
    {0.0, 1.0, 0.0}};
  chain.joints[3] = {"wrist_level_joint", "wrist_link", {0.0, 0.0, 0.480}, {0.0, 0.0, 0.0},
    {0.0, 1.0, 0.0}};
  chain.joints[4] = {"flange_yaw_joint", "flange_link", {0.040, 0.0, 0.0}, {0.0, 0.0, 0.0},
    {0.0, 0.0, 1.0}};
  return chain;
}

void validateKinematicChain(const KinematicChain & chain)
{
  std::set<std::string> names{chain.base_link};
  for (const auto & joint : chain.joints) {
    const double n = norm(joint.axis);
    if (!std::isfinite(n) || std::abs(n - 1.0) > 1e-9) {
      throw std::invalid_argument("joint '" + joint.name + "' axis must be a unit vector");
    }
    if (!isFinite(joint.origin_xyz) || !isFinite(joint.origin_rpy)) {
      throw std::invalid_argument("joint '" + joint.name + "' origin must be finite");
    }
    if (joint.child_link.empty() || !names.insert(joint.child_link).second) {
      throw std::invalid_argument("joint '" + joint.name + "' child link name must be unique");
    }
  }
}

KinematicChain loadKinematicChain(const YamlNode & node)
{
  KinematicChain chain;
  chain.world_frame = node.at("world_frame").asString();
  chain.base_link = node.at("base_link").asString();
  chain.world_to_base = makeTransform(
    toVector3(node.at("world_to_base_xyz"), "world_to_base_xyz"),
    toVector3(node.at("world_to_base_rpy"), "world_to_base_rpy"));
  const YamlNode & joints = node.at("joints");
  if (!joints.isList() || joints.list.size() != kUrdfJointCount) {
    throw std::invalid_argument("kinematic_chain.joints must list exactly 5 revolute joints");
  }
  for (std::size_t i = 0; i < kUrdfJointCount; ++i) {
    const YamlNode & j = joints.list[i];
    if (j.has("type") && j.at("type").asString() != "revolute" &&
      j.at("type").asString() != "continuous")
    {
      throw std::invalid_argument("only revolute joints are supported");
    }
    UrdfJoint & out = chain.joints[i];
    out.name = j.at("name").asString();
    out.child_link = j.at("child_link").asString();
    out.origin_xyz = toVector3(j.at("origin_xyz"), out.name + ".origin_xyz");
    out.origin_rpy = toVector3(j.at("origin_rpy"), out.name + ".origin_rpy");
    out.axis = toVector3(j.at("axis"), out.name + ".axis");
    const double n = norm(out.axis);
    if (n > 0.0 && std::isfinite(n)) {
      out.axis = out.axis / n;
    }
  }
  validateKinematicChain(chain);
  return chain;
}

std::array<Transform, kLinkCount> forwardKinematics(
  const KinematicChain & chain, const UrdfJointArray & q_urdf)
{
  std::array<Transform, kLinkCount> frames{};
  frames[0] = chain.world_to_base;
  for (std::size_t i = 0; i < kUrdfJointCount; ++i) {
    const UrdfJoint & joint = chain.joints[i];
    const Transform origin = makeTransform(joint.origin_xyz, joint.origin_rpy);
    const Transform rotation{axisAngle(joint.axis, q_urdf[i]), {0.0, 0.0, 0.0}};
    frames[i + 1] = frames[i] * origin * rotation;
  }
  return frames;
}

int findLinkIndex(const KinematicChain & chain, const std::string & link_name)
{
  if (link_name == chain.base_link) {
    return 0;
  }
  for (std::size_t i = 0; i < kUrdfJointCount; ++i) {
    if (chain.joints[i].child_link == link_name) {
      return static_cast<int>(i + 1);
    }
  }
  return -1;
}

double normalizeWristAngle(double wrist_rad)
{
  const double full_turn = 2.0 * kPi;
  double normalized = wrist_rad;
  if (normalized > 0.0 || normalized < -full_turn) {
    normalized = std::fmod(normalized, full_turn);
    if (normalized > 0.0) {
      normalized -= full_turn;
    }
  }
  return normalized;
}

CartesianPose forwardKinematicsPose(
  const KinematicChain & chain,
  const State & theta)
{
  requireTheta(theta);
  const auto q_urdf = toUrdfJointPositions(theta);
  const auto frames = forwardKinematics(chain, q_urdf);
  const Transform & flange = frames[kUrdfJointCount];
  CartesianPose pose;
  pose.x_mm = flange.translation.x * 1000.0;
  pose.y_mm = flange.translation.y * 1000.0;
  pose.z_mm = flange.translation.z * 1000.0;
  pose.phi_rad = theta[0] + theta[3];
  return pose;
}

bool inverseKinematics(
  const KinematicChain & chain,
  const CartesianPose & target,
  State & out_theta,
  bool normalize_wrist)
{
  if (!std::isfinite(target.x_mm) || !std::isfinite(target.y_mm) ||
    !std::isfinite(target.z_mm) || !std::isfinite(target.phi_rad))
  {
    return false;
  }

  const double base_x_m = chain.world_to_base.translation.x;
  const double base_y_m = chain.world_to_base.translation.y;
  const double base_z_m = chain.world_to_base.translation.z;

  const double shoulder_offset_x_m = chain.joints[1].origin_xyz.x;  // -0.040
  const double shoulder_z_m = base_z_m + chain.joints[1].origin_xyz.z;  // 0.090
  const double l1_m = chain.joints[2].origin_xyz.z;  // 0.480
  const double l2_m = chain.joints[3].origin_xyz.z;  // 0.480
  const double flange_offset_x_m = chain.joints[4].origin_xyz.x;  // +0.040

  if (l1_m <= 0.0 || l2_m <= 0.0) {
    return false;
  }

  const double dx_m = target.x_mm / 1000.0 - base_x_m;
  const double dy_m = target.y_mm / 1000.0 - base_y_m;
  const double radial_m = std::hypot(dx_m, dy_m) - (shoulder_offset_x_m + flange_offset_x_m);
  const double dz_m = target.z_mm / 1000.0 - shoulder_z_m;

  const double theta1 = std::atan2(dx_m, dy_m);
  const double dist_sq = radial_m * radial_m + dz_m * dz_m;
  const double cos_diff = (dist_sq - l1_m * l1_m - l2_m * l2_m) / (2.0 * l1_m * l2_m);
  if (!std::isfinite(cos_diff) || cos_diff < -1.0 - 1e-6 || cos_diff > 1.0 + 1e-6) {
    return false;
  }
  const double clamped_cos = std::max(-1.0, std::min(1.0, cos_diff));
  const double delta = std::acos(clamped_cos);  // theta3 - theta2 >= 0 (elbow-up branch)

  const double k1 = l1_m + l2_m * std::cos(delta);
  const double k2 = l2_m * std::sin(delta);
  const double alpha = std::atan2(radial_m, dz_m);
  const double gamma = std::atan2(k2, k1);

  const double theta2 = alpha - gamma;
  const double theta3 = theta2 + delta;
  double theta4 = target.phi_rad - theta1;
  if (normalize_wrist) {
    theta4 = normalizeWristAngle(theta4);
  }

  out_theta = {theta1, theta2, theta3, theta4};
  return isFiniteState(out_theta);
}

bool JointBounds::contains(const State & q, double tolerance) const
{
  if (q.size() != kDof || lower.size() != kDof || upper.size() != kDof) {
    return false;
  }
  for (std::size_t i = 0; i < kDof; ++i) {
    if (!std::isfinite(q[i]) || q[i] < lower[i] - tolerance || q[i] > upper[i] + tolerance) {
      return false;
    }
  }
  return true;
}

}  // namespace path_planner
