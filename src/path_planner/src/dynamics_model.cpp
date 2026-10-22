#include "path_planner/dynamics_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "path_planner/yaml_lite.hpp"

namespace path_planner
{
namespace
{
Matrix3 parallelAxisShift(double mass, const Vector3 & d)
{
  const double d2 = dot(d, d);
  Matrix3 out = Matrix3::zero();
  out(0, 0) = mass * (d2 - d.x * d.x);
  out(0, 1) = -mass * d.x * d.y;
  out(0, 2) = -mass * d.x * d.z;
  out(1, 0) = -mass * d.y * d.x;
  out(1, 1) = mass * (d2 - d.y * d.y);
  out(1, 2) = -mass * d.y * d.z;
  out(2, 0) = -mass * d.z * d.x;
  out(2, 1) = -mass * d.z * d.y;
  out(2, 2) = mass * (d2 - d.z * d.z);
  return out;
}

LinkInertial combineInertial(const LinkInertial & base, const Payload & payload)
{
  if (payload.mass <= 0.0) {
    return base;
  }
  const Matrix3 r_p = payload.mount_pose.rotation;
  const Vector3 c_p = payload.mount_pose.apply(payload.center_of_mass);
  const Matrix3 i_p_local = inertiaFromSix(payload.inertia);
  const Matrix3 i_p = r_p * i_p_local * transpose(r_p);

  const double total_mass = base.mass + payload.mass;
  if (total_mass <= 0.0) {
    return base;
  }
  const Vector3 com = (base.center_of_mass * base.mass + c_p * payload.mass) / total_mass;
  const Vector3 d_base = base.center_of_mass - com;
  const Vector3 d_pay = c_p - com;
  LinkInertial out;
  out.mass = total_mass;
  out.center_of_mass = com;
  out.inertia = base.inertia + parallelAxisShift(base.mass, d_base) +
    i_p + parallelAxisShift(payload.mass, d_pay);
  return out;
}

bool isValidCurve(const std::vector<SpeedTorquePoint> & curve)
{
  if (curve.size() < 2) {
    return false;
  }
  if (!std::isfinite(curve.front().speed) || std::abs(curve.front().speed) > 1e-9) {
    return false;
  }
  for (std::size_t i = 0; i < curve.size(); ++i) {
    if (!std::isfinite(curve[i].speed) || !std::isfinite(curve[i].torque) ||
      curve[i].speed < 0.0 || curve[i].torque < 0.0)
    {
      return false;
    }
    if (i > 0 && curve[i].speed <= curve[i - 1].speed) {
      return false;
    }
  }
  if (curve.front().torque <= 0.0) {
    return false;
  }
  return true;
}

bool interpolateCurve(
  const std::vector<SpeedTorquePoint> & curve,
  double abs_speed,
  double & out_torque)
{
  if (!std::isfinite(abs_speed) || abs_speed < 0.0 || curve.size() < 2) {
    return false;
  }
  // 範囲外速度は外挿せず実行不可能とする（仕様書 第11.2節）。
  if (abs_speed < curve.front().speed - 1e-9 || abs_speed > curve.back().speed + 1e-9) {
    return false;
  }
  if (abs_speed <= curve.front().speed) {
    out_torque = curve.front().torque;
    return true;
  }
  if (abs_speed >= curve.back().speed) {
    out_torque = curve.back().torque;
    return true;
  }
  for (std::size_t i = 0; i + 1 < curve.size(); ++i) {
    if (abs_speed >= curve[i].speed && abs_speed <= curve[i + 1].speed) {
      const double ds = curve[i + 1].speed - curve[i].speed;
      const double ratio = ds > 1e-12 ? (abs_speed - curve[i].speed) / ds : 0.0;
      out_torque = curve[i].torque + ratio * (curve[i + 1].torque - curve[i].torque);
      return true;
    }
  }
  return false;
}

std::vector<SpeedTorquePoint> parseCurve(const YamlNode & node, const std::string & name)
{
  if (!node.isList()) {
    throw std::invalid_argument(name + " must be a list");
  }
  std::vector<SpeedTorquePoint> out;
  for (const auto & item : node.list) {
    SpeedTorquePoint pt;
    if (item.isList()) {
      const auto vals = item.asDoubleList();
      if (vals.size() != 2) {
        throw std::invalid_argument(name + " entries must be [speed, torque]");
      }
      pt.speed = vals[0];
      pt.torque = vals[1];
    } else if (item.isMap()) {
      pt.speed = item.at("speed").asDouble();
      pt.torque = item.at("torque").asDouble();
    } else {
      throw std::invalid_argument(name + " entries must be [speed, torque] or {speed, torque}");
    }
    out.push_back(pt);
  }
  return out;
}

LinkInertial parseLinkInertial(const YamlNode & node, const std::string & name)
{
  LinkInertial link;
  link.mass = node.at("mass").asDouble();
  const auto com = node.at("center_of_mass").asDoubleList();
  if (com.size() != 3) {
    throw std::invalid_argument(name + ".center_of_mass must have 3 elements");
  }
  link.center_of_mass = {com[0], com[1], com[2]};
  const auto inertia = node.at("inertia").asDoubleList();
  if (inertia.size() != 6) {
    throw std::invalid_argument(name + ".inertia must have 6 elements [ixx,ixy,ixz,iyy,iyz,izz]");
  }
  std::array<double, 6> six{};
  for (std::size_t i = 0; i < 6; ++i) {
    six[i] = inertia[i];
  }
  link.inertia = inertiaFromSix(six);
  return link;
}
}  // namespace

bool Payload::isValid() const
{
  if (!std::isfinite(mass) || mass < 0.0 || !isFinite(center_of_mass)) {
    return false;
  }
  for (double v : inertia) {
    if (!std::isfinite(v)) {
      return false;
    }
  }
  return inertia[0] >= 0.0 && inertia[3] >= 0.0 && inertia[5] >= 0.0;
}

bool RobotDynamicsConfig::hasValidDynamics(std::string * reason) const
{
  const auto fail = [&](const std::string & msg) {
      if (reason != nullptr) {
        *reason = msg;
      }
      return false;
    };
  if (!std::isfinite(safety_factor) || safety_factor <= 0.0 || safety_factor > 1.0) {
    return fail("safety_factor must be in (0, 1]");
  }
  if (!std::isfinite(model_error_margin) || model_error_margin < 0.0) {
    return fail("model_error_margin must be >= 0");
  }
  if (!isFinite(gravity)) {
    return fail("gravity must be finite");
  }
  for (std::size_t i = 0; i < kUrdfJointCount; ++i) {
    const auto & link = links[i];
    if (!std::isfinite(link.mass) || link.mass <= 0.0) {
      return fail("link[" + std::to_string(i) + "] mass must be > 0");
    }
    if (!isFinite(link.center_of_mass)) {
      return fail("link[" + std::to_string(i) + "] center_of_mass must be finite");
    }
    if (link.inertia(0, 0) < 0.0 || link.inertia(1, 1) < 0.0 || link.inertia(2, 2) < 0.0) {
      return fail("link[" + std::to_string(i) + "] diagonal inertia must be >= 0");
    }
  }
  if (!payload.isValid()) {
    return fail("payload parameters are invalid");
  }
  if (!std::isfinite(theta1.continuous_torque) || theta1.continuous_torque <= 0.0) {
    return fail("theta1.continuous_torque must be > 0");
  }
  if (!std::isfinite(theta1.peak_torque) || theta1.peak_torque <= 0.0) {
    return fail("theta1.peak_torque must be > 0");
  }
  if (!std::isfinite(theta1.gear_ratio) || theta1.gear_ratio <= 0.0) {
    return fail("theta1.gear_ratio must be > 0");
  }
  if (!std::isfinite(theta1.efficiency) || theta1.efficiency <= 0.0 || theta1.efficiency > 1.0) {
    return fail("theta1.efficiency must be in (0, 1]");
  }
  if (!isValidCurve(theta1.positive_speed_torque_curve)) {
    return fail("theta1.positive_speed_torque_curve is unset or invalid");
  }
  if (!isValidCurve(theta1.negative_speed_torque_curve)) {
    return fail("theta1.negative_speed_torque_curve is unset or invalid");
  }
  if (!theta1.braking_speed_torque_curve.empty() &&
    !isValidCurve(theta1.braking_speed_torque_curve))
  {
    return fail("theta1.braking_speed_torque_curve is invalid");
  }
  return true;
}

bool RobotDynamicsConfig::isCompleteForHardware(std::string * reason) const
{
  const auto fail = [&](const std::string & msg) {
      if (reason != nullptr) {
        *reason = msg;
      }
      return false;
    };
  if (source.empty() || source == "UNSET") {
    return fail("robot_dynamics.source is UNSET");
  }
  if (measured_at.empty() || measured_at == "UNSET") {
    return fail("robot_dynamics.measured_at is UNSET");
  }
  return hasValidDynamics(reason);
}

RobotDynamicsConfig loadRobotDynamicsConfig(const YamlNode & root)
{
  const YamlNode & node = root.has("robot_dynamics") ? root.at("robot_dynamics") : root;
  RobotDynamicsConfig cfg;
  if (node.has("source")) {
    cfg.source = node.at("source").asString();
  }
  if (node.has("measured_at")) {
    cfg.measured_at = node.at("measured_at").asString();
  }
  if (node.has("safety_factor")) {
    cfg.safety_factor = node.at("safety_factor").asDouble();
  }
  if (node.has("model_error_margin")) {
    cfg.model_error_margin = node.at("model_error_margin").asDouble();
  }
  if (node.has("gravity")) {
    const auto g = node.at("gravity").asDoubleList();
    if (g.size() != 3) {
      throw std::invalid_argument("gravity must have 3 elements");
    }
    cfg.gravity = {g[0], g[1], g[2]};
  }
  if (node.has("kinematic_chain")) {
    cfg.chain = loadKinematicChain(node.at("kinematic_chain"));
  }
  if (node.has("links")) {
    const YamlNode & links = node.at("links");
    if (!links.isList() || links.list.size() != kUrdfJointCount) {
      throw std::invalid_argument("robot_dynamics.links must have 5 entries");
    }
    for (std::size_t i = 0; i < kUrdfJointCount; ++i) {
      cfg.links[i] = parseLinkInertial(links.list[i], "links[" + std::to_string(i) + "]");
    }
  }
  if (node.has("friction")) {
    const YamlNode & fr = node.at("friction");
    if (!fr.isList() || fr.list.size() != kDof) {
      throw std::invalid_argument("robot_dynamics.friction must have 4 entries");
    }
    for (std::size_t i = 0; i < kDof; ++i) {
      const YamlNode & item = fr.list[i];
      cfg.friction[i].coulomb_positive = item.at("coulomb_positive").asDouble();
      cfg.friction[i].coulomb_negative = item.at("coulomb_negative").asDouble();
      cfg.friction[i].viscous = item.at("viscous").asDouble();
    }
  }
  if (node.has("joint_peak_torque")) {
    const auto peaks = node.at("joint_peak_torque").asDoubleList();
    if (peaks.size() != kDof) {
      throw std::invalid_argument("joint_peak_torque must have 4 elements");
    }
    for (std::size_t i = 0; i < kDof; ++i) {
      cfg.joint_peak_torque[i] = peaks[i];
    }
  }
  if (node.has("theta1")) {
    const YamlNode & t1 = node.at("theta1");
    if (t1.has("continuous_torque")) {
      cfg.theta1.continuous_torque = t1.at("continuous_torque").asDouble();
    }
    if (t1.has("peak_torque")) {
      cfg.theta1.peak_torque = t1.at("peak_torque").asDouble();
    }
    if (t1.has("gear_ratio")) {
      cfg.theta1.gear_ratio = t1.at("gear_ratio").asDouble();
    }
    if (t1.has("efficiency")) {
      cfg.theta1.efficiency = t1.at("efficiency").asDouble();
    }
    if (t1.has("curve_in_motor_side")) {
      cfg.theta1.curve_in_motor_side = t1.at("curve_in_motor_side").asBool();
    }
    if (t1.has("positive_speed_torque_curve")) {
      cfg.theta1.positive_speed_torque_curve =
        parseCurve(t1.at("positive_speed_torque_curve"), "positive_speed_torque_curve");
    }
    if (t1.has("negative_speed_torque_curve")) {
      cfg.theta1.negative_speed_torque_curve =
        parseCurve(t1.at("negative_speed_torque_curve"), "negative_speed_torque_curve");
    }
    if (t1.has("braking_speed_torque_curve")) {
      cfg.theta1.braking_speed_torque_curve =
        parseCurve(t1.at("braking_speed_torque_curve"), "braking_speed_torque_curve");
    }
  }
  if (node.has("payload")) {
    const YamlNode & p = node.at("payload");
    cfg.payload.mass = p.at("mass").asDouble();
    const auto com = p.at("center_of_mass").asDoubleList();
    if (com.size() != 3) {
      throw std::invalid_argument("payload.center_of_mass must have 3 elements");
    }
    cfg.payload.center_of_mass = {com[0], com[1], com[2]};
    const auto inertia = p.at("inertia").asDoubleList();
    if (inertia.size() != 6) {
      throw std::invalid_argument("payload.inertia must have 6 elements");
    }
    for (std::size_t i = 0; i < 6; ++i) {
      cfg.payload.inertia[i] = inertia[i];
    }
  }
  return cfg;
}

RneaDynamicsModel::RneaDynamicsModel(RobotDynamicsConfig config, std::uint64_t version)
: config_(std::move(config)), combined_links_(config_.links), version_(version)
{
  validateKinematicChain(config_.chain);
  if (!config_.payload.isValid()) {
    throw std::invalid_argument("invalid payload in RneaDynamicsModel");
  }
  combined_links_[kUrdfJointCount - 1] =
    combineInertial(config_.links[kUrdfJointCount - 1], config_.payload);
}

RneaDynamicsModel RneaDynamicsModel::withPayload(const Payload & payload) const
{
  if (!payload.isValid()) {
    throw std::invalid_argument("invalid payload passed to withPayload");
  }
  RobotDynamicsConfig updated = config_;
  updated.payload = payload;
  return RneaDynamicsModel(std::move(updated), version_ + 1);
}

State RneaDynamicsModel::frictionTorque(const State & qd) const
{
  if (qd.size() != kDof) {
    throw std::invalid_argument("frictionTorque requires 4D velocity");
  }
  State tau(kDof, 0.0);
  // 速度 0 近傍での符号反転を滑らかにするため小閾値で区分線形化する。
  constexpr double kVelThreshold = 1e-3;
  for (std::size_t j = 0; j < kDof; ++j) {
    const auto & f = config_.friction[j];
    const double v = qd[j];
    double coulomb = 0.0;
    if (v >= kVelThreshold) {
      coulomb = f.coulomb_positive;
    } else if (v <= -kVelThreshold) {
      coulomb = -f.coulomb_negative;
    } else if (v > 0.0) {
      coulomb = (v / kVelThreshold) * f.coulomb_positive;
    } else if (v < 0.0) {
      coulomb = (v / kVelThreshold) * f.coulomb_negative;
    }
    tau[j] = coulomb + f.viscous * v;
  }
  return tau;
}

State RneaDynamicsModel::rnea(
  const State & q, const State & qd, const State & qdd,
  bool include_gravity, bool include_friction) const
{
  if (!isFiniteState(q) || !isFiniteState(qd) || !isFiniteState(qdd)) {
    throw std::invalid_argument("RNEA requires finite 4D q, qd, qdd");
  }

  const UrdfJointArray q_u = toUrdfJointPositions(q);
  const UrdfJointArray qd_u = toUrdfJointRates(qd);
  const UrdfJointArray qdd_u = toUrdfJointRates(qdd);

  // 각リンクの親からの変換 R_parent_to_i (親座標系での子基底), p_parent_to_i (親座標系での子原点)
  std::array<Matrix3, kUrdfJointCount> r_parent_to_child{};
  std::array<Vector3, kUrdfJointCount> p_parent_to_child{};
  std::array<Vector3, kUrdfJointCount> z_axis{};
  for (std::size_t i = 0; i < kUrdfJointCount; ++i) {
    const UrdfJoint & joint = config_.chain.joints[i];
    const Matrix3 r_origin = rpyToMatrix(
      joint.origin_rpy.x, joint.origin_rpy.y, joint.origin_rpy.z);
    const Matrix3 r_joint = axisAngle(joint.axis, q_u[i]);
    r_parent_to_child[i] = r_origin * r_joint;
    p_parent_to_child[i] = joint.origin_xyz;
    z_axis[i] = joint.axis;
  }

  // Forward recursion (i = 0..4 corresponds to link 1..5 in child frame)
  std::array<Vector3, kUrdfJointCount> omega{};
  std::array<Vector3, kUrdfJointCount> omega_dot{};
  std::array<Vector3, kUrdfJointCount> lin_acc{};
  std::array<Vector3, kUrdfJointCount> force_inertial{};
  std::array<Vector3, kUrdfJointCount> moment_inertial{};

  Vector3 omega_prev{0.0, 0.0, 0.0};
  Vector3 omega_dot_prev{0.0, 0.0, 0.0};
  // ベース座標系での上向き固有加速度 a_0 = R_world_to_base^T * (-gravity)
  Vector3 acc_prev{0.0, 0.0, 0.0};
  if (include_gravity) {
    acc_prev = transpose(config_.chain.world_to_base.rotation) * (-config_.gravity);
  }

  for (std::size_t i = 0; i < kUrdfJointCount; ++i) {
    const Matrix3 r_t = transpose(r_parent_to_child[i]);
    const Vector3 z_i = z_axis[i];
    const Vector3 w_parent_in_i = r_t * omega_prev;
    omega[i] = w_parent_in_i + z_i * qd_u[i];
    omega_dot[i] = (r_t * omega_dot_prev) + cross(w_parent_in_i, z_i * qd_u[i]) + z_i * qdd_u[i];

    const Vector3 p_i = p_parent_to_child[i];
    const Vector3 acc_parent_at_joint =
      acc_prev + cross(omega_dot_prev, p_i) + cross(omega_prev, cross(omega_prev, p_i));
    lin_acc[i] = r_t * acc_parent_at_joint;

    const LinkInertial & link = combined_links_[i];
    const Vector3 com_acc =
      lin_acc[i] + cross(omega_dot[i], link.center_of_mass) +
      cross(omega[i], cross(omega[i], link.center_of_mass));
    force_inertial[i] = com_acc * link.mass;
    moment_inertial[i] =
      (link.inertia * omega_dot[i]) + cross(omega[i], link.inertia * omega[i]);

    omega_prev = omega[i];
    omega_dot_prev = omega_dot[i];
    acc_prev = lin_acc[i];
  }

  // Backward recursion (i = 4..0)
  UrdfJointArray tau_u{};
  Vector3 f_child{0.0, 0.0, 0.0};
  Vector3 n_child{0.0, 0.0, 0.0};
  for (std::size_t idx = kUrdfJointCount; idx-- > 0;) {
    Vector3 f_from_child{0.0, 0.0, 0.0};
    Vector3 n_from_child{0.0, 0.0, 0.0};
    if (idx + 1 < kUrdfJointCount) {
      f_from_child = r_parent_to_child[idx + 1] * f_child;
      n_from_child = (r_parent_to_child[idx + 1] * n_child) +
        cross(p_parent_to_child[idx + 1], f_from_child);
    }
    const Vector3 f_i = force_inertial[idx] + f_from_child;
    const Vector3 n_i = moment_inertial[idx] +
      cross(combined_links_[idx].center_of_mass, force_inertial[idx]) + n_from_child;
    tau_u[idx] = dot(n_i, z_axis[idx]);
    f_child = f_i;
    n_child = n_i;
  }

  // 仮想仕事の原理 tau_theta = J^T * tau_u
  const CouplingMatrix & j_mat = couplingMatrix();
  State tau_theta(kDof, 0.0);
  for (std::size_t c = 0; c < kDof; ++c) {
    double s = 0.0;
    for (std::size_t r = 0; r < kUrdfJointCount; ++r) {
      s += j_mat[r][c] * tau_u[r];
    }
    tau_theta[c] = s;
  }

  if (include_friction) {
    const State tau_f = frictionTorque(qd);
    for (std::size_t j = 0; j < kDof; ++j) {
      tau_theta[j] += tau_f[j];
    }
  }
  return tau_theta;
}

State RneaDynamicsModel::inverseDynamics(
  const State & q, const State & qd, const State & qdd) const
{
  return rnea(q, qd, qdd, true, true);
}

State RneaDynamicsModel::inertialTorque(const State & q, const State & v) const
{
  const State zero(kDof, 0.0);
  return rnea(q, zero, v, false, false);
}

State RneaDynamicsModel::coriolisTorque(const State & q, const State & qd) const
{
  const State zero(kDof, 0.0);
  return rnea(q, qd, zero, false, false);
}

State RneaDynamicsModel::gravityTorque(const State & q) const
{
  const State zero(kDof, 0.0);
  return rnea(q, zero, zero, true, false);
}

double RneaDynamicsModel::effectiveInertia(std::size_t joint, const State & q) const
{
  if (joint >= kDof) {
    throw std::out_of_range("joint index out of range in effectiveInertia");
  }
  State unit(kDof, 0.0);
  unit[joint] = 1.0;
  const State col = inertialTorque(q, unit);
  return col[joint];
}

double RneaDynamicsModel::theta1ContinuousAllowedTorque() const
{
  if (config_.safety_factor <= 0.0 || config_.theta1.continuous_torque <= 0.0) {
    return 0.0;
  }
  double tau = config_.theta1.continuous_torque;
  if (config_.theta1.curve_in_motor_side) {
    tau *= config_.theta1.gear_ratio * config_.theta1.efficiency;
  }
  return std::max(0.0, tau * config_.safety_factor - config_.model_error_margin);
}

double RneaDynamicsModel::theta1PeakAllowedTorque() const
{
  if (config_.safety_factor <= 0.0 || config_.theta1.peak_torque <= 0.0) {
    return 0.0;
  }
  double tau = config_.theta1.peak_torque;
  if (config_.theta1.curve_in_motor_side) {
    tau *= config_.theta1.gear_ratio * config_.theta1.efficiency;
  }
  return std::max(0.0, tau * config_.safety_factor - config_.model_error_margin);
}

TorqueBounds RneaDynamicsModel::torqueBounds(std::size_t joint, double joint_velocity) const
{
  TorqueBounds bounds;
  if (joint >= kDof || !std::isfinite(joint_velocity)) {
    return bounds;
  }
  const double sf = config_.safety_factor;
  if (!std::isfinite(sf) || sf <= 0.0 || sf > 1.0) {
    return bounds;
  }

  if (joint != 0) {
    // 第2～4軸: 設定されている場合は安全率を掛けた定格上限を返し、未設定時は広帯域とする。
    const double peak = config_.joint_peak_torque[joint];
    if (peak > 0.0 && std::isfinite(peak)) {
      const double allowed = std::max(0.0, peak * sf - config_.model_error_margin);
      bounds.feasible = allowed > 0.0;
      bounds.max_torque = allowed;
      bounds.min_torque = -allowed;
    } else {
      bounds.feasible = true;
      bounds.max_torque = 1e6;
      bounds.min_torque = -1e6;
    }
    return bounds;
  }

  // 第1軸: 速度―トルク曲線を補間する（範囲外は外挿せず feasible = false）。
  const auto & t1 = config_.theta1;
  const double scale_drive =
    t1.curve_in_motor_side ? (t1.gear_ratio * t1.efficiency) : 1.0;
  const double scale_brake =
    t1.curve_in_motor_side ? (t1.gear_ratio / std::max(1e-6, t1.efficiency)) : 1.0;
  const double query_speed =
    t1.curve_in_motor_side ? std::abs(joint_velocity) * t1.gear_ratio : std::abs(joint_velocity);

  double pos_curve = 0.0;
  double neg_curve = 0.0;
  if (!interpolateCurve(t1.positive_speed_torque_curve, query_speed, pos_curve) ||
    !interpolateCurve(t1.negative_speed_torque_curve, query_speed, neg_curve))
  {
    bounds.feasible = false;
    return bounds;
  }

  double brake_curve = 0.0;
  const bool has_brake = !t1.braking_speed_torque_curve.empty();
  if (has_brake && !interpolateCurve(t1.braking_speed_torque_curve, query_speed, brake_curve)) {
    bounds.feasible = false;
    return bounds;
  }

  const double peak_joint =
    t1.curve_in_motor_side ? (t1.peak_torque * scale_drive) : t1.peak_torque;

  double max_rated = 0.0;
  double min_rated = 0.0;
  if (joint_velocity >= 0.0) {
    // 正転中: 正トルクは駆動側、負トルクは回生（制動）側
    max_rated = std::min(peak_joint, pos_curve * scale_drive);
    const double neg_base = has_brake ? (brake_curve * scale_brake) : (neg_curve * scale_drive);
    min_rated = -std::min(peak_joint, neg_base);
  } else {
    // 逆転中: 負トルクは逆転駆動側、正トルクは回生（制動）側
    const double pos_base = has_brake ? (brake_curve * scale_brake) : (pos_curve * scale_drive);
    max_rated = std::min(peak_joint, pos_base);
    min_rated = -std::min(peak_joint, neg_curve * scale_drive);
  }

  const double max_allowed = max_rated * sf - config_.model_error_margin;
  const double min_allowed = min_rated * sf + config_.model_error_margin;
  if (max_allowed < min_allowed || max_allowed <= 0.0 || min_allowed >= 0.0) {
    bounds.feasible = false;
    return bounds;
  }
  bounds.feasible = true;
  bounds.max_torque = max_allowed;
  bounds.min_torque = min_allowed;
  return bounds;
}

}  // namespace path_planner
