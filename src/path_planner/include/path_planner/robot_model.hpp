#pragma once

// ロボット機構モデル: 関節変換の一元管理と順運動学（仕様書 第4節, 第7.4節）。
//
// 4自由度の状態 theta = [theta1, theta2, theta3, theta4] と、URDF (robot_arm.xacro)
// の5つの回転関節 q_u の関係は次式のみで定義し、他モジュールで再実装しない。
//
//   q_u = [theta1, theta2, theta3 - theta2, -theta3, 2*theta1 + theta4 - pi/2]
//       = J * theta + offset
//
// J は定数行列（kCouplingMatrix）であり、速度・加速度も q_u_dot = J theta_dot で写像される。
// 動力学では仮想仕事の原理により tau_theta = J^T tau_u とする（dynamics_model.hpp）。

#include <array>
#include <string>

#include "path_planner/math3d.hpp"
#include "path_planner/types.hpp"

namespace path_planner
{
class YamlNode;

constexpr std::size_t kUrdfJointCount = 5;
/// リンク数: base_link(0) + 各関節の子リンク(1..5)
constexpr std::size_t kLinkCount = kUrdfJointCount + 1;

using UrdfJointArray = std::array<double, kUrdfJointCount>;
using CouplingMatrix = std::array<std::array<double, kDof>, kUrdfJointCount>;

/// q_u = J theta + offset の J。
const CouplingMatrix & couplingMatrix();
/// q_u = J theta + offset の offset。
const UrdfJointArray & couplingOffset();

/// 位置（オフセット込み）の変換。theta は4要素であること。
UrdfJointArray toUrdfJointPositions(const State & theta);
/// 速度・加速度など微分量の変換（オフセットなし）。
UrdfJointArray toUrdfJointRates(const State & theta_rate);

/// URDF 回転関節1つ分の固定パラメータ。
struct UrdfJoint
{
  std::string name;
  std::string child_link;
  Vector3 origin_xyz;   // 親リンク座標系での関節原点
  Vector3 origin_rpy;   // 親リンク座標系での関節姿勢（URDF rpy）
  Vector3 axis{0.0, 0.0, 1.0};  // 関節座標系での単位回転軸
};

/// 直列5関節の運動学チェーン（map -> base_link -> ... -> flange_link）。
struct KinematicChain
{
  std::string world_frame{"map"};
  std::string base_link{"base_link"};
  Transform world_to_base{};  // map -> base_link の固定変換
  std::array<UrdfJoint, kUrdfJointCount> joints{};
};

/// src/arm/urdf/robot_arm.xacro と同一の既定チェーン。
KinematicChain defaultKinematicChain();

/// 4自由度・直列・回転関節の前提（関節数・単位軸・リンク名の一意性）を検証する。
/// 不正なら std::invalid_argument。
void validateKinematicChain(const KinematicChain & chain);

/// YAML の `kinematic_chain` ノードから読み込む（検証込み）。
KinematicChain loadKinematicChain(const YamlNode & node);

/// 各リンク座標系のワールド姿勢。index 0 = base_link, i = joints[i-1].child_link。
std::array<Transform, kLinkCount> forwardKinematics(
  const KinematicChain & chain, const UrdfJointArray & q_urdf);

/// リンク名からリンク番号を返す。無ければ -1。
int findLinkIndex(const KinematicChain & chain, const std::string & link_name);

/// 関節位置の上下限。
struct JointBounds
{
  State lower;
  State upper;

  bool contains(const State & q, double tolerance = 0.0) const;
};

}  // namespace path_planner
