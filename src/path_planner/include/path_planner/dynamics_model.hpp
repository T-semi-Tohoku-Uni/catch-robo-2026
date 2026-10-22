#pragma once

// 4自由度ロボットアームの動力学モデル（仕様書 第3, 10, 11.2, 14節）。
//
// - 再帰 Newton-Euler 法（RNEA）を自前実装し、仮想仕事の原理 tau_theta = J^T tau_urdf により
//   [theta1, theta2, theta3, theta4] 軸換算トルクを返す。
// - 把持物（Payload）は平行軸の定理で末端リンク（flange_link）へ剛体合成する。
// - 第1軸の速度―トルク曲線は区分線形補間とし、範囲外速度は外挿せず実行不可能とする。
// - 許容トルクには安全率 safety_factor in (0, 1] を掛ける。

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "path_planner/math3d.hpp"
#include "path_planner/robot_model.hpp"
#include "path_planner/types.hpp"

namespace path_planner
{
class YamlNode;

/// リンク1個分の慣性パラメータ（リンク座標系基準）。
struct LinkInertial
{
  double mass{0.0};
  Vector3 center_of_mass{0.0, 0.0, 0.0};
  Matrix3 inertia{Matrix3::zero()};  // 重心まわりの慣性テンソル (kg m^2)
};

/// 把持物パラメータ（末端リンク座標系基準、仕様書 第10.3節）。
struct Payload
{
  double mass{0.0};
  Vector3 center_of_mass{0.0, 0.0, 0.0};
  std::array<double, 6> inertia{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // [ixx, ixy, ixz, iyy, iyz, izz]
  Transform mount_pose{Transform::identity()};

  bool isValid() const;
};

/// 回転方向別クーロン摩擦 + 粘性摩擦（関節軸換算、N m）。
struct JointFriction
{
  double coulomb_positive{0.0};  // qd > 0 での摩擦トルク (>= 0)
  double coulomb_negative{0.0};  // qd < 0 での摩擦トルク大きさ (>= 0)
  double viscous{0.0};           // 粘性摩擦係数 (N m s / rad, >= 0)
};

/// 速度―トルク曲線の1節点（speed >= 0 [rad/s], torque > 0 [N m]）。
struct SpeedTorquePoint
{
  double speed{0.0};
  double torque{0.0};
};

/// 第1軸ドライブ・減速機・熱・速度―トルク曲線設定（仕様書 第3.1, 10.2, 11.2, 14節）。
struct Theta1DriveConfig
{
  double continuous_torque{0.0};   // 連続定格トルク（関節軸換算またはモータ軸、N m）
  double peak_torque{0.0};         // 瞬時最大トルク（N m）
  double gear_ratio{0.0};          // 減速比 (>= 1)
  double efficiency{0.0};          // 伝達効率 (0, 1]
  bool curve_in_motor_side{false}; // true のとき曲線・定格はモータ軸値で gear_ratio 等で関節換算する
  std::vector<SpeedTorquePoint> positive_speed_torque_curve;  // 正転駆動側 (qd >= 0)
  std::vector<SpeedTorquePoint> negative_speed_torque_curve;  // 逆転駆動側 (qd < 0, |qd| で指定)
  std::vector<SpeedTorquePoint> braking_speed_torque_curve;   // 回生・制動側（省略時は駆動曲線を使用）
};

struct TorqueBounds
{
  bool feasible{false};
  double min_torque{0.0};  // 許容最小トルク（負、安全率反映済み）[N m]
  double max_torque{0.0};  // 許容最大トルク（正、安全率反映済み）[N m]
};

struct RobotDynamicsConfig
{
  std::string source{"UNSET"};
  std::string measured_at{"UNSET"};
  double safety_factor{0.0};
  double model_error_margin{0.0};  // モデル誤差の最大値 [N m]（仕様書 第10.4節）
  Vector3 gravity{0.0, 0.0, -9.80665};  // ワールド座標系の重力加速度ベクトル (m/s^2)

  KinematicChain chain{defaultKinematicChain()};
  /// 各可動リンク（index 0..4 = base_yaw, upper_arm, forearm, wrist, flange）の慣性。
  std::array<LinkInertial, kUrdfJointCount> links{};
  std::array<JointFriction, kDof> friction{};

  Theta1DriveConfig theta1{};
  /// 第2～4軸の瞬時許容トルク上限（定格値、0 以下は未設定）。
  std::array<double, kDof> joint_peak_torque{0.0, 0.0, 0.0, 0.0};
  Payload payload{};

  /// 実機出力に必要なすべての必須値が設定されているか検査する（仕様書 第3.1, 3.2節）。
  bool isCompleteForHardware(std::string * reason = nullptr) const;
  /// 動力学計算（Phase 2/3）に必要なリンク質量・安全率・第1軸上限が有効か検査する。
  bool hasValidDynamics(std::string * reason = nullptr) const;
};

/// YAML から `robot_dynamics` を読み込む（仕様書 第14節の最小形式および完全形式の両方に対応）。
RobotDynamicsConfig loadRobotDynamicsConfig(const YamlNode & root);

class DynamicsModel
{
public:
  virtual ~DynamicsModel() = default;
  virtual State inverseDynamics(
    const State & q, const State & qd,
    const State & qdd) const = 0;
  virtual double effectiveInertia(
    std::size_t joint,
    const State & q) const = 0;
};

/// 4自由度 RNEA 動力学モデルの具体実装。
class RneaDynamicsModel : public DynamicsModel
{
public:
  explicit RneaDynamicsModel(RobotDynamicsConfig config, std::uint64_t version = 1);

  State inverseDynamics(
    const State & q, const State & qd,
    const State & qdd) const override;
  double effectiveInertia(
    std::size_t joint,
    const State & q) const override;

  /// M(q) * v（重力=0, 速度=0, 摩擦=0 での慣性項）。
  State inertialTorque(const State & q, const State & v) const;
  /// C(q, qd) * qd（重力=0, 加速度=0, 摩擦=0 での遠心・コリオリ項）。
  State coriolisTorque(const State & q, const State & qd) const;
  /// g(q)（速度=0, 加速度=0, 摩擦=0 での重力項）。
  State gravityTorque(const State & q) const;
  /// tau_f(qd)（関節摩擦トルク）。
  State frictionTorque(const State & qd) const;

  /// 関節速度 qd [rad/s] における各軸の安全率込み許容トルク上下限 [N m]。
  /// 第1軸は速度―トルク曲線を補間し、範囲外速度なら feasible = false を返す。
  TorqueBounds torqueBounds(std::size_t joint, double joint_velocity) const;

  /// 安全率込みの第1軸連続許容トルク（RMS上限）[N m]。
  double theta1ContinuousAllowedTorque() const;
  /// 安全率込みの第1軸ピーク許容トルク（速度0付近の代表値）[N m]。
  double theta1PeakAllowedTorque() const;

  /// ペイロードを更新した新しいモデルを返す（version を +1 する）。
  RneaDynamicsModel withPayload(const Payload & payload) const;

  const RobotDynamicsConfig & config() const {return config_;}
  std::uint64_t version() const {return version_;}

private:
  RobotDynamicsConfig config_;
  std::array<LinkInertial, kUrdfJointCount> combined_links_{};
  std::uint64_t version_{1};

  State rnea(
    const State & q, const State & qd, const State & qdd,
    bool include_gravity, bool include_friction) const;
};

}  // namespace path_planner
