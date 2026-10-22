#pragma once

#include <array>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "path_planning/informed_rrt_star.hpp"

namespace path_planning {
using LinkPair = std::pair<std::string, std::string>;

struct CollisionCheckResult {
  bool valid{false};
  // First colliding pair, or empty names for malformed input / valid states.
  LinkPair links;
  std::string reason;
};

// URDF + FCL adapter. No ROS messages, node, or executor dependencies.
// Input: theta1, theta2, theta3, theta4, in radians, in that order.
class ArmCollisionChecker {
public:
  // Only explicitly listed pairs are tested; ordering of each pair is ignored.
  static std::vector<LinkPair> defaultCheckedPairs();
  explicit ArmCollisionChecker(const std::string & urdf_xml,
    const std::vector<LinkPair> & checked_pairs = defaultCheckedPairs());
  ~ArmCollisionChecker();
  ArmCollisionChecker(const ArmCollisionChecker &) = delete;
  ArmCollisionChecker & operator=(const ArmCollisionChecker &) = delete;

  CollisionCheckResult check(const State & absolute_angles) const;
  bool isStateValid(const State & absolute_angles) const;
  // Position in the URDF root frame, for visualization (does not reject collisions).
  std::array<double, 3> linkPosition(const State & absolute_angles,
    const std::string & link_name) const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace path_planning
