#ifndef NAV_DIRECTOR__ROUTE_PLANNER_HPP_
#define NAV_DIRECTOR__ROUTE_PLANNER_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "catchrobo2026_msgs/srv/plan_rotation_group.hpp"

class robot_kinematics;

namespace nav_director
{

struct Point3D
{
  double x, y, z, phi;
};

struct PlannerState
{
  Point3D pose;
  double base;
  double wrist;
};

using PlanRotationGroup = catchrobo2026_msgs::srv::PlanRotationGroup;

// Plans geometry and wrist commands without a ROS node or communication.
class RoutePlanner
{
public:
  RoutePlanner();
  ~RoutePlanner();
  RoutePlanner(RoutePlanner &&) noexcept;
  RoutePlanner & operator=(RoutePlanner &&) noexcept;

  PlannerState stateFromJoints(const std::array<float, 4> & joints);
  bool solveIK(const Point3D & point, std::array<float, 4> & joints);

  // Targets exclude the start pose. Lengths are millimeters; angles are radians.
  std::vector<Point3D> generateRoute(
    const PlannerState & state, const std::vector<Point3D> & targets);
  PlanRotationGroup::Response planGroup(
    const PlannerState & state, const PlanRotationGroup::Request & request);

private:
  bool solveIK(const Point3D & point, float (&joints)[4]);
  void planRotationGroup(
    const PlanRotationGroup::Request & req, PlanRotationGroup::Response & res);
  bool buildWristRoutes(
    const std::vector<Point3D> & targets,
    const std::vector<uint32_t> & route_ends,
    const std::vector<double> & selected, bool strict_base,
    std::vector<catchrobo2026_msgs::msg::RotationGroupRoute> & routes,
    double & travel, std::string & error, std::vector<double> & route_travel);
  void planMinimumPhi(
    const std::vector<Point3D> & targets, const std::vector<double> & raw_angles,
    const PlanRotationGroup::Request & request,
    PlanRotationGroup::Response & response, int direction = 0, bool wrist_interpolation = false);
  static bool finitePoint(const Point3D & point);
  std::vector<Point3D> densifyPoints(const std::vector<Point3D> & points);
  Point3D interpolatePoint(const Point3D & p0, const Point3D & p1, double ratio);
  std::vector<Point3D> generate3DSpline(
    const std::vector<Point3D> & points, std::vector<double> * parameters = nullptr);

  Point3D cur_pose_{};
  double current_base_{0.0};
  double current_wrist_{0.0};
  int sample_resolution_{50};
  std::unique_ptr<robot_kinematics> kin_;
};

}  // namespace nav_director

#endif  // NAV_DIRECTOR__ROUTE_PLANNER_HPP_
