#include "path_planning/arm_collision_checker.hpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <regex>
#include <sstream>
#include <iomanip>
#include <stdexcept>

using namespace path_planning;
namespace {
void require(bool ok, const std::string & message) {
  if (!ok) { throw std::runtime_error(message); }
}
}
int main(int argc, char ** argv) {
  require(argc >= 2, "Pass expanded robot URDF path");
  std::ifstream stream(argv[1]);
  require(stream.good(), "Cannot read URDF");
  const std::string xml((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
  ArmCollisionChecker checker(xml);
  const double pi = std::acos(-1.0);
  const State free_state{0, 0, 1.5, 0};
  require(checker.isStateValid(free_state), "extended arm must be free");
  const State colliding_state{0, pi/2, -pi/2, 0};
  auto folded = checker.check(colliding_state);
  require(!folded.valid && folded.links == LinkPair{"upper_arm_link", "flange_link"},
    "folded hand must collide with upper arm");
  require(checker.isStateValid({0, 0, 0, 0}),
    "forearm-hand overlap is outside the selected pair and must be ignored");
  require(ArmCollisionChecker(xml, {}).isStateValid(colliding_state),
    "empty checked pairs must disable pair collision tests");
  require(!ArmCollisionChecker(xml, {{"upper_arm_link", "flange_link"}}).isStateValid(colliding_state),
    "reversed checked pair must detect the same collision");
  ArmCollisionChecker forearm_check(xml, {{"forearm_link", "flange_link"}});
  require(!forearm_check.isStateValid({0, 0, 0, 0}), "excluded overlap really exists in URDF");

  PlannerOptions options;
  options.lower_bounds = {-pi, -pi, -pi, -pi};
  options.upper_bounds = {pi, pi, pi, pi};
  InformedRrtStar planner(options, [&checker](const State & q) { return checker.isStateValid(q); });
  const State other_end{0, 0, 1.5, pi};
  require(checker.isStateValid(other_end), "second endpoint is collision free");
  require(checker.isStateValid({0, 0, 1.5, pi/2}), "hand-forearm collision is not selected");
  require(planner.isMotionValid(free_state, other_end), "ignore unselected collisions along edges");
  require(!planner.plan(free_state, colliding_state), "planner must reject colliding goal");
  auto safe_path = planner.plan(free_state, {0.2, 0.2, 1.5, 0});
  require(static_cast<bool>(safe_path), "collision-free planning example");
  for (std::size_t i = 1; i < safe_path.path.size(); ++i) {
    for (int k = 0; k <= 100; ++k) {
      State q(4);
      for (int j = 0; j < 4; ++j) {
        q[j] = safe_path.path[i-1][j] + (safe_path.path[i][j]-safe_path.path[i-1][j])*k/100.0;
      }
      require(checker.isStateValid(q), "planned path must be collision free at finer resolution");
    }
  }

  // Independently derived field-frame kinematics: replace all collision bodies
  // by small probes, and place a fixed probe at the analytic hand position.
  // This checks URDF joint origins, axes, dependent angles and collision offsets.
  const std::regex collision_pattern(R"(<collision>[\s\S]*?</collision>)");
  std::string probes = std::regex_replace(xml, collision_pattern,
    "<collision><geometry><sphere radius='0.00001'/></geometry></collision>");
  const auto flange_begin = probes.find("<link name=\"flange_link\">");
  const auto flange_collision = probes.find("<collision>", flange_begin);
  probes.insert(flange_collision + std::string("<collision>").size(),
    "<origin xyz='0.07 0.02 -0.03'/>");
  for (int i = 0; i < 12; ++i) {
    const State q{-1.0 + i*0.17, -0.6 + i*0.08, 0.4 + i*0.1, -0.8 + i*0.13};
    const double radial = -0.04 + 0.48*std::sin(q[1]) + 0.48*std::sin(q[2]) + 0.04;
    const double yaw = q[0] + q[3];
    const double x = 0.675 + radial*std::sin(q[0]) + 0.07*std::cos(yaw)-0.02*std::sin(yaw);
    const double y = -0.190 + radial*std::cos(q[0]) + 0.07*std::sin(yaw)+0.02*std::cos(yaw);
    const double z = 0.090 + 0.48*std::cos(q[1]) + 0.48*std::cos(q[2]) - 0.03;
    std::ostringstream extra;
    extra << std::setprecision(17)
      << "<link name='probe'><collision><geometry><sphere radius='0.00001'/></geometry></collision></link>"
      << "<joint name='probe_fixed' type='fixed'><parent link='map'/><child link='probe'/><origin xyz='"
      << x << ' ' << y << ' ' << z << "'/></joint>";
    std::string fixture = probes;
    fixture.insert(fixture.find("</robot>"), extra.str());
    auto contact = ArmCollisionChecker(fixture, {{"flange_link", "probe"}}).check(q);
    require(!contact.valid && (contact.links == LinkPair{"flange_link", "probe"} ||
      contact.links == LinkPair{"probe", "flange_link"}), "independent kinematics contact probe");
    require(ArmCollisionChecker(probes).isStateValid(q), "probe fixture alone must be free");
  }
  require(!checker.isStateValid({0, 0, 0}), "wrong dimension");
  require(!checker.isStateValid({0, 0, std::numeric_limits<double>::quiet_NaN(), 0}), "NaN");
  bool threw = false;
  try { ArmCollisionChecker invalid("not XML"); } catch (const std::invalid_argument &) { threw = true; }
  require(threw, "invalid URDF must fail closed");
  threw = false;
  try { ArmCollisionChecker invalid(xml, {{"missing", "flange_link"}}); }
  catch (const std::invalid_argument &) { threw = true; }
  require(threw, "unknown checked pairs must fail");
  std::string unsupported = xml;
  const auto collision_start = unsupported.find("<collision>");
  const auto geometry_start = unsupported.find("<geometry>", collision_start);
  const auto geometry_end = unsupported.find("</geometry>", geometry_start);
  unsupported.replace(geometry_start, geometry_end + 11 - geometry_start,
    "<geometry><mesh filename='missing.stl'/></geometry>");
  threw = false;
  try { ArmCollisionChecker invalid(unsupported); }
  catch (const std::invalid_argument &) { threw = true; }
  require(threw, "unsupported mesh must fail closed");
  std::cout << "Arm collision checks passed\n";
}
