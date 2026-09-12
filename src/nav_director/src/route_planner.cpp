#include "nav_director/route_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "rotation_constraints.hpp"
#include "ros2_inverse_kinematics/robot_kinematics.h"

namespace nav_director
{

RoutePlanner::RoutePlanner() : kin_(std::make_unique<robot_kinematics>()) {}
RoutePlanner::~RoutePlanner() = default;
RoutePlanner::RoutePlanner(RoutePlanner &&) noexcept = default;
RoutePlanner & RoutePlanner::operator=(RoutePlanner &&) noexcept = default;

PlannerState RoutePlanner::stateFromJoints(const std::array<float, 4> &joints)
{
    if (!std::all_of(joints.begin(), joints.end(), [](float value) {
            return std::isfinite(value);
        })) {
        throw std::runtime_error("Initial joint angles must be finite");
    }
    auto input = joints;
    float pose[6]{};
    kin_->forward_kinematics(pose, input.data());
    if (!std::all_of(pose, pose + 4, [](float value) { return std::isfinite(value); })) {
        throw std::runtime_error("Initial forward kinematics is non-finite");
    }
    return {{pose[0], pose[1], pose[2], pose[3]}, joints[0], joints[3]};
}

bool RoutePlanner::solveIK(const Point3D &point, std::array<float, 4> &joints)
{
    float output[4]{};
    const bool success = solveIK(point, output);
    std::copy(output, output + 4, joints.begin());
    return success;
}

std::vector<Point3D> RoutePlanner::generateRoute(
    const PlannerState &state, const std::vector<Point3D> &targets)
{
    if (targets.empty()) throw std::runtime_error("Route requires at least one target");
    std::vector<Point3D> points{state.pose};
    points.insert(points.end(), targets.begin(), targets.end());
    if (!std::all_of(points.begin(), points.end(), finitePoint)) {
        throw std::runtime_error("Route contains a non-finite position or angle");
    }
    while (points.size() < 4) points = densifyPoints(points);
    auto samples = generate3DSpline(points);
    if (!std::all_of(samples.begin(), samples.end(), finitePoint)) {
        throw std::runtime_error("Route generation produced a non-finite pose");
    }
    return samples;
}

PlanRotationGroup::Response RoutePlanner::planGroup(
    const PlannerState &state, const PlanRotationGroup::Request &request,
    double wrist_feedback_tolerance_rad)
{
    PlanRotationGroup::Response response;
    if (!std::isfinite(wrist_feedback_tolerance_rad) || wrist_feedback_tolerance_rad < 0.0 ||
        wrist_feedback_tolerance_rad >= rotation_constraints::kTurn / 2.0) {
        response.message = "Wrist feedback tolerance must be finite and within [0, pi) radians";
        return response;
    }
    if (!finitePoint(state.pose) || !std::isfinite(state.base)) {
        response.message = "Finite initial state within the wrist limits is required";
        return response;
    }
    if (!rotation_constraints::legal_feedback(state.wrist, wrist_feedback_tolerance_rad)) {
        response.message = "Finite initial wrist angle within the feedback tolerance is required";
        return response;
    }
    cur_pose_ = state.pose;
    current_base_ = state.base;
    current_wrist_ = rotation_constraints::clamp(state.wrist);
    if (current_wrist_ != state.wrist) {
        // Correct only the planning copy, preserving the measured physical turn.
        cur_pose_.phi = current_base_ + current_wrist_;
    }
    planRotationGroup(request, response);
    return response;
}

bool RoutePlanner::solveIK(const Point3D &point, float (&joints)[4]) {
    if (!finitePoint(point)) return false;
    float pose[6] = {static_cast<float>(point.x), static_cast<float>(point.y),
        static_cast<float>(point.z), static_cast<float>(point.phi),
        static_cast<float>(-M_PI / 2.0), 0.0F};
    kin_->inverse_kinematics(pose, joints);
    return std::all_of(joints, joints + 4,
        [](float value) { return std::isfinite(value); });
}

void RoutePlanner::planRotationGroup(const PlanRotationGroup::Request &req,
                                       PlanRotationGroup::Response &res) {
    res.success = false;
    if (req.limit_phi_travel && (!std::isfinite(req.max_phi_travel) ||
        req.max_phi_travel < 0.0)) {
        res.message = "Phi travel limit must be finite and nonnegative";
        return;
    }
    if (!finitePoint(cur_pose_) || !std::isfinite(current_base_) ||
        !rotation_constraints::legal(current_wrist_)) {
        res.message = "Finite initial state within the wrist limits is required";
        return;
    }
    if (req.targets.empty() || req.targets.size() > 10000 || req.route_ends.empty() ||
        req.route_ends.back() != req.targets.size()) {
        res.message = "Rotation group requires targets and matching route ends";
        return;
    }
    size_t previous_end = 0;
    for (const auto end : req.route_ends) {
        if (end <= previous_end || end > req.targets.size()) {
            res.message = "Rotation group route ends must increase within targets";
            return;
        }
        previous_end = end;
    }
    previous_end = 0;
    for (const auto & interval : req.phi_travel_intervals) {
        if (interval.start_target < previous_end || interval.start_target >= interval.end_target ||
            interval.end_target > req.targets.size() ||
            !std::isfinite(interval.max_phi_travel) || interval.max_phi_travel < 0.0) {
            res.message = "Phi intervals must be ordered, nonoverlapping MOVE ranges with finite limits";
            return;
        }
        previous_end = interval.end_target;
    }
    std::vector<Point3D> targets;
    std::vector<double> raw_angles;
    for (const auto &pose : req.targets) {
        const auto &q = pose.orientation;
        const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
        if (!std::isfinite(norm) || norm < 1e-12) {
            res.message = "Rotation group target has an invalid quaternion";
            return;
        }
        tf2::Quaternion quaternion(q.x, q.y, q.z, q.w);
        quaternion.normalize();
        const tf2::Matrix3x3 rotation(quaternion);
        Point3D point{pose.position.x * 1000.0, pose.position.y * 1000.0,
            pose.position.z * 1000.0, std::atan2(-rotation[0][1], rotation[1][1])};
        float joints[4];
        if (!solveIK(point, joints)) {
            res.message = "Rotation group target is non-finite or unreachable";
            return;
        }
        targets.push_back(point);
        raw_angles.push_back(point.phi - joints[0]);
    }

    const double start_wrist = rotation_constraints::clamp(current_wrist_);
    if (req.allow_wrist_reversal) {
        planMinimumPhi(targets, raw_angles, req, res);
        if (!res.success && !req.phi_travel_intervals.empty()) {
            res = PlanRotationGroup::Response{};
            planMinimumPhi(targets, raw_angles, req, res, 0, true);
        }
        return;
    }
    if (req.limit_phi_travel || !req.phi_travel_intervals.empty()) {
        // Phi interpolation can satisfy both a zero travel budget and wrist direction.
        for (int direction : {1, -1}) {
            PlanRotationGroup::Response candidate;
            planMinimumPhi(targets, raw_angles, req, candidate, direction);
            if (candidate.success) {
                res = std::move(candidate);
                return;
            }
        }
        if (!req.phi_travel_intervals.empty()) {
            for (int direction : {1, -1}) {
                PlanRotationGroup::Response candidate;
                planMinimumPhi(targets, raw_angles, req, candidate, direction, true);
                if (candidate.success) {
                    res = std::move(candidate);
                    return;
                }
            }
        }
    }
    std::vector<double> increasing, decreasing;
    const bool can_increase = rotation_constraints::solve(start_wrist, raw_angles, 1, increasing);
    const bool can_decrease = rotation_constraints::solve(start_wrist, raw_angles, -1, decreasing);
    if (!can_increase && !can_decrease) {
        res.message = "No single wrist direction can reach every target within [-2pi, 0]";
        return;
    }
    const int preferred_direction = can_increase && (!can_decrease ||
        std::abs(increasing.back() - start_wrist) <=
            std::abs(decreasing.back() - start_wrist)) ? 1 : -1;
    std::vector<catchrobo2026_msgs::msg::RotationGroupRoute> selected_routes;
    std::vector<double> selected_interval_travel;
    double selected_travel = std::numeric_limits<double>::infinity();
    int selected_direction = 0;
    std::string error;
    for (int direction : {preferred_direction, -preferred_direction}) {
        if ((direction > 0 && !can_increase) || (direction < 0 && !can_decrease)) continue;
        const auto &selected = direction > 0 ? increasing : decreasing;
        std::vector<catchrobo2026_msgs::msg::RotationGroupRoute> routes;
        std::vector<double> target_travel, interval_travel;
        double travel = 0.0;
        const bool limited = req.limit_phi_travel || !req.phi_travel_intervals.empty();
        if (!buildWristRoutes(targets, req.route_ends, selected, limited,
                routes, travel, error, target_travel)) {
            if (!limited) break;
            continue;
        }
        bool intervals_valid = true;
        for (const auto & interval : req.phi_travel_intervals) {
            double amount = 0.0;
            for (size_t i = interval.start_target; i < interval.end_target; ++i) {
                amount += target_travel[i];
            }
            interval_travel.push_back(amount);
            intervals_valid &= amount <= interval.max_phi_travel + rotation_constraints::kTolerance;
        }
        if (!intervals_valid) {
            error = "Phi travel exceeds a configured subinterval limit";
            continue;
        }
        if (req.limit_phi_travel && travel > req.max_phi_travel +
                rotation_constraints::kTolerance) {
            error = "No single wrist direction satisfies the phi travel limit";
            continue;
        }
        if (selected_direction == 0 || travel < selected_travel) {
            selected_routes = std::move(routes);
            selected_travel = travel;
            selected_interval_travel = std::move(interval_travel);
            selected_direction = direction;
        }
        if (!limited) break;
    }
    if (selected_direction == 0) {
        res.message = error;
        return;
    }
    res.routes = std::move(selected_routes);
    res.direction = selected_direction;
    res.phi_travel = selected_travel;
    res.interval_phi_travel = std::move(selected_interval_travel);
    res.success = true;
    res.message = "Rotation group planned";
}

bool RoutePlanner::buildWristRoutes(const std::vector<Point3D> &targets,
                      const std::vector<uint32_t> &route_ends,
                      const std::vector<double> &selected, bool strict_base,
                      std::vector<catchrobo2026_msgs::msg::RotationGroupRoute> &routes,
                      double &travel, std::string &error, std::vector<double> &target_travel) {
    // Finish every route before exposing any plan or touching shared waypoints.
    Point3D start_pose = cur_pose_;
    double route_start_wrist = rotation_constraints::clamp(current_wrist_);
    travel = 0.0;
    size_t begin = 0;
    for (const auto end : route_ends) {
        const size_t route_target_count = end - begin;
        std::vector<Point3D> points{start_pose};
        std::vector<double> wrists{route_start_wrist};
        for (size_t i = begin; i < end; ++i) {
            points.push_back(targets[i]);
            wrists.push_back(selected[i]);
        }
        while (points.size() < 4) {
            std::vector<double> dense_wrists;
            for (size_t i = 0; i + 1 < wrists.size(); ++i) {
                for (int third = 0; third < 3; ++third) {
                    dense_wrists.push_back(wrists[i] +
                        (wrists[i + 1] - wrists[i]) * third / 3.0);
                }
            }
            dense_wrists.push_back(wrists.back());
            points = densifyPoints(points);
            wrists = std::move(dense_wrists);
        }
        std::vector<double> parameters;
        auto samples = generate3DSpline(points, &parameters);
        catchrobo2026_msgs::msg::RotationGroupRoute route;
        route.path.header.frame_id = "map";
        for (size_t target = 1; target <= route_target_count; ++target) {
            const double parameter = static_cast<double>(target) / route_target_count;
            const auto found = std::find_if(parameters.begin(), parameters.end(), [parameter](double value) {
                return std::abs(value - parameter) < 1e-12;
            });
            if (found == parameters.end()) {
                error = "Rotation group route omitted a target sample";
                return false;
            }
            route.target_sample_indices.push_back(static_cast<uint32_t>(found - parameters.begin()));
        }
        std::vector<double> local_target_travel(route_target_count, 0.0);
        for (size_t i = 0; i < samples.size(); ++i) {
            const double scaled = parameters[i] * (points.size() - 1);
            const size_t index = std::min(static_cast<size_t>(scaled), wrists.size() - 2);
            const double wrist = wrists[index] +
                (wrists[index + 1] - wrists[index]) * (scaled - index);
            float joints[4];
            if (!solveIK(samples[i], joints)) {
                error = "Rotation group spline leaves the reachable workspace";
                return false;
            }
            samples[i].phi = joints[0] + wrist;
            if (strict_base && begin == 0 && i == 0 &&
                std::abs(joints[0] - current_base_) > 1e-4) {
                error = "Current base angle does not match the principal IK branch";
                return false;
            }
            if (!solveIK(samples[i], joints)) {
                error = "Rotation group spline has invalid joint angles";
                return false;
            }
            if (i > 0) {
                const double amount = rotation_constraints::wrist_segment_phi_travel(
                    samples[i-1].x - robot_pos[0], samples[i-1].y - robot_pos[1],
                    samples[i].x - robot_pos[0], samples[i].y - robot_pos[1],
                    route.wrist_angles.back(), wrist);
                if (strict_base && !std::isfinite(amount)) {
                    error = "Phi-limited route crosses the base axis or atan2 branch";
                    return false;
                }
                travel += std::isfinite(amount) ? amount :
                    std::abs(samples[i].phi - samples[i-1].phi);
                const size_t target = static_cast<size_t>(std::lower_bound(
                    route.target_sample_indices.begin(), route.target_sample_indices.end(), i) -
                    route.target_sample_indices.begin());
                if (target >= local_target_travel.size()) {
                    error = "Rotation group sample lies beyond its final target";
                    return false;
                }
                local_target_travel[target] += std::isfinite(amount) ? amount :
                    std::abs(samples[i].phi - samples[i-1].phi);
            }
            geometry_msgs::msg::PoseStamped pose;
            pose.header = route.path.header;
            pose.pose.position.x = samples[i].x / 1000.0;
            pose.pose.position.y = samples[i].y / 1000.0;
            pose.pose.position.z = samples[i].z / 1000.0;
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, samples[i].phi);
            pose.pose.orientation = tf2::toMsg(q);
            route.path.poses.push_back(pose);
            route.wrist_angles.push_back(wrist);
        }
        routes.push_back(std::move(route));
        target_travel.insert(target_travel.end(),
            local_target_travel.begin(), local_target_travel.end());
        start_pose = targets[end - 1];
        route_start_wrist = selected[end - 1];
        begin = end;
    }
    return true;
}

void RoutePlanner::planMinimumPhi(const std::vector<Point3D> &targets,
                    const std::vector<double> &raw_angles,
                    const PlanRotationGroup::Request &request,
                    PlanRotationGroup::Response &response, int direction, bool wrist_interpolation) {
    struct Geometry {
        size_t begin, end;
        std::vector<Point3D> samples;
        std::vector<double> parameters, bases;
    };
    struct EdgeSample {Point3D point; double ratio;};
    std::vector<Geometry> geometry;
    std::vector<std::vector<EdgeSample>> edges(targets.size());
    std::vector<double> target_bases;
    for (const auto &target : targets) {
        float joints[4];
        if (!solveIK(target, joints)) {
            response.message = "Sequence group target is unreachable";
            return;
        }
        target_bases.push_back(joints[0]);
    }
    Point3D start_pose = cur_pose_;
    size_t begin = 0;
    for (const size_t end : request.route_ends) {
        std::vector<Point3D> points{start_pose};
        points.insert(points.end(), targets.begin() + begin, targets.begin() + end);
        while (points.size() < 4) points = densifyPoints(points);
        Geometry route;
        route.begin = begin;
        route.end = end;
        route.samples = generate3DSpline(points, &route.parameters);
        for (size_t j = 0; j < route.samples.size(); ++j) {
            float joints[4];
            if (!solveIK(route.samples[j], joints)) {
                response.message = "Sequence group spline leaves the reachable workspace";
                return;
            }
            if (begin == 0 && j == 0 && std::abs(joints[0] - current_base_) > 1e-4) {
                response.message = "Current base angle does not match the principal IK branch";
                return;
            }
            route.bases.push_back(joints[0]);
            double scaled = route.parameters[j] * (end - begin);
            const double nearest = std::round(scaled);
            if (std::abs(scaled - nearest) < 1e-9) scaled = nearest;
            const size_t index = std::min(static_cast<size_t>(scaled), end - begin - 1);
            const double ratio = scaled - index;
            if (ratio == 0.0 && index > 0) {
                edges[begin + index - 1].push_back({route.samples[j], 1.0});
            }
            edges[begin + index].push_back({route.samples[j], ratio});
        }
        geometry.push_back(std::move(route));
        start_pose = targets[end - 1];
        begin = end;
    }
    const auto edge_cost = [this, &edges, &target_bases, direction, wrist_interpolation](
            size_t index, double phi0, double phi1) {
        const double invalid = std::numeric_limits<double>::infinity();
        const auto &samples = edges[index];
        if (samples.size() < 2 || samples.front().ratio != 0.0 ||
            samples.back().ratio != 1.0) return invalid;
        const double wrist0 = phi0 - (index == 0 ? current_base_ : target_bases[index - 1]);
        const double wrist1 = phi1 - target_bases[index];
        if (wrist_interpolation && direction * (wrist1 - wrist0) <
                -rotation_constraints::kTolerance) return invalid;
        double travel = 0.0;
        for (size_t j = 1; j < samples.size(); ++j) {
            const auto &a = samples[j - 1];
            const auto &b = samples[j];
            if (wrist_interpolation) {
                const double amount = rotation_constraints::wrist_segment_phi_travel(
                    a.point.x - robot_pos[0], a.point.y - robot_pos[1],
                    b.point.x - robot_pos[0], b.point.y - robot_pos[1],
                    wrist0 + (wrist1 - wrist0) * a.ratio,
                    wrist0 + (wrist1 - wrist0) * b.ratio);
                if (!std::isfinite(amount)) return invalid;
                travel += amount;
                continue;
            }
            if (!rotation_constraints::legal_phi_segment(
                    a.point.x - robot_pos[0], a.point.y - robot_pos[1],
                    b.point.x - robot_pos[0], b.point.y - robot_pos[1],
                    phi0 + (phi1 - phi0) * a.ratio,
                    phi0 + (phi1 - phi0) * b.ratio, direction)) return invalid;
        }
        return wrist_interpolation ? travel : std::abs(phi1 - phi0);
    };
    std::vector<double> wrists;
    double travel = 0.0;
    const double start_phi = current_base_ + rotation_constraints::clamp(current_wrist_);
    std::vector<rotation_constraints::PhiTravelInterval> intervals;
    for (const auto & interval : request.phi_travel_intervals) {
        intervals.push_back({interval.start_target, interval.end_target, interval.max_phi_travel});
    }
    if (!rotation_constraints::solve_minimum_phi_cost(start_phi, target_bases, raw_angles,
            edge_cost, wrists, travel, intervals)) {
        response.message = "No continuous phi route fits the wrist limits, base branch and phi intervals";
        return;
    }
    if (request.limit_phi_travel && travel > request.max_phi_travel +
            rotation_constraints::kTolerance) {
        response.message = "Minimum phi travel exceeds the configured limit";
        return;
    }
    if (wrist_interpolation) {
        std::vector<double> target_travel;
        if (!buildWristRoutes(targets, request.route_ends, wrists, true,
                response.routes, travel, response.message, target_travel)) {
            response.routes.clear();
            return;
        }
        for (const auto & interval : intervals) {
            double amount = 0.0;
            for (size_t i = interval.start_target; i < interval.end_target; ++i) {
                amount += target_travel[i];
            }
            if (amount > interval.max_phi_travel + rotation_constraints::kTolerance) {
                response.routes.clear();
                response.message = "Wrist interpolation exceeds a phi interval limit";
                return;
            }
            response.interval_phi_travel.push_back(amount);
        }
        if (request.limit_phi_travel && travel > request.max_phi_travel +
                rotation_constraints::kTolerance) {
            response.routes.clear();
            response.message = "Wrist interpolation exceeds the whole-group phi limit";
            return;
        }
        response.direction = direction;
        response.phi_travel = travel;
        response.success = true;
        response.message = "Sequence group planned with wrist interpolation and phi intervals";
        return;
    }
    std::vector<double> phis{start_phi};
    for (size_t i = 0; i < wrists.size(); ++i) phis.push_back(target_bases[i] + wrists[i]);
    for (const auto & interval : intervals) {
        double amount = 0.0;
        for (size_t i = interval.start_target; i < interval.end_target; ++i) {
            amount += std::abs(phis[i + 1] - phis[i]);
        }
        response.interval_phi_travel.push_back(amount);
    }
    std::vector<catchrobo2026_msgs::msg::RotationGroupRoute> routes;
    for (const auto &shape : geometry) {
        catchrobo2026_msgs::msg::RotationGroupRoute route;
        route.path.header.frame_id = "map";
        for (size_t target = shape.begin; target < shape.end; ++target) {
            const double parameter = static_cast<double>(target - shape.begin + 1) /
                (shape.end - shape.begin);
            const auto found = std::find_if(shape.parameters.begin(), shape.parameters.end(),
                [parameter](double value) {return std::abs(value - parameter) < 1e-12;});
            if (found == shape.parameters.end()) {
                response.message = "Rotation group route omitted a target sample";
                return;
            }
            route.target_sample_indices.push_back(
                static_cast<uint32_t>(found - shape.parameters.begin()));
        }
        for (size_t j = 0; j < shape.samples.size(); ++j) {
            const double scaled = shape.parameters[j] * (shape.end - shape.begin);
            const size_t index = std::min(static_cast<size_t>(scaled),
                shape.end - shape.begin - 1);
            const double ratio = scaled - index;
            const double phi = phis[shape.begin + index] + ratio *
                (phis[shape.begin + index + 1] - phis[shape.begin + index]);
            const double wrist = phi - shape.bases[j];
            if (!rotation_constraints::legal(wrist)) {
                response.message = "Selected phi route has an invalid wrist sample";
                return;
            }
            geometry_msgs::msg::PoseStamped pose;
            pose.header = route.path.header;
            pose.pose.position.x = shape.samples[j].x / 1000.0;
            pose.pose.position.y = shape.samples[j].y / 1000.0;
            pose.pose.position.z = shape.samples[j].z / 1000.0;
            tf2::Quaternion quaternion;
            quaternion.setRPY(0.0, 0.0, phi);
            pose.pose.orientation = tf2::toMsg(quaternion);
            route.path.poses.push_back(pose);
            route.wrist_angles.push_back(rotation_constraints::clamp(wrist));
            route.phi_angles.push_back(phi);
        }
        routes.push_back(std::move(route));
    }
    response.routes = std::move(routes);
    response.direction = direction;
    response.phi_travel = travel;
    response.success = true;
    response.message = "Sequence group planned with minimum phi travel";
}

bool RoutePlanner::finitePoint(const Point3D &point) {
    return std::isfinite(point.x) && std::isfinite(point.y) &&
           std::isfinite(point.z) && std::isfinite(point.phi);
}

// Subdivide each segment into thirds.
std::vector<Point3D> RoutePlanner::densifyPoints(const std::vector<Point3D>& points) {
    std::vector<Point3D> densified;
    if (points.size() < 2) return points;

    for (size_t i = 0; i < points.size() - 1; ++i) {
        Point3D p0 = points[i];
        Point3D p1 = points[i + 1];

        densified.push_back(p0);

        // First third.
        densified.push_back(interpolatePoint(p0, p1, 1.0 / 3.0));

        // Second third.
        densified.push_back(interpolatePoint(p0, p1, 2.0 / 3.0));
    }
    densified.push_back(points.back());
    return densified;
}

// Interpolate position and orientation.
Point3D RoutePlanner::interpolatePoint(const Point3D& p0, const Point3D& p1, double ratio) {
    Point3D p;
    p.x = p0.x + (p1.x - p0.x) * ratio;
    p.y = p0.y + (p1.y - p0.y) * ratio;
    p.z = p0.z + (p1.z - p0.z) * ratio;

    // Interpolate phi with quaternion Slerp.
    tf2::Quaternion q0, q1;
    q0.setRPY(0, 0, p0.phi);
    q1.setRPY(0, 0, p1.phi);
    tf2::Quaternion q_interp = q0.slerp(q1, ratio);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q_interp).getRPY(roll, pitch, yaw);
    p.phi = yaw;

    return p;
}

std::vector<Point3D> RoutePlanner::generate3DSpline(const std::vector<Point3D>& points,
                                   std::vector<double> *parameters) {
    using Spline3D = Eigen::Spline<double, 3>;
    Eigen::Matrix<double, 3, Eigen::Dynamic> p_matrix(3, points.size());

    for (size_t i = 0; i < points.size(); ++i) {
        p_matrix(0, i) = points[i].x;
        p_matrix(1, i) = points[i].y;
        p_matrix(2, i) = points[i].z;
    }

    Eigen::RowVectorXd u(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        u(i) = static_cast<double>(i) / static_cast<double>(points.size() - 1);
    }

    // Fit a cubic position spline.
    const int degree = 3;
    Spline3D spline = Eigen::SplineFitting<Spline3D>::Interpolate(p_matrix, degree, u);

    std::vector<Point3D> smoothed;
    int total_samples = points.size() * sample_resolution_;
    std::vector<double> times;
    for (int i = 0; i <= total_samples; ++i) {
        times.push_back(static_cast<double>(i) / total_samples);
    }
    if (parameters != nullptr) {
        // Include every waypoint exactly when evaluating alternative wrist branches.
        for (size_t i = 0; i < points.size(); ++i) {
            times.push_back(static_cast<double>(i) / (points.size() - 1));
        }
        std::sort(times.begin(), times.end());
        times.erase(std::unique(times.begin(), times.end(), [](double a, double b) {
            return std::abs(a - b) < 1e-12;
        }), times.end());
        *parameters = times;
    }
    for (const double t : times) {
        Eigen::Vector3d pv = spline(t);

        // Interpolate orientation along the spline.
        double scaled_t = t * (points.size() - 1);
        int idx = static_cast<int>(std::floor(scaled_t));
        
        if (idx >= static_cast<int>(points.size() - 1)) {
            idx = points.size() - 2;
            scaled_t = points.size() - 1;
        }
        
        double local_t = scaled_t - idx;

        tf2::Quaternion q_start, q_end;
        q_start.setRPY(0, 0, points[idx].phi);
        q_end.setRPY(0, 0, points[idx + 1].phi);

        tf2::Quaternion q_interp = q_start.slerp(q_end, local_t);

        double roll, pitch, yaw;
        tf2::Matrix3x3(q_interp).getRPY(roll, pitch, yaw);

        smoothed.push_back({pv.x(), pv.y(), pv.z(), yaw});
    }
    
    return smoothed;
}

}  // namespace nav_director
