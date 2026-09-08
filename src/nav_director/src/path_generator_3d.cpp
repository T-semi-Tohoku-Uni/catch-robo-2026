#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp> // 追加: ジョイントメッセージ用
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <unsupported/Eigen/Splines>
#include <Eigen/Dense>
#include <cmath>
#include <chrono>
#include <stdexcept>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

// 独自メッセージパッケージと運動学ライブラリのヘッダー
#include <catchrobo2026_msgs/srv/waypoint.hpp>
#include <catchrobo2026_msgs/srv/generate_route.hpp>
#include <catchrobo2026_msgs/srv/plan_rotation_group.hpp>
#include "ros2_inverse_kinematics/robot_kinematics.h" // 追加: 運動学計算用
#include "rotation_constraints.hpp"

using WaypointSrv = catchrobo2026_msgs::srv::Waypoint;
using GenRouteSrv = catchrobo2026_msgs::srv::GenerateRoute;
using PlanRotationGroup = catchrobo2026_msgs::srv::PlanRotationGroup;

struct Point3D {
    double x, y, z, phi;
};

class PathGenerator3D : public rclcpp::Node {
public:
    PathGenerator3D() : Node("path_generator_3d") {
        rotation_joint_timeout_sec_ = declare_parameter("rotation_joint_timeout_sec", 1.0);
        if (!std::isfinite(rotation_joint_timeout_sec_) || rotation_joint_timeout_sec_ <= 0.0) {
            throw std::invalid_argument("Invalid rotation_joint_timeout_sec");
        }
        // パブリッシャーの初期化
        pub_path_ = this->create_publisher<nav_msgs::msg::Path>("route", 10);
        pub_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("path_orientations", 10);
        
        // サブスクライバーの初期化 (現在位置のPoseではなく、ジョイント角度を受信するように変更)
        sub_joints_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "current_joints", 10, std::bind(&PathGenerator3D::jointCallback, this, std::placeholders::_1));

        // サービスの初期化
        srv_waypoint_ = this->create_service<WaypointSrv>(
            "waypoint", std::bind(&PathGenerator3D::waypointCallback, this, std::placeholders::_1, std::placeholders::_2));
        srv_gen_route_ = this->create_service<GenRouteSrv>(
            "generate_route", std::bind(&PathGenerator3D::genRouteCallback, this, std::placeholders::_1, std::placeholders::_2));
        srv_plan_rotation_group_ = create_service<PlanRotationGroup>(
            "plan_rotation_group", std::bind(&PathGenerator3D::planRotationGroup, this,
                std::placeholders::_1, std::placeholders::_2));

        // 初期パラメータの設定
        cur_pose_ = {0.0, 0.0, 0.0, 0.0};
        sample_resolution_ = 50; // 各ウェイポイント間の分割数
        
        RCLCPP_INFO(this->get_logger(), "3D Path Generator Initialized (with Forward Kinematics).");
    }

private:
    // 変更: ジョイント値から順運動学を用いて現在地を計算するコールバック
    void jointCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 4) {
            joints_valid_ = false;
            RCLCPP_WARN(this->get_logger(), "Received joint data size is less than 4.");
            return;
        }

        float joint_angles[4];
        for (int i = 0; i < 4; ++i) {
            if (!std::isfinite(msg->data[i])) {
                joints_valid_ = false;
                return;
            }
            joint_angles[i] = msg->data[i];
        }

        float current_posrot[6] = {0.0};
        
        // 順運動学で手先位置姿勢を計算
        kin_.forward_kinematics(current_posrot, joint_angles);

        if (!std::all_of(current_posrot, current_posrot + 4,
                [](float value) { return std::isfinite(value); })) {
            joints_valid_ = false;
            return;
        }

        // cur_pose_ を更新 (X, Y, Z, PHI)
        cur_pose_.x = current_posrot[0];
        cur_pose_.y = current_posrot[1];
        cur_pose_.z = current_posrot[2];
        cur_pose_.phi = current_posrot[3]; // PHI (Yawに相当)
        current_wrist_ = joint_angles[3];
        joints_valid_ = true;
        joints_received_at_ = std::chrono::steady_clock::now();
    }

    bool solveIK(const Point3D &point, float (&joints)[4]) {
        if (!finitePoint(point)) return false;
        float pose[6] = {static_cast<float>(point.x), static_cast<float>(point.y),
            static_cast<float>(point.z), static_cast<float>(point.phi),
            static_cast<float>(-M_PI / 2.0), 0.0F};
        kin_.inverse_kinematics(pose, joints);
        return std::all_of(joints, joints + 4,
            [](float value) { return std::isfinite(value); });
    }

    void planRotationGroup(const std::shared_ptr<PlanRotationGroup::Request> req,
                           std::shared_ptr<PlanRotationGroup::Response> res) {
        res->success = false;
        if (!joints_valid_ || !rotation_constraints::legal(current_wrist_) ||
            std::chrono::duration<double>(std::chrono::steady_clock::now() -
                joints_received_at_).count() > rotation_joint_timeout_sec_) {
            res->message = "Fresh finite current_joints within the wrist limits are required";
            return;
        }
        if (req->targets.empty() || req->route_ends.empty() ||
            req->route_ends.back() != req->targets.size()) {
            res->message = "Rotation group requires targets and matching route ends";
            return;
        }
        size_t previous_end = 0;
        for (const auto end : req->route_ends) {
            if (end <= previous_end || end > req->targets.size()) {
                res->message = "Rotation group route ends must increase within targets";
                return;
            }
            previous_end = end;
        }
        std::vector<Point3D> targets;
        std::vector<double> raw_angles;
        for (const auto &pose : req->targets) {
            const auto &q = pose.orientation;
            const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
            if (!std::isfinite(norm) || norm < 1e-12) {
                res->message = "Rotation group target has an invalid quaternion";
                return;
            }
            tf2::Quaternion quaternion(q.x, q.y, q.z, q.w);
            quaternion.normalize();
            const tf2::Matrix3x3 rotation(quaternion);
            Point3D point{pose.position.x * 1000.0, pose.position.y * 1000.0,
                pose.position.z * 1000.0, std::atan2(-rotation[0][1], rotation[1][1])};
            float joints[4];
            if (!solveIK(point, joints)) {
                res->message = "Rotation group target is non-finite or unreachable";
                return;
            }
            targets.push_back(point);
            raw_angles.push_back(point.phi - joints[0]);
        }

        const double start_wrist = rotation_constraints::clamp(current_wrist_);
        std::vector<double> increasing, decreasing;
        const bool can_increase = rotation_constraints::solve(start_wrist, raw_angles, 1, increasing);
        const bool can_decrease = rotation_constraints::solve(start_wrist, raw_angles, -1, decreasing);
        if (!can_increase && !can_decrease) {
            res->message = "No single wrist direction can reach every target within [-2pi, 0]";
            return;
        }
        const int direction = can_increase && (!can_decrease ||
            std::abs(increasing.back() - start_wrist) <=
                std::abs(decreasing.back() - start_wrist)) ? 1 : -1;
        const auto &selected = direction > 0 ? increasing : decreasing;

        // Finish every route before exposing any plan or touching shared waypoints.
        std::vector<catchrobo2026_msgs::msg::RotationGroupRoute> routes;
        Point3D start_pose = cur_pose_;
        double route_start_wrist = start_wrist;
        size_t begin = 0;
        for (const auto end : req->route_ends) {
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
            auto samples = generate3DSpline(points);
            catchrobo2026_msgs::msg::RotationGroupRoute route;
            route.path.header.stamp = now();
            route.path.header.frame_id = "map";
            for (size_t i = 0; i < samples.size(); ++i) {
                const double scaled = static_cast<double>(i) * (points.size() - 1) /
                    static_cast<double>(samples.size() - 1);
                const size_t index = std::min(static_cast<size_t>(scaled), wrists.size() - 2);
                const double wrist = wrists[index] +
                    (wrists[index + 1] - wrists[index]) * (scaled - index);
                float joints[4];
                if (!solveIK(samples[i], joints)) {
                    res->message = "Rotation group spline leaves the reachable workspace";
                    return;
                }
                samples[i].phi = joints[0] + wrist;
                if (!solveIK(samples[i], joints)) {
                    res->message = "Rotation group spline has invalid joint angles";
                    return;
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
            start_pose = targets[end - 1];
            route_start_wrist = selected[end - 1];
            begin = end;
        }
        res->routes = std::move(routes);
        res->direction = direction;
        res->success = true;
        res->message = "Rotation group planned";
    }

    void waypointCallback(const std::shared_ptr<WaypointSrv::Request> req,
                          std::shared_ptr<WaypointSrv::Response> res) {
        waypoints_.push_back({req->x, req->y, req->z, req->phi});
        res->success = true;
        RCLCPP_INFO(this->get_logger(), "Added 3D Waypoint: [%.2f, %.2f, %.2f, phi: %.2f]", 
                    req->x, req->y, req->z, req->phi);
    }

    void genRouteCallback(const std::shared_ptr<GenRouteSrv::Request> req,
                          std::shared_ptr<GenRouteSrv::Response> res) {
        // Build locally so rejected requests cannot alter the legacy queue.
        res->success = false;
        std::vector<Point3D> route_points{cur_pose_};
        if (req->use_explicit_waypoints) {
            for (const auto &pose : req->waypoints) {
                const auto &q = pose.orientation;
                const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
                if (!std::isfinite(norm) || norm < 1e-12) {
                    RCLCPP_WARN(get_logger(), "Route waypoint has an invalid quaternion.");
                    return;
                }
                tf2::Quaternion orientation(q.x, q.y, q.z, q.w);
                orientation.normalize();
                const tf2::Matrix3x3 rotation(orientation);
                const double yaw = std::atan2(-rotation[0][1], rotation[1][1]);
                route_points.push_back({pose.position.x * 1000.0,
                    pose.position.y * 1000.0, pose.position.z * 1000.0, yaw});
            }
        } else {
            route_points.insert(route_points.end(), waypoints_.begin(), waypoints_.end());
        }
        route_points.push_back({req->x, req->y, req->z, req->phi});
        for (const auto &point : route_points) {
            if (!finitePoint(point)) {
                RCLCPP_WARN(get_logger(), "Route contains a non-finite position or angle.");
                return;
            }
        }

        // 点数が4点未満の場合、区間を3等分して中間の2点（1/3, 2/3地点）を挿入し、絶対に4点以上にする
        while (route_points.size() < 4) {
            route_points = densifyPoints(route_points);
        }

        // 3DスプラインとSlerpを用いた経路生成
        auto smoothed_path = generate3DSpline(route_points);
        for (const auto &point : smoothed_path) {
            if (!finitePoint(point)) {
                RCLCPP_WARN(get_logger(), "Route generation produced a non-finite pose.");
                return;
            }
        }
        res->path = publishPath(smoothed_path);

        // Explicit requests neither consume nor clear manual waypoints.
        if (!req->use_explicit_waypoints) waypoints_.clear();
        res->success = true;
        RCLCPP_INFO(this->get_logger(), "Route generation completed and published.");
    }

    static bool finitePoint(const Point3D &point) {
        return std::isfinite(point.x) && std::isfinite(point.y) &&
               std::isfinite(point.z) && std::isfinite(point.phi);
    }

    // 隣接する点の間を3等分して2点を作り出し、点数を増やす関数
    std::vector<Point3D> densifyPoints(const std::vector<Point3D>& points) {
        std::vector<Point3D> densified;
        if (points.size() < 2) return points;

        for (size_t i = 0; i < points.size() - 1; ++i) {
            Point3D p0 = points[i];
            Point3D p1 = points[i + 1];

            densified.push_back(p0);

            // 1/3 地点のポイントを生成
            densified.push_back(interpolatePoint(p0, p1, 1.0 / 3.0));

            // 2/3 地点のポイントを生成
            densified.push_back(interpolatePoint(p0, p1, 2.0 / 3.0));
        }
        densified.push_back(points.back());
        return densified;
    }

    // 位置と姿勢（Slerp）を考慮した内分点計算
    Point3D interpolatePoint(const Point3D& p0, const Point3D& p1, double ratio) {
        Point3D p;
        p.x = p0.x + (p1.x - p0.x) * ratio;
        p.y = p0.y + (p1.y - p0.y) * ratio;
        p.z = p0.z + (p1.z - p0.z) * ratio;

        // 姿勢(phi)はクォータニオンのSlerpで自然に補間
        tf2::Quaternion q0, q1;
        q0.setRPY(0, 0, p0.phi);
        q1.setRPY(0, 0, p1.phi);
        tf2::Quaternion q_interp = q0.slerp(q1, ratio);

        double roll, pitch, yaw;
        tf2::Matrix3x3(q_interp).getRPY(roll, pitch, yaw);
        p.phi = yaw;

        return p;
    }

    std::vector<Point3D> generate3DSpline(const std::vector<Point3D>& points) {
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

        // 3次スプライン曲線による位置情報のフィッティング
        const int degree = 3;
        Spline3D spline = Eigen::SplineFitting<Spline3D>::Interpolate(p_matrix, degree, u);

        std::vector<Point3D> smoothed;
        int total_samples = points.size() * sample_resolution_;
        
        for (int i = 0; i <= total_samples; ++i) {
            // tは経路全体の進行度合 (0.0 から 1.0)
            double t = static_cast<double>(i) / total_samples;
            Eigen::Vector3d pv = spline(t);

            // --- 姿勢(PHI)のSlerp補間 ---
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

    nav_msgs::msg::Path publishPath(const std::vector<Point3D>& path_points) {
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "map";

        visualization_msgs::msg::MarkerArray marker_array;
        
        visualization_msgs::msg::Marker del_marker;
        del_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(del_marker);

        for (size_t i = 0; i < path_points.size(); ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            
            pose.pose.position.x = path_points[i].x/1000;
            pose.pose.position.y = path_points[i].y/1000;
            pose.pose.position.z = path_points[i].z/1000;

            tf2::Quaternion q;
            q.setRPY(0, 0, path_points[i].phi);
            pose.pose.orientation = tf2::toMsg(q);
            
            path_msg.poses.push_back(pose);

            if (i % 10 == 0) {
                visualization_msgs::msg::Marker arrow;
                arrow.header = path_msg.header;
                arrow.ns = "path_orientations";
                arrow.id = static_cast<int>(i);
                arrow.type = visualization_msgs::msg::Marker::ARROW;
                arrow.action = visualization_msgs::msg::Marker::ADD;
                arrow.pose = pose.pose;
                
                arrow.scale.x = 0.05;
                arrow.scale.y = 0.01;
                arrow.scale.z = 0.01;

                arrow.color.r = 1.0f;
                arrow.color.g = 0.0f;
                arrow.color.b = 0.0f;
                arrow.color.a = 1.0f;
                
                marker_array.markers.push_back(arrow);
            }
        }
        pub_path_->publish(path_msg);
        pub_marker_->publish(marker_array);
        return path_msg;
    }

    Point3D cur_pose_;
    std::vector<Point3D> waypoints_;
    int sample_resolution_;
    double rotation_joint_timeout_sec_;
    double current_wrist_ = 0.0;
    bool joints_valid_ = false;
    std::chrono::steady_clock::time_point joints_received_at_;
    robot_kinematics kin_; // 追加: 運動学インスタンス

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_joints_; // 変更: PoseStampedから変更
    rclcpp::Service<WaypointSrv>::SharedPtr srv_waypoint_;
    rclcpp::Service<GenRouteSrv>::SharedPtr srv_gen_route_;
    rclcpp::Service<PlanRotationGroup>::SharedPtr srv_plan_rotation_group_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathGenerator3D>());
    rclcpp::shutdown();
    return 0;
}
