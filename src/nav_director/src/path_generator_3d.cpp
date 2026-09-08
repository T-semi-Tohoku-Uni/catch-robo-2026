#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp> // 追加: ジョイントメッセージ用
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <chrono>
#include <limits>
#include <stdexcept>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

// 独自メッセージパッケージと運動学ライブラリのヘッダー
#include <catchrobo2026_msgs/srv/waypoint.hpp>
#include <catchrobo2026_msgs/srv/generate_route.hpp>
#include <catchrobo2026_msgs/srv/plan_rotation_group.hpp>
#include "rotation_constraints.hpp"
#include "nav_director/route_planner.hpp"

using WaypointSrv = catchrobo2026_msgs::srv::Waypoint;
using GenRouteSrv = catchrobo2026_msgs::srv::GenerateRoute;
using PlanRotationGroup = catchrobo2026_msgs::srv::PlanRotationGroup;

using nav_director::Point3D;

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

        try {
            const auto state = planner_.stateFromJoints(
                {joint_angles[0], joint_angles[1], joint_angles[2], joint_angles[3]});
            cur_pose_ = state.pose;
            current_wrist_ = state.wrist;
            current_base_ = state.base;
        } catch (const std::runtime_error &) {
            joints_valid_ = false;
            return;
        }
        joints_valid_ = true;
        joints_received_at_ = std::chrono::steady_clock::now();
    }

    void planRotationGroup(const std::shared_ptr<PlanRotationGroup::Request> req,
                           std::shared_ptr<PlanRotationGroup::Response> res) {
        res->success = false;
        if (req->limit_phi_travel && (!std::isfinite(req->max_phi_travel) ||
            req->max_phi_travel < 0.0)) {
            res->message = "Phi travel limit must be finite and nonnegative";
            return;
        }
        if (!joints_valid_ || !rotation_constraints::legal(current_wrist_) ||
            std::chrono::duration<double>(std::chrono::steady_clock::now() -
                joints_received_at_).count() > rotation_joint_timeout_sec_) {
            res->message = "Fresh finite current_joints within the wrist limits are required";
            return;
        }
        *res = planner_.planGroup({cur_pose_, current_base_, current_wrist_}, *req);
        for (auto &route : res->routes) {
            route.path.header.stamp = now();
            for (auto &pose : route.path.poses) pose.header = route.path.header;
        }
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
        std::vector<Point3D> route_points;
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
        std::vector<Point3D> smoothed_path;
        try {
            smoothed_path = planner_.generateRoute(
                {cur_pose_, current_base_, current_wrist_}, route_points);
        } catch (const std::runtime_error &error) {
            RCLCPP_WARN(get_logger(), "%s", error.what());
            return;
        }
        res->path = publishPath(smoothed_path);

        // Explicit requests neither consume nor clear manual waypoints.
        if (!req->use_explicit_waypoints) waypoints_.clear();
        res->success = true;
        RCLCPP_INFO(this->get_logger(), "Route generation completed and published.");
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
    double rotation_joint_timeout_sec_;
    double current_wrist_ = 0.0;
    double current_base_ = 0.0;
    bool joints_valid_ = false;
    std::chrono::steady_clock::time_point joints_received_at_;
    nav_director::RoutePlanner planner_;

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
