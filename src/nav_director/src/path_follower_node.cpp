#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp> // 追加: PoseStamped用
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <algorithm>
#include <chrono>
#include <limits>
#include <stdexcept>
#include <atomic>
#include <thread>
#include <cmath>
#include <mutex>
#include <vector>

// 提供された運動学ライブラリと独自メッセージ
#include "ros2_inverse_kinematics/robot_kinematics.h"
#include "catchrobo2026_msgs/action/follow_route.hpp"
#include "catchrobo2026_msgs/srv/wrist_control.hpp"
#include "rotation_constraints.hpp"

using FollowRoute = catchrobo2026_msgs::action::FollowRoute;
using GoalHandleFollowRoute = rclcpp_action::ServerGoalHandle<FollowRoute>;
using WristControl = catchrobo2026_msgs::srv::WristControl;

class PathFollowerNode : public rclcpp::Node {
public:
    PathFollowerNode() : Node("path_follower_node") {
        lookahead_near_mm_ = declare_parameter("lookahead_near_mm", 40.0);
        lookahead_far_mm_ = declare_parameter("lookahead_far_mm", 150.0);
        goal_distance_near_mm_ = declare_parameter("goal_distance_near_mm", 100.0);
        goal_distance_far_mm_ = declare_parameter("goal_distance_far_mm", 500.0);
        lookahead_smoothing_sec_ = declare_parameter("lookahead_smoothing_sec", 0.2);
        rotation_joint_timeout_sec_ = declare_parameter("rotation_joint_timeout_sec", 1.0);
        rotation_speed_rad_sec_ = declare_parameter("rotation_speed_rad_sec", 1.0);
        if (!std::isfinite(rotation_joint_timeout_sec_) || rotation_joint_timeout_sec_ <= 0.0 ||
            !std::isfinite(rotation_speed_rad_sec_) || rotation_speed_rad_sec_ <= 0.0) {
            throw std::invalid_argument("Invalid rotation group parameters");
        }
        wrist_client_ = create_client<WristControl>("wrist_control");
        if (!std::isfinite(lookahead_near_mm_) || !std::isfinite(lookahead_far_mm_) ||
            !std::isfinite(goal_distance_near_mm_) || !std::isfinite(goal_distance_far_mm_) ||
            !std::isfinite(lookahead_smoothing_sec_) || lookahead_near_mm_ <= 0.0 ||
            lookahead_far_mm_ < lookahead_near_mm_ || goal_distance_near_mm_ < 0.0 ||
            goal_distance_far_mm_ <= goal_distance_near_mm_ || lookahead_smoothing_sec_ <= 0.0) {
            throw std::invalid_argument("Invalid lookahead parameters");
        }

        // パブリッシャーとサブスクライバーの初期化
        pub_joints_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("target_joint_angles", 10);
        
        // 追加: target_pose用のパブリッシャー
        pub_target_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", 10);
        pub_lookahead_marker_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "lookahead_marker", 10);
        
        sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
            "route", 10, std::bind(&PathFollowerNode::pathCallback, this, std::placeholders::_1));
            
        sub_current_joints_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "current_joints", 10, std::bind(&PathFollowerNode::jointCallback, this, std::placeholders::_1));

        // アクションサーバーの初期化 (20Hzでの制御ループ用)
        action_server_ = rclcpp_action::create_server<FollowRoute>(
            this,
            "follow_route",
            std::bind(&PathFollowerNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&PathFollowerNode::handleCancel, this, std::placeholders::_1),
            std::bind(&PathFollowerNode::handleAccepted, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Path Follower Node Initialized.");
    }

    ~PathFollowerNode() override {
        stopping_ = true;
        if (worker_.joinable()) worker_.join();
    }

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(path_mutex_);
        current_path_ = *msg;
    }

    void jointCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(joint_mutex_);
        joints_valid_ = msg->data.size() >= 4 &&
            std::all_of(msg->data.begin(), msg->data.begin() + 4,
                [](float value) { return std::isfinite(value); });
        if (joints_valid_) {
            for (int i = 0; i < 4; ++i) {
                current_joint_angles_[i] = msg->data[i];
            }
            joints_received_at_ = std::chrono::steady_clock::now();
        }
    }

    rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const FollowRoute::Goal> goal) {
        (void)uuid;
        if (!goal->start || busy_ || stopping_) {
            return rclcpp_action::GoalResponse::REJECT;
        }
        const bool constrained = goal->rotation_group_id != 0;
        if (constrained) {
            if (goal->path.poses.empty() || goal->wrist_angles.size() != goal->path.poses.size() ||
                goal->wrist_direction < -1 || goal->wrist_direction > 1 ||
                (!goal->phi_angles.empty() &&
                 goal->phi_angles.size() != goal->path.poses.size())) {
                return rclcpp_action::GoalResponse::REJECT;
            }
            if (!std::all_of(goal->phi_angles.begin(), goal->phi_angles.end(),
                    [](double value) { return std::isfinite(value); })) {
                return rclcpp_action::GoalResponse::REJECT;
            }
            for (size_t i = 0; i < goal->wrist_angles.size(); ++i) {
                if (!rotation_constraints::legal(goal->wrist_angles[i]) ||
                    (i > 0 && goal->wrist_direction *
                        (goal->wrist_angles[i] - goal->wrist_angles[i - 1]) <
                            -rotation_constraints::kTolerance)) {
                    return rclcpp_action::GoalResponse::REJECT;
                }
            }
            std::lock_guard<std::mutex> lock(joint_mutex_);
            if (!freshJoints() || std::abs(current_joint_angles_[3] -
                    goal->wrist_angles.front()) > 0.05) {
                return rclcpp_action::GoalResponse::REJECT;
            }
        } else if (!goal->wrist_angles.empty() || !goal->phi_angles.empty() ||
                   goal->wrist_direction != 0) {
            return rclcpp_action::GoalResponse::REJECT;
        }
        // Empty goals retain the manual API, using a snapshot at acceptance.
        auto path = goal->path;
        if (path.poses.empty()) {
            std::lock_guard<std::mutex> lock(path_mutex_);
            path = current_path_;
        }
        if (path.poses.empty()) {
            return rclcpp_action::GoalResponse::REJECT;
        }
        for (const auto &pose : path.poses) {
            const auto &p = pose.pose.position;
            const auto &q = pose.pose.orientation;
            const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
                !std::isfinite(norm) || norm < 1e-12) {
                return rclcpp_action::GoalResponse::REJECT;
            }
        }
        if (!goal->phi_angles.empty()) {
            try {
                for (size_t i = 0; i < path.poses.size(); ++i) {
                    const double base = baseAt(path, i);
                    tf2::Quaternion orientation;
                    tf2::fromMsg(path.poses[i].pose.orientation, orientation);
                    orientation.normalize();
                    const auto y_axis = tf2::Matrix3x3(orientation).getColumn(1);
                    const double pose_phi = std::atan2(-y_axis.x(), y_axis.y());
                    if (std::hypot(y_axis.x(), y_axis.y()) < 1e-6 ||
                        std::abs(std::remainder(goal->phi_angles[i] - pose_phi, 2.0 * M_PI)) >
                            1e-4 ||
                        std::abs(goal->phi_angles[i] - base - goal->wrist_angles[i]) > 1e-4) {
                        return rclcpp_action::GoalResponse::REJECT;
                    }
                    const auto &a = path.poses[i == 0 ? i : i - 1].pose.position;
                    const auto &b = path.poses[i].pose.position;
                    if (!rotation_constraints::legal_phi_segment(
                            a.x * 1000.0 - robot_pos[0], a.y * 1000.0 - robot_pos[1],
                            b.x * 1000.0 - robot_pos[0], b.y * 1000.0 - robot_pos[1],
                            goal->phi_angles[i == 0 ? i : i - 1], goal->phi_angles[i],
                            goal->wrist_direction)) {
                        return rclcpp_action::GoalResponse::REJECT;
                    }
                }
            } catch (const std::exception &) {
                return rclcpp_action::GoalResponse::REJECT;
            }
        }
        accepted_path_ = std::move(path);
        busy_ = true;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // The caller holds joint_mutex_.
    bool freshJoints() const {
        return joints_valid_ && rotation_constraints::legal(current_joint_angles_[3]) &&
            std::chrono::duration<double>(std::chrono::steady_clock::now() -
                joints_received_at_).count() <= rotation_joint_timeout_sec_;
    }

    static double angleAt(const std::vector<double> &angles, double progress) {
        if (angles.size() == 1) return angles.front();
        const size_t index = std::min(static_cast<size_t>(progress), angles.size() - 2);
        return angles[index] + (angles[index + 1] - angles[index]) * (progress - index);
    }

    static double wristAt(const std::vector<double> &angles, double progress) {
        return rotation_constraints::clamp(angleAt(angles, progress));
    }

    static geometry_msgs::msg::Pose poseAt(const nav_msgs::msg::Path &path, double progress) {
        if (path.poses.size() == 1) return path.poses.front().pose;
        const size_t index = std::min(static_cast<size_t>(progress), path.poses.size() - 2);
        const double t = progress - index;
        const auto &a = path.poses[index].pose;
        const auto &b = path.poses[index + 1].pose;
        auto pose = a;
        pose.position.x += t * (b.position.x - a.position.x);
        pose.position.y += t * (b.position.y - a.position.y);
        pose.position.z += t * (b.position.z - a.position.z);
        tf2::Quaternion qa, qb;
        tf2::fromMsg(a.orientation, qa);
        tf2::fromMsg(b.orientation, qb);
        qa.normalize();
        qb.normalize();
        pose.orientation = tf2::toMsg(qa.slerp(qb, t));
        return pose;
    }

    double baseAt(const nav_msgs::msg::Path &path, double progress) {
        const auto pose = poseAt(path, progress);
        float position[6] = {static_cast<float>(pose.position.x * 1000.0),
            static_cast<float>(pose.position.y * 1000.0),
            static_cast<float>(pose.position.z * 1000.0), 0.0F,
            static_cast<float>(-M_PI / 2.0), 0.0F};
        float joints[4];
        kin_.inverse_kinematics(position, joints);
        if (!std::all_of(joints, joints + 4,
                [](float value) { return std::isfinite(value); })) {
            throw std::runtime_error("Constrained route target is unreachable");
        }
        return joints[0];
    }

    double limitedProgress(const FollowRoute::Goal &goal, const nav_msgs::msg::Path &path,
                           double start, double desired, double budget) {
        const bool interpolate_phi = !goal.phi_angles.empty();
        double progress = start;
        while (progress < desired) {
            const double end = std::min(desired, std::floor(progress) + 1.0);
            const auto cost = [&](double target) {
                if (!interpolate_phi) {
                    return std::abs(wristAt(goal.wrist_angles, target) -
                        wristAt(goal.wrist_angles, progress));
                }
                // On each valid XY segment, base angle is monotonic.
                return std::abs(angleAt(goal.phi_angles, target) -
                    angleAt(goal.phi_angles, progress)) +
                    std::abs(baseAt(path, target) - baseAt(path, progress));
            };
            const double segment_cost = cost(end);
            if (segment_cost > budget) {
                double low = progress;
                double high = end;
                for (int iteration = 0; iteration < 50; ++iteration) {
                    const double middle = (low + high) / 2.0;
                    if (cost(middle) <= budget) low = middle;
                    else high = middle;
                }
                return low;
            }
            budget -= segment_cost;
            progress = end;
        }
        return progress;
    }

    bool sendWristTarget(const std::shared_ptr<GoalHandleFollowRoute> &goal_handle,
                         const geometry_msgs::msg::PoseStamped &target, double wrist) {
        if (goal_handle->is_canceling() || stopping_ || !wrist_client_->service_is_ready()) {
            return false;
        }
        auto request = std::make_shared<WristControl::Request>();
        request->operation = WristControl::Request::TARGET;
        request->group_id = goal_handle->get_goal()->rotation_group_id;
        request->direction = goal_handle->get_goal()->wrist_direction;
        request->wrist_angle = wrist;
        request->target = target;
        auto pending = wrist_client_->async_send_request(request);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
        while (rclcpp::ok() && !stopping_ && !goal_handle->is_canceling()) {
            if (pending.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready) {
                const auto response = pending.get();
                if (!response->success) {
                    RCLCPP_ERROR(get_logger(), "Wrist target rejected: %s", response->message.c_str());
                }
                return response->success;
            }
            if (std::chrono::steady_clock::now() >= deadline) break;
        }
        wrist_client_->remove_pending_request(pending);
        return false;
    }

    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleFollowRoute> goal_handle) {
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(const std::shared_ptr<GoalHandleFollowRoute> goal_handle) {
        if (worker_.joinable()) worker_.join();
        worker_ = std::thread([this, goal_handle, path = std::move(accepted_path_)]() {
            try {
                executeLoop(goal_handle, path);
            } catch (const std::exception &error) {
                RCLCPP_ERROR(get_logger(), "Route execution failed: %s", error.what());
                if (rclcpp::ok() && goal_handle->is_active()) {
                    auto result = std::make_shared<FollowRoute::Result>();
                    result->success = false;
                    busy_ = false;
                    try {
                        goal_handle->abort(result);
                    } catch (const std::exception &terminal_error) {
                        RCLCPP_ERROR(get_logger(), "Route abort failed: %s", terminal_error.what());
                    }
                }
            }
        });
    }

    void executeLoop(const std::shared_ptr<GoalHandleFollowRoute> goal_handle,
                     const nav_msgs::msg::Path &local_path) {
        size_t current_path_index = 0;
        rclcpp::Rate loop_rate(20); // 20Hzで実行
        auto feedback = std::make_shared<FollowRoute::Feedback>();
        auto result = std::make_shared<FollowRoute::Result>();
        const auto goal = goal_handle->get_goal();
        const bool constrained = goal->rotation_group_id != 0;
        const auto &wrist_angles = goal->wrist_angles;
        double commanded_progress = 0.0;

        bool lookahead_initialized = false;
        double smoothed_lookahead = lookahead_near_mm_;
        auto last_update = std::chrono::steady_clock::now();

        while (rclcpp::ok() && !stopping_) {
            if (goal_handle->is_canceling()) {
                result->success = false;
                busy_ = false;
                goal_handle->canceled(result);
                return;
            }

            float current_posrot[6] = {0.0};
            float target_posrot[6] = {0.0};
            float target_joints[4] = {0.0};
            
            float local_joints[4];

            // データの排他制御コピー
            {
                std::lock_guard<std::mutex> lock_j(joint_mutex_);
                if (constrained && !freshJoints()) {
                    throw std::runtime_error("Fresh current_joints required during rotation group");
                }
                for(int i=0; i<4; i++) local_joints[i] = current_joint_angles_[i];
            }

            // 1. 順運動学で現在地を計算
            kin_.forward_kinematics(current_posrot, local_joints);
            
            // cx, cy, cz は [mm] 単位
            double cx = current_posrot[0]; 
            double cy = current_posrot[1]; 
            double cz = current_posrot[2]; 

            const auto& final_pose = local_path.poses.back().pose;
            const double goal_distance = std::hypot(
                std::hypot(final_pose.position.x * 1000.0 - cx,
                           final_pose.position.y * 1000.0 - cy),
                final_pose.position.z * 1000.0 - cz);

            // 終点までの直線距離を smoothstep で先読み距離に変換する。
            const double ratio = std::max(0.0, std::min(1.0,
                (goal_distance - goal_distance_near_mm_) /
                (goal_distance_far_mm_ - goal_distance_near_mm_)));
            const double blend = ratio * ratio * (3.0 - 2.0 * ratio);
            const double desired_lookahead = lookahead_near_mm_ +
                blend * (lookahead_far_mm_ - lookahead_near_mm_);
            const auto now = std::chrono::steady_clock::now();
            const double rotation_dt = std::max(0.0, std::min(0.05,
                std::chrono::duration<double>(now - last_update).count()));
            if (!lookahead_initialized) {
                lookahead_initialized = true;
                smoothed_lookahead = desired_lookahead;
            } else {
                const double dt = std::chrono::duration<double>(now - last_update).count();
                smoothed_lookahead += (1.0 - std::exp(-dt / lookahead_smoothing_sec_)) *
                    (desired_lookahead - smoothed_lookahead);
            }
            last_update = now;

            // 現在地を経路の線分に射影。先読み目標ではなく足元の線分を記録し、
            // 先読み距離が短くなったときも適切な位置を選べるようにする。
            double nearest_distance_sq = std::numeric_limits<double>::max();
            double segment_fraction = 0.0;
            for (size_t i = current_path_index; i + 1 < local_path.poses.size(); ++i) {
                const auto& a = local_path.poses[i].pose.position;
                const auto& b = local_path.poses[i + 1].pose.position;
                const double vx = (b.x - a.x) * 1000.0;
                const double vy = (b.y - a.y) * 1000.0;
                const double vz = (b.z - a.z) * 1000.0;
                const double length_sq = vx * vx + vy * vy + vz * vz;
                const double t = length_sq > 0.0 ? std::max(0.0, std::min(1.0,
                    ((cx - a.x * 1000.0) * vx + (cy - a.y * 1000.0) * vy +
                     (cz - a.z * 1000.0) * vz) / length_sq)) : 0.0;
                const double dx = a.x * 1000.0 + t * vx - cx;
                const double dy = a.y * 1000.0 + t * vy - cy;
                const double dz = a.z * 1000.0 + t * vz - cz;
                const double distance_sq = dx * dx + dy * dy + dz * dz;
                if (distance_sq < nearest_distance_sq) {
                    nearest_distance_sq = distance_sq;
                    current_path_index = i;
                    segment_fraction = t;
                }
            }

            // 経路に沿って先読み距離だけ進み、線分内の位置・姿勢を補間する。
            auto target_pose = final_pose;
            bool targeting_final = true;
            double target_progress = local_path.poses.size() - 1;
            double remaining = smoothed_lookahead;
            for (size_t i = current_path_index; i + 1 < local_path.poses.size(); ++i) {
                const auto& a = local_path.poses[i].pose;
                const auto& b = local_path.poses[i + 1].pose;
                const double length = 1000.0 * std::hypot(
                    std::hypot(b.position.x - a.position.x, b.position.y - a.position.y),
                    b.position.z - a.position.z);
                const double start = i == current_path_index ? segment_fraction : 0.0;
                const double available = length * (1.0 - start);
                if (length > 0.0 && remaining < available) {
                    const double t = start + remaining / length;
                    target_pose.position.x = a.position.x + t * (b.position.x - a.position.x);
                    target_pose.position.y = a.position.y + t * (b.position.y - a.position.y);
                    target_pose.position.z = a.position.z + t * (b.position.z - a.position.z);
                    tf2::Quaternion qa, qb;
                    tf2::fromMsg(a.orientation, qa);
                    tf2::fromMsg(b.orientation, qb);
                    target_pose.orientation = tf2::toMsg(qa.slerp(qb, t));
                    target_progress = i + t;
                    targeting_final = false;
                    break;
                }
                remaining -= available;
            }

            double target_wrist = 0.0;
            if (constrained) {
                // Keep XYZ and wrist on the same monotonically advancing path parameter.
                target_progress = std::max(commanded_progress, target_progress);
                const double max_step = rotation_speed_rad_sec_ * rotation_dt;
                target_progress = limitedProgress(*goal, local_path,
                    commanded_progress, target_progress, max_step);
                target_wrist = goal->phi_angles.empty() ? wristAt(wrist_angles, target_progress) :
                    angleAt(goal->phi_angles, target_progress) - baseAt(local_path, target_progress);
                target_pose = poseAt(local_path, target_progress);
                targeting_final = target_progress >= local_path.poses.size() - 1 - 1e-9;
            }

            target_posrot[0] = target_pose.position.x * 1000.0;
            target_posrot[1] = target_pose.position.y * 1000.0;
            target_posrot[2] = target_pose.position.z * 1000.0;
            tf2::Quaternion q;
            tf2::fromMsg(target_pose.orientation, q);
            // roll = 0 の姿勢規約では、手先の Y 軸は pitch によらず
            // (-sin(yaw), cos(yaw), 0)。pitch = -PI/2 で RPY が一意に
            // 定まらない場合も、この軸から同じ物理的な向きを取り出せる。
            const tf2::Matrix3x3 target_rotation(q);
            double target_yaw = std::atan2(
                -target_rotation[0][1], target_rotation[1][1]);
            target_posrot[3] = static_cast<float>(target_yaw);
            target_posrot[4] = -M_PI / 2.0F;
            target_posrot[5] = 0.0F;
            if (constrained) {
                kin_.inverse_kinematics(target_posrot, target_joints);
                if (!std::all_of(target_joints, target_joints + 4,
                        [](float value) { return std::isfinite(value); })) {
                    throw std::runtime_error("Constrained route target is unreachable");
                }
                target_yaw = target_joints[0] + target_wrist;
                target_posrot[3] = static_cast<float>(target_yaw);
            }

            // 2PI の整数倍だけ異なる角度を同一視した最短の符号付き角度差。
            const double err_yaw = std::atan2(
                std::sin(target_yaw - current_posrot[3]),
                std::cos(target_yaw - current_posrot[3]));
            if (targeting_final && goal_distance <= 20.0 && std::abs(err_yaw) <= 0.05 &&
                (!constrained || (commanded_progress >= local_path.poses.size() - 1 - 1e-9 &&
                    std::abs(local_joints[3] - wrist_angles.back()) <= 0.05))) {
                result->success = true;
                busy_ = false;
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Reached the end of the path with correct position and orientation.");
                return;
            }

            // ===============================================================
            // 【追加】現在の目標位置と現在位置との誤差を計算しINFO出力
            // ===============================================================
            double err_x = target_posrot[0] - current_posrot[0];
            double err_y = target_posrot[1] - current_posrot[1];
            double err_z = target_posrot[2] - current_posrot[2];
            double err_dist = std::sqrt(err_x * err_x + err_y * err_y + err_z * err_z);
            
            // 1秒(1000ms)に1回だけ出力 (スパム防止)
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Error -> Dist: %6.2f [mm] (X:%6.2f, Y:%6.2f, Z:%6.2f) | Yaw: %6.4f [rad]",
                err_dist, err_x, err_y, err_z, err_yaw);
            // ===============================================================


            // 3. 逆運動学で目標ジョイント角を計算
            kin_.inverse_kinematics(target_posrot, target_joints);

            // 4. Float32MultiArrayで出力
            std_msgs::msg::Float32MultiArray msg_out;
            msg_out.data.resize(4);
            for (int i = 0; i < 4; ++i) {
                msg_out.data[i] = target_joints[i];
            }
            //pub_joints_->publish(msg_out);

            // 5. target_poseをPoseStampedで出力
            geometry_msgs::msg::PoseStamped target_pose_msg;
            target_pose_msg.header.stamp = this->now();
            
            // パスと同じフレームIDを使用。設定されていなければ"map"をデフォルトにする
            target_pose_msg.header.frame_id = local_path.header.frame_id; 
            if (target_pose_msg.header.frame_id.empty()) {
                target_pose_msg.header.frame_id = "map"; 
            }

            // 単位を [mm] からROSの標準である [m] に戻す
            target_pose_msg.pose.position.x = target_posrot[0] / 1000.0;
            target_pose_msg.pose.position.y = target_posrot[1] / 1000.0;
            target_pose_msg.pose.position.z = target_posrot[2] / 1000.0;

            // Roll, Pitch, Yawからクォータニオンへ変換
            tf2::Quaternion q_out;
            // 順序は (roll, pitch, yaw) : (PSI, THE, PHI) = (target_posrot[5], target_posrot[4], target_posrot[3])
            q_out.setRPY(target_posrot[5], target_posrot[4], target_posrot[3]);
            target_pose_msg.pose.orientation = tf2::toMsg(q_out);

            if (constrained) {
                if (!sendWristTarget(goal_handle, target_pose_msg, target_wrist)) {
                    if (goal_handle->is_canceling()) {
                        result->success = false;
                        busy_ = false;
                        goal_handle->canceled(result);
                        return;
                    }
                    throw std::runtime_error("Wrist target was rejected or timed out");
                }
                commanded_progress = target_progress;
            } else {
                pub_target_pose_->publish(target_pose_msg);
            }

            // 制御に使用した先読み目標位置を直径50mmの球で表示する。
            visualization_msgs::msg::Marker marker;
            marker.header = target_pose_msg.header;
            marker.ns = "lookahead_target";
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position = target_pose_msg.pose.position;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = marker.scale.y = marker.scale.z = 0.05;
            marker.color.r = 1.0;
            marker.color.g = 0.4;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            // 完了・キャンセル・異常終了後に古い参照点を残さない。
            marker.lifetime = rclcpp::Duration::from_seconds(0.3);
            pub_lookahead_marker_->publish(marker);

            // フィードバックの送信
            feedback->distance_remaining = local_path.poses.size() - current_path_index;
            goal_handle->publish_feedback(feedback);

            loop_rate.sleep();
        }
    }

    robot_kinematics kin_; // 運動学クラスのインスタンス
    
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_joints_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_target_pose_; // 追加: ターゲット姿勢用
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_lookahead_marker_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_current_joints_;
    rclcpp_action::Server<FollowRoute>::SharedPtr action_server_;
    rclcpp::Client<WristControl>::SharedPtr wrist_client_;

    std::mutex path_mutex_;
    std::mutex joint_mutex_;
    nav_msgs::msg::Path current_path_;
    float current_joint_angles_[4] = {0.0};
    bool joints_valid_ = false;
    std::chrono::steady_clock::time_point joints_received_at_;
    nav_msgs::msg::Path accepted_path_;
    std::atomic<bool> busy_{false};
    std::atomic<bool> stopping_{false};
    std::thread worker_;
    double lookahead_near_mm_;
    double lookahead_far_mm_;
    double goal_distance_near_mm_;
    double goal_distance_far_mm_;
    double lookahead_smoothing_sec_;
    double rotation_joint_timeout_sec_;
    double rotation_speed_rad_sec_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathFollowerNode>());
    rclcpp::shutdown();
    return 0;
}
