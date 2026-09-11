#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
// 追加: マーカー用インクルード
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "ros2_inverse_kinematics/robot_kinematics.h"

// サービス型のインクルード
#include "catchrobo2026_msgs/srv/pump_control.hpp"
#include "catchrobo2026_msgs/srv/endeffector_control.hpp"
#include "catchrobo2026_msgs/srv/wrist_control.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class JoyControllerNode : public rclcpp::Node {
public:
    JoyControllerNode() : Node("joy_controller_node") {
        
        // 1. パブリッシャー (各モーターの4つの角度を出力)
        joint_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("target_joint_angles", 10);
        
        // 追加: マーカーパブリッシャー
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("target_arm_markers", 10);

        // 2. サブスクライバー
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&JoyControllerNode::joy_callback, this, std::placeholders::_1));
            
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "target_pose", 10, std::bind(&JoyControllerNode::pose_callback, this, std::placeholders::_1));

        current_joints_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "current_joints", 10,
            std::bind(&JoyControllerNode::current_joints_callback, this, std::placeholders::_1));
            
        endeffector_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "endeffector_state", 10, std::bind(&JoyControllerNode::endeffector_callback, this, std::placeholders::_1));

        // 3. サービスクライアント
        pump_client_ = this->create_client<catchrobo2026_msgs::srv::PumpControl>("set_pump_state");
        endeffector_client_ = this->create_client<catchrobo2026_msgs::srv::EndeffectorControl>("set_endeffector_state");
        state_client_ = this->create_client<std_srvs::srv::Trigger>("request_initialization");

        rotation_joint_timeout_sec_ = declare_parameter("rotation_joint_timeout_sec", 1.0);
        if (!std::isfinite(rotation_joint_timeout_sec_) || rotation_joint_timeout_sec_ <= 0.0) {
            throw std::invalid_argument("rotation_joint_timeout_sec must be finite and positive");
        }
        const double wrist_feedback_tolerance_deg =
            declare_parameter("wrist_feedback_tolerance_deg", 6.0);
        if (!std::isfinite(wrist_feedback_tolerance_deg) ||
            wrist_feedback_tolerance_deg < 0.0 || wrist_feedback_tolerance_deg >= 180.0) {
            throw std::invalid_argument("wrist_feedback_tolerance_deg must be finite in [0, 180)");
        }
        wrist_feedback_tolerance_rad_ = wrist_feedback_tolerance_deg * M_PI / 180.0;
        require_current_joints_on_start_ =
            declare_parameter("require_current_joints_on_start", false);
        wrist_service_ = create_service<WristControl>("wrist_control",
            std::bind(&JoyControllerNode::wrist_callback, this,
                std::placeholders::_1, std::placeholders::_2));

        // 4. IK計算とパブリッシュを行うメインループタイマー (例: 20ms = 50Hz)
        publish_timer_ = this->create_wall_timer(
            2ms, std::bind(&JoyControllerNode::publish_timer_callback, this));

        // 目標座標の初期値設定 [mm] および [rad]
        current_pose_[0] = 600.0f;  // X
        current_pose_[1] = 200.0f;  // Y
        current_pose_[2] = 200.0f;  // Z
        current_pose_[3] = 0.0f;    // Phi (Yaw相当)
        current_pose_[4] = -M_PI / 2.0f; // Theta (Pitch相当、デフォルト姿勢)
        current_pose_[5] = 0.0f;    // Psi (Roll相当)

        RCLCPP_INFO(this->get_logger(), "Joy Controller Node started with integrated IK, Pump, Endeffector control, and Markers.");
    }

private:
    using PumpRequest = catchrobo2026_msgs::srv::PumpControl::Request;
    using WristControl = catchrobo2026_msgs::srv::WristControl;

    static bool wrist_in_range(double angle) {
        return std::isfinite(angle) && angle >= -2.0 * M_PI - 1e-5 && angle <= 1e-5;
    }

    bool wrist_feedback_in_range(double angle) const {
        return std::isfinite(angle) &&
            angle >= -2.0 * M_PI - wrist_feedback_tolerance_rad_ - 1e-6 &&
            angle <= wrist_feedback_tolerance_rad_ + 1e-6;
    }

    static bool read_pose(const geometry_msgs::msg::PoseStamped &msg, float (&pose)[6]) {
        const auto &p = msg.pose.position;
        const auto &q = msg.pose.orientation;
        const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
            !std::isfinite(norm) || std::abs(norm - 1.0) > 1e-3) {
            return false;
        }
        tf2::Quaternion rotation(q.x, q.y, q.z, q.w);
        rotation.normalize();
        const auto y_axis = tf2::Matrix3x3(rotation).getColumn(1);
        if (std::hypot(y_axis.x(), y_axis.y()) < 1e-6) {
            return false;
        }
        pose[0] = static_cast<float>(p.x * 1000.0);
        pose[1] = static_cast<float>(p.y * 1000.0);
        pose[2] = static_cast<float>(p.z * 1000.0);
        pose[3] = static_cast<float>(std::atan2(-y_axis.x(), y_axis.y()));
        pose[4] = -static_cast<float>(M_PI) / 2.0f;
        pose[5] = 0.0f;
        return std::all_of(std::begin(pose), std::end(pose),
            [](float value) { return std::isfinite(value); });
    }

    void clear_velocity() {
        vel_x_ = vel_y_ = vel_z_ = vel_phi_ = 0.0f;
    }

    void current_joints_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        current_joints_valid_ = msg->data.size() >= 4 &&
            std::all_of(msg->data.begin(), msg->data.begin() + 4,
                [](float value) { return std::isfinite(value); });
        if (current_joints_valid_) {
            std::copy_n(msg->data.begin(), 4, current_joints_.begin());
            current_joints_time_ = std::chrono::steady_clock::now();
            if (require_current_joints_on_start_ && !startup_feedback_received_ &&
                wrist_feedback_in_range(current_joints_[3])) {
                hold_joints_ = current_joints_;
                hold_joints_[3] = static_cast<float>(std::clamp(
                    static_cast<double>(hold_joints_[3]), -2.0 * M_PI, 0.0));
                float actual_pose[6];
                kin_.forward_kinematics(actual_pose, hold_joints_.data());
                if (!std::all_of(std::begin(actual_pose), std::end(actual_pose),
                        [](float value) { return std::isfinite(value); })) {
                    RCLCPP_ERROR(get_logger(),
                        "Initial current_joints cannot produce a finite hold pose");
                    return;
                }
                std::copy(std::begin(actual_pose), std::end(actual_pose), current_pose_);
                hold_joints_active_ = true;
                startup_feedback_received_ = true;
                clear_velocity();
                RCLCPP_INFO(get_logger(), "Holding initial current_joints after startup");
            }
        }
    }

    void wrist_callback(const std::shared_ptr<WristControl::Request> request,
                        std::shared_ptr<WristControl::Response> response) {
        response->success = false;
        if (request->operation == WristControl::Request::END) {
            if (request->group_id == 0 ||
                (rotation_active_ && request->group_id != rotation_group_id_) ||
                (!rotation_active_ && request->group_id != last_ended_group_id_)) {
                response->message = "Rotation group ID does not match";
                return;
            }
            if (!rotation_active_) {
                response->success = true;
                response->message = "Rotation group already ended";
                return;
            }
            rotation_active_ = false;
            phi_interval_active_ = false;
            last_ended_group_id_ = request->group_id;
            rotation_fault_.clear();
            clear_velocity();
            response->success = true;
            response->message = "Rotation group ended; holding last joint command";
            return;
        }

        if (request->operation == WristControl::Request::BEGIN) {
            if (rotation_active_ || state_request_pending_) {
                response->message = "A rotation group or initialization request is active";
                return;
            }
            if (request->group_id == 0 || request->group_id <= highest_group_id_) {
                response->message = "Rotation group ID is zero or stale";
                return;
            }
            if (request->direction < -1 || request->direction > 1) {
                response->message = "Rotation direction must be -1, 0, or +1";
                return;
            }
            if (request->limit_phi_travel &&
                (!std::isfinite(request->max_phi_travel) || request->max_phi_travel < 0.0)) {
                response->message = "Phi travel limit must be finite and nonnegative";
                return;
            }
            const double age = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - current_joints_time_).count();
            if (!current_joints_valid_ || age > rotation_joint_timeout_sec_ ||
                !wrist_feedback_in_range(current_joints_[3])) {
                response->message = "Fresh finite current_joints within the wrist limit are required";
                return;
            }
            // Validate raw feedback before using its bounded planning start.
            auto start_joints = current_joints_;
            const double start_wrist = std::clamp(
                static_cast<double>(start_joints[3]), -2.0 * M_PI, 0.0);
            start_joints[3] = static_cast<float>(start_wrist);
            if (!wrist_in_range(request->wrist_angle) ||
                std::abs(request->wrist_angle - start_wrist) > 0.05) {
                response->message = "Rotation start wrist angle does not match current_joints";
                return;
            }
            float requested_pose[6];
            float actual_pose[6];
            kin_.forward_kinematics(actual_pose, start_joints.data());
            if (!read_pose(request->target, requested_pose) ||
                !std::all_of(std::begin(actual_pose), std::end(actual_pose),
                    [](float value) { return std::isfinite(value); }) ||
                std::sqrt(std::pow(actual_pose[0] - requested_pose[0], 2) +
                    std::pow(actual_pose[1] - requested_pose[1], 2) +
                    std::pow(actual_pose[2] - requested_pose[2], 2)) > 20.0 ||
                std::abs(std::remainder(actual_pose[3] - requested_pose[3], 2.0 * M_PI)) > 0.05) {
                response->message = "Rotation start pose does not match current_joints";
                return;
            }
            rotation_active_ = true;
            rotation_group_id_ = request->group_id;
            highest_group_id_ = request->group_id;
            rotation_direction_ = request->direction;
            limit_phi_travel_ = request->limit_phi_travel;
            max_phi_travel_ = request->max_phi_travel;
            phi_travel_ = 0.0;
            phi_interval_active_ = false;
            interval_phi_travel_ = 0.0;
            last_commanded_phi_ = static_cast<double>(start_joints[0]) + start_wrist;
            rotation_fault_.clear();
            hold_joints_ = start_joints;
            hold_joints_active_ = true;
            std::copy(std::begin(actual_pose), std::end(actual_pose), current_pose_);
            clear_velocity();
            response->success = true;
            response->message = "Rotation group started; holding bounded start joints";
            return;
        }

        if (request->operation != WristControl::Request::TARGET &&
            request->operation != WristControl::Request::PHI_BEGIN &&
            request->operation != WristControl::Request::PHI_END) {
            response->message = "Unknown wrist operation";
            return;
        }
        if (!rotation_active_ || request->group_id != rotation_group_id_) {
            response->message = "Rotation group ID does not match an active group";
            return;
        }
        if (!rotation_fault_.empty()) {
            response->message = rotation_fault_;
            return;
        }
        const auto reject = [this, &response](const std::string &message) {
            rotation_fault_ = message;
            response->message = message;
            RCLCPP_ERROR(get_logger(), "Rotation group rejected target: %s", message.c_str());
        };
        if (request->operation == WristControl::Request::PHI_BEGIN) {
            if (phi_interval_active_ || !request->limit_phi_travel ||
                !std::isfinite(request->max_phi_travel) || request->max_phi_travel < 0.0) {
                reject("Phi interval is already active or its limit is invalid");
                return;
            }
            phi_interval_active_ = true;
            interval_max_phi_travel_ = request->max_phi_travel;
            interval_phi_travel_ = 0.0;
            response->success = true;
            response->message = "Phi interval started at the last accepted joint command";
            return;
        }
        if (request->operation == WristControl::Request::PHI_END) {
            if (!phi_interval_active_) {
                reject("Phi interval end needs an active interval");
                return;
            }
            phi_interval_active_ = false;
            response->success = true;
            response->message = "Phi interval ended; whole-group constraints remain active";
            return;
        }
        if (!wrist_in_range(request->wrist_angle)) {
            reject("Wrist target is nonfinite or outside [-2pi, 0]");
            return;
        }
        double wrist_angle = std::clamp(request->wrist_angle, -2.0 * M_PI, 0.0);
        const double increment = wrist_angle - hold_joints_[3];
        if (rotation_direction_ * increment < -1e-5) {
            reject("Wrist target reverses the group direction");
            return;
        }
        if (rotation_direction_ * increment < 0.0) {
            wrist_angle = hold_joints_[3];
        }
        float pose[6];
        if (!read_pose(request->target, pose)) {
            reject("Wrist target pose is invalid");
            return;
        }
        std::array<float, 4> joints;
        kin_.inverse_kinematics(pose, joints.data());
        if (!std::all_of(joints.begin(), joints.end(),
                [](float value) { return std::isfinite(value); })) {
            reject("Wrist target inverse kinematics is nonfinite");
            return;
        }
        if (std::abs(std::remainder(pose[3] - joints[0] - wrist_angle, 2.0 * M_PI)) > 1e-4) {
            reject("Wrist target angle does not match its pose yaw");
            return;
        }
        // Preserve turns and accumulate before Float32 output rounding.
        const double commanded_phi = static_cast<double>(joints[0]) + wrist_angle;
        const double next_phi_travel = phi_travel_ +
            std::abs(commanded_phi - last_commanded_phi_);
        const double next_interval_travel = interval_phi_travel_ +
            std::abs(commanded_phi - last_commanded_phi_);
        // Apply this tolerance once to the group total, never to each increment.
        if (limit_phi_travel_ && next_phi_travel > max_phi_travel_ + 1e-5) {
            reject("Phi total travel exceeds the sequence group limit");
            return;
        }
        if (phi_interval_active_ && next_interval_travel > interval_max_phi_travel_ + 1e-5) {
            reject("Phi total travel exceeds the active interval limit");
            return;
        }
        joints[3] = static_cast<float>(wrist_angle);
        hold_joints_ = joints;
        phi_travel_ = next_phi_travel;
        if (phi_interval_active_) interval_phi_travel_ = next_interval_travel;
        last_commanded_phi_ = commanded_phi;
        std::copy(std::begin(pose), std::end(pose), current_pose_);
        response->success = true;
        response->message = "Wrist target accepted";
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (rotation_active_) {
            return;
        }
        if (hold_joints_active_) {
            float pose[6];
            if (!read_pose(*msg, pose)) {
                return;
            }
            hold_joints_active_ = false;
        }
        // 単位を [m] から [mm] に変換
        current_pose_[0] = msg->pose.position.x * 1000.0;
        current_pose_[1] = msg->pose.position.y * 1000.0;
        current_pose_[2] = msg->pose.position.z * 1000.0;

        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.orientation, q);

        // クォータニオンを回転行列に変換
        tf2::Matrix3x3 mat(q);

        // 【修正】Pitch = -90度の特異点（ジンバルロック）を回避するため、
        // 常に水平面上に残る「ローカルY軸」のベクトルを抽出してYawを計算します。
        tf2::Vector3 y_axis = mat.getColumn(1); 
        
        // Y軸ベクトルは [-sin(yaw), cos(yaw), 0]^T の形になるため、atan2でYawを逆算
        double yaw = std::atan2(-y_axis.x(), y_axis.y());

        current_pose_[3] = static_cast<float>(yaw);   // PHI
        
        // 4DoF IKでは THE(Pitch) と PSI(Roll) は使用しないため更新不要です
    }

    void endeffector_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (!msg->data.empty()) {
            current_endeffector_val_ = msg->data[0];
        }
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (msg->axes.size() < 6 || msg->buttons.size() < 6) return;
        
        // --- 速度として入力値を保持 ---
        if (!rotation_active_) {
            // 左スティック・十字キーともに横を X、縦を Y に割り当てる。
            // 十字キーの軸がないコントローラーではスティックのみ使用する。
            const float dpad_x = msg->axes.size() > 6 ? msg->axes[6] : 0.0f;
            const float dpad_y = msg->axes.size() > 7 ? msg->axes[7] : 0.0f;
            vel_x_ = std::clamp(msg->axes[0] + dpad_x, -1.0f, 1.0f);
            vel_y_ = std::clamp(msg->axes[1] + dpad_y, -1.0f, 1.0f);
            vel_z_ = msg->axes[4];
            vel_phi_ = msg->axes[3];
            if (std::isfinite(vel_x_) && std::isfinite(vel_y_) &&
                std::isfinite(vel_z_) && std::isfinite(vel_phi_) &&
                (vel_x_ != 0.0f || vel_y_ != 0.0f || vel_z_ != 0.0f || vel_phi_ != 0.0f)) {
                hold_joints_active_ = false;
            } else if (hold_joints_active_) {
                clear_velocity();
            }
        }

        // Request a shared initialization toggle on each X button press.
        const bool current_x_button = msg->buttons[0] != 0;
        if (!rotation_active_ && current_x_button && !prev_x_button_ && !state_request_pending_) {
            if (!state_client_->service_is_ready()) {
                RCLCPP_WARN(this->get_logger(), "State service not ready.");
            } else {
                auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
                state_request_pending_ = true;
                state_request_time_ = std::chrono::steady_clock::now();
                state_request_id_ = state_client_->async_send_request(request,
                    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                        state_request_pending_ = false;
                        if (!future.get()->success) {
                            RCLCPP_WARN(this->get_logger(), "State request was rejected.");
                            return;
                        }
                        RCLCPP_INFO(this->get_logger(), "Initialization request accepted.");
                    }).request_id;
            }
        }
        prev_x_button_ = current_x_button;

        // --- 〇ボタン(buttons[1])によるPump状態の遷移 ---
        bool current_o_button = msg->buttons[1]; 

        if (current_o_button && !prev_o_button_ && !pump_request_pending_) {
            if (!pump_client_->service_is_ready()) {
                RCLCPP_WARN(this->get_logger(), "Pump service not ready.");
            } else {
                auto request = std::make_shared<PumpRequest>();
                request->left = next_pump_state_;
                request->center = next_pump_state_;
                request->right = next_pump_state_;
                pump_request_pending_ = true;
                pump_request_time_ = std::chrono::steady_clock::now();
                pump_request_id_ = pump_client_->async_send_request(request,
                    [this](rclcpp::Client<catchrobo2026_msgs::srv::PumpControl>::SharedFuture future) {
                        pump_request_pending_ = false;
                        if (!future.get()->success) {
                            RCLCPP_WARN(this->get_logger(), "Pump request was rejected.");
                            return;
                        }
                        if (next_pump_state_ == PumpRequest::OFF) {
                            next_pump_state_ = PumpRequest::SUCTION;
                        } else if (next_pump_state_ == PumpRequest::SUCTION) {
                            next_pump_state_ = PumpRequest::RELEASE;
                        } else {
                            next_pump_state_ = PumpRequest::OFF;
                        }
                    }).request_id;
            }
        }
        prev_o_button_ = current_o_button;

        // --- buttons[2]によるEndeffector状態の切り替え ---
        bool current_endeffector_button = msg->buttons[2];

        if (current_endeffector_button && !prev_endeffector_button_) {
            if (!endeffector_client_->service_is_ready()) {
                RCLCPP_WARN(this->get_logger(), "Endeffector service not ready.");
            } else {
                auto request = std::make_shared<catchrobo2026_msgs::srv::EndeffectorControl::Request>();

                if (current_endeffector_val_ == 0) {
                    request->command = 1;
                } else {
                    request->command = 0; 
                }

                endeffector_client_->async_send_request(request);
                RCLCPP_INFO(this->get_logger(), "Requested Endeffector change. Sent command: %d", request->command);
            }
        }
        prev_endeffector_button_ = current_endeffector_button;
    }

    void publish_timer_callback() {
        if (state_request_pending_ &&
            std::chrono::steady_clock::now() - state_request_time_ >= 1s) {
            state_client_->remove_pending_request(state_request_id_);
            state_request_pending_ = false;
            RCLCPP_WARN(this->get_logger(), "State request timed out. Press again to retry.");
        }

        // 応答が途絶えても、次のボタン操作で同じ状態を再要求できるようにする。
        if (pump_request_pending_ &&
            std::chrono::steady_clock::now() - pump_request_time_ >= 1s) {
            pump_client_->remove_pending_request(pump_request_id_);
            pump_request_pending_ = false;
            RCLCPP_WARN(this->get_logger(), "Pump request timed out. Press again to retry.");
        }

        if (require_current_joints_on_start_ && !startup_feedback_received_) {
            return;
        }

        if (hold_joints_active_) {
            std_msgs::msg::Float32MultiArray msg_out;
            msg_out.data.assign(hold_joints_.begin(), hold_joints_.end());
            joint_pub_->publish(msg_out);
            publish_arm_markers(hold_joints_.data());
            return;
        }

        // 1. Joy入力による手動介入 (位置の微調整)
        const float pos_gain = 1.0f;  
        const float rot_gain = 0.01f; 

        current_pose_[0] -= vel_x_ * pos_gain;
        current_pose_[1] += vel_y_ * pos_gain;
        current_pose_[2] += vel_z_ * pos_gain;
        current_pose_[3] += vel_phi_ * rot_gain;

        // 2. ローカルで逆運動学(IK)を計算
        float target_joints[4] = {0.0f};
        kin_.inverse_kinematics(current_pose_, target_joints);

        // 同じ向きを保ったまま第4関節を [-2π, 0] [rad] に収める。
        // 例: +π は 0 に切り詰めず -π とする。範囲内の値は端点も保持する。
        const float full_turn = 2.0f * static_cast<float>(M_PI);
        if (target_joints[3] > 0.0f || target_joints[3] < -full_turn) {
            target_joints[3] = std::fmod(target_joints[3], full_turn);
            if (target_joints[3] > 0.0f) {
                target_joints[3] -= full_turn;
            }
        }

        // 3. 計算結果をパブリッシュ
        std_msgs::msg::Float32MultiArray msg_out;
        msg_out.data.resize(4);
        for(int i = 0; i < 4; ++i) {
            msg_out.data[i] = target_joints[i];
        }
        joint_pub_->publish(msg_out);

        // 4. アームの姿勢マーカー (矢印) をパブリッシュ
        publish_arm_markers(target_joints);
    }

    void publish_arm_markers(float* target_joints) {
        float positions[6][3];
        // 運動学モデルから6つのジョイント位置(0~5)の座標配列を取得
        kin_.get_joint_positions(target_joints, positions);

        visualization_msgs::msg::MarkerArray marker_array;
        
        // 6つの点から5本の矢印(リンク)を生成する
        for (int i = 0; i < 5; ++i) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map"; // 適切な固定フレーム名に変更してください (例: "base_link" や "map")
            marker.header.stamp = this->now();
            marker.ns = "arm_links";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;

            geometry_msgs::msg::Point p_start, p_end;
            // robot_kinematicsは[mm]単位なので、RViz用に[m]に変換する[cite: 1]
            p_start.x = positions[i][0] / 1000.0;
            p_start.y = positions[i][1] / 1000.0;
            p_start.z = positions[i][2] / 1000.0;

            p_end.x = positions[i+1][0] / 1000.0;
            p_end.y = positions[i+1][1] / 1000.0;
            p_end.z = positions[i+1][2] / 1000.0;

            marker.points.push_back(p_start);
            marker.points.push_back(p_end);

            // 矢印の太さ設定
            marker.scale.x = 0.015; // シャフトの直径 (1.5cm)
            marker.scale.y = 0.03;  // ヘッドの直径 (3.0cm)
            marker.scale.z = 0.03;  // ヘッドの長さ (3.0cm)

            // 色の設定 (例として緑色)
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.8f; // 若干透過

            marker_array.markers.push_back(marker);
        }

        marker_pub_->publish(marker_array);
    }

    // --- 変数定義 ---
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr current_joints_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr endeffector_sub_;
    
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joint_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_; // 追加
    
    rclcpp::Client<catchrobo2026_msgs::srv::PumpControl>::SharedPtr pump_client_;
    rclcpp::Client<catchrobo2026_msgs::srv::EndeffectorControl>::SharedPtr endeffector_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr state_client_;
    rclcpp::Service<WristControl>::SharedPtr wrist_service_;
    
    rclcpp::TimerBase::SharedPtr publish_timer_; 
    
    robot_kinematics kin_; // 運動学クラスのインスタンス

    float current_pose_[6];
    std::array<float, 4> current_joints_{};
    std::chrono::steady_clock::time_point current_joints_time_;
    bool current_joints_valid_ = false;
    bool require_current_joints_on_start_ = false;
    bool startup_feedback_received_ = false;
    double rotation_joint_timeout_sec_ = 1.0;
    double wrist_feedback_tolerance_rad_ = 6.0 * M_PI / 180.0;
    bool rotation_active_ = false;
    uint64_t rotation_group_id_ = 0;
    uint64_t highest_group_id_ = 0;
    uint64_t last_ended_group_id_ = 0;
    int rotation_direction_ = 0;
    bool limit_phi_travel_ = false;
    double max_phi_travel_ = 0.0;
    double phi_travel_ = 0.0;
    bool phi_interval_active_ = false;
    double interval_max_phi_travel_ = 0.0;
    double interval_phi_travel_ = 0.0;
    double last_commanded_phi_ = 0.0;
    std::string rotation_fault_;
    std::array<float, 4> hold_joints_{};
    bool hold_joints_active_ = false;
    bool prev_x_button_ = false;
    bool state_request_pending_ = false;
    int64_t state_request_id_ = 0;
    std::chrono::steady_clock::time_point state_request_time_;
    int8_t next_pump_state_ = PumpRequest::RELEASE;
    bool pump_request_pending_ = false;
    int64_t pump_request_id_ = 0;
    std::chrono::steady_clock::time_point pump_request_time_;
    int current_endeffector_val_ = 56; 
    bool prev_o_button_ = false; 
    bool prev_endeffector_button_ = false;
    
    float vel_x_ = 0.0f;
    float vel_y_ = 0.0f;
    float vel_z_ = 0.0f;
    float vel_phi_ = 0.0f;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyControllerNode>());
    rclcpp::shutdown();
    return 0;
}
