#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "catchrobo2026_msgs/srv/pump_control.hpp"
#include "catchrobo2026_msgs/srv/check_suction.hpp"
#include "canrpc_bridge/srv/call.hpp"

using namespace std::chrono_literals;
using PumpControl = catchrobo2026_msgs::srv::PumpControl;
using RpcCall = canrpc_bridge::srv::Call;
using CheckSuction = catchrobo2026_msgs::srv::CheckSuction;

class PumpControllerNode : public rclcpp::Node {
public:
    PumpControllerNode() : Node("pump_controller_node") {
        // Configure wiring and polarity at startup, in L/C/R order.
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = true;
        pump_bits_ = this->declare_parameter<std::vector<int64_t>>(
            "pump_bits", {0, 1, 2}, descriptor);
        valve_bits_ = this->declare_parameter<std::vector<int64_t>>(
            "valve_bits", {3, 4, 5}, descriptor);
        pump_on_level_ = this->declare_parameter<bool>("pump_on_level", false, descriptor);
        valve_on_level_ = this->declare_parameter<bool>("valve_on_level", false, descriptor);
        selected_pump_ = declare_parameter<int>("selected_pump", 1, descriptor);
        if (selected_pump_ < 1 || selected_pump_ > 3) {
            throw std::invalid_argument("selected_pump must be 1, 2 or 3");
        }

        if (pump_bits_.size() != 3 || valve_bits_.size() != 3) {
            throw std::invalid_argument("pump_bits and valve_bits must each contain 3 entries (L/C/R)");
        }
        auto bits = pump_bits_;
        bits.insert(bits.end(), valve_bits_.begin(), valve_bits_.end());
        std::array<bool, 6> used_bits{};
        for (const auto bit : bits) {
            if (bit < 0 || bit > 5 || used_bits[bit]) {
                throw std::invalid_argument("pump_bits and valve_bits must use bits 0..5 without duplicates");
            }
            used_bits[bit] = true;
        }

        pressure_enabled_ = declare_parameter<bool>("pressure_enabled", true, descriptor);
        pressure_indices_ = this->declare_parameter<std::vector<int64_t>>(
            "pressure_indices", {0, 1, 2}, descriptor);
        if (pressure_indices_.size() != 3) {
            throw std::invalid_argument("pressure_indices must contain 3 entries (L/C/R)");
        }
        std::array<bool, 3> used_indices{};
        for (const auto index : pressure_indices_) {
            if (index < 0 || index > 2 || used_indices[index]) {
                throw std::invalid_argument("pressure_indices must use indices 0..2 without duplicates");
            }
            used_indices[index] = true;
        }
        const auto threshold = this->declare_parameter<int64_t>(
            "pressure_threshold", 0x60, descriptor);
        if (threshold < std::numeric_limits<int32_t>::min() ||
            threshold > std::numeric_limits<int32_t>::max()) {
            throw std::invalid_argument("pressure_threshold must fit int32");
        }
        pressure_threshold_ = static_cast<int32_t>(threshold);
        pressure_comparison_ = this->declare_parameter<std::string>(
            "pressure_comparison", "unconfigured", descriptor);
        if (pressure_comparison_ != "ge" && pressure_comparison_ != "le" &&
            pressure_comparison_ != "unconfigured") {
            throw std::invalid_argument("pressure_comparison must be ge, le or unconfigured");
        }
        const auto max_pending = this->declare_parameter<int64_t>(
            "max_pending_suction_checks", 8, descriptor);
        if (max_pending < 1 || max_pending > 128) {
            throw std::invalid_argument("max_pending_suction_checks must be in 1..128");
        }
        max_pending_suction_checks_ = static_cast<size_t>(max_pending);
        pending_suction_checks_.reserve(max_pending_suction_checks_);

        pump_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("pump_state", 10);
        const auto rpc_service = declare_parameter<std::string>("canrpc_service", "canrpc/call", descriptor);
        rpc_endpoint_ = declare_parameter<std::string>("pump_rpc_endpoint", "pump_set", descriptor);
        rpc_timeout_sec_ = declare_parameter<double>("rpc_timeout_sec", 1.5, descriptor);
        if (!std::isfinite(rpc_timeout_sec_) || rpc_timeout_sec_ <= 0.0) {
            throw std::invalid_argument("rpc_timeout_sec must be finite and positive");
        }
        rpc_ = create_client<RpcCall>(rpc_service);
        service_server_ = this->create_service<PumpControl>("set_pump_state",
            [this](const std::shared_ptr<rmw_request_id_t> header,
                   const std::shared_ptr<PumpControl::Request> request) {
                handle_service(header, request);
            });
        rpc_timer_ = create_wall_timer(10ms, [this]() {
            if (pending_ && SteadyClock::now() >= pending_deadline_) {
                rpc_->remove_pending_request(pending_rpc_id_);
                complete_pump(pending_token_, false, "pump RPC timed out; applied state unknown");
            }
        });

        // Defer responses without blocking the executor or creating worker threads.
        suction_service_ = this->create_service<CheckSuction>(
            "check_suction",
            [this](const std::shared_ptr<rmw_request_id_t> header,
                   const std::shared_ptr<CheckSuction::Request> request) {
                handle_suction_request(header, request);
            });
        if (pressure_enabled_) {
            pressure_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
                "pressure_sensor", rclcpp::SensorDataQoS().keep_last(1),
                std::bind(&PumpControllerNode::pressure_callback, this, std::placeholders::_1));
            suction_timer_ = this->create_wall_timer(
                10ms, [this]() { expire_suction_checks(SteadyClock::now()); });
        }

        RCLCPP_INFO(this->get_logger(), "Pump Controller Node started.");
        if (!pressure_enabled_) {
            RCLCPP_INFO(get_logger(), "Pressure input disabled; check_suction is unavailable.");
        } else if (pressure_comparison_ == "unconfigured") {
            RCLCPP_WARN(this->get_logger(),
                        "check_suction is unavailable until pressure_comparison is set to ge or le.");
        }
    }

private:
    using SteadyClock = std::chrono::steady_clock;

    struct PendingSuctionCheck {
        std::shared_ptr<rmw_request_id_t> header;
        uint8_t collector_mask;
        SteadyClock::time_point deadline;
        CheckSuction::Response response;
        bool received_sample = false;
    };

    void send_suction_response(
        const std::shared_ptr<rmw_request_id_t> &header, CheckSuction::Response &response)
    {
        try {
            suction_service_->send_response(*header, response);
        } catch (const std::exception &error) {
            RCLCPP_WARN(this->get_logger(), "Failed to send check_suction response: %s", error.what());
        }
    }

    void handle_suction_request(
        const std::shared_ptr<rmw_request_id_t> &header,
        const std::shared_ptr<CheckSuction::Request> &request)
    {
        const auto now = SteadyClock::now();
        expire_suction_checks(now);
        CheckSuction::Response response;
        response.pressure.fill(std::numeric_limits<int32_t>::min());
        if (!pressure_enabled_) {
            response.message = "pressure input disabled; suction cannot be confirmed";
        } else if (request->collector_mask == 0 || (request->collector_mask & 0xf8) != 0) {
            response.message = "collector_mask must be in 1..7 (L/C/R bits 0/1/2)";
        } else if (!std::isfinite(request->timeout_sec) || request->timeout_sec <= 0.0 ||
                   request->timeout_sec > 86400.0) {
            response.message = "timeout_sec must be finite and in (0, 86400]";
        } else if (pressure_comparison_ == "unconfigured") {
            response.message = "pressure_comparison must be configured as ge or le";
        } else if (pending_suction_checks_.size() >= max_pending_suction_checks_) {
            response.message = "too many pending check_suction requests";
        } else {
            const auto duration = std::chrono::duration_cast<SteadyClock::duration>(
                std::chrono::duration<double>(request->timeout_sec));
            pending_suction_checks_.push_back(
                {header, request->collector_mask, now + duration, response, false});
            return;
        }
        send_suction_response(header, response);
    }

    void expire_suction_checks(SteadyClock::time_point now) {
        for (auto it = pending_suction_checks_.begin(); it != pending_suction_checks_.end();) {
            if (now < it->deadline) {
                ++it;
                continue;
            }
            it->response.timed_out = true;
            it->response.message = it->received_sample ?
                "suction threshold was not reached before timeout" :
                "no valid pressure sample received before timeout";
            send_suction_response(it->header, it->response);
            it = pending_suction_checks_.erase(it);
        }
    }

    void pressure_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (!pressure_enabled_) return;
        const auto now = SteadyClock::now();
        // Expire first so a late sample cannot turn a timeout into success.
        expire_suction_checks(now);
        const auto &dimensions = msg->layout.dim;
        if (msg->data.size() != 3 || msg->layout.data_offset != 0 ||
            (!dimensions.empty() && (dimensions.size() != 1 ||
             dimensions[0].size != 3 || dimensions[0].stride != 3))) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                "Ignoring malformed pressure_sensor array (expected 3 values, offset 0).");
            return;
        }
        std::array<int32_t, 3> pressure;
        uint8_t suction_mask = 0;
        for (size_t i = 0; i < pressure.size(); ++i) {
            pressure[i] = msg->data[pressure_indices_[i]];
            if ((pressure_comparison_ == "ge" && pressure[i] >= pressure_threshold_) ||
                (pressure_comparison_ == "le" && pressure[i] <= pressure_threshold_)) {
                suction_mask |= static_cast<uint8_t>(1u << i);
            }
        }
        for (auto it = pending_suction_checks_.begin(); it != pending_suction_checks_.end();) {
            // Each request only observes callbacks after its acceptance.
            it->received_sample = true;
            it->response.pressure = pressure;
            it->response.suction_mask = suction_mask;
            if ((suction_mask & it->collector_mask) != it->collector_mask) {
                ++it;
                continue;
            }
            it->response.success = true;
            it->response.message = "all selected collectors reached the suction threshold";
            send_suction_response(it->header, it->response);
            it = pending_suction_checks_.erase(it);
        }
    }

    static uint32_t read_u32(const std::vector<uint8_t> &data, size_t offset) {
        return uint32_t(data[offset]) | uint32_t(data[offset + 1]) << 8 |
            uint32_t(data[offset + 2]) << 16 | uint32_t(data[offset + 3]) << 24;
    }

    uint32_t command_mask(const std::array<int8_t, 3> &states) const {
        uint32_t mask = 0;
        for (size_t i = 0; i < states.size(); ++i) {
            const bool pump_on = states[i] == PumpControl::Request::SUCTION;
            const bool valve_on = states[i] == PumpControl::Request::RELEASE;
            if (pump_on == pump_on_level_) mask |= 1u << pump_bits_[i];
            if (valve_on == valve_on_level_) mask |= 1u << valve_bits_[i];
        }
        return mask;
    }

    void publish_pump_status(bool confirmed, uint32_t mask = 0) {
        std_msgs::msg::Int32MultiArray message;
        if (confirmed) message.data = {static_cast<int32_t>(mask)};
        // Empty means pending/unknown, never a guessed physical output.
        pump_pub_->publish(message);
    }

    void complete_pump(uint64_t token, bool success, const std::string &reason) {
        if (!pending_ || token != pending_token_) return;
        success = success && SteadyClock::now() < pending_deadline_;
        auto header = pending_header_;
        pending_ = false;
        publish_pump_status(success, pending_mask_);
        PumpControl::Response response;
        response.success = success;
        if (!success) RCLCPP_WARN(get_logger(), "%s", reason.c_str());
        try { service_server_->send_response(*header, response); }
        catch (const std::exception &error) {
            RCLCPP_WARN(get_logger(), "PumpControl response failed: %s", error.what());
        }
    }

    void handle_service(const std::shared_ptr<rmw_request_id_t> &header,
                        const std::shared_ptr<PumpControl::Request> &request) {
        const std::array<int8_t, 3> requested = {request->left, request->center, request->right};
        bool valid = true;
        for (auto state : requested) valid = valid && state >= PumpControl::Request::RELEASE &&
            state <= PumpControl::Request::SUCTION;
        if (!valid || pending_ || !rpc_->service_is_ready()) {
            PumpControl::Response response;
            response.success = false;
            service_server_->send_response(*header, response);
            return;
        }
        std::array<int8_t, 3> selected{};
        selected[selected_pump_ - 1] = requested[selected_pump_ - 1];
        const uint32_t mask = command_mask(selected);
        auto call = std::make_shared<RpcCall::Request>();
        call->endpoint = rpc_endpoint_;
        call->data = {uint8_t(mask), uint8_t(mask >> 8), uint8_t(mask >> 16), uint8_t(mask >> 24)};
        pending_ = true;
        const uint64_t token = ++pending_token_;
        pending_header_ = header;
        pending_mask_ = mask;
        pending_deadline_ = SteadyClock::now() + std::chrono::duration_cast<SteadyClock::duration>(
            std::chrono::duration<double>(rpc_timeout_sec_));
        publish_pump_status(false);
        try {
            pending_rpc_id_ = rpc_->async_send_request(call,
                [this, token, mask](rclcpp::Client<RpcCall>::SharedFuture future) {
                    if (!pending_ || pending_token_ != token) return;
                    try {
                        const auto reply = future.get();
                        const bool applied = reply->outcome == 0 && reply->data.size() == 8 &&
                            read_u32(reply->data, 0) == 0 && read_u32(reply->data, 4) == mask;
                        complete_pump(token, applied, "pump not confirmed: " + reply->message);
                    } catch (const std::exception &error) { complete_pump(token, false, error.what()); }
                }).request_id;
        } catch (const std::exception &error) { complete_pump(token, false, error.what()); }
    }

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pump_pub_;
    rclcpp::Service<PumpControl>::SharedPtr service_server_;
    rclcpp::Client<RpcCall>::SharedPtr rpc_;
    rclcpp::TimerBase::SharedPtr rpc_timer_;
    std::string rpc_endpoint_;
    double rpc_timeout_sec_ = 1.5;
    bool pending_ = false;
    uint64_t pending_token_ = 0;
    int64_t pending_rpc_id_ = 0;
    uint32_t pending_mask_ = 0;
    SteadyClock::time_point pending_deadline_;
    std::shared_ptr<rmw_request_id_t> pending_header_;
    rclcpp::Service<CheckSuction>::SharedPtr suction_service_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr pressure_sub_;
    rclcpp::TimerBase::SharedPtr suction_timer_;

    bool pressure_enabled_ = true;
    std::vector<int64_t> pressure_indices_;
    int32_t pressure_threshold_;
    std::string pressure_comparison_;
    size_t max_pending_suction_checks_;
    std::vector<PendingSuctionCheck> pending_suction_checks_;

    int selected_pump_{1};
    std::vector<int64_t> pump_bits_;
    std::vector<int64_t> valve_bits_;
    bool pump_on_level_;
    bool valve_on_level_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<PumpControllerNode>());
    } catch (const std::exception &error) {
        RCLCPP_FATAL(rclcpp::get_logger("pump_controller_node"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
