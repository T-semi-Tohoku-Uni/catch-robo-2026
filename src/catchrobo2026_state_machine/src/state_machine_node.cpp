#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <sstream>
#include <iomanip>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "canrpc_bridge/srv/call.hpp"
#include "catchrobo2026_msgs/srv/arm_target_gate.hpp"

using namespace std::chrono_literals;
using Call = canrpc_bridge::srv::Call;
using Trigger = std_srvs::srv::Trigger;
using Gate = catchrobo2026_msgs::srv::ArmTargetGate;

class ReadinessNode : public rclcpp::Node {
public:
  ReadinessNode() : Node("repeater_node") {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    const auto service = declare_parameter<std::string>("canrpc_service", "canrpc/call", descriptor);
    endpoint_ = declare_parameter<std::string>("arm_ready_endpoint", "arm_ready", descriptor);
    timeout_ = declare_parameter<double>("rpc_timeout_sec", 1.5, descriptor);
    if (!std::isfinite(timeout_) || timeout_ <= 0.0) {
      throw std::invalid_argument("rpc_timeout_sec must be finite and positive");
    }
    rpc_ = create_client<Call>(service);
    gate_ = create_client<Gate>(declare_parameter<std::string>("target_gate_service", "arm_target/gate", descriptor));
    service_ = create_service<Trigger>("request_initialization",
      [this](const std::shared_ptr<rmw_request_id_t> header,
             const std::shared_ptr<Trigger::Request>) {
        if (pending_ || !rpc_->service_is_ready()) {
          Trigger::Response response;
          response.message = pending_ ? "arm readiness query busy" : "CAN-RPC unavailable";
          service_->send_response(*header, response);
          return;
        }
        pending_ = true;
        header_ = header;
        const auto token = ++token_;
        deadline_ = Clock::now() + std::chrono::duration_cast<Clock::duration>(
          std::chrono::duration<double>(timeout_));
        if(!gate_->service_is_ready()) {complete(token,false,"target pause unavailable");return;}
        phase_=Phase::Pause;
        try {request_id_=gate_->async_send_request(gate_request(true),
          [this,token](rclcpp::Client<Gate>::SharedFuture future) {
            if(!pending_ || token!=token_) return;
            try {const auto result=future.get();
              if(!result->success) {complete(token,false,"target pause failed: "+result->message);return;}
              if(Clock::now()>=deadline_) {complete(token,false,"arm readiness query timed out; outcome unknown");return;}
              gate_token_=result->token;
              query(token);
            } catch(const std::exception &error) {complete(token,false,error.what());}
          }).request_id;
        } catch(const std::exception &error) {complete(token,false,error.what());}
      });
    timer_ = create_wall_timer(10ms, [this]() {
      if (pending_ && Clock::now() >= deadline_) {
        if(phase_==Phase::Readiness) rpc_->remove_pending_request(request_id_);
        else gate_->remove_pending_request(request_id_);
        complete(token_, false, "arm readiness query timed out; outcome unknown");
      }
    });
    RCLCPP_INFO(get_logger(), "Arm readiness query service started; no rehoming is requested");
  }
private:
  using Clock = std::chrono::steady_clock;
  enum class Phase {Pause,Readiness,Resume};
  Phase phase_=Phase::Pause;
  std::shared_ptr<Gate::Request> gate_request(bool pause) const {
    auto request=std::make_shared<Gate::Request>();
    request->pause=pause;request->token=pause?0:gate_token_;
    request->deadline_steady_ns=static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      deadline_.time_since_epoch()).count());
    return request;
  }
  void query(uint64_t token) {
        phase_=Phase::Readiness;
        auto request = std::make_shared<Call::Request>();
        request->endpoint = endpoint_;
        try {
          request_id_ = rpc_->async_send_request(request,
            [this, token](rclcpp::Client<Call>::SharedFuture future) {
              if (!pending_ || token != token_) return;
              try {
                const auto reply = future.get();
                const bool valid = reply->outcome == 0 && reply->data.size() == 48 && reply->data[11] == 1;
                const bool ready = valid && read_u32(reply->data, 0) == 0 &&
                  (read_u32(reply->data, 4) & 0x1f) == 0x1f;
                if (valid) RCLCPP_INFO(get_logger(), "%s raw=%s", describe(reply->data).c_str(), hex(reply->data).c_str());
                const auto detail=valid ? describe(reply->data) :
                  "arm readiness not confirmed: " + reply->message + " raw=" + hex(reply->data);
                if (!ready) {complete(token, false, detail);return;}
                if(Clock::now()>=deadline_) {complete(token,false,"arm readiness query timed out; outcome unknown");return;}
                if (!gate_->service_is_ready()) {complete(token, false, detail+"; target resume unavailable");return;}
                phase_=Phase::Resume;
                request_id_=gate_->async_send_request(gate_request(false),
                  [this,token,detail](rclcpp::Client<Gate>::SharedFuture answer) {
                    if(!pending_ || token!=token_) return;
                    try {const auto result=answer.get();complete(token,result->success && result->token==gate_token_,
                      detail+"; target resume: "+result->message);}
                    catch(const std::exception &error) {complete(token,false,error.what());}
                  }).request_id;
              } catch (const std::exception &error) { complete(token, false, error.what()); }
            }).request_id;
        } catch (const std::exception &error) { complete(token, false, error.what()); }
  }
  static uint32_t read_u32(const std::vector<uint8_t> &data, size_t at) {
    return uint32_t(data[at]) | uint32_t(data[at + 1]) << 8 |
      uint32_t(data[at + 2]) << 16 | uint32_t(data[at + 3]) << 24;
  }
  static std::string hex(const std::vector<uint8_t> &data) {
    std::ostringstream out; out << std::hex << std::setfill('0');
    for (auto byte : data) out << std::setw(2) << unsigned(byte);
    return out.str();
  }
  static const char *name(unsigned value, const std::vector<const char *> &names) {
    return value < names.size() ? names[value] : "UNKNOWN";
  }
  static std::string describe(const std::vector<uint8_t> &data) {
    std::ostringstream out;
    out << "arm result=" << read_u32(data, 0) << " flags=" << read_u32(data, 4)
        << " stage=" << unsigned(data[8]) << " reason=" << name(data[9],
          {"NONE","TX_ENQUEUE_TIMEOUT","TX_COMPLETE_TIMEOUT","RESPONSE_TIMEOUT","MOTOR_FAULT",
           "MODE_MISMATCH","HOME_TIMEOUT","FINAL_FEEDBACK_TIMEOUT","CONTROL_FAILURE"})
        << '(' << unsigned(data[9]) << ") failed_axis=" << unsigned(data[10]);
    out << " snapshot(age@failure)=" << ((data[14]|data[26]|data[38])&128 ? 1 : 0);
    for (size_t axis = 0; axis < 3; ++axis) {
      const size_t at = 12 + axis * 12;
      const bool seen=(data[at+2]&1)!=0;
      out << "; q" << axis << " id=" << unsigned(data[at+3])
          << " m=" << (seen?std::to_string(data[at]):"unknown")
          << " f=" << (seen?std::to_string(data[at+1]):"unknown") << " v=" << unsigned(data[at+2])
          << " age=" << (seen?std::to_string(read_u32(data,at+4)):"never") << " cmd=" << name(data[at+8],
            {"NONE","STOP","MODE_OPERATION","ENABLE","MODE_SPEED","VELOCITY","ZERO","MODE_PP",
             "PP_VMAX","PP_ACCEL","CURRENT_LIMIT","MODE_CURRENT","CURRENT_ZERO","REFRESH","POSITION_TARGET","CURRENT_REFERENCE"})
          << " rsp=" << name(data[at+9],
            {"NONE","QUEUE_WAIT","TX_WAIT","RESPONSE_WAIT","CONFIRMED","FAILED"});
    }
    return out.str();
  }
  void pause_after_unknown() {
    auto request=gate_request(true);
    request->deadline_steady_ns=static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      (Clock::now()+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(timeout_))).time_since_epoch()).count());
    try {const auto future=gate_->async_send_request(request);gate_->remove_pending_request(future.request_id);
      RCLCPP_WARN(get_logger(),"Resume outcome unknown; fresh-deadline PAUSE sent without confirmation (physical stop not confirmed)");}
    catch(const std::exception &error) {
      RCLCPP_ERROR(get_logger(),"Resume outcome unknown; PAUSE could not be confirmed: %s",error.what());
    }
  }
  void complete(uint64_t token, bool ready, const std::string &message) {
    if (!pending_ || token != token_) return;
    const bool expired = Clock::now() >= deadline_;
    ready = ready && !expired;
    if(!ready && phase_==Phase::Resume) pause_after_unknown();
    auto header = header_;
    pending_ = false;
    Trigger::Response response;
    response.success = ready;
    response.message = expired ? "arm readiness query timed out; outcome unknown" : message;
    if (!ready) RCLCPP_ERROR(get_logger(), "%s", response.message.c_str());
    try { service_->send_response(*header, response); }
    catch (const std::exception &error) {
      RCLCPP_WARN(get_logger(), "Readiness response failed: %s", error.what());
    }
  }
  rclcpp::Client<Call>::SharedPtr rpc_;
  rclcpp::Client<Gate>::SharedPtr gate_;
  uint64_t gate_token_=0;
  rclcpp::Service<Trigger>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string endpoint_;
  double timeout_ = 1.5;
  bool pending_ = false;
  uint64_t token_ = 0;
  int64_t request_id_ = 0;
  Clock::time_point deadline_;
  std::shared_ptr<rmw_request_id_t> header_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReadinessNode>());
  rclcpp::shutdown();
  return 0;
}
