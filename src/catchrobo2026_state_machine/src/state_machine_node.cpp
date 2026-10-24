#include <algorithm>
#include <array>
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
    initialize_endpoint_ = declare_parameter<std::string>("arm_initialize_endpoint", "arm_initialize", descriptor);
    poll_period_ = declare_parameter<double>("readiness_poll_period_sec", 0.1, descriptor);
    if (!std::isfinite(poll_period_) || poll_period_ <= 0.0) {
      throw std::invalid_argument("readiness_poll_period_sec must be finite and positive");
    }
    timeout_ = declare_parameter<double>("rpc_timeout_sec", 21.3, descriptor);
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
          response.message = pending_ ? "arm initialization busy" : "CAN-RPC unavailable";
          service_->send_response(*header, response);
          return;
        }
        pending_ = true;
        last_detail_.clear();
        boot_.fill(0);
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
              query(token, true);
            } catch(const std::exception &error) {complete(token,false,error.what());}
          }).request_id;
        } catch(const std::exception &error) {complete(token,false,error.what());}
      });
    timer_ = create_wall_timer(10ms, [this]() {
      if (!pending_) return;
      if (Clock::now() >= deadline_) {
        if (phase_ == Phase::Initialize || phase_ == Phase::Readiness || phase_ == Phase::FailureDiagnostic) rpc_->remove_pending_request(request_id_);
        else if (phase_ != Phase::PollWait) gate_->remove_pending_request(request_id_);
        complete(token_, false, "arm initialization timed out; outcome unknown");
      } else if (phase_ == Phase::PollWait && Clock::now() >= poll_due_) {
        query(token_, false);
      }
    });
    RCLCPP_INFO(get_logger(), "Arm initialization service started; explicit start then same-boot readiness polling");
  }
private:
  using Clock = std::chrono::steady_clock;
  enum class Phase {Pause,Initialize,Readiness,FailureDiagnostic,PollWait,Resume};
  Phase phase_=Phase::Pause;
  std::shared_ptr<Gate::Request> gate_request(bool pause) const {
    auto request=std::make_shared<Gate::Request>();
    request->pause=pause;request->token=pause?0:gate_token_;
    request->deadline_steady_ns=static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      deadline_.time_since_epoch()).count());
    return request;
  }
  static bool valid_diagnostic(const std::vector<uint8_t> &data) {
    return data.size() == 64 && data[11] == 2 &&
      std::any_of(data.begin() + 48, data.end(), [](uint8_t value) { return value != 0; });
  }
  std::string diagnostic_detail(const Call::Response &reply) {
    const bool valid = valid_diagnostic(reply.data);
    if (valid) RCLCPP_INFO(get_logger(), "%s outcome=%u reason=%s raw=%s",
      describe(reply.data).c_str(), unsigned(reply.outcome), reply.message.c_str(), hex(reply.data).c_str());
    auto detail = valid ? describe(reply.data) :
      "arm diagnostic invalid: " + reply.message + " raw=" + hex(reply.data);
    if (valid && reply.outcome != Call::Response::OK) {
      const auto reason = reply.message.size() > 64 ? reply.message.substr(0, 61) + "..." : reply.message;
      detail = "CANRPC[" + std::to_string(reply.outcome) + "] " + reason + "; " + detail;
    }
    if (valid) last_detail_ = detail;
    return detail;
  }
  void query(uint64_t token, bool initialize) {
    if (!pending_ || token != token_ || Clock::now() >= deadline_) {
      complete(token, false, "arm initialization timed out; outcome unknown");
      return;
    }
    phase_ = initialize ? Phase::Initialize : Phase::Readiness;
    const auto expected_phase = phase_;
    auto request = std::make_shared<Call::Request>();
    request->endpoint = initialize ? initialize_endpoint_ : endpoint_;
    try {
      request_id_ = rpc_->async_send_request(request,
        [this, token, initialize, expected_phase](rclcpp::Client<Call>::SharedFuture future) {
          if (!pending_ || token != token_ || phase_ != expected_phase) return;
          try {
            const auto reply = future.get();
            const auto detail = diagnostic_detail(*reply);
            if (Clock::now() >= deadline_) {complete(token, false, detail); return;}
            if (reply->outcome != Call::Response::OK || !valid_diagnostic(reply->data)) {
              if (initialize) failure_diagnostic(token, detail);
              else complete(token, false, detail);
              return;
            }
            if (initialize) std::copy_n(reply->data.begin() + 48, boot_.size(), boot_.begin());
            else if (!std::equal(boot_.begin(), boot_.end(), reply->data.begin() + 48)) {
              complete(token, false, "arm boot changed; " + detail); return;
            }
            const auto result = read_u32(reply->data, 0);
            if (result > 1 || reply->data[8] == 31) {
              complete(token, false, detail); return;
            }
            if (initialize) {
              // A lost start response is terminal; never issue another start automatically.
              query(token, false);
              return;
            }
            const bool ready = result == 0 && (read_u32(reply->data, 4) & 0x1f) == 0x1f;
            if (!ready) {
              phase_ = Phase::PollWait;
              poll_due_ = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(poll_period_));
              return;
            }
            resume(token, detail);
          } catch (const std::exception &error) {
            if (initialize) failure_diagnostic(token, error.what());
            else complete(token, false, error.what());
          }
        }).request_id;
    } catch (const std::exception &error) {
      if (initialize) failure_diagnostic(token, error.what());
      else complete(token, false, error.what());
    }
  }
  void failure_diagnostic(uint64_t token, const std::string &original) {
    last_detail_ = original;
    RCLCPP_ERROR(get_logger(), "Initialization start failed; no automatic restart: %s", original.c_str());
    if (Clock::now() >= deadline_ || !rpc_->service_is_ready()) {complete(token, false, original); return;}
    phase_ = Phase::FailureDiagnostic;
    auto request = std::make_shared<Call::Request>();
    request->endpoint = endpoint_;
    try {
      request_id_ = rpc_->async_send_request(request,
        [this, token, original](rclcpp::Client<Call>::SharedFuture future) {
          if (!pending_ || token != token_ || phase_ != Phase::FailureDiagnostic) return;
          try {
            const auto reply = future.get();
            (void)diagnostic_detail(*reply);
            if (valid_diagnostic(reply->data)) {
              const auto reason = original.size() > 64 ? original.substr(0, 61) + "..." : original;
              last_detail_ = "init: " + reason + "; " + describe(reply->data);
              complete(token, false, last_detail_);
            } else {last_detail_ = original; complete(token, false, original);}
          } catch (...) {last_detail_ = original; complete(token, false, original);}
        }).request_id;
    } catch (...) {complete(token, false, original);}
  }
  void resume(uint64_t token, const std::string &detail) {
    if (!gate_->service_is_ready()) {complete(token, false, detail + "; target resume unavailable"); return;}
    if (Clock::now() >= deadline_) {complete(token, false, detail); return;}
    phase_ = Phase::Resume;
    request_id_ = gate_->async_send_request(gate_request(false),
      [this, token, detail](rclcpp::Client<Gate>::SharedFuture answer) {
        if (!pending_ || token != token_ || phase_ != Phase::Resume) return;
        try {
          const auto result = answer.get();
          complete(token, result->success && result->token == gate_token_,
            detail + "; target resume: " + result->message);
        } catch (const std::exception &error) {complete(token, false, error.what());}
      }).request_id;
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
           "MODE_MISMATCH","HOME_TIMEOUT","FINAL_FEEDBACK_TIMEOUT","CONTROL_FAILURE","INITIALIZATION_TIMEOUT"})
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
    response.message = expired ? (last_detail_.empty() ?
      "arm initialization timed out; outcome unknown" : "init timeout; " + last_detail_) : message;
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
  std::string endpoint_, initialize_endpoint_, last_detail_;
  std::array<uint8_t, 16> boot_{};
  double timeout_ = 21.3, poll_period_ = 0.1;
  Clock::time_point poll_due_;
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
