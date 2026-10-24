#include "canrpc_bridge/canrpc.h"
#include "canrpc_bridge/socketcan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include <array>
#include <algorithm>
#include <cmath>
#include <sys/random.h>
#include <thread>

class CanRpcBridge : public rclcpp_lifecycle::LifecycleNode {
  using Return = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using Floats = std_msgs::msg::Float32MultiArray;
  using Ints = std_msgs::msg::Int32MultiArray;
  std::unique_ptr<CanBridge> socket_;
  rclcpp::Subscription<Floats>::SharedPtr target_;
  rclcpp::Subscription<Ints>::SharedPtr pump_;
  rclcpp::Publisher<Floats>::SharedPtr current_;
  rclcpp::Publisher<Ints>::SharedPtr pressure_;
  rclcpp::TimerBase::SharedPtr poll_, announce_;
  std::array<uint8_t,16> boot_{};
  std::array<uint8_t,3> seq_{};
  std::array<canrpc_peer_t,5> peers_{};
  struct ResetOperation {
    std::vector<uint8_t> request;
    uint32_t created_ms=0, last_send_ms=0;
    uint8_t tag=0;
    bool accepted=false, changed=false;
  };
  std::array<ResetOperation,5> resets_{};
  std::string startup_error_;
  static void random_bytes(uint8_t *data,size_t size) {
    size_t offset=0;
    while (offset<size) {
      const auto n=getrandom(data+offset,size-offset,0);
      if (n<0 && errno==EINTR) continue;
      if (n<=0) throw std::runtime_error("Random source failed");
      offset+=static_cast<size_t>(n);
    }
  }
  uint32_t uptime() const { return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-start_).count()); }
  std::chrono::steady_clock::time_point start_;

  bool send(uint32_t id, const std::vector<uint8_t>& data) {
    if (!socket_) return false;
    try { socket_->send_bytes(static_cast<int>(id | CAN_EFF_FLAG),data); return true; }
    catch (const std::exception &e) { RCLCPP_ERROR(get_logger(),"CAN TX: %s",e.what()); return false; }
  }
  void boot() {
    std::vector<uint8_t> data(24);
    const auto ms=std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now()-start_).count();
    canrpc_boot_payload(data.data(),boot_.data(),static_cast<uint32_t>(ms),CANRPC_READY);
    send(canrpc_id(4,CANRPC_CONTROL,127,CANRPC_ANNOUNCE,1,seq_[0]++),data);
  }
  void receive(bool startup=false) {
    try {
      CanBridge::RxData_struct frame;
      // Bound work so a busy bus cannot starve ROS callbacks.
      for (int count=0; count<32 && socket_->receive_data(frame); ++count) {
        const uint32_t id=static_cast<uint32_t>(frame.canid);
        if (!canrpc_valid_source(id)) continue;
        const auto &d=frame.data;
        if (canrpc_class(id)==CANRPC_CONTROL && canrpc_destination(id)==127 &&
            canrpc_service(id)==CANRPC_ANNOUNCE && canrpc_source(id)<peers_.size()) {
          canrpc_peer_announce(&peers_[canrpc_source(id)],d.data(),static_cast<uint32_t>(d.size()),uptime());
        } else if (startup && canrpc_class(id)==CANRPC_CONTROL && canrpc_destination(id)==1 &&
                   canrpc_service(id)==CANRPC_RESET_STATUS && canrpc_source(id)>=2 &&
                   canrpc_source(id)<=4 && d.size()==48) {
          auto &operation=resets_[canrpc_source(id)];
          if (operation.request.empty() || operation.changed ||
              (peers_[canrpc_source(id)].known && std::memcmp(peers_[canrpc_source(id)].boot,operation.request.data(),16)!=0) ||
              canrpc_tag(id)!=operation.tag ||
              !std::equal(d.begin(),d.begin()+16,operation.request.begin()) ||
              !std::equal(d.begin()+16,d.begin()+32,operation.request.begin()) ||
              !std::equal(d.begin()+32,d.begin()+40,operation.request.begin()+32) ||
              d[45]!=0 || d[46]!=0 || d[47]!=0) continue;
          if (d[44]==CANRPC_RESET_ACCEPTED) operation.accepted=true;
          else if (d[44]!=CANRPC_RESET_PREPARING)
            startup_error_="MCU "+std::to_string(canrpc_source(id))+" rejected reset: "+std::to_string(d[44]);
        } else if (canrpc_class(id)==CANRPC_CONTROL && canrpc_destination(id)==1 &&
            canrpc_service(id)==CANRPC_ANNOUNCE_QUERY && canrpc_tag(id)==0 && d.empty()) {
          boot();
        } else if (canrpc_class(id)==CANRPC_CONTROL && canrpc_destination(id)==1 &&
                   canrpc_service(id)==CANRPC_RESET_REQUEST && d.size()==48) {
          std::vector<uint8_t> reply(48);
          std::copy_n(d.begin(),16,reply.begin());
          std::copy(boot_.begin(),boot_.end(),reply.begin()+16);
          std::copy_n(d.begin()+32,8,reply.begin()+32);
          canrpc_put_u32(reply.data()+40,uptime());
          reply[44]=CANRPC_RESET_REJECT_AUTHORITY; reply[45]=d[44];
          send(canrpc_id(4,CANRPC_CONTROL,canrpc_source(id),CANRPC_RESET_STATUS,1,canrpc_tag(id)),reply);
        } else if (canrpc_class(id)==CANRPC_REQUEST && canrpc_destination(id)==1) {
          send(canrpc_id(4,CANRPC_CONTROL,canrpc_source(id),CANRPC_REJECT_SERVICE,1,canrpc_tag(id)),{});
        } else if (!startup && canrpc_class(id)==CANRPC_PUBLISH && d.size()==12 &&
                   canrpc_source(id)<peers_.size() && canrpc_peer_ready(&peers_[canrpc_source(id)],uptime())) {
          if (canrpc_subject(id)==CANRPC_SUBJECT_CURRENT && canrpc_source(id)==CANRPC_NODE_ARM) {
            Floats msg; bool finite=true;
            for (int i=0;i<3;++i) { const float value=canrpc_get_float(d.data()+4*i);
              finite=finite && std::isfinite(value); msg.data.push_back(value); }
            if (finite) current_->publish(msg);
          } else if (canrpc_subject(id)==CANRPC_SUBJECT_PRESSURE && canrpc_source(id)==CANRPC_NODE_PRESSURE) {
            Ints msg;
            for (int i=0;i<3;++i) msg.data.push_back(static_cast<int32_t>(canrpc_get_u32(d.data()+4*i)));
            pressure_->publish(msg);
          }
        }
      }
    } catch (const std::exception &e) {
      if (startup) throw;
      RCLCPP_ERROR(get_logger(),"CAN RX: %s",e.what());
      trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    }
  }
  void reset_boards() {
    const double timeout=get_parameter("startup_timeout_sec").as_double();
    if (!std::isfinite(timeout) || timeout<=0 || timeout>600)
      throw std::runtime_error("startup_timeout_sec must be in (0,600]");
    const uint32_t deadline=uptime()+static_cast<uint32_t>(timeout*1000);
    uint32_t next_announce=uptime()+CANRPC_ANNOUNCE_PERIOD_MS;
    bool requested=false;
    resets_={}; startup_error_.clear();
    while (rclcpp::ok() && (int32_t)(deadline-uptime())>0) {
      receive(true);
      const uint32_t now=uptime();
      if (!startup_error_.empty()) throw std::runtime_error(startup_error_);
      if ((int32_t)(now-next_announce)>=0) {
        boot(); next_announce=now+CANRPC_ANNOUNCE_PERIOD_MS;
      }
      if (!requested) {
        bool ready=true;
        for (uint8_t node=2;node<=4;++node)
          ready=ready && canrpc_peer_ready(&peers_[node],now) &&
                (peers_[node].capabilities & CANRPC_CAP_SYSTEM_RESET)!=0;
        if (ready) {
          for (uint8_t node=2;node<=4;++node) {
            auto &operation=resets_[node];
            operation.request.resize(48); operation.created_ms=now;
            operation.last_send_ms=now; operation.tag=0;
            uint64_t nonce; random_bytes(reinterpret_cast<uint8_t*>(&nonce),sizeof(nonce));
            const auto &peer=peers_[node];
            canrpc_reset_payload(operation.request.data(),peer.boot,boot_.data(),nonce,
              peer.uptime_ms+(uint32_t)(now-peer.seen_ms)+1000u);
            if (!send(canrpc_id(4,CANRPC_CONTROL,node,CANRPC_RESET_REQUEST,1,operation.tag),operation.request))
              throw std::runtime_error("Failed to send startup reset");
          }
          requested=true;
          RCLCPP_INFO(get_logger(),"Reset requested for MCU nodes 2,3,4; waiting for new READY generations");
        }
      } else {
        bool completed=true;
        for (uint8_t node=2;node<=4;++node) {
          auto &operation=resets_[node]; const auto &peer=peers_[node];
          if (peer.known && std::memcmp(peer.boot,operation.request.data(),16)!=0) operation.changed=true;
          completed=completed && operation.changed && canrpc_peer_ready(&peer,now);
          if (!operation.changed && !operation.accepted && now-operation.created_ms<900u &&
              now-operation.last_send_ms>=200u) {
            if (!send(canrpc_id(4,CANRPC_CONTROL,node,CANRPC_RESET_REQUEST,1,operation.tag),operation.request))
              throw std::runtime_error("Failed to resend startup reset");
            operation.last_send_ms=now;
          }
        }
        if (completed) { RCLCPP_INFO(get_logger(),"All MCU reset generations are READY"); return; }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    throw std::runtime_error("MCU startup initialization timed out or was interrupted; reset outcome unknown");
  }
  void stop() {
    poll_.reset(); announce_.reset(); target_.reset(); pump_.reset(); socket_.reset();
    current_.reset(); pressure_.reset();
  }
public:
  CanRpcBridge():LifecycleNode("canrpc_bridge") {
    declare_parameter("ifname","can0"); declare_parameter("non_blocking",true);
    declare_parameter("tx_retry_timeout_ms",5);
    declare_parameter("reset_on_startup",false);
    declare_parameter("startup_timeout_sec",30.0);
  }
  Return on_configure(const rclcpp_lifecycle::State&) override { return Return::SUCCESS; }
  Return on_activate(const rclcpp_lifecycle::State&) override {
    try {
      random_bytes(boot_.data(),boot_.size());
      bool nonzero=false; for (auto b:boot_) nonzero=nonzero || b!=0;
      if (!nonzero) throw std::runtime_error("Zero Boot ID");
      start_=std::chrono::steady_clock::now(); seq_.fill(0); peers_={};
      socket_=std::make_unique<CanBridge>(get_parameter("ifname").as_string(),
        get_parameter("non_blocking").as_bool()?CanBridge::SocketMode::NonBlocking:CanBridge::SocketMode::Blocking,
        static_cast<int>(get_parameter("tx_retry_timeout_ms").as_int()));
      boot();
      if (get_parameter("reset_on_startup").as_bool()) reset_boards();
      current_=rclcpp::create_publisher<Floats>(this,"current_joints",10);
      pressure_=rclcpp::create_publisher<Ints>(this,"pressure_sensor",10);
      target_=create_subscription<Floats>("target_joint_angles",1,[this](Floats::ConstSharedPtr msg) {
        if (msg->data.size()!=3) return;
        std::vector<uint8_t> d(12);
        for (int i=0;i<3;++i) { if (!std::isfinite(msg->data[i])) return; canrpc_put_float(d.data()+4*i,msg->data[i]); }
        send(canrpc_publish_id(CANRPC_SUBJECT_TARGET,1,seq_[1]++),d);
      });
      pump_=create_subscription<Ints>("pump_state",1,[this](Ints::ConstSharedPtr msg) {
        if (msg->data.size()!=1 || msg->data[0]<0 || msg->data[0]>63) return;
        std::vector<uint8_t> d(4); canrpc_put_u32(d.data(),static_cast<uint32_t>(msg->data[0]));
        send(canrpc_publish_id(CANRPC_SUBJECT_PUMP,1,seq_[2]++),d);
      });
      boot();
      announce_=create_wall_timer(std::chrono::milliseconds(200),[this]{boot();});
      poll_=create_wall_timer(std::chrono::milliseconds(2),[this]{receive();});
      return Return::SUCCESS;
    } catch (const std::exception &e) { RCLCPP_ERROR(get_logger(),"%s",e.what()); stop(); return Return::FAILURE; }
  }
  Return on_deactivate(const rclcpp_lifecycle::State&) override {stop();return Return::SUCCESS;}
  Return on_cleanup(const rclcpp_lifecycle::State&) override {stop();return Return::SUCCESS;}
  Return on_shutdown(const rclcpp_lifecycle::State&) override {stop();return Return::SUCCESS;}
  Return on_error(const rclcpp_lifecycle::State&) override {stop();return Return::SUCCESS;}
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv); rclcpp::spin(std::make_shared<CanRpcBridge>()->get_node_base_interface());
  rclcpp::shutdown(); return 0;
}
