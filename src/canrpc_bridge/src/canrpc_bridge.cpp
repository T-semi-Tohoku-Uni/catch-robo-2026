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
  uint32_t uptime() const { return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-start_).count()); }
  std::chrono::steady_clock::time_point start_;

  void send(uint32_t id, const std::vector<uint8_t>& data) {
    if (!socket_) return;
    try { socket_->send_bytes(static_cast<int>(id | CAN_EFF_FLAG),data); }
    catch (const std::exception &e) { RCLCPP_ERROR(get_logger(),"CAN TX: %s",e.what()); }
  }
  void boot() {
    std::vector<uint8_t> data(24);
    const auto ms=std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now()-start_).count();
    canrpc_boot_payload(data.data(),boot_.data(),static_cast<uint32_t>(ms),CANRPC_READY);
    send(canrpc_id(4,CANRPC_CONTROL,127,CANRPC_ANNOUNCE,1,seq_[0]++),data);
  }
  void receive() {
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
        } else if (canrpc_class(id)==CANRPC_PUBLISH && d.size()==12 &&
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
      RCLCPP_ERROR(get_logger(),"CAN RX: %s",e.what());
      trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    }
  }
  void stop() {
    poll_.reset(); announce_.reset(); target_.reset(); pump_.reset(); socket_.reset();
    current_.reset(); pressure_.reset();
  }
public:
  CanRpcBridge():LifecycleNode("canrpc_bridge") {
    declare_parameter("ifname","can0"); declare_parameter("non_blocking",true);
    declare_parameter("tx_retry_timeout_ms",5);
  }
  Return on_configure(const rclcpp_lifecycle::State&) override { return Return::SUCCESS; }
  Return on_activate(const rclcpp_lifecycle::State&) override {
    try {
      size_t offset=0;
      while (offset<boot_.size()) {
        const auto n=getrandom(boot_.data()+offset,boot_.size()-offset,0);
        if (n<0 && errno==EINTR) continue;
        if (n<=0) throw std::runtime_error("Boot ID random source failed");
        offset+=static_cast<size_t>(n);
      }
      bool nonzero=false; for (auto b:boot_) nonzero=nonzero || b!=0;
      if (!nonzero) throw std::runtime_error("Zero Boot ID");
      start_=std::chrono::steady_clock::now(); seq_.fill(0); peers_={};
      socket_=std::make_unique<CanBridge>(get_parameter("ifname").as_string(),
        get_parameter("non_blocking").as_bool()?CanBridge::SocketMode::NonBlocking:CanBridge::SocketMode::Blocking,
        static_cast<int>(get_parameter("tx_retry_timeout_ms").as_int()));
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
