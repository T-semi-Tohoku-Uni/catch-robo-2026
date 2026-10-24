#include "canrpc_core.h"
#include "canrpc_bridge/config.hpp"
#include "canrpc_bridge/socketcan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include "canrpc_bridge/msg/bridge_status.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "canrpc_bridge/srv/call.hpp"
#include <algorithm>
#include <array>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <sys/random.h>
#include <thread>
#include <type_traits>

class CanRpcBridge : public rclcpp_lifecycle::LifecycleNode {
  using Return=rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using Floats=std_msgs::msg::Float32MultiArray;
  using Ints=std_msgs::msg::Int32MultiArray;
  using Bytes=std_msgs::msg::ByteMultiArray;
  using Config=canrpc_bridge::Config;
  using Route=canrpc_bridge::Route;
  using Type=canrpc_bridge::ValueType;
  using Call=canrpc_bridge::srv::Call;
  struct PendingCall {
    std::shared_ptr<rmw_request_id_t> header;
    size_t endpoint=0;
    canrpc_rpc_client_t operation{};
    bool diagnostic_only=false;
    std::string unavailable_reason;
  };
  rclcpp::Service<Call>::SharedPtr call_service_;
  std::array<canrpc_rpc_pair_t,127> rpc_pairs_{};
  std::array<PendingCall,127> calls_{};
  canrpc_rpc_config_t rpc_config_{};
  std::array<canrpc_rpc_config_t,127> peer_rpc_config_{};
  struct RuntimeRoute {
    uint8_t sequence=0;
    rclcpp::SubscriptionBase::SharedPtr subscription;
    rclcpp::Publisher<Floats>::SharedPtr floats;
    rclcpp::Publisher<Ints>::SharedPtr ints;
    rclcpp::Publisher<Bytes>::SharedPtr bytes;
  };
  Config config_;
  std::unique_ptr<CanBridge> socket_;
  std::vector<RuntimeRoute> routes_;
  std::array<canrpc_peer_t,127> peers_{};
  std::array<canrpc_reset_client_t,127> resets_{};
  std::array<uint8_t,16> boot_{};
  uint8_t announce_sequence_=0;
  std::string startup_error_;
  enum class Health {Starting,Ready,Degraded};
  Health health_=Health::Degraded;
  bool runtime_active_=false, reset_required_=false, reset_requested_=false;
  uint32_t startup_deadline_=0;
  uint64_t command_generation_=0, call_generation_=0;
  std::string health_reason_="Not configured", transport_error_;
  std::array<std::array<uint8_t,16>,127> initialized_boots_{};
  rclcpp::Publisher<canrpc_bridge::msg::BridgeStatus>::SharedPtr status_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr poll_,announce_;
  std::chrono::steady_clock::time_point start_;
  uint32_t uptime() const {
    return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-start_).count());
  }
  static void random_bytes(uint8_t *data,size_t size) {
    size_t offset=0;
    while(offset<size) {const auto n=getrandom(data+offset,size-offset,0);
      if(n<0 && errno==EINTR) continue;
      if(n<=0) throw std::runtime_error("Random source failed");
      offset+=static_cast<size_t>(n);}
  }
  bool send(uint32_t id,const std::vector<uint8_t> &data) {
    try {socket_->send_bytes(static_cast<int>(id|CAN_EFF_FLAG),data);return true;}
    catch(const std::exception &e) {transport_error_=std::string("CAN TX: ")+e.what();return false;}
  }
  void announce() {
    if(!socket_) return;
    std::vector<uint8_t> data(24);
    canrpc_boot_payload(data.data(),boot_.data(),uptime(),CANRPC_READY,0);
    send(canrpc_id(config_.announce_priority,CANRPC_CONTROL,CANRPC_BROADCAST,CANRPC_ANNOUNCE,
                   config_.local_node,announce_sequence_++),data);
  }
  static bool allowed(const Route &route,double value) {
    if(route.finite && !std::isfinite(value)) return false;
    if(route.has_min && !(value>=route.minimum)) return false;
    if(route.has_max && !(value<=route.maximum)) return false;
    return true;
  }
  template<class Message>
  void transmit(size_t index,typename Message::ConstSharedPtr message) {
    if(health_!=Health::Ready || !runtime_active_ || !socket_) return;
    const auto &route=config_.routes[index];
    if(message->data.size()!=route.count || message->layout.data_offset!=0) return;
    std::vector<uint8_t> data(canrpc_wire_length(static_cast<uint8_t>(route.logical_size())),0);
    for(size_t i=0;i<route.count;++i) {
      const auto value=message->data[i];if(!allowed(route,static_cast<double>(value))) return;
      if constexpr(std::is_same<Message,Floats>::value) canrpc_put_float(data.data()+4*i,value);
      else if constexpr(std::is_same<Message,Ints>::value) canrpc_put_u32(data.data()+4*i,static_cast<uint32_t>(value));
      else data[i]=value;
    }
    if(!send(canrpc_publish_id(route.subject,config_.local_node,routes_[index].sequence++,route.priority),data))
      degrade(transport_error_,true);
  }
  template<class Message>
  void subscribe(size_t index) {
    routes_[index].subscription=create_subscription<Message>(config_.routes[index].topic,1,
      [this,index,generation=command_generation_](typename Message::ConstSharedPtr message){if(generation==command_generation_) transmit<Message>(index,message);});
  }
  void create_routes() {
    routes_.resize(config_.routes.size());
    for(size_t i=0;i<config_.routes.size();++i) {
      const auto &route=config_.routes[i];
      if(!route.transmit) {
        if(route.type==Type::Float32) routes_[i].floats=rclcpp::create_publisher<Floats>(this,route.topic,10);
        else if(route.type==Type::Int32) routes_[i].ints=rclcpp::create_publisher<Ints>(this,route.topic,10);
        else routes_[i].bytes=rclcpp::create_publisher<Bytes>(this,route.topic,10);
      }
    }
  }
  void enable_transmit_routes() {
    ++command_generation_;
    for(size_t i=0;i<config_.routes.size();++i) if(config_.routes[i].transmit) {
      if(config_.routes[i].type==Type::Float32) subscribe<Floats>(i);
      else if(config_.routes[i].type==Type::Int32) subscribe<Ints>(i);
      else subscribe<Bytes>(i);
    }
  }
  void publish_status() {
    using Status=canrpc_bridge::msg::BridgeStatus;
    Status message;
    message.state=health_==Health::Ready?Status::READY:health_==Health::Starting?Status::STARTING:Status::DEGRADED;
    message.commands_enabled=runtime_active_ && health_==Health::Ready;
    message.socket_open=bool(socket_);message.reason=health_reason_;
    std::ostringstream id;id << std::hex << std::setfill('0');
    for(auto value:boot_) id << std::setw(2) << unsigned(value);
    message.session_id=id.str();status_->publish(message);
  }
  void degrade(std::string reason,bool close_socket=false,uint8_t control=CANRPC_RPC_TIMEOUT) {
    health_=Health::Degraded;health_reason_=reason;++command_generation_;
    for(auto &route:routes_) route.subscription.reset();
    for(uint8_t node=1;node<127;++node) if(calls_[node].header) {
      canrpc_rpc_client_finish(&rpc_pairs_[node],&calls_[node].operation,&peer_rpc_config_[node],CANRPC_RPC_UNKNOWN,control,uptime());
      finish_call(node);
    }
    if(close_socket) socket_.reset();
    transport_error_.clear();startup_error_.clear();
    RCLCPP_ERROR(get_logger(),"Bridge DEGRADED: %s; explicit deactivate/activate required",reason.c_str());publish_status();
  }
  void recreate_call_service() {
    ++call_generation_;call_service_.reset();
    if(!config_.rpc.enabled) return;
    const auto generation=call_generation_;
    call_service_=create_service<Call>(config_.rpc.ros_service,
      [this,generation](const std::shared_ptr<rmw_request_id_t> header,const std::shared_ptr<Call::Request> request) {
        if(generation!=call_generation_) {respond(header,Call::Response::UNAVAILABLE,{},"Request belongs to an expired bridge phase");return;}
        handle_call(header,request);
      });
  }
  void ready() {
    recreate_call_service();
    health_=Health::Ready;health_reason_="Initialization confirmed";
    for(auto node:config_.reset_targets) std::copy_n(peers_[node].boot,16,initialized_boots_[node].begin());
    enable_transmit_routes();publish_status();
  }
  void publish(size_t index,const std::vector<uint8_t> &data) {
    const auto &route=config_.routes[index];
    if(!canrpc_padding_valid(data.data(),static_cast<uint8_t>(route.logical_size()),static_cast<uint8_t>(data.size()))) return;
    Floats floats;Ints ints;Bytes bytes;
    for(size_t i=0;i<route.count;++i) {
      if(route.type==Type::Float32) {const auto v=canrpc_get_float(data.data()+4*i);if(!allowed(route,v)) return;floats.data.push_back(v);}
      else if(route.type==Type::Int32) {const auto v=static_cast<int32_t>(canrpc_get_u32(data.data()+4*i));if(!allowed(route,v)) return;ints.data.push_back(v);}
      else {const auto v=data[i];if(!allowed(route,v)) return;bytes.data.push_back(v);}
    }
    if(route.type==Type::Float32) routes_[index].floats->publish(floats);
    else if(route.type==Type::Int32) routes_[index].ints->publish(ints);
    else routes_[index].bytes->publish(bytes);
  }
  void respond(const std::shared_ptr<rmw_request_id_t> &header,uint8_t outcome,
               const std::vector<uint8_t> &data,const std::string &message) {
    if(!call_service_) return;
    Call::Response response;response.outcome=outcome;response.data=data;response.message=message;
    try {call_service_->send_response(*header,response);}
    catch(const std::exception &error) {RCLCPP_WARN(get_logger(),"RPC ROS response failed: %s",error.what());}
  }
  void finish_call(uint8_t node) {
    auto &call=calls_[node];if(!call.header || call.operation.state==CANRPC_RPC_PENDING) return;
    const auto &operation=call.operation;const auto &ep=config_.rpc_endpoints[call.endpoint];
    uint8_t outcome=Call::Response::TIMEOUT_UNKNOWN;std::vector<uint8_t> data;std::string message;
    if(operation.state==CANRPC_RPC_SUCCESS) {
      outcome=Call::Response::OK;data.assign(operation.response,operation.response+ep.response_bytes);message="Remote final response received";
    } else if(operation.state==CANRPC_RPC_REJECTED) {
      outcome=Call::Response::REJECTED;message="Remote rejected RPC: control="+std::to_string(operation.control);
    } else if(operation.control==CANRPC_RPC_PEER_CHANGED) {
      outcome=Call::Response::PEER_CHANGED;message="Remote generation changed; execution outcome unknown";
    } else message="RPC outcome unknown: control="+std::to_string(operation.control);
    if(call.diagnostic_only) {outcome=Call::Response::UNAVAILABLE;message=call.unavailable_reason+"; diagnostic: "+message;}
    respond(call.header,outcome,data,message);call.header.reset();
  }
  void handle_call(const std::shared_ptr<rmw_request_id_t> &header,const std::shared_ptr<Call::Request> &request) {
    auto found=std::find_if(config_.rpc_endpoints.begin(),config_.rpc_endpoints.end(),
      [&request](const auto &ep){return ep.name==request->endpoint;});
    if(found==config_.rpc_endpoints.end() || request->data.size()!=found->request_bytes) {
      respond(header,Call::Response::INVALID_REQUEST,{},"Unknown endpoint or invalid logical request length");return;
    }
    if(!runtime_active_ || !socket_ || health_==Health::Starting ||
       (health_==Health::Degraded && !found->startup_diagnostic)) {
      respond(header,Call::Response::UNAVAILABLE,{},health_reason_);return;
    }
    const auto node=found->destination;
    if(calls_[node].header || rpc_pairs_[node].active) {respond(header,Call::Response::BUSY,{},"One RPC is already pending for this peer");return;}
    std::vector<uint8_t> data(canrpc_wire_length(found->request_bytes),0);std::copy(request->data.begin(),request->data.end(),data.begin());
    auto &call=calls_[node];call.diagnostic_only=health_!=Health::Ready;call.unavailable_reason=health_reason_;call.endpoint=static_cast<size_t>(found-config_.rpc_endpoints.begin());
    if(!canrpc_rpc_client_begin(&rpc_pairs_[node],&call.operation,&peer_rpc_config_[node],boot_.data(),&peers_[node],
         config_.local_node,node,found->service,data.data(),static_cast<uint8_t>(data.size()),uptime(),config_.peer_timeout_ms,found->require_ready)) {
      respond(header,Call::Response::UNAVAILABLE,{},(call.diagnostic_only?health_reason_+"; ":"")+"Peer is unavailable or no guarded Tag can be allocated");return;
    }
    call.header=header;advance_calls();
  }
  void advance_calls() {
    const uint32_t now=uptime();
    for(uint8_t node=1;node<127;++node) {
      auto &call=calls_[node];if(!call.header) continue;
      const auto action=canrpc_rpc_client_next(&rpc_pairs_[node],&call.operation,&peer_rpc_config_[node],&peers_[node],now,config_.peer_timeout_ms);
      if(action!=CANRPC_RPC_NONE) {
        const auto &op=call.operation;const bool query=action==CANRPC_RPC_SEND_QUERY;
        const auto id=canrpc_id(config_.priority,query?CANRPC_CONTROL:CANRPC_REQUEST,node,
          query?CANRPC_QUERY_STATE:op.service,config_.local_node,op.tag);
        const std::vector<uint8_t> data=query?std::vector<uint8_t>{}:std::vector<uint8_t>(op.request,op.request+op.request_length);
        if(!send(id,data)) {
          canrpc_rpc_client_finish(&rpc_pairs_[node],&call.operation,&peer_rpc_config_[node],CANRPC_RPC_UNKNOWN,CANRPC_RPC_TIMEOUT,now);
          finish_call(node);degrade(transport_error_,true);return;
        }
      }
      finish_call(node);
    }
  }
  void receive() {
    if(!socket_) return;
    try {
      CanBridge::RxData_struct frame;
      for(uint32_t count=0;count<config_.rx_budget && transport_error_.empty() && socket_->receive_data(frame);++count) {
        const uint32_t id=static_cast<uint32_t>(frame.canid);if(!canrpc_valid_source(id)) continue;
        const uint8_t source=canrpc_source(id);const auto &d=frame.data;
        if(calls_[source].header) {
          const auto &ep=config_.rpc_endpoints[calls_[source].endpoint];
          const bool valid_response=canrpc_class(id)!=CANRPC_RESPONSE ||
            canrpc_padding_valid(d.data(),ep.response_bytes,static_cast<uint8_t>(d.size()));
          if(valid_response && canrpc_rpc_client_receive(&rpc_pairs_[source],&calls_[source].operation,&peer_rpc_config_[source],
              &peers_[source],id,d.data(),static_cast<uint8_t>(d.size()),uptime(),config_.peer_timeout_ms)) {
            finish_call(source);continue;
          }
        }
        if(canrpc_class(id)==CANRPC_CONTROL && canrpc_destination(id)==CANRPC_BROADCAST && canrpc_service(id)==CANRPC_ANNOUNCE) {
          canrpc_peer_announce(&peers_[source],d.data(),static_cast<uint32_t>(d.size()),uptime());
        } else if(health_==Health::Starting && canrpc_class(id)==CANRPC_CONTROL && canrpc_service(id)==CANRPC_RESET_STATUS) {
          if(canrpc_reset_client_status(&resets_[source],id,d.data(),static_cast<uint32_t>(d.size()),config_.local_node,source,&peers_[source])<0)
            startup_error_="Node "+std::to_string(source)+" rejected reset: "+std::to_string(d[44]);
        } else if(canrpc_class(id)==CANRPC_CONTROL && canrpc_destination(id)==config_.local_node &&
                  canrpc_service(id)==CANRPC_ANNOUNCE_QUERY && canrpc_tag(id)==0 && d.empty()) {
          announce();
        } else if(canrpc_class(id)==CANRPC_CONTROL && canrpc_destination(id)==config_.local_node &&
                  canrpc_service(id)==CANRPC_RESET_REQUEST && d.size()==48) {
          std::vector<uint8_t> reply(48);canrpc_reset_status_payload(reply.data(),d.data(),boot_.data(),uptime(),CANRPC_RESET_REJECT_AUTHORITY);
          send(canrpc_id(config_.priority,CANRPC_CONTROL,source,CANRPC_RESET_STATUS,config_.local_node,canrpc_tag(id)),reply);
        } else if(canrpc_class(id)==CANRPC_REQUEST && canrpc_destination(id)==config_.local_node) {
          send(canrpc_id(config_.priority,CANRPC_CONTROL,source,CANRPC_REJECT_SERVICE,config_.local_node,canrpc_tag(id)),{});
        } else if(health_!=Health::Starting && canrpc_class(id)==CANRPC_PUBLISH) {
          for(size_t i=0;i<config_.routes.size();++i) {const auto &route=config_.routes[i];
            if(!route.transmit && route.subject==canrpc_subject(id) && route.source==source &&
               (!route.require_peer || canrpc_peer_ready(&peers_[source],uptime(),config_.peer_timeout_ms))) {publish(i,d);break;}}
        }
      }
    } catch(const std::exception &e) {
      transport_error_=std::string("CAN RX: ")+e.what();
    }
  }
  void send_reset(uint8_t node) {
    auto &op=resets_[node];
    const std::vector<uint8_t> data(op.request,op.request+48);
    if(!send(canrpc_id(config_.priority,CANRPC_CONTROL,node,CANRPC_RESET_REQUEST,config_.local_node,op.tag),data))
      throw std::runtime_error("Failed to send startup reset");
    canrpc_reset_client_sent(&op,uptime());
  }
  void startup_tick() {
    if(health_!=Health::Starting) return;
    const auto now=uptime();
    if(!startup_error_.empty()) {degrade(startup_error_);return;}
    if(static_cast<int32_t>(now-startup_deadline_)>=0) {degrade("Startup initialization timed out; reset outcome unknown");return;}
    if(!reset_requested_) {
      for(auto node:config_.reset_targets) if(!canrpc_peer_ready(&peers_[node],now,config_.peer_timeout_ms) ||
          !(peers_[node].capabilities&CANRPC_CAP_SYSTEM_RESET)) return;
      for(auto node:config_.reset_targets) {
        uint64_t nonce;random_bytes(reinterpret_cast<uint8_t*>(&nonce),sizeof(nonce));
        canrpc_reset_client_begin(&resets_[node],&peers_[node],boot_.data(),nonce,now,config_.request_window_ms,0);
        send_reset(node);
      }
      reset_requested_=true;health_reason_="Reset requested; waiting for all new READY generations";publish_status();
    } else {
      bool complete=true;
      for(auto node:config_.reset_targets) {
        auto &operation=resets_[node];
        complete=canrpc_reset_client_complete(&operation,&peers_[node],now,config_.peer_timeout_ms) && complete;
        if(canrpc_reset_client_should_retry(&operation,now,config_.retry_interval_ms,config_.retry_window_ms)) send_reset(node);
      }
      if(complete) ready();
    }
  }
  void tick() {
    if(!runtime_active_ || !socket_) return;
    try {
      receive();
      if(!transport_error_.empty()) {degrade(transport_error_,true);return;}
      startup_tick();
      if(!transport_error_.empty()) {degrade(transport_error_,true);return;}
      if(health_==Health::Ready && reset_required_) {
        for(auto node:config_.reset_targets) {
          if(!std::equal(initialized_boots_[node].begin(),initialized_boots_[node].end(),peers_[node].boot)) {
            degrade("Required peer "+std::to_string(node)+" changed generation",false,CANRPC_RPC_PEER_CHANGED);return;
          }
          if(!canrpc_peer_ready(&peers_[node],uptime(),config_.peer_timeout_ms)) {
            degrade("Required peer "+std::to_string(node)+" lost fresh READY");return;
          }
        }
      }
      advance_calls();
      if(!transport_error_.empty()) degrade(transport_error_,true);
    } catch(const std::exception &error) {degrade(error.what(),true);}
  }
  void stop() {
    runtime_active_=false;health_=Health::Degraded;health_reason_="Bridge inactive; explicit activation required";
    ++command_generation_;poll_.reset();announce_.reset();
    for(uint8_t node=1;node<127;++node) if(calls_[node].header) {
      canrpc_rpc_client_finish(&rpc_pairs_[node],&calls_[node].operation,&peer_rpc_config_[node],CANRPC_RPC_UNKNOWN,CANRPC_RPC_TIMEOUT,uptime());finish_call(node);
    }
    routes_.clear();socket_.reset();recreate_call_service();publish_status();
  }
public:
  CanRpcBridge():LifecycleNode("canrpc_bridge") {
    rcl_interfaces::msg::ParameterDescriptor descriptor;descriptor.read_only=true;
    const auto status_topic=declare_parameter("status_topic",std::string("canrpc/status"),descriptor);
    start_=std::chrono::steady_clock::now();
    status_=rclcpp::create_publisher<canrpc_bridge::msg::BridgeStatus>(this,status_topic,rclcpp::QoS(1).transient_local());
    status_timer_=create_wall_timer(std::chrono::seconds(1),[this]{publish_status();});
    publish_status();
    declare_parameter("config_file",std::string(),descriptor);
    declare_parameter("reset_on_startup",false,descriptor);
    declare_parameter("startup_timeout_sec",0.0,descriptor);
    declare_parameter("startup_block_reason",std::string(),descriptor);
  }
  Return on_configure(const rclcpp_lifecycle::State&) override {
    try {config_=canrpc_bridge::load_config(get_parameter("config_file").as_string());
      const auto &rpc=config_.rpc;
      rpc_config_={rpc.retry_interval_ms,rpc.retry_window_ms,rpc.query_interval_ms,rpc.response_timeout_ms,
                   rpc.result_retention_ms,rpc.tag_guard_ms,rpc.max_frame_age_ms,static_cast<uint8_t>(rpc.cache_capacity)};
      peer_rpc_config_.fill(rpc_config_);
      for(const auto &entry:config_.nodes) if(entry.second.rpc.enabled) {
        const auto &p=entry.second.rpc;
        peer_rpc_config_[entry.second.id]={p.retry_interval_ms,p.retry_window_ms,p.query_interval_ms,p.response_timeout_ms,
          p.result_retention_ms,p.tag_guard_ms,p.max_frame_age_ms,static_cast<uint8_t>(p.cache_capacity)};
      }
      recreate_call_service();
      health_reason_=get_parameter("startup_block_reason").as_string();
      if(health_reason_.empty()) health_reason_="Configured; activation required";
      publish_status();
      RCLCPP_INFO(get_logger(),"Loaded project %s, local node %u, %zu routes",config_.project.c_str(),config_.local_node,config_.routes.size());return Return::SUCCESS;}
    catch(const std::exception &e) {health_reason_=e.what();publish_status();RCLCPP_ERROR(get_logger(),"%s",e.what());return Return::FAILURE;}
  }
  Return on_activate(const rclcpp_lifecycle::State&) override {
    runtime_active_=true;health_=Health::Starting;health_reason_="Opening CAN transport";
    recreate_call_service();
    try {
      random_bytes(boot_.data(),boot_.size());bool nonzero=false;for(auto value:boot_) nonzero=nonzero || value!=0;
      if(!nonzero) throw std::runtime_error("Zero Boot ID");
      announce_sequence_=0;peers_={};resets_={};reset_requested_=false;transport_error_.clear();startup_error_.clear();
      reset_required_=get_parameter("reset_on_startup").as_bool();
      const auto override_timeout=get_parameter("startup_timeout_sec").as_double();
      if(!std::isfinite(override_timeout) || override_timeout<0 || override_timeout>=INT32_MAX/1000.0)
        throw std::runtime_error("Invalid startup_timeout_sec override");
      if(reset_required_ && config_.reset_targets.empty()) throw std::runtime_error("reset_on_startup requires reset.targets");
      startup_deadline_=uptime()+static_cast<uint32_t>((override_timeout>0?override_timeout:config_.startup_timeout_sec)*1000);
      socket_=std::make_unique<CanBridge>(config_.interface,config_.non_blocking?CanBridge::SocketMode::NonBlocking:CanBridge::SocketMode::Blocking,
                                        static_cast<int>(config_.tx_retry_timeout_ms),config_.brs);
      create_routes();announce();
      if(!transport_error_.empty()) throw std::runtime_error(transport_error_);
      if(!reset_required_) {ready();health_reason_="Transport ready; peer conditions are checked per endpoint";publish_status();}
      else {health_reason_="Waiting for configured peers before startup reset";publish_status();}
      announce_=create_wall_timer(std::chrono::milliseconds(config_.announce_period_ms),[this]{
        announce();if(!transport_error_.empty()) degrade(transport_error_,true);});
      poll_=create_wall_timer(std::chrono::milliseconds(config_.poll_period_ms),[this]{tick();});
    } catch(const std::exception &error) {degrade(error.what(),true);}
    return Return::SUCCESS;
  }
  Return on_deactivate(const rclcpp_lifecycle::State&) override {stop();return Return::SUCCESS;}
  Return on_cleanup(const rclcpp_lifecycle::State&) override {stop();call_service_.reset();return Return::SUCCESS;}
  Return on_shutdown(const rclcpp_lifecycle::State&) override {stop();return Return::SUCCESS;}
  Return on_error(const rclcpp_lifecycle::State&) override {stop();return Return::SUCCESS;}
};
int main(int argc,char **argv) {rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<CanRpcBridge>()->get_node_base_interface());rclcpp::shutdown();return 0;}
