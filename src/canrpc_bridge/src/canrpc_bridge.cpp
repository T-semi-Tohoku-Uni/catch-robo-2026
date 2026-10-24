#include "canrpc_core.h"
#include "canrpc_bridge/config.hpp"
#include "canrpc_bridge/socketcan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
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
    catch(const std::exception &e) {RCLCPP_ERROR(get_logger(),"CAN TX: %s",e.what());return false;}
  }
  void announce() {
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
    const auto &route=config_.routes[index];
    if(message->data.size()!=route.count || message->layout.data_offset!=0) return;
    std::vector<uint8_t> data(canrpc_wire_length(static_cast<uint8_t>(route.logical_size())),0);
    for(size_t i=0;i<route.count;++i) {
      const auto value=message->data[i];if(!allowed(route,static_cast<double>(value))) return;
      if constexpr(std::is_same<Message,Floats>::value) canrpc_put_float(data.data()+4*i,value);
      else if constexpr(std::is_same<Message,Ints>::value) canrpc_put_u32(data.data()+4*i,static_cast<uint32_t>(value));
      else data[i]=value;
    }
    send(canrpc_publish_id(route.subject,config_.local_node,routes_[index].sequence++,route.priority),data);
  }
  template<class Message>
  void subscribe(size_t index) {
    routes_[index].subscription=create_subscription<Message>(config_.routes[index].topic,1,
      [this,index](typename Message::ConstSharedPtr message){transmit<Message>(index,message);});
  }
  void create_routes() {
    routes_.resize(config_.routes.size());
    for(size_t i=0;i<config_.routes.size();++i) {
      const auto &route=config_.routes[i];
      if(route.transmit) {
        if(route.type==Type::Float32) subscribe<Floats>(i);
        else if(route.type==Type::Int32) subscribe<Ints>(i);
        else subscribe<Bytes>(i);
      } else {
        if(route.type==Type::Float32) routes_[i].floats=rclcpp::create_publisher<Floats>(this,route.topic,10);
        else if(route.type==Type::Int32) routes_[i].ints=rclcpp::create_publisher<Ints>(this,route.topic,10);
        else routes_[i].bytes=rclcpp::create_publisher<Bytes>(this,route.topic,10);
      }
    }
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
    respond(call.header,outcome,data,message);call.header.reset();
  }
  void handle_call(const std::shared_ptr<rmw_request_id_t> &header,const std::shared_ptr<Call::Request> &request) {
    auto found=std::find_if(config_.rpc_endpoints.begin(),config_.rpc_endpoints.end(),
      [&request](const auto &ep){return ep.name==request->endpoint;});
    if(found==config_.rpc_endpoints.end() || request->data.size()!=found->request_bytes) {
      respond(header,Call::Response::INVALID_REQUEST,{},"Unknown endpoint or invalid logical request length");return;
    }
    const auto node=found->destination;
    if(calls_[node].header || rpc_pairs_[node].active) {respond(header,Call::Response::BUSY,{},"One RPC is already pending for this peer");return;}
    std::vector<uint8_t> data(canrpc_wire_length(found->request_bytes),0);std::copy(request->data.begin(),request->data.end(),data.begin());
    auto &call=calls_[node];call.endpoint=static_cast<size_t>(found-config_.rpc_endpoints.begin());
    if(!canrpc_rpc_client_begin(&rpc_pairs_[node],&call.operation,&peer_rpc_config_[node],boot_.data(),&peers_[node],
         config_.local_node,node,found->service,data.data(),static_cast<uint8_t>(data.size()),uptime(),config_.peer_timeout_ms,found->require_ready)) {
      respond(header,Call::Response::UNAVAILABLE,{},"Peer is unavailable or no guarded Tag can be allocated");return;
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
        if(!send(id,data)) canrpc_rpc_client_finish(&rpc_pairs_[node],&call.operation,&peer_rpc_config_[node],CANRPC_RPC_UNKNOWN,CANRPC_RPC_TIMEOUT,now);
      }
      finish_call(node);
    }
  }
  void receive(bool startup=false) {
    try {
      CanBridge::RxData_struct frame;
      for(uint32_t count=0;count<config_.rx_budget && socket_->receive_data(frame);++count) {
        const uint32_t id=static_cast<uint32_t>(frame.canid);if(!canrpc_valid_source(id)) continue;
        const uint8_t source=canrpc_source(id);const auto &d=frame.data;
        if(!startup && calls_[source].header) {
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
        } else if(startup && canrpc_class(id)==CANRPC_CONTROL && canrpc_service(id)==CANRPC_RESET_STATUS) {
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
        } else if(!startup && canrpc_class(id)==CANRPC_PUBLISH) {
          for(size_t i=0;i<config_.routes.size();++i) {const auto &route=config_.routes[i];
            if(!route.transmit && route.subject==canrpc_subject(id) && route.source==source &&
               (!route.require_peer || canrpc_peer_ready(&peers_[source],uptime(),config_.peer_timeout_ms))) {publish(i,d);break;}}
        }
      }
    } catch(const std::exception &e) {
      if(startup) throw;
      RCLCPP_ERROR(get_logger(),"CAN RX: %s",e.what());
      trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    }
  }
  void send_reset(uint8_t node) {
    auto &op=resets_[node];
    const std::vector<uint8_t> data(op.request,op.request+48);
    if(!send(canrpc_id(config_.priority,CANRPC_CONTROL,node,CANRPC_RESET_REQUEST,config_.local_node,op.tag),data))
      throw std::runtime_error("Failed to send startup reset");
    canrpc_reset_client_sent(&op,uptime());
  }
  void startup_diagnostics() {
    const uint32_t deadline=uptime()+config_.rpc.response_timeout_ms;
    std::array<canrpc_rpc_client_t,127> operations{};
    std::array<const canrpc_bridge::RpcEndpoint*,127> endpoints{};
    for(const auto &ep:config_.rpc_endpoints) {
      if(!ep.startup_diagnostic || endpoints[ep.destination]) continue;
      if(canrpc_rpc_client_begin(&rpc_pairs_[ep.destination],&operations[ep.destination],&peer_rpc_config_[ep.destination],
           boot_.data(),&peers_[ep.destination],config_.local_node,ep.destination,ep.service,nullptr,0,
           uptime(),config_.peer_timeout_ms,false)) endpoints[ep.destination]=&ep;
      else RCLCPP_ERROR(get_logger(),"Startup diagnostic %s: peer unavailable",ep.name.c_str());
    }
    uint32_t next_announce=uptime();
    while(rclcpp::ok() && static_cast<int32_t>(deadline-uptime())>0) {
      const auto now=uptime();
      if(static_cast<int32_t>(now-next_announce)>=0) {announce();next_announce=now+config_.announce_period_ms;}
      CanBridge::RxData_struct frame;
      for(uint32_t count=0;count<config_.rx_budget && socket_->receive_data(frame);++count) {
        const auto id=static_cast<uint32_t>(frame.canid);
        if(!canrpc_valid_source(id)) continue;
        const auto source=canrpc_source(id);const auto &data=frame.data;
        if(canrpc_class(id)==CANRPC_CONTROL && canrpc_destination(id)==CANRPC_BROADCAST && canrpc_service(id)==CANRPC_ANNOUNCE)
          canrpc_peer_announce(&peers_[source],data.data(),data.size(),now);
        else if(endpoints[source] && (canrpc_class(id)!=CANRPC_RESPONSE ||
            canrpc_padding_valid(data.data(),endpoints[source]->response_bytes,data.size())))
          canrpc_rpc_client_receive(&rpc_pairs_[source],&operations[source],&peer_rpc_config_[source],&peers_[source],
            id,data.data(),data.size(),now,config_.peer_timeout_ms);
      }
      bool pending=false;
      for(uint8_t node=1;node<127;++node) {
        if(!endpoints[node]) continue;
        auto &op=operations[node];
        const auto action=canrpc_rpc_client_next(&rpc_pairs_[node],&op,&peer_rpc_config_[node],&peers_[node],now,config_.peer_timeout_ms);
        if(action!=CANRPC_RPC_NONE) {
          const bool query=action==CANRPC_RPC_SEND_QUERY;
          if(!send(canrpc_id(config_.priority,query?CANRPC_CONTROL:CANRPC_REQUEST,node,
               query?CANRPC_QUERY_STATE:op.service,config_.local_node,op.tag),{}))
            canrpc_rpc_client_finish(&rpc_pairs_[node],&op,&peer_rpc_config_[node],CANRPC_RPC_UNKNOWN,CANRPC_RPC_TIMEOUT,now);
        }
        if(op.state==CANRPC_RPC_PENDING) {pending=true;continue;}
        std::ostringstream raw;raw<<std::hex<<std::setfill('0');
        for(size_t i=0;i<op.response_length;++i) raw<<std::setw(2)<<unsigned(op.response[i]);
        RCLCPP_ERROR(get_logger(),"Startup diagnostic endpoint=%s node=%u outcome=%u control=%u raw=%s",
          endpoints[node]->name.c_str(),node,op.state,op.control,raw.str().c_str());
        endpoints[node]=nullptr;
      }
      if(!pending) return;
      std::this_thread::sleep_for(std::chrono::milliseconds(config_.poll_period_ms));
    }
    for(uint8_t node=1;node<127;++node) if(endpoints[node]) {
      canrpc_rpc_client_finish(&rpc_pairs_[node],&operations[node],&peer_rpc_config_[node],CANRPC_RPC_UNKNOWN,CANRPC_RPC_TIMEOUT,uptime());
      RCLCPP_ERROR(get_logger(),"Startup diagnostic endpoint=%s timed out",endpoints[node]->name.c_str());
    }
  }
  void reset_peers() {
    if(config_.reset_targets.empty()) throw std::runtime_error("reset_on_startup requires reset.targets");
    const double override_timeout=get_parameter("startup_timeout_sec").as_double();
    if(!std::isfinite(override_timeout) || override_timeout<0 || override_timeout>=INT32_MAX/1000.0)
      throw std::runtime_error("Invalid startup_timeout_sec override");
    const double timeout=override_timeout>0?override_timeout:config_.startup_timeout_sec;
    const uint32_t deadline=uptime()+static_cast<uint32_t>(timeout*1000),period=config_.announce_period_ms;
    uint32_t next_announce=uptime()+period;bool requested=false;resets_={};startup_error_.clear();
    while(rclcpp::ok() && static_cast<int32_t>(deadline-uptime())>0) {
      receive(true);const uint32_t now=uptime();
      if(!startup_error_.empty()) throw std::runtime_error(startup_error_);
      if(static_cast<int32_t>(now-next_announce)>=0) {announce();next_announce=now+period;}
      if(!requested) {
        bool ready=true;
        for(auto node:config_.reset_targets) ready=ready && canrpc_peer_ready(&peers_[node],now,config_.peer_timeout_ms) &&
          (peers_[node].capabilities&CANRPC_CAP_SYSTEM_RESET)!=0;
        if(ready) {
          for(auto node:config_.reset_targets) {uint64_t nonce;random_bytes(reinterpret_cast<uint8_t*>(&nonce),sizeof(nonce));
            canrpc_reset_client_begin(&resets_[node],&peers_[node],boot_.data(),nonce,now,config_.request_window_ms,0);send_reset(node);}
          requested=true;RCLCPP_INFO(get_logger(),"Reset requested; waiting for all configured new READY generations");
        }
      } else {
        bool completed=true;
        for(auto node:config_.reset_targets) {auto &op=resets_[node];
          const bool done=canrpc_reset_client_complete(&op,&peers_[node],now,config_.peer_timeout_ms);completed=completed && done;
          if(canrpc_reset_client_should_retry(&op,now,config_.retry_interval_ms,config_.retry_window_ms)) send_reset(node);}
        if(completed) {RCLCPP_INFO(get_logger(),"All configured reset generations are READY");return;}
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(config_.poll_period_ms));
    }
    throw std::runtime_error("Startup initialization timed out or was interrupted; reset outcome unknown");
  }
  void stop() {
    poll_.reset();announce_.reset();
    for(uint8_t node=1;node<127;++node) if(calls_[node].header) {
      canrpc_rpc_client_finish(&rpc_pairs_[node],&calls_[node].operation,&peer_rpc_config_[node],CANRPC_RPC_UNKNOWN,CANRPC_RPC_TIMEOUT,uptime());finish_call(node);
    }
    call_service_.reset();routes_.clear();socket_.reset();
  }
public:
  CanRpcBridge():LifecycleNode("canrpc_bridge") {
    start_=std::chrono::steady_clock::now();
    rcl_interfaces::msg::ParameterDescriptor descriptor;descriptor.read_only=true;
    declare_parameter("config_file",std::string(),descriptor);
    declare_parameter("reset_on_startup",false,descriptor);
    declare_parameter("startup_timeout_sec",0.0,descriptor);
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
      RCLCPP_INFO(get_logger(),"Loaded project %s, local node %u, %zu routes",config_.project.c_str(),config_.local_node,config_.routes.size());return Return::SUCCESS;}
    catch(const std::exception &e) {RCLCPP_ERROR(get_logger(),"%s",e.what());return Return::FAILURE;}
  }
  Return on_activate(const rclcpp_lifecycle::State&) override {
    try {
      random_bytes(boot_.data(),boot_.size());bool nonzero=false;for(auto value:boot_) nonzero=nonzero || value!=0;
      if(!nonzero) throw std::runtime_error("Zero Boot ID");
      announce_sequence_=0;peers_={};
      socket_=std::make_unique<CanBridge>(config_.interface,config_.non_blocking?CanBridge::SocketMode::NonBlocking:CanBridge::SocketMode::Blocking,
                                        static_cast<int>(config_.tx_retry_timeout_ms),config_.brs);
      announce();if(get_parameter("reset_on_startup").as_bool()) {
        try {reset_peers();}
        catch(const std::exception &failure) {
          const std::string original=failure.what();
          try {startup_diagnostics();} catch(const std::exception &diagnostic) {
            RCLCPP_ERROR(get_logger(),"Startup diagnostic failed: %s",diagnostic.what());
          }
          throw std::runtime_error(original);
        }
      }
      create_routes();
      if(config_.rpc.enabled) call_service_=create_service<Call>(config_.rpc.ros_service,
        [this](const std::shared_ptr<rmw_request_id_t> header,const std::shared_ptr<Call::Request> request){handle_call(header,request);});
      announce();
      announce_=create_wall_timer(std::chrono::milliseconds(config_.announce_period_ms),[this]{announce();});
      poll_=create_wall_timer(std::chrono::milliseconds(config_.poll_period_ms),[this]{receive();advance_calls();});return Return::SUCCESS;
    } catch(const std::exception &e) {RCLCPP_ERROR(get_logger(),"%s",e.what());stop();return Return::FAILURE;}
  }
  Return on_deactivate(const rclcpp_lifecycle::State&) override {stop();return Return::SUCCESS;}
  Return on_cleanup(const rclcpp_lifecycle::State&) override {stop();return Return::SUCCESS;}
  Return on_shutdown(const rclcpp_lifecycle::State&) override {stop();return Return::SUCCESS;}
  Return on_error(const rclcpp_lifecycle::State&) override {stop();return Return::SUCCESS;}
};
int main(int argc,char **argv) {rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<CanRpcBridge>()->get_node_base_interface());rclcpp::shutdown();return 0;}
