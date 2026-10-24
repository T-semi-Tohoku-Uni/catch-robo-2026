#include "canrpc_bridge/config.hpp"
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <algorithm>
#include <cctype>
#include <limits>
#include <regex>
#include <set>
#include <stdexcept>

namespace canrpc_bridge {
namespace {
void require(bool ok,const std::string &why) { if(!ok) throw std::runtime_error("CAN-RPC config: "+why); }
void keys(const YAML::Node &node,std::initializer_list<const char*> names,const std::string &where) {
  require(node.IsMap(),where+" must be a map"); std::set<std::string> allowed,seen;
  for(auto name:names) allowed.insert(name);
  for(auto item:node) {const auto key=item.first.as<std::string>();
    require(allowed.count(key)!=0,where+": unknown key "+key);
    require(seen.insert(key).second,where+": duplicate key "+key);}
}
uint32_t number(const YAML::Node &node,const char *key,uint32_t low,uint32_t high) {
  require(node[key] && node[key].IsScalar(),std::string("missing integer ")+key);
  require(node[key].Tag()!="!" && node[key].Tag()!="tag:yaml.org,2002:str",std::string(key)+" must be an integer");
  const auto n=node[key].as<int64_t>(); require(n>=low && n<=high,std::string(key)+" is out of range");
  return static_cast<uint32_t>(n);
}
bool boolean(const YAML::Node &node,const char *key) {
  require(node[key] && node[key].IsScalar(),std::string("missing boolean ")+key);
  require(node[key].Tag()!="!" && node[key].Tag()!="tag:yaml.org,2002:str",std::string(key)+" must be boolean");
  auto value=node[key].Scalar();std::transform(value.begin(),value.end(),value.begin(),[](unsigned char ch){return static_cast<char>(std::tolower(ch));});
  require(value=="true" || value=="false" || value=="yes" || value=="no" || value=="on" || value=="off",std::string(key)+" must be boolean");
  return value=="true" || value=="yes" || value=="on";
}
std::string text(const YAML::Node &node,const char *key) {
  require(node[key] && node[key].IsScalar(),std::string("missing string ")+key);
  const auto result=node[key].as<std::string>(); require(std::any_of(result.begin(),result.end(),[](unsigned char ch){return !std::isspace(ch);}),std::string(key)+" must not be empty");
  if(node[key].Tag()=="?") {
    try {node[key].as<double>();throw std::runtime_error(std::string(key)+" must be a string");} catch(const YAML::BadConversion&) {}
    try {node[key].as<bool>();throw std::runtime_error(std::string(key)+" must be a string");} catch(const YAML::BadConversion&) {}
  }
  return result;
}
std::vector<uint8_t> ids(const YAML::Node &node,const std::string &where) {
  require(node.IsSequence(),where+" must be a list"); std::set<int> seen; std::vector<uint8_t> result;
  for(auto item:node) {require(item.IsScalar() && item.Tag()!="!" && item.Tag()!="tag:yaml.org,2002:str",where+": node must be integer");const int n=item.as<int>();require(n>=1 && n<=126 && seen.insert(n).second,where+": invalid/duplicate node");result.push_back(static_cast<uint8_t>(n));}
  return result;
}
RpcConfig rpc_policy(const YAML::Node &rpc) {
  RpcConfig result; result.enabled=true;
  result.retry_interval_ms=number(rpc,"retry_interval_ms",1,INT32_MAX);
  result.retry_window_ms=number(rpc,"retry_window_ms",1,INT32_MAX);
  result.query_interval_ms=number(rpc,"query_interval_ms",1,INT32_MAX);
  result.response_timeout_ms=number(rpc,"response_timeout_ms",1,INT32_MAX);
  result.result_retention_ms=number(rpc,"result_retention_ms",1,INT32_MAX);
  result.tag_guard_ms=number(rpc,"tag_guard_ms",1,INT32_MAX);
  result.max_frame_age_ms=number(rpc,"max_frame_age_ms",1,INT32_MAX);
  result.cache_capacity=number(rpc,"cache_capacity",1,32);
  require(result.retry_interval_ms<=result.retry_window_ms && result.retry_window_ms<result.response_timeout_ms,"invalid RPC retry window");
  require(result.query_interval_ms<result.response_timeout_ms,"RPC query interval must be shorter than response timeout");
  require(uint64_t(result.result_retention_ms)>=uint64_t(result.response_timeout_ms)+result.max_frame_age_ms,"RPC result retention is too short");
  require(uint64_t(result.tag_guard_ms)>=uint64_t(result.response_timeout_ms)+result.result_retention_ms+result.max_frame_age_ms,"RPC Tag guard is too short");
  return result;
}

}
Config load_config(const std::string &path) {
  require(!path.empty(),"config_file is required"); const auto root=YAML::LoadFile(path); Config c;
  keys(root,{"schema_version","project","local_node","bus","timing","reset","nodes","routes","rpc","rpc_endpoints","application"},"profile");
  require(number(root,"schema_version",1,1)==1,"unsupported schema"); c.project=text(root,"project");
  c.local_node=number(root,"local_node",1,126);
  const auto bus=root["bus"];
  keys(bus,{"interface","configure","nominal_bitrate","data_bitrate","brs","non_blocking","tx_retry_timeout_ms"},"bus");
  c.interface=text(bus,"interface"); require(c.interface.size()<16 && std::regex_match(c.interface,std::regex("[A-Za-z0-9_.-]+")),"invalid interface");
  c.configure_bus=boolean(bus,"configure"); c.brs=boolean(bus,"brs"); c.non_blocking=boolean(bus,"non_blocking");
  c.nominal_bitrate=number(bus,"nominal_bitrate",1,1000000);c.data_bitrate=number(bus,"data_bitrate",1,8000000);
  require(c.data_bitrate>=c.nominal_bitrate,"data_bitrate must be >= nominal_bitrate");
  c.tx_retry_timeout_ms=number(bus,"tx_retry_timeout_ms",1,1000);
  const auto t=root["timing"];
  keys(t,{"priority","announce_priority","announce_period_ms","peer_timeout_ms","poll_period_ms","rx_budget"},"timing");
  c.priority=number(t,"priority",0,7); c.announce_priority=number(t,"announce_priority",0,7);
  c.announce_period_ms=number(t,"announce_period_ms",1,INT32_MAX); c.peer_timeout_ms=number(t,"peer_timeout_ms",1,INT32_MAX);
  c.poll_period_ms=number(t,"poll_period_ms",1,INT32_MAX); c.rx_budget=number(t,"rx_budget",1,65536);
  require(c.peer_timeout_ms>c.announce_period_ms && c.poll_period_ms<=c.announce_period_ms,"poll <= announce < peer_timeout is required");
  const auto r=root["reset"];
  keys(r,{"targets","timeout_sec","request_window_ms","max_window_ms","retry_interval_ms","retry_window_ms","execution_timeout_ms"},"reset");
  c.reset_targets=ids(r["targets"],"reset.targets");
  require(r["timeout_sec"] && r["timeout_sec"].IsScalar() && r["timeout_sec"].Tag()!="!" && r["timeout_sec"].Tag()!="tag:yaml.org,2002:str","missing/nonnumeric timeout_sec");c.startup_timeout_sec=r["timeout_sec"].as<double>();
  require(std::isfinite(c.startup_timeout_sec) && c.startup_timeout_sec>0 && c.startup_timeout_sec<INT32_MAX/1000.0,"invalid timeout_sec");
  c.request_window_ms=number(r,"request_window_ms",1,INT32_MAX);c.max_window_ms=number(r,"max_window_ms",1,INT32_MAX);
  c.retry_interval_ms=number(r,"retry_interval_ms",1,INT32_MAX);c.retry_window_ms=number(r,"retry_window_ms",1,INT32_MAX);
  c.execution_timeout_ms=number(r,"execution_timeout_ms",1,INT32_MAX);
  require(c.request_window_ms<=c.max_window_ms,"request_window_ms exceeds max_window_ms");
  require(c.retry_interval_ms<=c.retry_window_ms && c.retry_window_ms<c.request_window_ms,"invalid reset retry interval/window");
  require(c.execution_timeout_ms<=c.request_window_ms && c.startup_timeout_sec*1000>c.request_window_ms,"execution_timeout <= request_window < startup timeout is required");
  require(root["nodes"].IsMap() && root["nodes"].size()>0,"nodes must be a nonempty map");std::set<uint8_t> declared;
  for(auto entry:root["nodes"]) {
    const auto name=entry.first.as<std::string>(); const auto n=entry.second; keys(n,{"id","reset_sources","announce_priority","rpc"},"nodes."+name);
    require(std::regex_match(name,std::regex("[A-Za-z_][A-Za-z0-9_]*")),"invalid node role");
    NodeConfig nc;nc.id=number(n,"id",1,126);nc.reset_sources=ids(n["reset_sources"],"reset_sources");
    nc.announce_priority=n["announce_priority"]?number(n,"announce_priority",0,7):c.announce_priority;
    if(n["rpc"]) {
      keys(n["rpc"],{"retry_interval_ms","retry_window_ms","query_interval_ms","response_timeout_ms","result_retention_ms","tag_guard_ms","max_frame_age_ms","cache_capacity"},"node rpc");
      nc.rpc=rpc_policy(n["rpc"]);
    }
    require(declared.insert(nc.id).second && c.nodes.emplace(name,nc).second,"duplicate node role/id");
  }
  require(declared.count(c.local_node)!=0,"local_node must be declared");
  for(const auto &entry:c.nodes) if(entry.second.id==c.local_node) c.announce_priority=entry.second.announce_priority;
  for(const auto &entry:c.nodes) require(entry.second.id!=c.local_node || entry.second.reset_sources.empty(),"ROS local_node cannot enable SYSTEM_RESET server");
  for(const auto &entry:c.nodes) for(auto source:entry.second.reset_sources)
    require(source!=entry.second.id && declared.count(source)!=0,"reset_sources must reference other declared nodes");
  for(auto target:c.reset_targets) require(target!=c.local_node && declared.count(target)!=0,"invalid reset target");
  require(root["routes"].IsSequence(),"routes must be a list");std::set<std::string> names,topics;std::set<uint16_t> tx;std::set<uint32_t> rx;
  std::map<uint16_t,std::pair<ValueType,size_t>> subjects;
  for(auto node:root["routes"]) {
    keys(node,{"name","direction","topic","subject","source","type","count","priority","require_peer","finite","min","max"},"route");
    Route route;route.name=text(node,"name");route.topic=text(node,"topic");const auto direction=text(node,"direction");
    require(direction=="tx" || direction=="rx","invalid route direction");route.transmit=direction=="tx";
    require(names.insert(route.name).second,"duplicate route name");
    require(std::regex_match(route.topic,std::regex("(~/|/)?[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*")),"invalid ROS topic name");
    require(topics.insert(direction+":"+route.topic).second,"duplicate topic in same direction");
    route.subject=number(node,"subject",0,4095);const auto type=text(node,"type");
    require(type=="float32" || type=="int32" || type=="uint8","unsupported route type");
    route.type=type=="float32"?ValueType::Float32:type=="int32"?ValueType::Int32:ValueType::UInt8;
    route.count=number(node,"count",1,route.type==ValueType::UInt8?64:16);
    route.priority=node["priority"]?number(node,"priority",0,7):c.priority;
    route.finite=node["finite"]?boolean(node,"finite"):true;
    require(!node["finite"] || route.type==ValueType::Float32,"finite applies only to float32");
    route.require_peer=node["require_peer"]?boolean(node,"require_peer"):!route.transmit;
    if(route.transmit) {require(!node["source"] && !route.require_peer,"TX source is local_node; peer gate is RX only");route.source=c.local_node;require(tx.insert(route.subject).second,"duplicate TX subject");}
    else {route.source=number(node,"source",1,126);require(route.source!=c.local_node && declared.count(route.source)!=0,"RX source must be a declared remote node");require(rx.insert((uint32_t(route.source)<<12)|route.subject).second,"ambiguous RX source/subject");}
    const auto signature=std::make_pair(route.type,route.count);const auto previous=subjects.find(route.subject);
    require(previous==subjects.end() || previous->second==signature,"Subject has conflicting type/count");subjects[route.subject]=signature;
    route.has_min=bool(node["min"]);route.has_max=bool(node["max"]);
    if(route.has_min) {require(node["min"].Tag()!="!" && node["min"].Tag()!="tag:yaml.org,2002:str","min must be numeric");route.minimum=node["min"].as<double>();}
    if(route.has_max) {require(node["max"].Tag()!="!" && node["max"].Tag()!="tag:yaml.org,2002:str","max must be numeric");route.maximum=node["max"].as<double>();}
    require((!route.has_min || std::isfinite(route.minimum)) && (!route.has_max || std::isfinite(route.maximum)),"bounds must be finite");
    require(!route.has_min || !route.has_max || route.minimum<=route.maximum,"min exceeds max");
    if(route.type!=ValueType::Float32) {const double lo=route.type==ValueType::UInt8?0:INT32_MIN,hi=route.type==ValueType::UInt8?255:INT32_MAX;
      require((!route.has_min || (route.minimum>=lo && route.minimum<=hi && std::floor(route.minimum)==route.minimum)) &&
              (!route.has_max || (route.maximum>=lo && route.maximum<=hi && std::floor(route.maximum)==route.maximum)),"integer bounds are out of range");}
    c.routes.push_back(route);
  }
  if(root["application"]) require(root["application"].IsMap(),"application must be a map");
  require(bool(root["rpc"])==bool(root["rpc_endpoints"]),"rpc and rpc_endpoints must be provided together");
  if(!root["rpc"]) {for(const auto &entry:c.nodes) require(!entry.second.rpc.enabled,"node RPC override requires global rpc");return c;}
  c.rpc.enabled=true;
  const auto rpc=root["rpc"];
  keys(rpc,{"ros_service","retry_interval_ms","retry_window_ms","query_interval_ms","response_timeout_ms","result_retention_ms","tag_guard_ms","max_frame_age_ms","cache_capacity"},"rpc");
  c.rpc.ros_service=text(rpc,"ros_service");
  require(std::regex_match(c.rpc.ros_service,std::regex("(~/|/)?[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*")),"invalid RPC ROS service name");
  {const auto service=c.rpc.ros_service;c.rpc=rpc_policy(rpc);c.rpc.ros_service=service;}
  require(root["rpc_endpoints"].IsSequence(),"rpc_endpoints must be a list");
  std::set<std::string> endpoint_names;std::set<uint32_t> endpoint_addresses;std::set<uint8_t> diagnostic_peers;
  for(auto node:root["rpc_endpoints"]) {
    keys(node,{"name","destination","service","request_bytes","response_bytes","require_ready","startup_diagnostic"},"RPC endpoint");
    RpcEndpoint ep;ep.name=text(node,"name");ep.destination=number(node,"destination",1,126);
    ep.service=number(node,"service",0,31);ep.request_bytes=number(node,"request_bytes",0,64);ep.response_bytes=number(node,"response_bytes",0,64);
    ep.require_ready=node["require_ready"]?boolean(node,"require_ready"):true;
    ep.startup_diagnostic=node["startup_diagnostic"]?boolean(node,"startup_diagnostic"):false;
    require(!ep.startup_diagnostic || (ep.request_bytes==0 && !ep.require_ready),"startup diagnostic requires empty request and require_ready false");
    require(!ep.startup_diagnostic || diagnostic_peers.insert(ep.destination).second,"only one startup diagnostic endpoint per peer is allowed");
    require(ep.destination!=c.local_node && declared.count(ep.destination)!=0,"RPC destination must be a declared remote node");
    require(endpoint_names.insert(ep.name).second && endpoint_addresses.insert((uint32_t(ep.destination)<<5)|ep.service).second,"duplicate RPC endpoint name/address");
    c.rpc_endpoints.push_back(ep);
  }
  return c;
}
}
