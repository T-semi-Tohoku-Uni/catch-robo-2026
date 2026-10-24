#include "canrpc_bridge/config.hpp"
#include <yaml-cpp/yaml.h>
#include <cassert>
#include <fstream>
#include <functional>
#include <iostream>
#include <unistd.h>

int main(int argc,char **argv) {
  assert(argc==3);
  auto profile=YAML::LoadFile(argv[1]);
  const auto default_config=canrpc_bridge::load_config(argv[1]);
  const auto alternate=canrpc_bridge::load_config(argv[2]);
  assert(default_config.local_node==1 && default_config.routes.size()==2);
  assert(alternate.local_node==17 && alternate.reset_targets.size()==2 && alternate.routes.back().count==9);
  assert(default_config.nodes.at("arm").rpc.response_timeout_ms==100);
  assert(default_config.rpc_endpoints[1].response_bytes==48 && default_config.rpc_endpoints[1].startup_diagnostic);
  // Retain a generic TX fixture for route validation independent of this project's RX-only profile.
  auto routes=YAML::Node(YAML::NodeType::Sequence);
  routes.push_back(YAML::Load("{name: target, direction: tx, topic: target_joint_angles, subject: 512, type: float32, count: 3, priority: 4, finite: true}"));
  for(auto route:profile["routes"]) routes.push_back(route);
  profile["routes"]=routes;
  char path[]="/tmp/canrpc-config-XXXXXX";const int fd=mkstemp(path);assert(fd>=0);close(fd);
  size_t rejected=0;
  const auto bad=[&](const std::function<void(YAML::Node&)> &edit) {
    auto node=YAML::Clone(profile);edit(node);std::ofstream(path)<<node;
    bool failed=false;try {canrpc_bridge::load_config(path);}catch(const std::exception&){failed=true;}
    assert(failed);++rejected;
  };
  bad([](auto &n){n["schema_version"]=2;});
  bad([](auto &n){n["local_node"]=0;});
  bad([](auto &n){n["local_node"]=127;});
  bad([](auto &n){n["local_node"]="1";n["local_node"].SetTag("tag:yaml.org,2002:str");});
  bad([](auto &n){n["unexpected"]=true;});
  bad([](auto &n){n["bus"]["interface"]="can0;echo";});
  bad([](auto &n){n["bus"]["tx_retry_timeout_ms"]=0;});
  bad([](auto &n){n["bus"]["data_bitrate"]=1;});
  bad([](auto &n){n["timing"]["priority"]=8;});
  bad([](auto &n){n["timing"]["peer_timeout_ms"]=200;});
  bad([](auto &n){n["timing"]["poll_period_ms"]=201;});
  bad([](auto &n){n["reset"]["targets"].push_back(2);});
  bad([](auto &n){n["reset"]["targets"][0]=126;});
  bad([](auto &n){n["reset"]["retry_window_ms"]=1000;});
  bad([](auto &n){n["reset"]["execution_timeout_ms"]=1001;});
  bad([](auto &n){n["reset"]["timeout_sec"]=0.5;});
  bad([](auto &n){n["nodes"]["ros"]["reset_sources"].push_back(2);});
  bad([](auto &n){n["nodes"]["arm"]["reset_sources"].push_back(1);});
  bad([](auto &n){n["nodes"]["arm"]["id"]=1;});
  bad([](auto &n){n["routes"][0]["subject"]=4096;});
  bad([](auto &n){n["routes"][0]["type"]="double";});
  bad([](auto &n){n["routes"][0]["count"]=17;});
  bad([](auto &n){n["routes"][0]["source"]=1;});
  bad([](auto &n){n["routes"][0]["topic"]="invalid topic";});
  bad([](auto &n){n["routes"][1]["source"]=1;});
  bad([](auto &n){n["routes"][1]["count"]=2;n["routes"][1]["subject"]=512;});
  bad([](auto &n){auto route=YAML::Clone(n["routes"][0]);route["name"]="second";route["topic"]="second";n["routes"].push_back(route);});
  bad([](auto &n){auto route=YAML::Clone(n["routes"][1]);route["name"]="second";route["topic"]="second";n["routes"].push_back(route);});
  bad([](auto &n){n["routes"][2]["min"]=64;n["routes"][2]["max"]=63;});
  bad([](auto &n){n["routes"][2]["min"]=0.5;});
  bad([](auto &n){n["routes"][2]["max"]=2147483648.0;});
  bad([](auto &n){n["routes"][2]["finite"]=false;});
  bad([](auto &n){n.remove("rpc");});
  bad([](auto &n){n["rpc"]["cache_capacity"]=33;});
  bad([](auto &n){n["rpc"]["retry_interval_ms"]=301;});
  bad([](auto &n){n["rpc"]["query_interval_ms"]=1000;});
  bad([](auto &n){n["rpc"]["result_retention_ms"]=1000;});
  bad([](auto &n){n["rpc"]["tag_guard_ms"]=4000;});
  bad([](auto &n){n["rpc_endpoints"][0]["destination"]=1;});
  bad([](auto &n){n["rpc_endpoints"][0]["service"]=32;});
  bad([](auto &n){n["rpc_endpoints"][0]["request_bytes"]=65;});
  bad([](auto &n){auto ep=YAML::Clone(n["rpc_endpoints"][0]);ep["name"]="duplicate_address";n["rpc_endpoints"].push_back(ep);});
  bad([](auto &n){n["rpc_endpoints"][1]["request_bytes"]=1;});
  bad([](auto &n){n["rpc_endpoints"][1]["require_ready"]=true;});
  bad([](auto &n){n["nodes"]["arm"]["rpc"]["cache_capacity"]=33;});
  bad([](auto &n){n["nodes"]["arm"]["rpc"]["tag_guard_ms"]=239;});
  bad([](auto &n){auto ep=YAML::Clone(n["rpc_endpoints"][1]);ep["name"]="second_diagnostic";ep["service"]=2;n["rpc_endpoints"].push_back(ep);});
  auto publish_only=YAML::Clone(profile);publish_only.remove("rpc");publish_only.remove("rpc_endpoints");publish_only["nodes"]["arm"].remove("rpc");std::ofstream(path)<<publish_only;
  assert(!canrpc_bridge::load_config(path).rpc.enabled);
  // A subject can be received from multiple sources with one shared layout.
  auto valid=YAML::Clone(profile);auto route=YAML::Clone(valid["routes"][1]);route["name"]="other_source";
  route["topic"]="other_source";route["source"]=3;valid["routes"].push_back(route);std::ofstream(path)<<valid;
  assert(canrpc_bridge::load_config(path).routes.size()==4);
  unlink(path);std::cout<<"profiles accepted; "<<rejected<<" invalid configurations rejected\n";
}
