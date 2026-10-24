#pragma once
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace canrpc_bridge {
enum class ValueType { Float32, Int32, UInt8 };
struct Route {
  std::string name, topic;
  bool transmit=false, require_peer=true, finite=true, has_min=false, has_max=false;
  uint16_t subject=0;
  uint8_t source=0, priority=0;
  ValueType type=ValueType::Float32;
  size_t count=0;
  double minimum=0, maximum=0;
  size_t logical_size() const { return count*(type==ValueType::UInt8?1:4); }
};
struct RpcEndpoint {
  std::string name;
  uint8_t destination=0, service=0, request_bytes=0, response_bytes=0;
  bool require_ready=true, startup_diagnostic=false;
};
struct RpcConfig {
  bool enabled=false;
  std::string ros_service;
  uint32_t retry_interval_ms=0, retry_window_ms=0, query_interval_ms=0, response_timeout_ms=0;
  uint32_t result_retention_ms=0, tag_guard_ms=0, max_frame_age_ms=0, cache_capacity=0;
};
struct NodeConfig { uint8_t id=0, announce_priority=0; std::vector<uint8_t> reset_sources; RpcConfig rpc; };
struct Config {
  std::string project, interface;
  uint8_t local_node=0, priority=0, announce_priority=0;
  bool configure_bus=false, brs=false, non_blocking=true;
  uint32_t nominal_bitrate=0, data_bitrate=0, tx_retry_timeout_ms=0;
  uint32_t announce_period_ms=0, peer_timeout_ms=0, poll_period_ms=0, rx_budget=0;
  uint32_t request_window_ms=0, max_window_ms=0, retry_interval_ms=0, retry_window_ms=0, execution_timeout_ms=0;
  double startup_timeout_sec=0;
  std::vector<uint8_t> reset_targets;
  std::map<std::string,NodeConfig> nodes;
  std::vector<Route> routes;
  RpcConfig rpc;
  std::vector<RpcEndpoint> rpc_endpoints;
};
Config load_config(const std::string &path);
}
