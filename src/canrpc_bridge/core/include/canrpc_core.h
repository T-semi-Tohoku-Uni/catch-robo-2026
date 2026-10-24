#ifndef CANRPC_CORE_H
#define CANRPC_CORE_H

#include <stdint.h>
#include <string.h>

#define CANRPC_BROADCAST 127u
#define CANRPC_CONTROL 0u
#define CANRPC_PUBLISH 1u
#define CANRPC_REQUEST 2u
#define CANRPC_RESPONSE 3u
#define CANRPC_REJECT_SERVICE 0x02u
#define CANRPC_ANNOUNCE 0x10u
#define CANRPC_ANNOUNCE_QUERY 0x11u
#define CANRPC_RESET_REQUEST 0x12u
#define CANRPC_RESET_STATUS 0x13u
#define CANRPC_RESET_PREPARING 0x00u
#define CANRPC_RESET_ACCEPTED 0x01u
#define CANRPC_RESET_REJECT_AUTHORITY 0x12u
#define CANRPC_CAP_SYSTEM_RESET 1u
#define CANRPC_STARTING 0u
#define CANRPC_READY 1u
#define CANRPC_DEGRADED 2u
#define CANRPC_RESET_PENDING 3u
#define CANRPC_FAULT 4u

static inline uint32_t canrpc_id(uint8_t priority, uint8_t cls, uint8_t destination,
                                uint8_t service, uint8_t source, uint8_t tag) {
  return ((uint32_t)(priority & 7u) << 26) | ((uint32_t)(cls & 3u) << 24) |
         ((uint32_t)(destination & 127u) << 17) | ((uint32_t)(service & 31u) << 12) |
         ((uint32_t)(source & 127u) << 5) | (tag & 31u);
}
static inline uint32_t canrpc_publish_id(uint16_t subject, uint8_t source, uint8_t seq, uint8_t priority) {
  return ((uint32_t)(priority & 7u) << 26) | (CANRPC_PUBLISH << 24) | ((uint32_t)(subject & 4095u) << 12) |
         ((uint32_t)(source & 127u) << 5) | (seq & 31u);
}
static inline uint8_t canrpc_class(uint32_t id) { return (uint8_t)((id >> 24) & 3u); }
static inline uint8_t canrpc_source(uint32_t id) { return (uint8_t)((id >> 5) & 127u); }
static inline uint8_t canrpc_destination(uint32_t id) { return (uint8_t)((id >> 17) & 127u); }
static inline uint8_t canrpc_service(uint32_t id) { return (uint8_t)((id >> 12) & 31u); }
static inline uint16_t canrpc_subject(uint32_t id) { return (uint16_t)((id >> 12) & 4095u); }
static inline uint8_t canrpc_tag(uint32_t id) { return (uint8_t)(id & 31u); }
static inline int canrpc_valid_source(uint32_t id) {
  const uint8_t s = canrpc_source(id); return s > 0u && s < CANRPC_BROADCAST;
}
static inline void canrpc_put_u32(uint8_t *p, uint32_t v) {
  p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); p[2]=(uint8_t)(v>>16); p[3]=(uint8_t)(v>>24);
}
static inline uint32_t canrpc_get_u32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1]<<8) | ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24);
}
static inline void canrpc_put_u64(uint8_t *p,uint64_t v) {
  canrpc_put_u32(p,(uint32_t)v); canrpc_put_u32(p+4,(uint32_t)(v>>32));
}
static inline uint64_t canrpc_get_u64(const uint8_t *p) {
  return (uint64_t)canrpc_get_u32(p) | ((uint64_t)canrpc_get_u32(p+4)<<32);
}
static inline void canrpc_reset_payload(uint8_t *p,const uint8_t *target_boot,
                                       const uint8_t *requester_boot,uint64_t nonce,
                                       uint32_t not_after) {
  memcpy(p,target_boot,16); memcpy(p+16,requester_boot,16);
  canrpc_put_u64(p+32,nonce); canrpc_put_u32(p+40,not_after);
  memset(p+44,0,4);
}
static inline void canrpc_put_float(uint8_t *p, float v) {
  uint32_t bits; memcpy(&bits,&v,4); canrpc_put_u32(p,bits);
}
static inline float canrpc_get_float(const uint8_t *p) {
  uint32_t bits=canrpc_get_u32(p); float v; memcpy(&v,&bits,4); return v;
}
static inline void canrpc_boot_payload(uint8_t *p, const uint8_t *boot_id,
                                       uint32_t uptime_ms, uint8_t state, uint8_t capabilities) {
  memcpy(p,boot_id,16); canrpc_put_u32(p+16,uptime_ms);
  p[20]=1; p[21]=0; p[22]=state; p[23]=capabilities;
}

/* Application peer gate; announcements never refresh sensor timestamps. */
typedef struct {
  uint8_t boot[16], retired[8][16];
  uint8_t known, state, retired_count, retired_next, capabilities;
  uint32_t uptime_ms, seen_ms;
} canrpc_peer_t;
static inline int canrpc_peer_announce(canrpc_peer_t *peer, const uint8_t *p,
                                       uint32_t length, uint32_t now_ms) {
  unsigned i; uint8_t nonzero=0;
  if (length!=24 || p[20]!=1 || p[21]!=0 || p[22]>CANRPC_FAULT || (p[23] & ~CANRPC_CAP_SYSTEM_RESET)!=0) return 0;
  for(i=0;i<16;++i) nonzero|=p[i];
  if(!nonzero) return 0;
  if(peer->known && memcmp(peer->boot,p,16)==0) {
    if((int32_t)(canrpc_get_u32(p+16)-peer->uptime_ms)<=0) return 0;
  } else {
    for(i=0;i<peer->retired_count;++i) if(memcmp(peer->retired[i],p,16)==0) return 0;
    if(peer->known) {
      memcpy(peer->retired[peer->retired_next],peer->boot,16);
      peer->retired_next=(uint8_t)((peer->retired_next+1u)%8u);
      if(peer->retired_count<8) ++peer->retired_count;
    }
    memcpy(peer->boot,p,16); peer->known=1;
  }
  peer->uptime_ms=canrpc_get_u32(p+16); peer->seen_ms=now_ms; peer->state=p[22]; peer->capabilities=p[23];
  return 1;
}
static inline int canrpc_peer_ready(const canrpc_peer_t *peer,uint32_t now_ms,uint32_t timeout_ms) {
  return peer->known && peer->state==CANRPC_READY && (uint32_t)(now_ms-peer->seen_ms)<timeout_ms;
}

#define CANRPC_RESET_REJECT_EPOCH 0x10u
#define CANRPC_RESET_REJECT_DEADLINE 0x11u
#define CANRPC_RESET_REJECT_MODE 0x13u
#define CANRPC_RESET_BUSY 0x14u
#define CANRPC_RESET_REJECT_REQUESTER 0x15u
#define CANRPC_RESET_SAFETY_FAILED 0x16u
#define CANRPC_RESET_BAD_FORMAT 0x17u
#define CANRPC_RESET_CONFLICT 0x18u
#define CANRPC_RESET_FAILED 0x1fu

/* The transport owns DLC encoding; the core uses byte lengths. */
static inline uint8_t canrpc_wire_length(uint8_t logical_length) {
  if (logical_length<=8u) return logical_length;
  if (logical_length<=12u) return 12u;
  if (logical_length<=16u) return 16u;
  if (logical_length<=20u) return 20u;
  if (logical_length<=24u) return 24u;
  if (logical_length<=32u) return 32u;
  if (logical_length<=48u) return 48u;
  if (logical_length<=64u) return 64u;
  return 0xffu;
}
static inline int canrpc_padding_valid(const uint8_t *data,uint8_t logical,uint8_t wire) {
  unsigned i;
  if (canrpc_wire_length(logical)!=wire) return 0;
  for (i=logical;i<wire;++i) if (data[i]!=0u) return 0;
  return 1;
}
static inline void canrpc_reset_status_payload(uint8_t *out,const uint8_t *request,
                                               const uint8_t *own_boot,uint32_t now,uint8_t status) {
  memcpy(out,request,16); memcpy(out+16,own_boot,16); memcpy(out+32,request+32,8);
  canrpc_put_u32(out+40,now);out[44]=status;out[45]=request[44];out[46]=out[47]=0;
}

typedef struct {
  uint8_t enabled;
  const uint8_t *allowed_sources;
  uint8_t allowed_source_count;
  uint32_t peer_timeout_ms,max_deadline_ms,execute_timeout_ms;
} canrpc_reset_server_config_t;
typedef struct {
  uint8_t request[48],source,tag,active,status;
  uint32_t started_ms;
} canrpc_reset_server_t;

/* Return 1 only for a newly accepted operation; duplicates never execute again. */
static inline int canrpc_reset_server_request(canrpc_reset_server_t *op,
    const canrpc_reset_server_config_t *config,const uint8_t *own_boot,
    const canrpc_peer_t *peer,uint8_t own_state,uint8_t source,uint8_t tag,
    const uint8_t *request,uint32_t length,uint32_t now,uint8_t *status) {
  unsigned i;int allowed=0;
  *status=CANRPC_RESET_BAD_FORMAT;
  if (length!=48u) return 0;
  for (i=0;i<config->allowed_source_count;++i)
    if (source==config->allowed_sources[i]) allowed=1;
  if (!config->enabled || !allowed) *status=CANRPC_RESET_REJECT_AUTHORITY;
  else if (memcmp(request,own_boot,16)!=0) *status=CANRPC_RESET_REJECT_EPOCH;
  else if (!peer->known || memcmp(request+16,peer->boot,16)!=0) *status=CANRPC_RESET_REJECT_REQUESTER;
  else if (op->active && source==op->source && memcmp(request+16,op->request+16,24)==0)
    *status=(tag==op->tag && memcmp(request,op->request,48)==0) ? op->status : CANRPC_RESET_CONFLICT;
  else if (op->active || own_state!=CANRPC_READY) *status=CANRPC_RESET_BUSY;
  else if (request[44]!=0u) *status=CANRPC_RESET_REJECT_MODE;
  else if (request[45]!=0u || request[46]!=0u || request[47]!=0u) *status=CANRPC_RESET_BAD_FORMAT;
  else if (!canrpc_peer_ready(peer,now,config->peer_timeout_ms)) *status=CANRPC_RESET_REJECT_REQUESTER;
  else {
    int32_t delta=(int32_t)(canrpc_get_u32(request+40)-now);
    if (delta<=0 || (uint32_t)delta>config->max_deadline_ms) *status=CANRPC_RESET_REJECT_DEADLINE;
    else {
      memcpy(op->request,request,48);op->source=source;op->tag=tag;
      op->status=CANRPC_RESET_PREPARING;op->started_ms=now;op->active=1;
      *status=CANRPC_RESET_PREPARING;return 1;
    }
  }
  return 0;
}
static inline int canrpc_reset_server_expired(const canrpc_reset_server_t *op,
                                             uint32_t now,uint32_t timeout_ms) {
  return op->active && (uint32_t)(now-op->started_ms)>=timeout_ms;
}
static inline int canrpc_reset_server_can_execute(const canrpc_reset_server_t *op,
                                                 uint32_t now,uint32_t timeout_ms) {
  return op->active && op->status==CANRPC_RESET_ACCEPTED &&
         !canrpc_reset_server_expired(op,now,timeout_ms);
}

typedef struct {
  uint8_t request[48],tag,active,accepted,changed;
  uint32_t created_ms,last_send_ms;
} canrpc_reset_client_t;
static inline void canrpc_reset_client_begin(canrpc_reset_client_t *op,
    const canrpc_peer_t *peer,const uint8_t *own_boot,uint64_t nonce,uint32_t now,
    uint32_t window_ms,uint8_t tag) {
  memset(op,0,sizeof(*op));op->active=1;op->tag=tag;op->created_ms=op->last_send_ms=now;
  canrpc_reset_payload(op->request,peer->boot,own_boot,nonce,
    peer->uptime_ms+(uint32_t)(now-peer->seen_ms)+window_ms);
}
static inline void canrpc_reset_client_observe(canrpc_reset_client_t *op,const canrpc_peer_t *peer) {
  if (op->active && peer->known && memcmp(peer->boot,op->request,16)!=0) op->changed=1;
}
/* 0 = unrelated/invalid, 1 = progress, -1 = final rejection in data[44]. */
static inline int canrpc_reset_client_status(canrpc_reset_client_t *op,uint32_t id,
    const uint8_t *data,uint32_t length,uint8_t local_node,uint8_t target_node,
    const canrpc_peer_t *peer) {
  if (!op->active || op->changed || length!=48u || canrpc_class(id)!=CANRPC_CONTROL ||
      canrpc_service(id)!=CANRPC_RESET_STATUS || canrpc_destination(id)!=local_node ||
      canrpc_source(id)!=target_node || canrpc_tag(id)!=op->tag ||
      (peer->known && memcmp(peer->boot,op->request,16)!=0) ||
      memcmp(data,op->request,16)!=0 || memcmp(data+16,op->request,16)!=0 ||
      memcmp(data+32,op->request+32,8)!=0 || data[45]!=0u || data[46]!=0u || data[47]!=0u) return 0;
  if (data[44]==CANRPC_RESET_ACCEPTED) op->accepted=1;
  else if (data[44]!=CANRPC_RESET_PREPARING) return -1;
  return 1;
}
static inline int canrpc_reset_client_complete(canrpc_reset_client_t *op,
                                               const canrpc_peer_t *peer,uint32_t now,uint32_t timeout_ms) {
  canrpc_reset_client_observe(op,peer);
  return op->changed && canrpc_peer_ready(peer,now,timeout_ms);
}
static inline int canrpc_reset_client_should_retry(const canrpc_reset_client_t *op,
                                                   uint32_t now,uint32_t interval_ms,uint32_t window_ms) {
  return op->active && !op->changed && !op->accepted &&
         (uint32_t)(now-op->created_ms)<window_ms &&
         (uint32_t)(now-op->last_send_ms)>=interval_ms;
}
static inline void canrpc_reset_client_sent(canrpc_reset_client_t *op,uint32_t now) {
  op->last_send_ms=now;
}


#define CANRPC_ACCEPTED 0x00u
#define CANRPC_REJECT_BUSY 0x01u
#define CANRPC_REJECT_REQUEST 0x03u
#define CANRPC_QUERY_STATE 0x04u
#define CANRPC_RUNNING 0x05u
#define CANRPC_UNKNOWN_OUTCOME 0x0bu
#define CANRPC_REJECT_NOT_READY 0x0du
#define CANRPC_CONFLICT 0x0eu
#define CANRPC_RPC_CACHE_MAX 32u
#define CANRPC_RPC_PENDING 0u
#define CANRPC_RPC_SUCCESS 1u
#define CANRPC_RPC_REJECTED 2u
#define CANRPC_RPC_UNKNOWN 3u
#define CANRPC_RPC_PEER_CHANGED 0x80u
#define CANRPC_RPC_PEER_EXPIRED 0x81u
#define CANRPC_RPC_TIMEOUT 0x82u
#define CANRPC_RPC_NONE 0u
#define CANRPC_RPC_SEND_REQUEST 1u
#define CANRPC_RPC_SEND_QUERY 2u
#define CANRPC_RPC_SERVER_NEW 1u
#define CANRPC_RPC_SERVER_RUNNING 2u
#define CANRPC_RPC_SERVER_RESPONSE 3u
#define CANRPC_RPC_SERVER_CONTROL 4u

typedef struct {
  uint32_t retry_interval_ms,retry_window_ms,query_interval_ms,response_timeout_ms;
  uint32_t result_retention_ms,tag_guard_ms,max_frame_age_ms;
  uint8_t cache_capacity;
} canrpc_rpc_config_t;
typedef struct {
  uint8_t next_tag,active,guarded[32];
  uint32_t guard_until[32];
} canrpc_rpc_pair_t;
typedef struct {
  uint8_t request[64],response[64],own_boot[16],peer_boot[16];
  uint8_t source,destination,service,tag,request_length,response_length;
  uint8_t state,control,accepted,sent,require_ready;
  uint32_t created_ms,last_send_ms;
} canrpc_rpc_client_t;
static inline int canrpc_rpc_peer_valid(const canrpc_peer_t *peer,uint32_t now,
                                        uint32_t timeout_ms,uint8_t require_ready) {
  return peer->known && (uint32_t)(now-peer->seen_ms)<timeout_ms &&
         (!require_ready || peer->state==CANRPC_READY);
}
static inline void canrpc_rpc_client_finish(canrpc_rpc_pair_t *pair,canrpc_rpc_client_t *op,
    const canrpc_rpc_config_t *config,uint8_t state,uint8_t control,uint32_t now) {
  op->state=state;op->control=control;pair->active=0;
  pair->guarded[op->tag]=1;pair->guard_until[op->tag]=now+config->tag_guard_ms;
}
static inline int canrpc_rpc_client_begin(canrpc_rpc_pair_t *pair,canrpc_rpc_client_t *op,
    const canrpc_rpc_config_t *config,const uint8_t *own_boot,const canrpc_peer_t *peer,
    uint8_t source,uint8_t destination,uint8_t service,const uint8_t *request,uint8_t length,
    uint32_t now,uint32_t peer_timeout_ms,uint8_t require_ready) {
  unsigned i;uint8_t tag=0;
  if(pair->active || source==0 || source>=127 || destination==0 || destination>=127 ||
     source==destination || service>31 || length>64 || canrpc_wire_length(length)!=length ||
     !canrpc_rpc_peer_valid(peer,now,peer_timeout_ms,require_ready)) return 0;
  for(i=0;i<32;++i) {
    tag=(uint8_t)((pair->next_tag+i)&31u);
    if(!pair->guarded[tag] || (int32_t)(now-pair->guard_until[tag])>=0) break;
  }
  if(i==32) return 0;
  memset(op,0,sizeof(*op));op->source=source;op->destination=destination;op->service=service;
  op->tag=tag;op->request_length=length;op->require_ready=require_ready;op->created_ms=now;
  if(length) memcpy(op->request,request,length);
  memcpy(op->own_boot,own_boot,16);memcpy(op->peer_boot,peer->boot,16);
  pair->active=1;pair->next_tag=(uint8_t)((tag+1u)&31u);
  pair->guarded[tag]=1;pair->guard_until[tag]=now+config->tag_guard_ms;
  return 1;
}
static inline uint8_t canrpc_rpc_client_next(canrpc_rpc_pair_t *pair,canrpc_rpc_client_t *op,
    const canrpc_rpc_config_t *config,const canrpc_peer_t *peer,uint32_t now,uint32_t peer_timeout_ms) {
  if(!pair->active || op->state!=CANRPC_RPC_PENDING) return CANRPC_RPC_NONE;
  if(!canrpc_rpc_peer_valid(peer,now,peer_timeout_ms,op->require_ready) ||
     memcmp(op->peer_boot,peer->boot,16)!=0 || (uint32_t)(now-op->created_ms)>=config->response_timeout_ms) {
    canrpc_rpc_client_finish(pair,op,config,CANRPC_RPC_UNKNOWN,
      memcmp(op->peer_boot,peer->boot,16)!=0 ? CANRPC_RPC_PEER_CHANGED :
      (!canrpc_rpc_peer_valid(peer,now,peer_timeout_ms,op->require_ready) ? CANRPC_RPC_PEER_EXPIRED : CANRPC_RPC_TIMEOUT),now);
    return CANRPC_RPC_NONE;
  }
  if(!op->sent) {op->sent=1;op->last_send_ms=now;return CANRPC_RPC_SEND_REQUEST;}
  if(op->accepted) {
    if((uint32_t)(now-op->last_send_ms)>=config->query_interval_ms) {
      op->last_send_ms=now;return CANRPC_RPC_SEND_QUERY;
    }
  } else if((uint32_t)(now-op->created_ms)<config->retry_window_ms &&
            (uint32_t)(now-op->last_send_ms)>=config->retry_interval_ms) {
    op->last_send_ms=now;return CANRPC_RPC_SEND_REQUEST;
  }
  return CANRPC_RPC_NONE;
}
static inline int canrpc_rpc_client_receive(canrpc_rpc_pair_t *pair,canrpc_rpc_client_t *op,
    const canrpc_rpc_config_t *config,const canrpc_peer_t *peer,uint32_t id,
    const uint8_t *data,uint8_t length,uint32_t now,uint32_t peer_timeout_ms) {
  uint8_t cls=canrpc_class(id),code=canrpc_service(id);
  if(!pair->active || op->state!=CANRPC_RPC_PENDING) return 0;
  if(!canrpc_rpc_peer_valid(peer,now,peer_timeout_ms,op->require_ready) ||
     memcmp(op->peer_boot,peer->boot,16)!=0 || (uint32_t)(now-op->created_ms)>=config->response_timeout_ms) {
    canrpc_rpc_client_finish(pair,op,config,CANRPC_RPC_UNKNOWN,
      memcmp(op->peer_boot,peer->boot,16)!=0 ? CANRPC_RPC_PEER_CHANGED :
      (!canrpc_rpc_peer_valid(peer,now,peer_timeout_ms,op->require_ready) ? CANRPC_RPC_PEER_EXPIRED : CANRPC_RPC_TIMEOUT),now);return 0;
  }
  if(canrpc_source(id)!=op->destination || canrpc_destination(id)!=op->source ||
     canrpc_tag(id)!=op->tag || length>64 || canrpc_wire_length(length)!=length) return 0;
  if(cls==CANRPC_RESPONSE && code==op->service) {
    if(length) memcpy(op->response,data,length);
    op->response_length=length;
    canrpc_rpc_client_finish(pair,op,config,CANRPC_RPC_SUCCESS,0,now);return 1;
  }
  if(cls!=CANRPC_CONTROL || length!=0) return 0;
  if(code==CANRPC_ACCEPTED || code==CANRPC_RUNNING) {
    op->accepted=1;op->last_send_ms=now;return 1;
  }
  if(code==CANRPC_UNKNOWN_OUTCOME) {
    canrpc_rpc_client_finish(pair,op,config,CANRPC_RPC_UNKNOWN,code,now);return 1;
  }
  if(code==CANRPC_REJECT_BUSY || code==CANRPC_REJECT_SERVICE || code==CANRPC_REJECT_REQUEST ||
     code==CANRPC_REJECT_NOT_READY || code==CANRPC_CONFLICT) {
    canrpc_rpc_client_finish(pair,op,config,CANRPC_RPC_REJECTED,code,now);return 1;
  }
  return 0;
}

typedef struct {
  uint8_t request[64],response[64],peer_boot[16];
  uint8_t source,service,tag,request_length,response_length,state;
  uint32_t created_ms,completed_ms,token;
} canrpc_rpc_cache_entry_t;
typedef struct {
  canrpc_rpc_cache_entry_t entries[CANRPC_RPC_CACHE_MAX];
  uint32_t next_token;
} canrpc_rpc_server_t;
static inline uint8_t canrpc_rpc_server_result(canrpc_rpc_cache_entry_t *entry,
    const canrpc_rpc_config_t *config,uint32_t now,uint8_t *control) {
  if(entry->state==2 && (uint32_t)(now-entry->completed_ms)<config->result_retention_ms)
    return CANRPC_RPC_SERVER_RESPONSE;
  if((entry->state==1 || entry->state==4) && (uint32_t)(now-entry->created_ms)<config->response_timeout_ms)
    return CANRPC_RPC_SERVER_RUNNING;
  *control=CANRPC_UNKNOWN_OUTCOME;return CANRPC_RPC_SERVER_CONTROL;
}
/* Lookup is source+Tag across all Services. Epoch changes never release guarded entries. */
static inline uint8_t canrpc_rpc_server_request(canrpc_rpc_server_t *server,
    const canrpc_rpc_config_t *config,const canrpc_peer_t *peer,uint8_t source,uint8_t service,uint8_t tag,
    const uint8_t *data,uint8_t length,uint32_t now,uint32_t peer_timeout_ms,uint8_t own_ready,
    uint8_t *slot,uint8_t *control) {
  unsigned i;int free_slot=-1;
  *control=CANRPC_REJECT_REQUEST;
  if(config->cache_capacity==0 || config->cache_capacity>CANRPC_RPC_CACHE_MAX ||
     length>64 || canrpc_wire_length(length)!=length || service>31 || tag>31 || source==0 || source>=127)
    return CANRPC_RPC_SERVER_CONTROL;
  if(!canrpc_rpc_peer_valid(peer,now,peer_timeout_ms,1)) {
    *control=CANRPC_REJECT_NOT_READY;return CANRPC_RPC_SERVER_CONTROL;
  }
  for(i=0;i<config->cache_capacity;++i) {
    canrpc_rpc_cache_entry_t *entry=&server->entries[i];
    if(entry->state==0 || (uint32_t)(now-entry->created_ms)>=config->tag_guard_ms) {
      if(free_slot<0) free_slot=(int)i;
    } else if(entry->source==source && entry->tag==tag) {
      *slot=(uint8_t)i;
      if(memcmp(entry->peer_boot,peer->boot,16)!=0 || entry->service!=service ||
         entry->request_length!=length || (length && memcmp(entry->request,data,length)!=0)) {
        *control=CANRPC_CONFLICT;return CANRPC_RPC_SERVER_CONTROL;
      }
      return canrpc_rpc_server_result(entry,config,now,control);
    }
  }
  if(!own_ready) {*control=CANRPC_REJECT_NOT_READY;return CANRPC_RPC_SERVER_CONTROL;}
  if(free_slot<0) {*control=CANRPC_REJECT_BUSY;return CANRPC_RPC_SERVER_CONTROL;}
  *slot=(uint8_t)free_slot;
  {
    canrpc_rpc_cache_entry_t *entry=&server->entries[*slot];memset(entry,0,sizeof(*entry));
    entry->source=source;entry->service=service;entry->tag=tag;entry->request_length=length;
    if(length) memcpy(entry->request,data,length);
    memcpy(entry->peer_boot,peer->boot,16);
    entry->created_ms=now;entry->state=1;
    if(++server->next_token==0) ++server->next_token;
    entry->token=server->next_token;
  }
  return CANRPC_RPC_SERVER_NEW;
}
static inline uint8_t canrpc_rpc_server_query(canrpc_rpc_server_t *server,
    const canrpc_rpc_config_t *config,const canrpc_peer_t *peer,uint8_t source,uint8_t tag,
    uint32_t now,uint32_t peer_timeout_ms,uint8_t *slot,uint8_t *control) {
  unsigned i;*control=CANRPC_UNKNOWN_OUTCOME;
  if(!canrpc_rpc_peer_valid(peer,now,peer_timeout_ms,1)) {
    *control=CANRPC_REJECT_NOT_READY;return CANRPC_RPC_SERVER_CONTROL;
  }
  for(i=0;i<config->cache_capacity && i<CANRPC_RPC_CACHE_MAX;++i) {
    canrpc_rpc_cache_entry_t *entry=&server->entries[i];
    if(entry->state && entry->source==source && entry->tag==tag &&
       (uint32_t)(now-entry->created_ms)<config->tag_guard_ms && memcmp(entry->peer_boot,peer->boot,16)==0) {
      *slot=(uint8_t)i;return canrpc_rpc_server_result(entry,config,now,control);
    }
  }
  return CANRPC_RPC_SERVER_CONTROL;
}
static inline int canrpc_rpc_server_complete(canrpc_rpc_server_t *server,const canrpc_rpc_config_t *config,uint8_t slot,
    const uint8_t *response,uint8_t length,uint32_t now) {
  canrpc_rpc_cache_entry_t *entry;
  if(slot>=CANRPC_RPC_CACHE_MAX || length>64 || canrpc_wire_length(length)!=length) return 0;
  entry=&server->entries[slot];
  if((entry->state!=1 && entry->state!=4) || (uint32_t)(now-entry->created_ms)>=config->response_timeout_ms) return 0;
  if(length) memcpy(entry->response,response,length);
  entry->response_length=length;
  entry->completed_ms=now;entry->state=2;return 1;
}
/* Deferred completion must carry the allocation token, never a slot index alone. */
static inline int canrpc_rpc_server_complete_checked(canrpc_rpc_server_t *server,
    const canrpc_rpc_config_t *config,uint8_t slot,uint32_t token,
    const uint8_t *response,uint8_t length,uint32_t now) {
  if(slot>=CANRPC_RPC_CACHE_MAX || server->entries[slot].token!=token) return 0;
  return canrpc_rpc_server_complete(server,config,slot,response,length,now);
}
#endif
