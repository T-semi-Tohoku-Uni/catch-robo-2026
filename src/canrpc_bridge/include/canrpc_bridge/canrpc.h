#ifndef CANRPC_H
#define CANRPC_H

#include <stdint.h>
#include <string.h>

#define CANRPC_NODE_ROS 1u
#define CANRPC_NODE_ARM 2u
#define CANRPC_NODE_PUMP 3u
#define CANRPC_NODE_PRESSURE 4u
#define CANRPC_BROADCAST 127u
#define CANRPC_SUBJECT_TARGET 0x200u
#define CANRPC_SUBJECT_CURRENT 0x210u
#define CANRPC_SUBJECT_PUMP 0x401u
#define CANRPC_SUBJECT_PRESSURE 0x406u
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
#define CANRPC_ANNOUNCE_PERIOD_MS 200u

static inline uint32_t canrpc_id(uint8_t priority, uint8_t cls, uint8_t destination,
                                uint8_t service, uint8_t source, uint8_t tag) {
  return ((uint32_t)(priority & 7u) << 26) | ((uint32_t)(cls & 3u) << 24) |
         ((uint32_t)(destination & 127u) << 17) | ((uint32_t)(service & 31u) << 12) |
         ((uint32_t)(source & 127u) << 5) | (tag & 31u);
}
static inline uint32_t canrpc_publish_id(uint16_t subject, uint8_t source, uint8_t seq) {
  return (4u << 26) | (CANRPC_PUBLISH << 24) | ((uint32_t)(subject & 4095u) << 12) |
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
                                       uint32_t uptime_ms, uint8_t state) {
  memcpy(p,boot_id,16); canrpc_put_u32(p+16,uptime_ms);
  p[20]=1; p[21]=0; p[22]=state; p[23]=0;
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
static inline int canrpc_peer_ready(const canrpc_peer_t *peer,uint32_t now_ms) {
  return peer->known && peer->state==CANRPC_READY && (uint32_t)(now_ms-peer->seen_ms)<750u;
}
#endif
