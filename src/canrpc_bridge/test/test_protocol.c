#include "canrpc_core.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
int main(void) {
  uint8_t bytes[24]={0},boot[16]={1};
  for (unsigned subject=0;subject<4096;++subject) for(unsigned seq=0;seq<32;++seq) {
    uint32_t id=canrpc_publish_id(subject,2,seq,4);
    assert(canrpc_class(id)==CANRPC_PUBLISH && canrpc_source(id)==2);
    assert(canrpc_subject(id)==subject && canrpc_tag(id)==seq);
  }
  assert(!canrpc_valid_source(canrpc_publish_id(0x210,0,0,4)));
  assert(!canrpc_valid_source(canrpc_publish_id(0x210,127,0,4)));
  canrpc_put_float(bytes,1.0f);assert(bytes[0]==0 && bytes[1]==0 && bytes[2]==128 && bytes[3]==63);
  assert(canrpc_get_float(bytes)==1.0f);
  canrpc_put_float(bytes,NAN);assert(isnan(canrpc_get_float(bytes)));
  canrpc_put_u32(bytes,0xdeadbeef);assert(canrpc_get_u32(bytes)==0xdeadbeef);
  canrpc_boot_payload(bytes,boot,123,CANRPC_READY,0);
  assert(bytes[0]==1 && canrpc_get_u32(bytes+16)==123 && bytes[20]==1 && bytes[22]==CANRPC_READY && bytes[23]==0);
  canrpc_peer_t peer={0};
  assert(!canrpc_peer_ready(&peer,0,750));
  assert(canrpc_peer_announce(&peer,bytes,24,10));
  assert(canrpc_peer_ready(&peer,759,750));assert(!canrpc_peer_ready(&peer,760,750));
  assert(!canrpc_peer_announce(&peer,bytes,24,759));
  assert(!canrpc_peer_ready(&peer,760,750));
  canrpc_put_u32(bytes+16,124);assert(canrpc_peer_announce(&peer,bytes,24,800));
  assert(canrpc_peer_ready(&peer,800,750));
  bytes[0]=2;canrpc_put_u32(bytes+16,1);bytes[22]=CANRPC_STARTING;
  assert(canrpc_peer_announce(&peer,bytes,24,801));assert(!canrpc_peer_ready(&peer,801,750));
  bytes[0]=1;canrpc_put_u32(bytes+16,125);bytes[22]=CANRPC_READY;
  assert(!canrpc_peer_announce(&peer,bytes,24,802));
  bytes[0]=2;canrpc_put_u32(bytes+16,2);assert(canrpc_peer_announce(&peer,bytes,24,803));
  assert(canrpc_peer_ready(&peer,803,750));
  bytes[20]=2;assert(!canrpc_peer_announce(&peer,bytes,24,804));
  assert(!canrpc_peer_announce(&peer,bytes,23,804));
  uint8_t reset[48], requester[16]={9};
  canrpc_reset_payload(reset,boot,requester,UINT64_C(0x123456789abcdef0),1234);
  assert(memcmp(reset,boot,16)==0 && memcmp(reset+16,requester,16)==0);
  assert(reset[32]==0xf0 && reset[39]==0x12);
  assert(canrpc_get_u64(reset+32)==UINT64_C(0x123456789abcdef0));
  assert(canrpc_get_u32(reset+40)==1234 && reset[44]==0 && reset[47]==0);
  canrpc_boot_payload(bytes,boot,500,CANRPC_READY,0); bytes[23]=CANRPC_CAP_SYSTEM_RESET;
  canrpc_peer_t capable={0};assert(canrpc_peer_announce(&capable,bytes,24,1));
  assert(capable.capabilities==CANRPC_CAP_SYSTEM_RESET);
  bytes[23]=2;assert(!canrpc_peer_announce(&capable,bytes,24,2));
  puts("protocol C99: ID 131072 combinations, LE and Boot payload passed");
}
