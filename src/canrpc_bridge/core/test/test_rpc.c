#include "canrpc_core.h"
#include <assert.h>
#include <stdio.h>

int main(void) {
  canrpc_rpc_config_t config={100,300,100,1000,3000,5000,250,8};
  canrpc_peer_t server_peer={0},client_peer={0};uint8_t a[16]={1},b[16]={2},boot[24];
  canrpc_boot_payload(boot,a,1,CANRPC_READY,0);assert(canrpc_peer_announce(&client_peer,boot,24,100));
  canrpc_boot_payload(boot,b,1,CANRPC_READY,0);assert(canrpc_peer_announce(&server_peer,boot,24,100));
  canrpc_rpc_pair_t pair={0};canrpc_rpc_client_t client={0};canrpc_rpc_server_t server={0};
  uint8_t data[4]={0x21,0,0,0},reply[8]={0,0,0,0,0x21,0,0,0},slot=0,code=0,action;
  assert(canrpc_rpc_client_begin(&pair,&client,&config,a,&server_peer,1,3,0,data,4,100,750,1));
  assert(!canrpc_rpc_client_begin(&pair,&client,&config,a,&server_peer,1,3,1,data,4,100,750,1));
  assert(canrpc_rpc_client_next(&pair,&client,&config,&server_peer,100,750)==CANRPC_RPC_SEND_REQUEST);
  assert(canrpc_rpc_client_next(&pair,&client,&config,&server_peer,200,750)==CANRPC_RPC_SEND_REQUEST);
  action=canrpc_rpc_server_request(&server,&config,&client_peer,1,0,client.tag,data,4,200,750,1,&slot,&code);
  assert(action==CANRPC_RPC_SERVER_NEW);
  assert(canrpc_rpc_server_request(&server,&config,&client_peer,1,1,client.tag,data,4,201,750,0,&slot,&code)==CANRPC_RPC_SERVER_CONTROL && code==CANRPC_CONFLICT);
  assert(canrpc_rpc_client_receive(&pair,&client,&config,&server_peer,
    canrpc_id(4,CANRPC_CONTROL,1,CANRPC_ACCEPTED,3,client.tag),data,0,201,750));
  assert(canrpc_rpc_client_next(&pair,&client,&config,&server_peer,301,750)==CANRPC_RPC_SEND_QUERY);
  assert(canrpc_rpc_server_query(&server,&config,&client_peer,1,client.tag,301,750,&slot,&code)==CANRPC_RPC_SERVER_RUNNING);
  assert(canrpc_rpc_server_complete(&server,&config,slot,reply,8,302));
  /* Saved result remains retrievable when new execution is not allowed. */
  assert(canrpc_rpc_server_request(&server,&config,&client_peer,1,0,client.tag,data,4,303,750,0,&slot,&code)==CANRPC_RPC_SERVER_RESPONSE);
  assert(canrpc_rpc_client_receive(&pair,&client,&config,&server_peer,
    canrpc_id(4,CANRPC_RESPONSE,1,0,3,client.tag),reply,8,304,750));
  assert(client.state==CANRPC_RPC_SUCCESS && !pair.active && client.response[4]==0x21);
  uint8_t used=client.tag;pair.next_tag=used;
  assert(canrpc_rpc_client_begin(&pair,&client,&config,a,&server_peer,1,3,1,data,4,305,750,1) && client.tag!=used);
  server_peer.boot[0]=3;
  assert(canrpc_rpc_client_next(&pair,&client,&config,&server_peer,306,750)==CANRPC_RPC_NONE);
  assert(client.state==CANRPC_RPC_UNKNOWN && client.control==CANRPC_RPC_PEER_CHANGED);
  client_peer.seen_ms=3302;
  assert(canrpc_rpc_server_query(&server,&config,&client_peer,1,used,3302,750,&slot,&code)==CANRPC_RPC_SERVER_CONTROL && code==CANRPC_UNKNOWN_OUTCOME);
  client_peer.seen_ms=100;
  /* Full guarded cache refuses eviction, including on peer generation changes. */
  memset(&server,0,sizeof(server));config.cache_capacity=1;
  assert(canrpc_rpc_server_request(&server,&config,&client_peer,1,0,0,data,4,100,750,1,&slot,&code)==CANRPC_RPC_SERVER_NEW);
  assert(canrpc_rpc_server_request(&server,&config,&client_peer,1,0,1,data,4,101,750,1,&slot,&code)==CANRPC_RPC_SERVER_CONTROL && code==CANRPC_REJECT_BUSY);
  client_peer.boot[0]=4;
  assert(canrpc_rpc_server_request(&server,&config,&client_peer,1,0,0,data,4,102,750,1,&slot,&code)==CANRPC_RPC_SERVER_CONTROL && code==CANRPC_CONFLICT);
  client_peer.boot[0]=1;client_peer.seen_ms=1100;
  assert(!canrpc_rpc_server_complete(&server,&config,0,reply,8,1100));
  assert(canrpc_rpc_server_query(&server,&config,&client_peer,1,0,1100,750,&slot,&code)==CANRPC_RPC_SERVER_CONTROL && code==CANRPC_UNKNOWN_OUTCOME);
  client_peer.seen_ms=5100;
  assert(canrpc_rpc_server_request(&server,&config,&client_peer,1,0,0,data,4,5100,750,1,&slot,&code)==CANRPC_RPC_SERVER_NEW);
  uint32_t stale_token=server.entries[slot].token;
  server.entries[slot].state=4;
  assert(canrpc_rpc_server_complete_checked(&server,&config,slot,stale_token,reply,8,5101));
  client_peer.seen_ms=10100;
  assert(canrpc_rpc_server_request(&server,&config,&client_peer,1,0,0,data,4,10100,750,1,&slot,&code)==CANRPC_RPC_SERVER_NEW);
  assert(!canrpc_rpc_server_complete_checked(&server,&config,slot,stale_token,reply,8,10101));
  assert(server.entries[slot].state==1);
  /* Wrap-aware expiry, result retention, and startup-only readiness exemption. */
  memset(&pair,0,sizeof(pair));server_peer.seen_ms=0xfffffff0u;server_peer.state=CANRPC_STARTING;
  assert(!canrpc_rpc_client_begin(&pair,&client,&config,a,&server_peer,1,3,0,data,4,0xfffffff0u,750,1));
  assert(canrpc_rpc_client_begin(&pair,&client,&config,a,&server_peer,1,3,0,data,4,0xfffffff0u,750,0));
  server_peer.seen_ms=984u;
  assert(canrpc_rpc_client_next(&pair,&client,&config,&server_peer,984u,750)==CANRPC_RPC_NONE && client.state==CANRPC_RPC_UNKNOWN);
  assert(client.control==CANRPC_RPC_TIMEOUT);
  memset(&pair,0,sizeof(pair));server_peer.state=CANRPC_READY;server_peer.seen_ms=500;
  for(unsigned i=0;i<32;++i) {
    assert(canrpc_rpc_client_begin(&pair,&client,&config,a,&server_peer,1,3,(uint8_t)(i%2),data,4,500,750,1));
    canrpc_rpc_client_finish(&pair,&client,&config,CANRPC_RPC_SUCCESS,0,500);
  }
  assert(!canrpc_rpc_client_begin(&pair,&client,&config,a,&server_peer,1,3,0,data,4,500,750,1));
  server_peer.seen_ms=5500;
  assert(canrpc_rpc_client_begin(&pair,&client,&config,a,&server_peer,1,3,0,data,4,5500,750,1));
  puts("PASS: ordinary RPC identity/retry/query/cache/guard/epoch/deadline/wrap/readiness");
  return 0;
}
