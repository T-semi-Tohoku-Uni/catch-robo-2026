#include "canrpc_core.h"
#include <assert.h>
#include <stdio.h>

int main(void) {
  uint8_t own[16]={3},remote[16]={7},data[64]={0},status;
  uint32_t id=canrpc_publish_id(0x456,42,31,2);
  assert(canrpc_source(id)==42 && canrpc_subject(id)==0x456 && canrpc_tag(id)==31 && id>>26==2);
  assert(canrpc_wire_length(9)==12 && canrpc_wire_length(65)==255);
  assert(canrpc_padding_valid(data,9,12));data[11]=1;assert(!canrpc_padding_valid(data,9,12));
  canrpc_peer_t peer={0};
  canrpc_boot_payload(data,remote,100,CANRPC_READY,CANRPC_CAP_SYSTEM_RESET);
  assert(canrpc_peer_announce(&peer,data,24,1000));
  assert(!canrpc_peer_announce(&peer,data,24,1100));
  assert(canrpc_peer_ready(&peer,1749,750) && !canrpc_peer_ready(&peer,1750,750));
  uint8_t allowed[1]={17};
  canrpc_reset_server_config_t config={1,allowed,1,750,2000,200};
  canrpc_reset_server_t server={0};canrpc_reset_client_t client={0};
  canrpc_reset_client_begin(&client,&peer,own,0x123456789abcdef0ULL,1100,1000,9);
  assert(canrpc_get_u64(client.request+32)==0x123456789abcdef0ULL);
  /* Server Boot ID is the target; requester peer has the client's Boot ID. */
  canrpc_peer_t requester={0};canrpc_boot_payload(data,own,200,CANRPC_READY,0);
  assert(canrpc_peer_announce(&requester,data,24,1100));
  assert(canrpc_reset_server_request(&server,&config,remote,&requester,CANRPC_READY,
    17,9,client.request,48,1100,&status));
  assert(status==CANRPC_RESET_PREPARING);
  assert(!canrpc_reset_server_request(&server,&config,remote,&requester,CANRPC_RESET_PENDING,
    17,9,client.request,48,1200,&status) && status==CANRPC_RESET_PREPARING);
  client.request[40]^=1;
  assert(!canrpc_reset_server_request(&server,&config,remote,&requester,CANRPC_RESET_PENDING,
    17,9,client.request,48,1200,&status) && status==CANRPC_RESET_CONFLICT);
  client.request[40]^=1;
  server.status=CANRPC_RESET_ACCEPTED;
  assert(canrpc_reset_server_can_execute(&server,1299,200));
  assert(!canrpc_reset_server_can_execute(&server,1300,200));
  canrpc_reset_status_payload(data,client.request,remote,1110,CANRPC_RESET_ACCEPTED);
  id=canrpc_id(3,CANRPC_CONTROL,17,CANRPC_RESET_STATUS,42,9);
  assert(canrpc_reset_client_status(&client,id,data,48,17,42,&peer)==1 && client.accepted);
  assert(!canrpc_reset_client_should_retry(&client,1400,200,900));
  client.accepted=0;assert(canrpc_reset_client_should_retry(&client,1300,200,900));
  canrpc_reset_client_sent(&client,1300);assert(!canrpc_reset_client_should_retry(&client,1301,200,900));
  remote[0]=8;canrpc_boot_payload(data,remote,1,CANRPC_STARTING,1);
  assert(canrpc_peer_announce(&peer,data,24,1400));
  assert(!canrpc_reset_client_complete(&client,&peer,1400,750) && client.changed);
  assert(!canrpc_reset_client_should_retry(&client,1500,200,900));
  canrpc_boot_payload(data,remote,2,CANRPC_READY,1);assert(canrpc_peer_announce(&peer,data,24,1401));
  assert(canrpc_reset_client_complete(&client,&peer,1401,750));
  remote[0]=7;canrpc_boot_payload(data,remote,101,CANRPC_READY,1);
  assert(!canrpc_peer_announce(&peer,data,24,1402));
  config.enabled=0;memset(&server,0,sizeof(server));
  assert(!canrpc_reset_server_request(&server,&config,remote,&requester,CANRPC_READY,
    17,9,client.request,48,1200,&status) && status==CANRPC_RESET_REJECT_AUTHORITY);
  puts("PASS: portable C99 codec, padding, peer generations, reset server/client");
  return 0;
}
