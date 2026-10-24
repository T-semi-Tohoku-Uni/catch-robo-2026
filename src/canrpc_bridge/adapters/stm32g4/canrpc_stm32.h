#ifndef CANRPC_STM32_H
#define CANRPC_STM32_H

#include "main.h"
#include "canrpc.h"

/* Board transport: FD with BRS disabled, native motor CAN stays unchanged. */
static uint8_t rpc_node, rpc_boot[16], rpc_boot_seq;
static volatile uint8_t rpc_state;
static uint32_t rpc_last_announce;
static volatile uint8_t rpc_announce_due;
static FDCAN_HandleTypeDef *rpc_can;
static canrpc_peer_t rpc_peer;
static canrpc_reset_server_t rpc_reset;
#define rpc_peer_boot rpc_peer.boot
#define rpc_peer_known rpc_peer.known
#define rpc_peer_state rpc_peer.state
#define rpc_peer_uptime rpc_peer.uptime_ms
#define rpc_peer_seen rpc_peer.seen_ms
#define rpc_reset_active rpc_reset.active
#define rpc_reset_status rpc_reset.status
#define rpc_reset_data rpc_reset.request
#define rpc_reset_source rpc_reset.source
#define rpc_reset_tag rpc_reset.tag
#define rpc_reset_started rpc_reset.started_ms
static volatile uint8_t rpc_tx_locked;
static uint32_t rpc_reset_tx_buffer;
static const uint8_t rpc_allowed_sources[] = CANRPC_PROJECT_RESET_SOURCES;
static const canrpc_reset_server_config_t rpc_reset_config = {
  CANRPC_PROJECT_RESET_ENABLED, rpc_allowed_sources, CANRPC_PROJECT_RESET_SOURCE_COUNT,
  CANRPC_PROJECT_PEER_TIMEOUT_MS, CANRPC_PROJECT_RESET_MAX_WINDOW_MS,
  CANRPC_PROJECT_RESET_EXECUTE_TIMEOUT_MS
};
static int (*rpc_prepare_reset)(void);
static uint8_t rpc_reset_prepared;

static canrpc_peer_t rpc_reset_peers[CANRPC_PROJECT_RESET_SOURCE_COUNT ? CANRPC_PROJECT_RESET_SOURCE_COUNT : 1];
static canrpc_peer_t rpc_unknown_peer;
static canrpc_peer_t *rpc_peer_for_source(uint8_t source) {
  if (source == CANRPC_NODE_ROS) return &rpc_peer;
  for (unsigned i = 0; i < CANRPC_PROJECT_RESET_SOURCE_COUNT; ++i)
    if (source == rpc_allowed_sources[i]) return &rpc_reset_peers[i];
  return &rpc_unknown_peer;
}

static canrpc_rpc_server_t rpc_server;
static const canrpc_rpc_config_t rpc_config = {
  CANRPC_PROJECT_RPC_RETRY_INTERVAL_MS,CANRPC_PROJECT_RPC_RETRY_WINDOW_MS,
  CANRPC_PROJECT_RPC_QUERY_INTERVAL_MS,CANRPC_PROJECT_RPC_RESPONSE_TIMEOUT_MS,
  CANRPC_PROJECT_RPC_RESULT_RETENTION_MS,CANRPC_PROJECT_RPC_TAG_GUARD_MS,
  CANRPC_PROJECT_RPC_MAX_FRAME_AGE_MS,CANRPC_PROJECT_RPC_CACHE_CAPACITY
};
typedef struct {uint8_t service,request_bytes,response_bytes,require_ready;} rpc_endpoint_t;
static const rpc_endpoint_t rpc_endpoints[]=CANRPC_PROJECT_RPC_ENDPOINTS;
static const rpc_endpoint_t *rpc_endpoint(uint8_t service) {
  for(unsigned i=0;i<CANRPC_PROJECT_RPC_ENDPOINT_COUNT;++i) if(rpc_endpoints[i].service==service) return &rpc_endpoints[i];
  return 0;
}
static int (*rpc_service_start)(uint8_t,const uint8_t *,uint8_t,uint8_t,uint32_t);
static int rpc_application_complete(uint8_t slot,uint32_t token,const uint8_t *reply,uint8_t length);

static int (*rpc_service_handler)(uint8_t,const uint8_t *,uint8_t,uint8_t *,uint8_t *);
static uint8_t rpc_dlc_length(uint32_t dlc) {
  static const uint8_t lengths[16]={0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
  return lengths[dlc & 15U];
}
static uint32_t rpc_length_dlc(uint8_t length) {
  static const uint8_t lengths[16]={0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
  for (unsigned i=0;i<16U;++i) if(lengths[i]==length) return i;
  return 0U;
}
static void rpc_application_poll(void);
static void rpc_application_receive(const FDCAN_RxHeaderTypeDef *,const uint8_t *);

static void rpc_reset_request(const FDCAN_RxHeaderTypeDef *header, const uint8_t *data);
static void rpc_reset_poll(void);

static inline int rpc_peer_ready(void) {
  return canrpc_peer_ready(&rpc_peer, HAL_GetTick(), CANRPC_PROJECT_PEER_TIMEOUT_MS);
}

static int rpc_entropy(void) {
  uint32_t start = HAL_GetTick();
  __HAL_RCC_HSI48_ENABLE();
  while (__HAL_RCC_GET_FLAG(RCC_FLAG_HSI48RDY) == RESET)
    if ((uint32_t)(HAL_GetTick() - start) > 100U) return 0;
  __HAL_RCC_RNG_CONFIG(RCC_RNGCLKSOURCE_HSI48);
  SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_RNGEN);
  (void)RCC->AHB2ENR;
  SET_BIT(RCC->AHB2RSTR, RCC_AHB2RSTR_RNGRST);
  CLEAR_BIT(RCC->AHB2RSTR, RCC_AHB2RSTR_RNGRST);
  RNG->CR = RNG_CR_RNGEN;
  uint32_t previous = 0U, nonzero = 0U;
  for (unsigned i = 0; i < 5U; ++i) {
    start = HAL_GetTick();
    while ((RNG->SR & RNG_SR_DRDY) == 0U) {
      if ((RNG->SR & (RNG_SR_CECS | RNG_SR_SECS)) != 0U ||
          (uint32_t)(HAL_GetTick() - start) > 100U) return 0;
    }
    if ((RNG->SR & (RNG_SR_CECS | RNG_SR_SECS)) != 0U) return 0;
    uint32_t word = RNG->DR;
    if (i != 0U) {
      if (word == previous) return 0;
      canrpc_put_u32(rpc_boot + (i - 1U) * 4U, word);
      nonzero |= word;
    }
    previous = word;
  }
  RNG->CR = 0U;
  return nonzero != 0U;
}

static int rpc_init(FDCAN_HandleTypeDef *can, uint8_t node) {
  rpc_can = can; rpc_node = node; rpc_state = 0U;
  rpc_announce_due = 1U;
  return rpc_entropy();
}

static int rpc_send(uint32_t id, uint32_t dlc, const uint8_t *data) {
  if (rpc_tx_locked) return 0;
  FDCAN_TxHeaderTypeDef header = {0};
  header.Identifier = id;
  header.IdType = FDCAN_EXTENDED_ID;
  header.TxFrameType = FDCAN_DATA_FRAME;
  header.DataLength = dlc;
  header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  header.BitRateSwitch = (CANRPC_PROJECT_BRS ? FDCAN_BRS_ON : FDCAN_BRS_OFF);
  header.FDFormat = FDCAN_FD_CAN;
  header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  uint32_t mask = __get_PRIMASK();
  __disable_irq();
  HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(rpc_can, &header, (uint8_t *)data);
  __set_PRIMASK(mask);
  return status == HAL_OK;
}

static void rpc_poll(void) {
  uint32_t now = HAL_GetTick();
  if (rpc_announce_due || (uint32_t)(now - rpc_last_announce) >= CANRPC_PROJECT_ANNOUNCE_PERIOD_MS) {
    uint8_t data[24];
    canrpc_boot_payload(data, rpc_boot, now, rpc_state,
                        CANRPC_PROJECT_RESET_ENABLED ? CANRPC_CAP_SYSTEM_RESET : 0U);
    if (rpc_send(canrpc_id(CANRPC_PROJECT_ANNOUNCE_PRIORITY, CANRPC_CONTROL, CANRPC_BROADCAST, CANRPC_ANNOUNCE,
                           rpc_node, rpc_boot_seq), FDCAN_DLC_BYTES_24, data)) {
      rpc_boot_seq = (rpc_boot_seq + 1U) & 31U;
      rpc_last_announce = now; rpc_announce_due = 0U;
    }
  }
}

static void rpc_ready(void) { rpc_state = CANRPC_READY; rpc_announce_due = 1U; }

static int rpc_valid(const FDCAN_RxHeaderTypeDef *header) {
  return header->IdType == FDCAN_EXTENDED_ID && header->FDFormat == FDCAN_FD_CAN &&
         header->RxFrameType == FDCAN_DATA_FRAME && header->Identifier <= 0x1fffffffU &&
         canrpc_valid_source(header->Identifier);
}

static void rpc_management(const FDCAN_RxHeaderTypeDef *header, const uint8_t *data) {
  uint32_t id = header->Identifier;
  if (!rpc_valid(header)) return;
  if (canrpc_class(id) == CANRPC_CONTROL && canrpc_service(id) == CANRPC_ANNOUNCE &&
      canrpc_destination(id) == CANRPC_BROADCAST &&
      header->DataLength == FDCAN_DLC_BYTES_24) {
    canrpc_peer_t *peer = rpc_peer_for_source(canrpc_source(id));
    if (peer != &rpc_unknown_peer) canrpc_peer_announce(peer, data, 24U, HAL_GetTick());
    return;
  }
  if (canrpc_destination(id) != rpc_node) return;
  uint8_t tag = canrpc_tag(id), opcode = canrpc_service(id);
  if (canrpc_class(id) == CANRPC_REQUEST) {
    rpc_application_receive(header,data);
  } else if (canrpc_class(id) == CANRPC_CONTROL) {
    if (opcode == CANRPC_QUERY_STATE && header->DataLength == FDCAN_DLC_BYTES_0) {
      rpc_application_receive(header,data);
    } else if (opcode == CANRPC_ANNOUNCE_QUERY && tag == 0U && header->DataLength == FDCAN_DLC_BYTES_0)
      rpc_announce_due = 1U;
    else if (opcode == CANRPC_RESET_REQUEST)
      rpc_reset_request(header, data);
  }
}

static int rpc_reset_reply(uint8_t source, uint8_t tag, const uint8_t *request, uint8_t status) {
  uint8_t reply[48];
  canrpc_reset_status_payload(reply, request, rpc_boot, HAL_GetTick(), status);
  return rpc_send(canrpc_id(CANRPC_PROJECT_PRIORITY, CANRPC_CONTROL, source, CANRPC_RESET_STATUS, rpc_node, tag),
                  FDCAN_DLC_BYTES_48, reply);
}

static void rpc_reset_request(const FDCAN_RxHeaderTypeDef *header, const uint8_t *data) {
  uint8_t source = canrpc_source(header->Identifier), tag = canrpc_tag(header->Identifier);
  if (header->DataLength != FDCAN_DLC_BYTES_48) return;
  uint8_t status;
  if (canrpc_reset_server_request(&rpc_reset, &rpc_reset_config, rpc_boot, rpc_peer_for_source(source),
      rpc_state, source, tag, data, 48U, HAL_GetTick(), &status)) {
    /* Gate application commands; each board prepares its actuators separately. */
    rpc_state = CANRPC_RESET_PENDING; rpc_announce_due = 1U;
  }
  rpc_reset_reply(source, tag, data, status);
}

/* Run only from main, after application command production has been gated. */
static void rpc_reset_poll_locked(void) {
  if (!rpc_reset_active || rpc_reset_status >= 0x10U) return;
  if (canrpc_reset_server_expired(&rpc_reset, HAL_GetTick(),
                                   CANRPC_PROJECT_RESET_EXECUTE_TIMEOUT_MS)) {
    /* A queued ACCEPTED without confirmed transmission never permits a reset. */
    rpc_tx_locked = 0U; rpc_reset_status =
      (rpc_prepare_reset && !rpc_reset_prepared) ? 0x16U : 0x1fU;
    rpc_state = CANRPC_FAULT; rpc_announce_due = 1U;
    rpc_reset_reply(rpc_reset_source, rpc_reset_tag, rpc_reset_data, rpc_reset_status);
    return;
  }
  if (rpc_reset_status == 0U) {
    if (!rpc_reset_prepared) {
      if (rpc_prepare_reset && !rpc_prepare_reset()) return;
      rpc_reset_prepared = 1U;
    }
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (rpc_reset_reply(rpc_reset_source, rpc_reset_tag, rpc_reset_data, 1U)) {
      rpc_reset_tx_buffer = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(rpc_can);
      rpc_tx_locked = 1U; rpc_reset_status = 1U;
    }
    __set_PRIMASK(mask);
  }
  if (canrpc_reset_server_can_execute(&rpc_reset, HAL_GetTick(),
                                      CANRPC_PROJECT_RESET_EXECUTE_TIMEOUT_MS) &&
      (rpc_can->Instance->TXBTO & rpc_reset_tx_buffer) != 0U) {
    NVIC_SystemReset();
    return;
  }

}
/* Serialize core state shared with the receive interrupt. */
static void rpc_reset_poll(void) {
  uint32_t mask = __get_PRIMASK();
  __disable_irq();
  rpc_reset_poll_locked();
  __set_PRIMASK(mask);
}


static void rpc_application_reply(uint8_t action,uint8_t slot,uint8_t control,uint8_t source,uint8_t tag) {
  uint8_t unused=0U;
  if(action==CANRPC_RPC_SERVER_RESPONSE) {
    canrpc_rpc_cache_entry_t *entry=&rpc_server.entries[slot];
    rpc_send(canrpc_id(CANRPC_PROJECT_PRIORITY,CANRPC_RESPONSE,source,entry->service,rpc_node,tag),
             rpc_length_dlc(entry->response_length),entry->response);
  } else {
    if(action==CANRPC_RPC_SERVER_NEW) control=CANRPC_ACCEPTED;
    if(action==CANRPC_RPC_SERVER_RUNNING) control=CANRPC_RUNNING;
    rpc_send(canrpc_id(CANRPC_PROJECT_PRIORITY,CANRPC_CONTROL,source,control,rpc_node,tag),
             FDCAN_DLC_BYTES_0,&unused);
  }
}
static void rpc_application_receive(const FDCAN_RxHeaderTypeDef *header,const uint8_t *data) {
  uint32_t id=header->Identifier;uint8_t source=canrpc_source(id),tag=canrpc_tag(id);
  uint8_t slot=0U,control=CANRPC_REJECT_SERVICE,action=CANRPC_RPC_SERVER_CONTROL;
  if(source!=CANRPC_NODE_ROS) control=CANRPC_REJECT_REQUEST;
  else if(canrpc_class(id)==CANRPC_CONTROL) {
    action=canrpc_rpc_server_query(&rpc_server,&rpc_config,rpc_peer_for_source(source),source,tag,
      HAL_GetTick(),CANRPC_PROJECT_PEER_TIMEOUT_MS,&slot,&control);
  } else {
    uint8_t length=rpc_dlc_length(header->DataLength);
    const rpc_endpoint_t *endpoint=rpc_endpoint(canrpc_service(id));
    int known=endpoint && rpc_service_handler;
    int shape=known && length==canrpc_wire_length(endpoint->request_bytes) &&
      canrpc_padding_valid(data,endpoint->request_bytes,length);
    action=canrpc_rpc_server_request(&rpc_server,&rpc_config,rpc_peer_for_source(source),source,
      canrpc_service(id),tag,data,length,HAL_GetTick(),CANRPC_PROJECT_PEER_TIMEOUT_MS,
      shape && rpc_state!=CANRPC_RESET_PENDING && (rpc_state==CANRPC_READY || !endpoint->require_ready),&slot,&control);
    if(action==CANRPC_RPC_SERVER_CONTROL && control==CANRPC_REJECT_NOT_READY && rpc_peer_ready()) {
      if(!known) control=CANRPC_REJECT_SERVICE;
      else if(!shape) control=CANRPC_REJECT_REQUEST;
    }
  }
  rpc_application_reply(action,slot,control,source,tag);
}
static void rpc_application_poll(void) {
  uint32_t mask=__get_PRIMASK();__disable_irq();
  for(unsigned i=0;i<CANRPC_PROJECT_RPC_CACHE_CAPACITY;++i) {
    canrpc_rpc_cache_entry_t *entry=&rpc_server.entries[i];
    if(entry->state==1U && rpc_service_handler &&
       (uint32_t)(HAL_GetTick()-entry->created_ms)<rpc_config.response_timeout_ms &&
       rpc_state!=CANRPC_RESET_PENDING) {
      canrpc_peer_t *peer=rpc_peer_for_source(entry->source);
      if(!canrpc_rpc_peer_valid(peer,HAL_GetTick(),CANRPC_PROJECT_PEER_TIMEOUT_MS,1) ||
         memcmp(peer->boot,entry->peer_boot,16)!=0) {entry->state=3U;continue;}
      uint8_t reply[64]={0},length=0;
      entry->state=4U;
      const rpc_endpoint_t *endpoint=rpc_endpoint(entry->service);
      if(rpc_service_handler(entry->service,entry->request,entry->request_length,reply,&length) &&
         endpoint && length==endpoint->response_bytes) {
        uint8_t wire=canrpc_wire_length(length);
        (void)rpc_application_complete((uint8_t)i,entry->token,reply,wire);
      } else if(!rpc_service_start || !rpc_service_start(entry->service,entry->request,entry->request_length,(uint8_t)i,entry->token)) {
        entry->state=3U;
      }
    }
  }
  __set_PRIMASK(mask);
}

static int rpc_application_complete(uint8_t slot,uint32_t token,const uint8_t *reply,uint8_t length) {
  uint32_t mask=__get_PRIMASK();__disable_irq();
  canrpc_rpc_cache_entry_t *entry=slot<CANRPC_RPC_CACHE_MAX ? &rpc_server.entries[slot]:0;
  const rpc_endpoint_t *endpoint=entry ? rpc_endpoint(entry->service):0;
  int completed=entry && endpoint && length==canrpc_wire_length(endpoint->response_bytes) &&
    canrpc_padding_valid(reply,endpoint->response_bytes,length) && entry->token==token &&
    canrpc_rpc_peer_valid(rpc_peer_for_source(entry->source),HAL_GetTick(),CANRPC_PROJECT_PEER_TIMEOUT_MS,1) &&
    memcmp(entry->peer_boot,rpc_peer_for_source(entry->source)->boot,16)==0 &&
    canrpc_rpc_server_complete_checked(&rpc_server,&rpc_config,slot,token,reply,length,HAL_GetTick());
  if(completed) rpc_application_reply(CANRPC_RPC_SERVER_RESPONSE,slot,0U,entry->source,entry->tag);
  __set_PRIMASK(mask);return completed;
}

#endif
