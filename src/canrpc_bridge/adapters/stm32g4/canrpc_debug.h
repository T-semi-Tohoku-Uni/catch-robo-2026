#ifndef CANRPC_DEBUG_H
#define CANRPC_DEBUG_H

/* Main-context producer and bounded, nonblocking USART drain. */
#define DEBUG_CAPACITY 2048U
static UART_HandleTypeDef *debug_uart;
static char debug_buffer[DEBUG_CAPACITY];
static unsigned debug_head, debug_tail;
static uint32_t debug_dropped;
static volatile uint32_t debug_can_tx_errors;
static const char *debug_stage = "hal-init";

static int debug_write(const char *data, int length) {
  if (length <= 0) return 0;
  if (!debug_uart || __get_IPSR() || __get_PRIMASK()) {
    debug_dropped++;
    return length;
  }
  unsigned available = (debug_tail + DEBUG_CAPACITY - debug_head - 1U) % DEBUG_CAPACITY;
  if ((unsigned)length > available) {
    debug_dropped++;
    return length;
  }
  for (int i = 0; i < length; ++i) {
    debug_buffer[debug_head] = data[i];
    debug_head = (debug_head + 1U) % DEBUG_CAPACITY;
  }
  return length;
}

static void debug_drain(void) {
  if (!debug_uart || __get_IPSR()) return;
  for (unsigned count = 0; count < 32U && debug_tail != debug_head; ++count) {
    if ((debug_uart->Instance->ISR & USART_ISR_TXE_TXFNF) == 0U) break;
    debug_uart->Instance->TDR = (uint8_t)debug_buffer[debug_tail];
    debug_tail = (debug_tail + 1U) % DEBUG_CAPACITY;
  }
}
#endif
