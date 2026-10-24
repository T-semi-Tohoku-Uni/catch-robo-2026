#ifndef CANRPC_BOARD_H
#define CANRPC_BOARD_H

#include "main.h"
#include "canrpc_project.h"
#include <stdint.h>

/* Read-only startup checks; generated GPIO initialization has already happened. */
static int canrpc_board_valid(const FDCAN_HandleTypeDef *can, int started) {
  if (!can || can->Instance != FDCAN1 ||
      __HAL_RCC_GET_FDCAN_SOURCE() != RCC_FDCANCLKSOURCE_PCLK1 ||
      HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN) != 80000000U ||
      HAL_RCC_GetSysClockFreq() != 80000000U || HAL_RCC_GetHCLKFreq() != 80000000U ||
      HAL_RCC_GetPCLK1Freq() != 80000000U ||
      FDCAN_CONFIG->CKDIV != FDCAN_CLOCK_DIV1) return 0;
  const FDCAN_InitTypeDef *init = &can->Init;
  if (init->ClockDivider != FDCAN_CLOCK_DIV1 || init->FrameFormat != FDCAN_FRAME_FD_BRS ||
      init->Mode != FDCAN_MODE_NORMAL || init->AutoRetransmission != DISABLE ||
      init->TransmitPause != DISABLE || init->ProtocolException != DISABLE ||
      init->NominalPrescaler != CANRPC_PROJECT_NOMINAL_PRESCALER ||
      init->NominalSyncJumpWidth != 1U || init->NominalTimeSeg1 != 15U || init->NominalTimeSeg2 != 4U ||
      init->DataPrescaler != CANRPC_PROJECT_DATA_PRESCALER || init->DataSyncJumpWidth != 1U ||
      init->DataTimeSeg1 != 15U || init->DataTimeSeg2 != 4U || init->StdFiltersNbr != 0U ||
      init->ExtFiltersNbr != 1U || init->TxFifoQueueMode != FDCAN_TX_FIFO_OPERATION) return 0;
  const uint32_t nominal = ((CANRPC_PROJECT_NOMINAL_PRESCALER-1U)<<16U) | (14U<<8U) | 3U;
  const uint32_t data = ((CANRPC_PROJECT_DATA_PRESCALER-1U)<<16U) | (14U<<8U) | (3U<<4U);
  if (can->Instance->NBTP != nominal || can->Instance->DBTP != data ||
      (can->Instance->CCCR & (FDCAN_CCCR_FDOE|FDCAN_CCCR_BRSE|FDCAN_CCCR_DAR|
                             FDCAN_CCCR_TEST|FDCAN_CCCR_MON|FDCAN_CCCR_ASM)) !=
      (FDCAN_CCCR_FDOE|FDCAN_CCCR_BRSE|FDCAN_CCCR_DAR)) return 0;
  if (((GPIOA->MODER>>22U)&15U) != 10U || ((GPIOA->AFR[1]>>12U)&255U) != 0x99U) return 0;
  if (!started) return 1;
  if ((can->Instance->CCCR & FDCAN_CCCR_INIT) ||
      can->Instance->RXGFC != ((1U<<24U)|(2U<<4U)|(2U<<2U)|3U) ||
      !(can->Instance->IE & FDCAN_IE_RF1NE) || !(can->Instance->ILE & FDCAN_ILE_EINT0) ||
      !can->msgRam.ExtendedFilterSA || !NVIC_GetEnableIRQ(FDCAN1_IT0_IRQn) ||
      !NVIC_GetEnableIRQ(FDCAN1_IT1_IRQn) || NVIC_GetPriority(FDCAN1_IT0_IRQn) != 0U ||
      NVIC_GetPriority(FDCAN1_IT1_IRQn) != 0U) return 0;
  const volatile uint32_t *filter = (const volatile uint32_t *)(uintptr_t)can->msgRam.ExtendedFilterSA;
  return filter[0] == ((uint32_t)FDCAN_FILTER_TO_RXFIFO1<<29U) &&
         filter[1] == ((uint32_t)FDCAN_FILTER_MASK<<30U);
}

#endif
