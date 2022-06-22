/**
  ********************************************************************************************************************************
  * @file    driver_usart2.h 
  * @author  Pamela Lee
  * @brief   Header of Driver function/s for serial protocol with Usart2 hardware
  * @details 
  ********************************************************************************************************************************
  */

// Define to prevent recursive inclusion ---------------------------------------
#ifndef _DRV_USART2_H_
#define _DRV_USART2_H_

// Includes --------------------------------------------------------------------
#include "typedef.h"

#include "sequential_memory.h"
#include "structured_memory.h"
#include "scheduler.h"

// Content ---------------------------------------------------------------------
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

// User parameters -------------------------------------------------------------
#define USART2_TX_RX_BUF_SIZE 60

#define BAUD_RATE_FACTOR  100
#define DEFAULT_BAUD_RATE 115200

// Setup -----------------------------------------------------------------------
static Ring_Buf_Handle usart2SeqMem_RawRx;
static Ring_Buf_Handle usart2SeqMemModbus;
static Ring_Buf_Handle usart2SeqMem_Tx;
static Ram_Buf_Handle usart2StructMem;


typedef struct {
    Ring_Buf_Handle seqMemModbus;
    Ring_Buf_Handle seqMemTX;
    Ring_Buf_Handle seqMem_RawRx;
    uint8_t errorCode_u8;
    uint8_t UsartMode_u8;
} Usart2_Control;

    

/* Public Function Declarations -------------------------------------------------------------------------------------------------------*/

/**
  * @brief Wrapper for USART2 Initialization Function
  * @param None
  * @retval None
  */
void Usart2_Init(void);

/**
  * @brief Periodic call to copy data to buffer for processing
  * @param None
  * @retval None
  */
void RxProcess_Usart2(void);

/**
  * @brief Periodic call to process outgoing data
  * @param None
  * @retval None
  */
void TxProcess_Usart2(void);

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 26.
  */
void USART2_IRQHandler(void);



#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _DRV_USART2_H_ */
