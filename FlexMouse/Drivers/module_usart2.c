/**
********************************************************************************************************************************
* @file    module_usart2.c 
* @author  Pamela Lee
* @brief   Main driver module for USART2 Communication.
* @details This module initializes the USART2 port and attaches the pre-selected fixed memory allocation to the module.
To Transmitt data in the RUN_MODULE case: put data into seqMemTX_u32, and call this function:
*             USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
********************************************************************************************************************************
*/

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "module_usart2.h"
#include "driver_usart2.h"
#include "module_modbus.h"

/* Content ---------------------------------------------------------------------------------------------------------------------*/
void assignModuleMem_USART2(void);

// -- Module States
enum {
  MEMORY_INIT_MODULE,
  INIT_MODULE,
  RUN_MODULE,
  // additional states to be added here as necessary.
  IRQ_MODULE = DEFAULT_IRQ_STATE,
  KILL_MODULE = KILL_APP
};

// - External Variables
extern Ram_Buf sharedMemArray[STRUCT_MEM_ARRAY_SIZE];
extern ProcessInfo processInfoTable[];
extern uint8_t usart2_CaptureLen;
extern uint8_t ModbusProtocolState;
extern __IO uint8_t indexTx_Usart2;

// - Global variables specific to this module
// -- Define Pointers that will be used as References to other Modules, where applicable
extern Usart2_Control usart2Control;
extern Ram_Buf_Handle usart2StructMem;
uint8_t usart2_RawRxBuf_u8[USART2_TX_RX_BUF_SIZE];
uint8_t usart2_ModbusRxBuf_u8[USART2_TX_RX_BUF_SIZE];
uint8_t usart2_TxBuf_u8[USART2_TX_RX_BUF_SIZE];

/**
********************************************************************************************************************************
* @brief   State machine for USART2 module
* @details
* @param   drv_identifier_u8, previous_state_u8, next_stat_u8, irq_identfier_u8
* @retval  return_state_u8
********************************************************************************************************************************
*/

uint8_t moduleUsart2(uint8_t drv_id_u8, uint8_t prev_state_u8, uint8_t next_state_u8, uint8_t irq_id_u8) {
  uint8_t return_state_u8 = MEMORY_INIT_MODULE;
  switch (next_state_u8) {
  case MEMORY_INIT_MODULE:
    {
      
      assignModuleMem_USART2(); // Assign structured memory
      return_state_u8 = INIT_MODULE;
      break;
    }
  case INIT_MODULE: {   
    Usart2_Init();
    return_state_u8 = RUN_MODULE;
    break;
  }
  case RUN_MODULE: { 
    // If items in buffer then transfer data to seqMem_ModbusR buffer for processing (incoming )
    if((RingBuf_GetUsedNumOfElements((usart2Control).seqMem_RawRx) >= 1 )) {
      RxProcess_Usart2();
    } 

    if((RingBuf_GetUsedNumOfElements((usart2Control).seqMemTX) >= MODBUS_MIN_TX_MESSAGE_LEN) && (LL_USART_IsActiveFlag_TXE(USART2))) {
      TxProcess_Usart2();
    }  
    return_state_u8 = RUN_MODULE;
    break;
  }
  case KILL_MODULE: {
    // The USART2 driver module must only be executed once.
    // Setting processStatus_u8 to PROCESS_STATUS_KILLED prevents the scheduler main loop from calling this module again.
    uint8_t table_index_u8 = getProcessInfoIndex(drv_id_u8);
    if (table_index_u8 != INDEX_NOT_FOUND) {
      processInfoTable[table_index_u8].Sched_DrvData.processStatus_u8 = PROCESS_STATUS_KILLED;
    }
    return_state_u8 = KILL_MODULE;
    break;
  }
  default: {
    return_state_u8 = KILL_MODULE;
    break;
  }
  }
  return return_state_u8;
}

/**
********************************************************************************************************************************
* @brief   Assign structured/sequential memory
* @details Assign structured/sequential memory for USART2 module
* @param   None 
* @return  None
********************************************************************************************************************************
*/
void assignModuleMem_USART2(){  
 
  usart2SeqMem_RawRx = SeqMem_CreateInstance(MODULE_USART2, USART2_TX_RX_BUF_SIZE, 
                                             ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);
  (*usart2SeqMem_RawRx).p_ringBuf_u8 = (uint8_t *)usart2_RawRxBuf_u8;
  
  usart2SeqMemModbus = SeqMem_CreateInstance(MODULE_USART2, USART2_TX_RX_BUF_SIZE, 
                                                ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);
  (*usart2SeqMemModbus).p_ringBuf_u8 = (uint8_t *)usart2_ModbusRxBuf_u8;
  
  usart2SeqMem_Tx = SeqMem_CreateInstance(MODULE_USART2, USART2_TX_RX_BUF_SIZE, 
                                          ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);
  (*usart2SeqMem_Tx).p_ringBuf_u8 = (uint8_t *)usart2_TxBuf_u8;
  
  usart2StructMem =  StructMem_CreateInstance(MODULE_USART2, sizeof(Usart2_Control), 
                                            ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);
  (*usart2StructMem).p_ramBuf_u8 = (uint8_t *)&usart2Control ;   
 
  uint8_t usart2_index_u8 = getProcessInfoIndex(MODULE_USART2);
  processInfoTable[usart2_index_u8].Sched_DrvData.irqState_u8 = DEFAULT_IRQ_STATE;
  processInfoTable[usart2_index_u8].Sched_DrvData.p_masterSharedMem_u32 = (Ram_Buf_Handle)usart2StructMem;
  
  
  /** assign all the new generated sequential-memory of USART2 to the structured-memory **/
  usart2Control.seqMemTX = usart2SeqMem_Tx;
  usart2Control.seqMemTX->is_OverwrittingAllowed_u8 = TRUE;
  usart2Control.seqMemModbus = usart2SeqMemModbus;
  usart2Control.seqMemModbus->is_OverwrittingAllowed_u8 = TRUE;
  usart2Control.seqMem_RawRx = usart2SeqMem_RawRx;
  usart2Control.seqMem_RawRx->is_OverwrittingAllowed_u8 = TRUE;
  usart2Control.errorCode_u8 = 0;
  usart2Control.UsartMode_u8 = 0;
}
