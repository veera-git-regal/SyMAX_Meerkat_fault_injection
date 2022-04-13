/**
  ***************************************************************************************************
  * @file    module_ShortCmd.h 
  * @author  Regal Pamela Lee
  * @version V1.0
  * @date    1-Jul-2020
  * @brief   Header of c++ function/s for APP example0 (simple template test APP header)
  * @note    
  ***************************************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MODULE_SHORTCMD_H
#define __MODULE_SHORTCMD_H

/* Includes ------------------------------------------------------------------*/
//#include "stm32F0xx.h"
//#include "stm32f0_discovery.h"
//#include "core_cm0.h"
#include "main.h"
#include "typedef.h"

#include "scheduler.h"
#include "sequential_memory.h"
#include "structured_memory.h"

#ifdef __cplusplus
extern "C" {
#endif
 
#define UP_SHORT_CMD_ID_UTILITY 0xFB
  
//static Ram_Buf_Handle 
void Utility_ExecuteOperation(uint8_t function_id, uint8_t function_parameter, uint8_t function_subparameter); // TODO: Move to it's own file   

#ifdef __cplusplus
}
#endif

#endif /* __MODULE_SHORTCMD_H */

