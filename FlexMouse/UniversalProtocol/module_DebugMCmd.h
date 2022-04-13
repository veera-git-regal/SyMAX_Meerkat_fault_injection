/**
  ***************************************************************************************************
  * @file    module_DebugMCmd.h 
  * @author  Regal Pamela Lee
  * @version V1.0
  * @date    27-Aug-2021
  * @brief   Header of debug Mode for universal protocol  
  * @note    
  ***************************************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MODULE_DEBUGMCMD_H
#define __MODULE_DEBUGMCMD_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "typedef.h"

#include "scheduler.h"
#include "sequential_memory.h"
#include "structured_memory.h"

#ifdef __cplusplus
extern "C" {
#endif
 
//static Ram_Buf_Handle 
int32_t DebugMRegRd(uint8_t bCh_var, uint8_t *success, uint8_t *p_outBuff);

#ifdef __cplusplus
}
#endif

#endif /* __MODULE_DEBUGMCMD_H*/

