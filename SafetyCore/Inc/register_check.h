/**
  *******************************************************************************************************************************************
  * @file    register_check.h 
  * @author  Regal Satya
  * @version V1.0
  * @date    17-Nov-2021
  * @brief   Header of safety Register Check Module
  * @note    The result will export to the Module data struct
  *******************************************************************************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SAFETY_REGISTERCHECK_H
#define __SAFETY_REGISTERCHECK_H

/* Includes ------------------------------------------------------------------*/
#include "safety_core_supervisor.h"
#include "main.h"
#include "stm32F3xx.h"

#ifdef __cplusplus
extern "C" {
#endif

#define REGISTER_CHECK_CALL_RATE  (800) // How frequently a module is called (in milliseconds), 0 is invalid
// - This module has 6 test phases, so general pass time will be 6*REGISTER_CHECK_CALL_RATE (1s when CALL_RATE is 167ms) 
#define REGISTER_CHECK_PASS_REQUIREMENT   (1) // How many times module must pass before pass is reported
    
typedef struct { //Private Static data for this module
    uint8_t stage_u8;
} registerCheck_private_OTYP; //= {ModuleInit, 0, 0, ROM_START_ADDR,0};

void RegCheckFail(void);

#ifdef __cplusplus
}
#endif

#endif /* __SAFETY_REGISTERCHECK_H */
