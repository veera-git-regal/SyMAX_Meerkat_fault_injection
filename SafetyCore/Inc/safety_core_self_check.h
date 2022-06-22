/**
  ********************************************************************************************************************************************
  * @file    safety_core_self_check.h 
  * @author  Myron Mychal
  * @version V1.0
  * @date    18-11-20
  * @brief   Independent self-test for all Meerkat test functions
  * @note    Module for testing Meerkat functionality independent of application; can run without hardware ties
  *******************************************************************************************************************************************
  */
#include <stdint.h>

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MEERKAT_SELFTEST_H_
    #define _MEERKAT_SELFTEST_H_

    /* Includes ------------------------------------------------------------------*/
    #include "shared_ram.h"

    #define MEERKAT_SELFTEST_ID        (30)   // Meerkat Selftest Module ID
    #define MEERKAT_SELFTEST_CALL_RATE (750)  // number of times module is called before a pass is determined
    #define FORCED_GOOD_CLOCK_COUNT    (400)  // ideal clock counter value
    #define FORCED_GOOD_AD_VALUE       (1000) // forced good AD value


/*** Public variable structure looks like this
typedef struct
{
  uint8_t	moduleID_u8;                                                       //moduleID defined above
  uint16_t	callRate_u16;
  uint32_t	faultCount_u32;               
  uint32_t	passCount_u32;
  uint32_t	errorCode_u32; 
}Module_Public_OTYP;
End of public variable structure ****/

// private variable structure
typedef struct {
    uint32_t index_u32;         // loop index
    uint8_t hysteresisCount_u8; // hysteresis count for module
    uint8_t stage_u8;           // next stage of module
} meerkatSelftest_private_OTYP;

// function prototypes
void Supervisor_Module_MeerkatSelftest(void);
void Supervisor_Module_PopulateSharedRAM(void);

#endif
