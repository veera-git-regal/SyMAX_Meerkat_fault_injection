/**
  ********************************************************************************************************************************
  * @file    Safety_safety_core_ram_structures.h 
  * @author  Regal Pamela Lee
  * @version V1.0
  * @date    17-Nov-2018
  * @brief   Header of safety core ram check module
  * @note    The result will export to the Module data struct
  ********************************************************************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SAFETY_RAMSTRUCT_H
#define __SAFETY_RAMSTRUCT_H

/* Includes ------------------------------------------------------------------*/
#include "stm32F3xx.h"
//refer to user manual
#include "adc_check.h"
#include "clock_check.h"
#include "ram_check.h"
#include "rom_check.h"
#include "register_check.h"
//include more module headers here
#include "safety_core_supervisor.h"

//
// RAM Memory Map for Safety Core, Shared and Bootware
//

// Safety Core RAM area
#define SAFETY_RAM_POSITIVE_RAM_INIT_VAL      (0x00)
#define SAFETY_RAM_NEGATIVE_RAM_INIT_VAL      (0xFF)
#define SAFETY_RAM_START_ADDR                 (0x20000000u)
#define SAFETY_RAM_END_ADDR                   (0x20000400u)
#define NRAM_OFFSET                           (0x0200) // offset between safety core PRAM and safety core NRAM
// - 0x000-0x200 = PRAM
#define SAFETY_RAM_MEERKAT_POSITIVE_RAM_START (0x20000000u)
#define SAFETY_RAM_MEERKAT_POSITIVE_RAM_END   (0x200001FFu)
// - 0x200-0x400 = NRAM
#define SAFETY_RAM_MEERKAT_NEGATIVE_RAM_START (0x20000200u)
#define SAFETY_RAM_MEERKAT_NEGATIVE_RAM_END   (0x200003FFu)
// - 0x400-0x500 = STACK (used for meerkat standalone only)
// - 0x500-0x700 = SHARED_RAM (512 Bytes)

////////////////////////////////////////////Private Module variable define ///////////////////////////////////////////////
/* each module should put their private structure in here for Safety ram management */

typedef struct {
    uint8_t moduleID_u8; //moduleID defined above
    uint16_t callRate_u16;
    uint32_t faultCount_u32;
    uint32_t passCount_u32;
    uint32_t errorCode_u32;
} Module_Public_OTYP;

typedef struct {
    Safety_Supervisor_Private_OTYP safety_supervisor_private;
    ModuleInfo_OTYP moduleInfo[LASTMODULE];
    //Refer to user manual
    //add more module public attribute structre here
    Module_Public_OTYP adcCheck_public;
    Module_Public_OTYP registerCheck_public;
    Module_Public_OTYP clockCheck_public;
    Module_Public_OTYP ramCheck_public;
    Module_Public_OTYP romCheck_public;
    //add more module public attribute structre here

    //Safety_RomCheck_Private safety_romCheck_private;
    //Refer to user manual
    //add more module private attribute structre here
    adcCheck_private_OTYP adcCheck_private;
    clockCheck_private_OTYP clockCheck_private;
    registerCheck_private_OTYP registerCheck_private;
    ramCheck_private_OTYP ramCheck_private;
    romCheck_private_OTYP romCheck_private;
    uint8_t motorState_u8; // 0 = IDLE, 1 = Running
    //uint8_t endStruct;
} Safety_RAMStruct_OTYP;

extern Safety_RAMStruct_OTYP Safety_RAMStruct_P;
extern Safety_RAMStruct_OTYP Safety_RAMStruct_N;

#endif /* __SAFETY_RAMSTRUCT_H */
