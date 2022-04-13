/**
  ********************************************************************************************************************************
  * @file    module_voltage doubler.h 
  * @author  Myron Mychal
  * @brief   This is a template non-driver app.
  * @details This module controls the behavior of the votlage doubling circuit in the Symax SRi
  ********************************************************************************************************************************
  */

/* Define to prevent recursive inclusion ---------------------------------------------------------------------------------------*/
#ifndef _MODULE_VOLTAGE_DOUBLER_H_
#define _MODULE_VOLTAGE_DOUBLER_H_

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "main.h"
#include "typedef.h"

#include "scheduler.h"
#include "sequential_memory.h"
#include "structured_memory.h"
#include "scheduler.h"
#include "ring_buffer.h"

/* Content ---------------------------------------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Setup -----------------------------------------------------------------------------------------------------------------------*/
static Ring_Buf_Handle doublerSeqMem0_u32;
static Ring_Buf_Handle doublerSeqMem1_u32;
static Ram_Buf_Handle doublerStructMem_u32;

/**
  ********************************************************************************************************************************
  * @brief   Module module_ecm_icl (inside shared memory)
  * @details 
  * @param   
  * @param   errorCode_u8       Error code of this app module.
  ********************************************************************************************************************************
  */
typedef struct {
   // Ring_Buf_Handle internalPipe_u32;
    bool isDoublerEngaged;
    uint8_t errorCode_u8;
} Module_VoltageDoublerControl;

/**
  ********************************************************************************************************************************
  * @brief   Module App control (inside shared memory)
  * @details 
  * @param   internalPipe_u32 
  * @param   errorCode_u8       Error code of this app module.
  ********************************************************************************************************************************
  */
//typedef struct {
 //   Ring_Buf_Handle internalPipe_u32;
 //   uint8_t errorCode_u8;
//} App_TemplateControl;

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _MODULE_APP_H_ */