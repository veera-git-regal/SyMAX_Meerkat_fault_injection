/**
  ********************************************************************************************************************************
  * @file    driver_rtc.h 
  * @author  Justin Moon
  * @brief   Main Driver function/s for Real Time Clock System
  * @details  Used by Meerkat Safetly Core to monitor LSI Tick Count
  ********************************************************************************************************************************
  */

/* Define to prevent recursive inclusion ---------------------------------------------------------------------------------------*/
#ifndef _DRIVER_GPIO_H_
#define _DRIVER_GPIO_H_

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include <stdint.h>
#include "stm32f3xx_ll_rtc.h" 
#include "typedef.h"

// #include "scheduler.h"
// #include "sequential_memory.h"
// #include "structured_memory.h"

/* Content ---------------------------------------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* User parameters -------------------------------------------------------------------------------------------------------------*/
// #define VALUE_INDICATING_TRIGGERED_INTERRUPT 0x55
#define RTC_SUBSECONDS_VALUE_OUT_OF_RANGE 0x8000 // The SSR register returns a 15-bit value on stm32f301 and 302s
#define RTC_PRESCALAR 0
#define RTC_SUBSECONDS_VALUE_MAX 0x7FFF // The SSR register returns a 15-bit value on stm32f301 and 302s


// #define SIZE_OF_GPIO_SEQUENTIAL_MEMORY 32
// #define ACCESS_MODE

/* Setup -----------------------------------------------------------------------------------------------------------------------*/
// static Ring_Buffer_Handle gpioSequentialMemory_u32;
// static Ram_Buffer_Handle gpioStructuredMemory_u32;


/* Function Declarations -------------------------------------------------------------------------------------------------------*/
/**
  ********************************************************************************************************************************
  * @brief    
  * @details 
  ********************************************************************************************************************************
  */

void RTC_Init(void);
uint32_t RTC_UpdateLsiTickCounter(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _DRIVER_GPIO_H_ */