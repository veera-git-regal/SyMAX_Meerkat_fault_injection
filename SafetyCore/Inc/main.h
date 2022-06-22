/**
  ******************************************************************************
  * @file    main.h 
  * @author  Kyle McBrady
  * @version V1.0.0
  * @date    07-Dec-2018
  * @brief   Header of main.c
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

#define ENABLE_BUILD_AS_APPLICATION 0 // Enable the main loop because you are going to run this application on it's own

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_ll_rcc.h"
#include "stm32f3xx_ll_bus.h"
#include "stm32f3xx_ll_system.h"
#include "stm32f3xx_ll_exti.h"
#include "stm32f3xx_ll_cortex.h"
#include "stm32f3xx_ll_utils.h"
#include "stm32f3xx_ll_pwr.h"
#include "stm32f3xx_ll_dma.h"
#include "stm32f3xx_ll_gpio.h"
#include "safety_core_supervisor.h"
#include "shared_ram.h"

/* Private define ------------------------------------------------------------*/
#define MEERKAT_VERSION 00.02.04

#if ENABLE_BUILD_AS_APPLICATION > 0
    // For debugging purposes only.
    void IncSysTick(void);
    // LED D1 
    #define LED1_PIN                           LL_GPIO_PIN_3
    #define LED1_GPIO_PORT                     GPIOB
    #define LED1_GPIO_CLK_ENABLE()             LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB)
    // function prototypes
    void LED_Init(void);
    void LED_On(void);
    void LED_Off(void);
#endif // ENABLE_BUILD_AS_APPLICATION > 0

#endif /* __MAIN_H__ */

