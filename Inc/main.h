/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "stm32f3xx_ll_adc.h"
#include "stm32f3xx_ll_crc.h"
#include "stm32f3xx_ll_dac.h"
#include "stm32f3xx_ll_i2c.h"
#include "stm32f3xx_ll_rcc.h"
#include "stm32f3xx_ll_bus.h"
#include "stm32f3xx_ll_system.h"
#include "stm32f3xx_ll_exti.h"
#include "stm32f3xx_ll_cortex.h"
#include "stm32f3xx_ll_utils.h"
#include "stm32f3xx_ll_pwr.h"
#include "stm32f3xx_ll_dma.h"
#include "stm32f3xx_ll_tim.h"
#include "stm32f3xx_ll_usart.h"
#include "stm32f3xx.h"
#include "stm32f3xx_ll_gpio.h"
#include "scheduler.h" //SPA
#include "hardware_config.h"
#include <arm_itm.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* SOFTWARE VERSION
Software Versioning Scheme used consists of three revision numbers, 
represented as VV.XX.YY. Motor control and Safety Core have revision numbers 
stored individually. Which version number is incremented is associated with the 
risk of the update.
YY: Patch 
This version number is incremented when a small patch is applied that is 
unlikely to affect any other features.
XX: Minor Revision
This version number is incremented when a new feature is applied that has some 
risk of impacting the performance of other features.
VV: Major Revision
This version number is incremented when new features are integrated that 
significantly impact the 	performance of other features. 
*/
//#define DRIVE_FW_VERSION 000000

// Drive firmware version in VV.XX.YY format
#define DRIVE_FW_VERSION_MAJOR 0x3032  // VV // In ASCII
#define DRIVE_FW_VERSION_MEDIAN 0x3030 // XX // In ASCII
#define DRIVE_FW_VERSION_MINOR 0x3032  // YY // In ASCII

#if ((HARDWARE_VERSION == HARDWARE_VERSION_4p5KW) || (HARDWARE_VERSION == HARDWARE_VERSION_8KW))
#define M1_ICL_SHUT_OUT_Pin LL_GPIO_PIN_15
#define M1_ICL_SHUT_OUT_GPIO_Port GPIOC
#endif
#define M1_CURR_AMPL_U_Pin LL_GPIO_PIN_0
#define M1_CURR_AMPL_U_GPIO_Port GPIOA
#define M1_CURR_AMPL_V_Pin LL_GPIO_PIN_1
#define M1_CURR_AMPL_V_GPIO_Port GPIOA
#define DBG_DAC_CH1_Pin LL_GPIO_PIN_4
#define DBG_DAC_CH1_GPIO_Port GPIOA
#define M1_CURR_AMPL_W_Pin LL_GPIO_PIN_6
#define M1_CURR_AMPL_W_GPIO_Port GPIOA
#define M1_BUS_VOLTAGE_Pin LL_GPIO_PIN_0
#define M1_BUS_VOLTAGE_GPIO_Port GPIOB
#if ((HARDWARE_VERSION == HARDWARE_VERSION_4p5KW) || (HARDWARE_VERSION == HARDWARE_VERSION_8KW))
#define M1_TEMPERATURE_Pin LL_GPIO_PIN_1
#define M1_TEMPERATURE_GPIO_Port GPIOB
#define M1_OVP_Pin LL_GPIO_PIN_12
#define M1_OVP_GPIO_Port GPIOB
#endif
#define M1_PWM_UL_Pin LL_GPIO_PIN_13
#define M1_PWM_UL_GPIO_Port GPIOB
#define M1_PWM_VL_Pin LL_GPIO_PIN_14
#define M1_PWM_VL_GPIO_Port GPIOB
#define M1_PWM_WL_Pin LL_GPIO_PIN_15
#define M1_PWM_WL_GPIO_Port GPIOB
#define M1_PWM_UH_Pin LL_GPIO_PIN_8
#define M1_PWM_UH_GPIO_Port GPIOA
#define M1_PWM_VH_Pin LL_GPIO_PIN_9
#define M1_PWM_VH_GPIO_Port GPIOA
#define M1_PWM_WH_Pin LL_GPIO_PIN_10
#define M1_PWM_WH_GPIO_Port GPIOA

#if ( (HARDWARE_VERSION == HARDWARE_VERSION_1p3KW_REVE_AND_BELOW) || (HARDWARE_VERSION == HARDWARE_VERSION_1p3KW_REVE_AND_BELOW_EXT_CRYSTAL) )
#define M1_OCP_Pin LL_GPIO_PIN_12
#define M1_OCP_GPIO_Port GPIOB
#else
#define M1_OCP_Pin LL_GPIO_PIN_12
#define M1_OCP_GPIO_Port GPIOB
#endif

#if ((HARDWARE_VERSION == HARDWARE_VERSION_4p5KW) || (HARDWARE_VERSION == HARDWARE_VERSION_8KW))
#define LED_Debug_Pin LL_GPIO_PIN_15
#define LED_Debug_GPIO_Port GPIOA
#endif
#define UART_TX_Pin LL_GPIO_PIN_6
#define UART_TX_GPIO_Port GPIOB
#define UART_RX_Pin LL_GPIO_PIN_7
#define UART_RX_GPIO_Port GPIOB



/* USER CODE BEGIN Private defines */
void initStack(uint32_t startingAddr, uint32_t length);
extern volatile uint16_t I_a_Peak,I_b_Peak;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
