/**
  ********************************************************************************************************************************
  * @file    driver_rtc.c 
  * @author  Justin Moon
  * @brief   Main Driver function/s for Real Time Clock System
  * @details  Used by Meerkat Safetly Core to monitor LSI Tick Count
  ********************************************************************************************************************************
  */

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "main.h"
#include "driver_rtc.h"
// #include "main.h"

/* Content ---------------------------------------------------------------------------------------------------------------------*/
LL_RTC_InitTypeDef hrtc_init;


void RTC_Init(void) {
    // !review (START)
    LL_RCC_LSI_Enable();

    /* Wait till LSI is ready */
    while(LL_RCC_LSI_IsReady() != 1)
    {

    }
    LL_PWR_EnableBkUpAccess();
    LL_RCC_ForceBackupDomainReset();
    LL_RCC_ReleaseBackupDomainReset();
    LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSI);
    LL_RCC_EnableRTC();

    // REVIEW: Configure before EnableRTC?
    hrtc_init.HourFormat = LL_RTC_HOURFORMAT_24HOUR;
    hrtc_init.AsynchPrescaler = RTC_PRESCALAR;  
    hrtc_init.SynchPrescaler = RTC_SUBSECONDS_VALUE_MAX;
    LL_RTC_Init(RTC, &hrtc_init);
    // - Note: our lsiFreq is 40000, RTC_SUBSECONDS_VALUE_MAX yields an 0.8192s rollover period if prescalar is 0
    // -- REVIEW: this configuration is to best support Meerkat Safety Core as an LSI Tick Counter, but is not Real Time Accurate 
    // --- To make realtime clock accurate, make AsyncPreDiv 1 and set Synch Prediv to read to 19999 (0.5 seconds). 
    // ---- Rollover period is one second under this scenario.
    // ----- Rollover Time in seconds = ((SynchPreDiv+1)*(AsynchPrediv+1)/LSI_FREQUENCY_HZ);
}

#define LSI_TICK_MAX_VALUE 0x7FFF // Inclusive: lsi_value: counts 0x7FFF down to 0. at 40kHz
#define LSI_TICK_VALUES 0x8000

// Meerkat Clock Check: LSI Tick Counter
// - Use RTC data to update an 'LSI Tick' count.
// - Note: Must be called at least every rollover period
// -- or lsi ticks time will not be incremented correctly
uint32_t RTC_UpdateLsiTickCounter(void) {
    static uint32_t value_base_u32 = 0; // Incremented at rollover points to give us a u32 instead of a u11
    static uint32_t last_value_inv_u32 = 0;
    uint32_t lsi_value_u32 = LL_RTC_TIME_GetSubSecond(RTC); // read lsi ticks (rtc subseconds register: 0x7FFF down to 0)
    uint32_t value_inv_u32 = LSI_TICK_MAX_VALUE - lsi_value_u32; // 0 up to 0x7FFF 

    if (value_inv_u32 < last_value_inv_u32) { // Rollover Case
        // update value base
        value_base_u32 += LSI_TICK_VALUES;
    }

    last_value_inv_u32 = value_inv_u32; // store value for rollover detection
    return (value_base_u32 + value_inv_u32);
}



// REVIEW: We have a hybrid of LL and HAL in this project.
// - the RTC module in particular seems to be forced to include HAL due to the way systick is setup
// -- so we are using LL per project specification, yet we are having to include both HAL and LL files for RTC when compiling.

// ---- Partial HAL Initialization Code ----
//RTC_HandleTypeDef hrtc;
/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
// void MX_RTC_Init(void)
// {

//   /* USER CODE BEGIN RTC_Init 0 */

//   /* USER CODE END RTC_Init 0 */

//   /* USER CODE BEGIN RTC_Init 1 */

//   /* USER CODE END RTC_Init 1 */
//   /** Initialize RTC Only 
//   */

//   // Configure Realtime Clock as an LSI Tick Counter
//   hrtc.Instance = RTC;
//   hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
//   // hrtc.Init.AsynchPrediv = 99; 
//   // hrtc.Init.SynchPrediv = 399; // (LsiFreq/(AsynchPrediv+1) - 1)  Note: our lsiFreq is 40000
//   hrtc.Init.AsynchPrediv = RTC_PRESCALAR;  // REVIEW: this method doesn't actually map properly, but the math is easy because it can serve as a counter
//   hrtc.Init.SynchPrediv = RTC_SUBSECONDS_VALUE_MAX; // SSR Register Start Value (LsiFreq/(AsynchPrediv+1) - 1)  
//   // - Note: our lsiFreq is 40000, RTC_SUBSECONDS_VALUE_MAX yields an 0.8192s rollover period if prescalar is 0
//   // -- this configuration is to best support Meerkat Safety Core as an LSI Tick Counter, but is not Real Time Accurate 
//   // --- To make realtime clock accurate, make AsyncPreDiv 1 and set Synch Prediv to read to 19999 (0.5 seconds). 
//   // ---- Rollover period is one second under this scenario.
//   // ----- Rollover Time in seconds = ((SynchPreDiv+1)*(AsynchPrediv+1)/LSI_FREQUENCY_HZ);
  
//   // This project does not use an output for the RTC, disable.
//   hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
//   hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
//   hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
//   if (HAL_RTC_Init(&hrtc) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   /* USER CODE BEGIN RTC_Init 2 */

//   /* USER CODE END RTC_Init 2 */
// }
// // REVIEW: (END)
