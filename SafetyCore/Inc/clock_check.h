/**
  ********************************************************************************************************************************************
  * @file    clock_check.h 
  * @author  Myron Mychal
  * @version V1.0
  * @date    03-09-2020
  * @brief   Check Processor Clock
  * @note    Header for periodically checking processor clock
  *******************************************************************************************************************************************
  */
#include <stdint.h>

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _CLOCK_CHECK_H_
    #define _CLOCK_CHECK_H_

    /* Includes ------------------------------------------------------------------*/
    #include "shared_ram.h"
    #include "shared_rom.h"

    // Common Test Parameters
    #define CLOCK_CHECK_CALL_RATE          (300) // How frequently a module is called (in milliseconds)
    #define CLOCK_CHECK_PASS_REQUIREMENT   (1) // How many times module must pass before pass is reported

    // Execution Time: 10us (20200701)
    #define TIME_TO_ACKNOWLEGE_A_PASS (10) // acceptable threshold of time between pasess expressed in 100s of milliseconds
    #define LSI_RING_BUFFER_SAMPLE_PERIOD_MS 100 // 100ms per sample
    // -  24 = 2.4 seconds

    #define CLOCK_CHECK_HYSTERESIS_LSITICK (MeerkatConfig_ClockCheck_HysteresisLsiTick_u8)
    #define CLOCK_CHECK_HYSTERESIS_SYSTICK (MeerkatConfig_ClockCheck_HysteresisSysTick_u8)

    // Clock Checker:
    // - Assumes SysTick 1ms interrupt in Application (FlexMouse), and that Application is storing LSI Tick Count
    // -- and accumulates them over 100 ms before storing them into the shared RAM ring buffer
    // --- For LSI frequency of 40kHz, LSI period = 1/40000 = 0.000025 sec; if HSI counts up to 100 ms; LSI should have 4000 counts
    #define IDEAL_LSI_CLOCK_COUNTS  (MeerkatConfig_ClockCheck_IdealLsiCounts_u32) // LSI Ticks in 100ms of samples
    #define CLOCK_THRESHOLD         (MeerkatConfig_ClockCheck_AcceptableVariance_f)
    #define CLOCK_THRESHOLD_MAX      (1 + (CLOCK_THRESHOLD))
    #define CLOCK_THRESHOLD_MIN      (1 - (CLOCK_THRESHOLD))
    #define MAX_LSI_CLOCK_COUNTS     ((CLOCK_THRESHOLD_MAX) * (IDEAL_LSI_CLOCK_COUNTS))
    #define MIN_LSI_CLOCK_COUNTS     ((CLOCK_THRESHOLD_MIN) * (IDEAL_LSI_CLOCK_COUNTS))
    #define NUM_LSI_COUNTS_IN_BUFFER (LENGTH_OF_LSI_RING_BUFFER)

enum 
{ 
  CLOCK_SYSTIC_CHECK = 1,
  CLOCK_LIS_CHECK = 2 
};

// variable structures for this module
typedef struct {
    uint8_t nextStage_u8;                                       // next stage of module
    uint8_t hysteresisCount_u8;                                 // hysteresis count for module
    uint8_t applicationHysteresisCount_u8;                      // hysteresis count for application clock
    uint8_t ring_buffer_position_u8;                             // pointer to head of ring buffer
    uint32_t indexN_u32;                                        // loop index N
    uint32_t previousClockCounts_u32;                           // previous value of shared clock counter
    uint32_t currentClockCounts_u32;                            // current value of shared clock counter
    uint32_t ringBufferLSIdelta_u32;                            // contains LSITick clock counts (each sample 100ms apart)
    uint32_t idealClockCounts_u32;                              // the number of ideal clock counts for a perfect LSI clock
    uint32_t lastClockReading_u32;                              // the last clock reading from previous cycle of clock checks
    uint32_t minClockCounts_u32;                                // the minimum number of clock counts withni the clock threshold
    uint32_t maxClockCounts_u32;                                // the maximum number of clock counts within the clock threshold
    uint32_t runningPassCount_u32;                              // a running accumulator of successful clock checks
    uint8_t stage_u8;                                           // the current stage of the clock count checker
} clockCheck_private_OTYP;


#endif