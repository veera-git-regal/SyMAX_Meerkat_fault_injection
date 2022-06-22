/**
  *******************************************************************************************************************************************
  * @file    adc_check.h 
  * @author  Regal Kyle McBrady
  * @version V1.0
  * @date    30-Nov-2018
  * @brief   Header of safety core ADC check module
  * @note    The result will export to the Module data struct
  *******************************************************************************************************************************************
  */

#include <stdint.h>

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADCCHECK_H
    #define __ADCCHECK_H

    /* Includes ------------------------------------------------------------------*/
    #include "shared_ram.h"
    #include "shared_rom.h"

    // Execution Time: 2us 20200701 - This will increase, when more tests are implemented
    #define ADC_CHECK_CALL_RATE  (1) // How frequently a module is called (in milliseconds), 0 is invalid
    #define ADC_CHECK_PASS_REQUIREMENT (2) // How many times module must pass before pass is reported
    
    #define V_REF_HYSTERESIS_LIMIT (MeerkatConfig_ADCCheck_Hysteresis_VRef_u8)
    #define MUX_HYSTERESIS_LIMIT (MeerkatConfig_ADCCheck_Hysteresis_Mux_u8)
    #define SHUNT_CURRENT_HYSTERESIS_LIMIT (MeerkatConfig_ADCCheck_Hysteresis_Shunt_current_u8)
    #define LOCKED_ROTOR_HYSTERESIS_LIMIT (MeerkatConfig_ADCCheck_Hysteresis_Locked_Rotor_u8)
    #define COHERENCE_HYSTERESIS_LIMIT (MeerkatConfig_ADCCheck_Hysteresis_Coherence_u8)
    #define NO_LOAD_OVER_SPEED_HYSTERESIS_LIMIT (MeerkatConfig_ADCCheck_Hysteresis_Over_Speed_u8)

    #define HYSTERESIS_LIMIT (MeerkatConfig_ADCCheck_Hysteresis_u8)
    #define V_REF_LIMIT_MIN (MeerkatConfig_ADCCheck_MinVref_u16)
    #define V_REF_LIMIT_MAX (MeerkatConfig_ADCCheck_MaxVref_u16)
    #define V_REF_TORERANCE (MeerkatConfig_ADCCheck_MaxVref_u16-MeerkatConfig_ADCCheck_MinVref_u16)

    #define TEMPERATURE_LIMIT_MIN (MeerkatConfig_ADCCheck_MinTemp_u16)
    #define TEMPERATURE_LIMIT_MAX (MeerkatConfig_ADCCheck_MaxTemp_u16)

    #define CURRENT_TOLERANCE     (MeerkatConfig_ADCCheck_CurrentVarianceTolerance_u16) 

    #define MODULE_MAX_CURRENT (MeerkatConfig_ADCCheck_MaxRatedCurrent_i16)
    #define MIN_SPEED    (MeerkatConfig_ADCCheck_MinRatedSpeed_i16)
    #define CURRENT_SPEED_SLOPE (MeerkatConfig_ADCCheck_CurrentSpeedSlope)
    #define CURRENT_SPEED_B (MeerkatConfig_ADCCheck_SpeedCurrentYIntercept)
    

    #define MUX_TOLERANCE (MeerkatConfig_ADCCheck_MultiplexerVarianceRequired_u16)
    #define MIN_SPEED_COHERENCE_CHECK_RPM (MeerkatConfig_ADCCoherence_MinSpeedRpm)
    #define ERROR_FLAG_NEGATIVE_CURRENT 0x80
    // - Overcurrent 
    #define CURRENT_CHANNEL_U_INDEX    0
    #define CURRENT_CHANNEL_V_INDEX    1
    #define CURRENT_CHANNEL_W_INDEX    2
    #define NUMBER_OF_CURRENT_CHANNELS 3



enum 
{ 
  ADC_VREF_CHECK = 0,
  ADC_STAGNANCY_CHECK,
  ADC_CURRENT_CHECK,
  ADC_MUX_CHECK,
  ADC_SHUNT_CURRENT_CHECK,
  ADC_LOCK_ROTOR_CHECK,
  ADC_OVER_SPEED_CHECK,
  ADC_MAX_STARTUP_CURRENT_CHECK,
  ADC_MAX_CHECK //must always be last, add additional tests before 
};




typedef struct { //Private Static data for this module
    uint8_t stage_u8;
    uint8_t index_u8;
    uint8_t hysteresisDelay_u8[ADC_MAX_CHECK];
    uint8_t minCurrentChannelIndex_u8;
    uint8_t maxCurrentChannelIndex_u8;
    uint16_t presentADCReading_u16;
    uint16_t previousADCReading_u16;
    uint16_t previousTemperatureReading_u16;
    uint16_t previousCurrentCounterReading_u16;
    // Min/Max Currents for Each Phase
    int16_t maxCurrentPhaseU_i16;
    int16_t minCurrentPhaseU_i16;
    int16_t maxCurrentPhaseV_i16;
    int16_t minCurrentPhaseV_i16;
    int16_t maxCurrentPhaseW_i16;
    int16_t minCurrentPhaseW_i16; 
    int16_t maxCurrent_i16;
    int16_t minCurrent_i16;
} adcCheck_private_OTYP;



extern __root void MeerkatCore_ADCCheck_CoherenceCheck(void);

#endif /* __ADCCHECK_H */

