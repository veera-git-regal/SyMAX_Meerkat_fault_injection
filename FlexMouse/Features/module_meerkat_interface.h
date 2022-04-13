/**
  ********************************************************************************************************************************
  * @file    module_meerkat_interface.h 
  * @author  Justin Moon
  * @brief   Header of c++ function/s for APP example0 (simple template test APP header)
  * @details    
  ********************************************************************************************************************************
  */

/* Define to prevent recursive inclusion ---------------------------------------------------------------------------------------*/
#ifndef _MODULE_MEERKAT_SAFETY_H_
#define _MODULE_MEERKAT_SAFETY_H_

/* Includes --------------------------------------------------------------------------------------------------------------------*/

/* Content ---------------------------------------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

enum {
    MEERKAT_TEST_ID_ADCCHECK,
    MEERKAT_TEST_ID_REGISTERCHECK,
    MEERKAT_TEST_ID_CLOCKCHECK,
    MEERKAT_TEST_ID_RAMCHECK,
    MEERKAT_TEST_ID_ROMCHECK,
    MEERKAT_TEST_ID_LASTMODULE //total number of module
};

#define MEERKAT_STATUS_TEST_ERROR_CODE_FLAGS 0x00FFFFFF
#define MEERKAT_STATUS_TEST_ID_SHIFT 24
#define MEERKAT_CLOCK_SAMPLE_PERIOD 100

// Library Contained Functions
uint64_t MeerkatCore_SupervisorRun(uint8_t motor_running);
void MeerkatCore_SupervisorInit(void);
void MeerkatCore_SupervisorInitSharedRam(void);
void MeerkatCore_AddShuntCurrentSample(int16_t phase_current_ia, int16_t phase_current_ib, int16_t phase_current_ic);

// User accessible functions
// - Init
//void MeerkatInterface_StartupInit(void);
// - Test Responses
void MeerkatInterface_RiskAddrStateSetCallback(void);
void MeerkatInterface_RiskAddrStateClearedCallback(void);
// - Data Input
void MeerkatInterface_AddShuntCurrentSample(int16_t phase_current_ia, int16_t phase_current_ib, int16_t phase_current_ic);
void MeerkatInterface_AddTemperature1Sample(uint16_t adc_value);
void MeerkatInterface_AddTemperature2Sample(uint16_t adc_value);
void MeerkatInterface_AddVrefSample(uint16_t adc_value);
void MeerkatInterface_AddMeasuredSpeedSample(int32_t speed_rpm);
void MeerkatInterface_AddBemfTorqueSamples(void);
void Meerkat_ClockCheck_AddClockReading(uint32_t systick_count, uint32_t lsitick_count);
//void MeerkatInterface_AddClockSample(uint32_t lsi_tick_count_u32);
// - Product Specific
void MeerkatInterface_RestartMotor(void); // ERM Gen0

// - SHOULD BE MOVED TO CORE
void MeerkatInterface_ClearRiskAddressedState(void);
void MeerkatInterface_RebootByWatchdogTimerReset(void);
void MeerkatInterface_StartupInit(void);

// typedef void (*Meerkat_SuperVisorInitPointer)(void);
//extern Meerkat_SuperVisorInitPointer MeerkatCore_SupervisorInit; // = ((Meerkat_SuperVisorInitPointer)(MEERKAT_SUPERVISOR_INIT_ROM_ADDRESS));

uint8_t MeerkatInterface_MotorIsRunning(void);
uint8_t MeerkatInterface_RunTests(void);
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _MODULE_MEERKAT_SAFETY_H_ */
