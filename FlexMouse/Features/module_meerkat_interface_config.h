/**
  ********************************************************************************************************************************
  * @file    module_meerkat_interface_config.h 
  * @author  Justin Moon
  * @brief   Header of parameters used by the Meerkat Safety Core
  * @details    
  ********************************************************************************************************************************
  */

// ^** Tips: APPs/Drivers adding process example step6  [refer to user manual )

/* Define to prevent recursive inclusion ---------------------------------------------------------------------------------------*/
#ifndef _MODULE_MEERKAT_SAFETY_CONFIG_H_
#define _MODULE_MEERKAT_SAFETY_CONFIG_H_

/* Includes --------------------------------------------------------------------------------------------------------------------*/

/* Content ---------------------------------------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdint.h>
    
    
// Debugging Options
// - during production, all values should be 0.
#define DISABLE_MEERKAT_SAFETY_MODULE 0 // Debugging Option: totally disable, so flexmouse can run stand-alone
#define DISABLE_MEERKAT_SAFETY_MODULE_INIT 0
#define DISABLE_MEERKAT_AUTO_START 0
#define DISABLE_MEERKAT_AUTO_RESTART 0
#define DISABLE_MEERKAT_REBOOT 0 // Debugging Option: don't allow rebooting the processor so you have more time to view fail state
#define MEERKAT_ENABLE_FAULT_INJECTION_PROTOTOCOL 1 // Allow reception of incoming commands to inject UL Faults


// Constant Parameters
// - Clock Check Buffer
#define NO_CLOCK_SAMPLES 0xFFFFFFFFu

// - Clock Check: Ring Buffer
#define MEERKAT_CLOCK_TEST_SAMPLES_ADDRESS  0x20000500u // FIXED_RAM_ADDRESS_LSI_RING_BUFFER_FIRST
#define MEERKAT_CLOCK_TEST_RING_BUFFER_SIZE 4 // // number of u32's, so a value of 4 creates 16 bytes of shared ram
#define MEERKAT_CLOCK_TEST_SAMPLE_TIME_MS 100 // Time in milliseconds of a single clock sample for the clock test


// Local Variables
// - Test Status
extern int meerkatInterface_FailedTestId; // Local Copy of the Active Safety Error Code (-1 = no failed test, 0=adc_check, etc...)
extern int meerkatInterface_FailedTestCode;

// Safety Core Variables (these items are located in the Safety Core's Partitioned RAM Space)
// - ADC Check: Single Sample Buffers
extern uint16_t meerkatCore_Vref_u16; // FIXED_RAM_ADDRESS_VREF
extern uint16_t meerkatCore_TemperatureValue1_u16; // FIXED_RAM_ADDRESS_TEMPERATURE
extern uint16_t meerkatCore_TemperatureValue2_u16; // FIXED_RAM_ADDRESS_TEMPERATURE2
extern uint16_t meerkatCore_Vbus_u16; // FIXED_RAM_ADDRESS_VBUS

// - Clock Check: Single Sample Buffers
extern uint32_t meerkatCore_LSIClockBuffer_u32[MEERKAT_CLOCK_TEST_RING_BUFFER_SIZE];
extern uint32_t meerkatCore_SysTickMs_u32; // FIXED_RAM_ADDRESS_SYSTICK_MS

// - General Motor State
extern int32_t meerkatCore_SpeedRpm_s32; //  Speed RPM
extern int32_t meerkatCore_MeasuredTorque_s32; // Torque Readings
extern int32_t meerkatCore_ExpectedBEMF_s32; // BEMF Expectations for Locked Rotor Detection
extern int32_t meerkatCore_ObservedBEMF_s32; // BEMF Readings for Locked Rotor Detection

// - Over Current/ Trip Current Protection:
extern uint32_t meerkatCore_ShuntCurrentSampleCounter_u32; // Sample Counter used by Overcurrent and Phase Coherence Checks

// - Test Status
extern uint32_t meerkatCore_FaultCode_u32; // Active Safety Error Code
extern uint32_t meerkatCore_ActiveTest_u32; // Currently Running Test (to ensure that ISRs do not inject data mid-test)
extern uint32_t meerkatCore_PassedTests_u32; // Passed Test List
extern uint16_t meerkatCore_RiskAddrRetryCount_u16;
extern uint32_t meerkatCore_TestInterface_u32; //unused

// Function Imports (from symbols file)
void MeerkatCore_CheckWatchdogResetFlag(void);
void MeerkatCore_AddClockSample(uint32_t lsi_tick_count_u32);
void MeerkatCore_StartupInit(void);

// Fault Injection Addresses (for Debugging Safety Core Checks)
#define MEERKAT_FAULT_INJECT_RAM_ADDRESS    0x20000204u // Negative RAM - 'InitVal' (magic number to signify module was initialized)
#define MEERKAT_FAULT_INJECT_CLOCK_ADDRESS  meerkatCore_LSIClockBuffer_u32 // LSI Tick Buffer
#define MEERKAT_FAULT_INJECT_ROM_STAGE_ADDRESS  0x20000155
#define MEERKAT_FAULT_INJECT_ROM_STAGE_N_ADDRESS  0x20000355
#define MEERKAT_FAULT_INJECT_ROM_CRC_ADDRESS    0x20000144
#define MEERKAT_FAULT_INJECT_ROM_CRC_N_ADDRESS    0x20000344
#define MEERKAT_FAULT_INJECT_ADC_ADDRESS          0x200000E4
#define MEERKAT_FAULT_INJECT_REGISTER_ADDRESS     0x2000009C // faultCount address


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _MODULE_MEERKAT_SAFETY_CONFIG_H_ */
