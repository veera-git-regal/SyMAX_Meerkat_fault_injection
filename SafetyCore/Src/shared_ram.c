/**
  *******************************************************************************************************************************************
  * @file    shared_ram.h 
  * @author  Doug Oda
  * @version V2.0
  * @date    5/20/21
  * @brief   Header containing definitions of variables listed in the .symbols file to be shared with the application
  *******************************************************************************************************************************************
  */

#include <stdint.h>
#include "shared_ram.h"

// - Clock Check: Ring Buffer 
__root uint32_t meerkatCore_LSIClockBuffer_u32[LENGTH_OF_LSI_RING_BUFFER];

// - ADC Check: Single Sample Buffers
__root uint16_t meerkatCore_Vref_u16;
__root uint16_t meerkatCore_TemperatureValue1_u16;
__root uint16_t meerkatCore_Vbus_u16;

// - Clock Check: Single Sample Buffers
__root uint32_t meerkatCore_SysTickMs_u32;

// - General Motor State: Speed RPM
__root int32_t meerkatCore_SpeedRpm_s32;

// - Locked Rotor Detection
__root int32_t meerkatCore_ExpectedBEMF_s32;
__root int32_t meerkatCore_ObservedBEMF_s32;

// - No Load Detection
__root int32_t meerkatCore_MeasuredTorque_s32;

// - Over Current/ Trip Current Protection: Sample Counter
__root uint32_t meerkatCore_ShuntCurrentSampleCounter_u32;

__root int32_t meerkatCore_TestValue1_i32;
__root int32_t meerkatCore_TestValue2_i32;

// - Test Status (Place at End)
__root uint32_t meerkatCore_FaultCode_u32;
__root uint32_t meerkatCore_LastFaultCode_u32;
__root uint32_t meerkatCore_ActiveTest_u32;
__root uint32_t meerkatCore_PassedTests_u32;
__root uint16_t meerkatCore_RiskAddrStateCount_u16; // how many cycles of Supervisor_Run have executed since Risk Addressed State was set.
volatile uint16_t meerkatCore_RiskAddrRetryCount_u16; // how many times Risk Addressed state has been cleared since last reboot
__root uint32_t meerkatCore_RiskAddrStateSetTime_u32; // what systick value when Risk Addressed State was last triggered.

// - version
__root uint16_t meerkatCore_MajorVersion_u16;
__root uint16_t meerkatCore_MedianVersion_u16;
__root uint16_t meerkatCore_MinorVersion_u16;
