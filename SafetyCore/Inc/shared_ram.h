/**
  *******************************************************************************************************************************************
  * @file    shared_ram.h 
  * @author  Regal Kyle McBrady
  * @version V1.0
  * @date    03-Dec-2018
  * @brief   Header containing Fixed Ram locations for varibles shared between FlexMouse and Meerkat
  * @note    This file needs to be shared between Meerkat Project and FlexMouse Project
  * @note    Fixed Shared FlexMouse Write / Meerkat Read Begins at 0x2000 0C00 and ends at 0x2000 0FFF
  * @note    Fixed Shared FlexMouse Read / Meerkat Write Begins at 0x2000 1000 and ends at 0x2000 13FF
  *******************************************************************************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SHARED_RAM_H
    #define SHARED_RAM_H

    #ifdef __cplusplus
extern "C" {
    #endif
 
// - Clock Check: Ring Buffer 
// -- Note: If this macro is changed then the same change MUST be made to all applicaitons that use this library
#define LENGTH_OF_LSI_RING_BUFFER               (4) // number of u32's, so a value of 4 creates 16 bytes of shared ram
extern uint32_t meerkatCore_LSIClockBuffer_u32[LENGTH_OF_LSI_RING_BUFFER]; 
// - ADC Check: Single Sample Buffers
extern uint16_t meerkatCore_Vref_u16; // meerkatCore_Vref_u16
extern uint16_t meerkatCore_TemperatureValue1_u16; // meerkatCore_TemperatureValue1_u16
extern uint16_t meerkatCore_Vbus_u16; // meerkatCore_Vbus_u16

// - Clock Check: Single Sample Buffers
extern uint32_t meerkatCore_SysTickMs_u32; // meerkatCore_SysTickMs_u32

// - General Motor State: Speed RPM
extern int32_t meerkatCore_SpeedRpm_s32; // meerkatCore_SpeedRpm_s32

// - Locked Rotor Detection
extern int32_t meerkatCore_ExpectedBEMF_s32; // meerkatCore_ExpectedBEMF_s32
extern int32_t meerkatCore_ObservedBEMF_s32; // meerkatCore_ObservedBEMF_s32

// - No Load Detection
extern int32_t meerkatCore_MeasuredTorque_s32; // meerkatCore_MeasuredTorque_s32

// - Over Current/ Trip Current Protection: Sample Counter
extern uint32_t meerkatCore_ShuntCurrentSampleCounter_u32; // meerkatCore_ShuntCurrentSampleCounter_u32


extern int32_t meerkatCore_TestValue1_i32;
extern int32_t meerkatCore_TestValue2_i32;

// - version
extern uint16_t meerkatCore_MajorVersion_u16;
extern uint16_t meerkatCore_MedianVersion_u16;
extern uint16_t meerkatCore_MinorVersion_u16;

// - Test Status (Place at End)
extern uint32_t meerkatCore_FaultCode_u32; // meerkatCore_FaultCode_u32
extern uint32_t meerkatCore_LastFaultCode_u32; // meerkatCore_FaultCode_u32
extern uint32_t meerkatCore_ActiveTest_u32; // meerkatCore_ActiveTest_u32
extern uint32_t meerkatCore_PassedTests_u32; // meerkatCore_PassedTests_u32
extern uint16_t meerkatCore_RiskAddrStateCount_u16; // how many cycles of Supervisor_Run have executed since Risk Addressed State was set.
extern volatile uint16_t meerkatCore_RiskAddrRetryCount_u16; // how many times Risk Addressed state has been cleared since last reboot
extern uint32_t meerkatCore_RiskAddrStateSetTime_u32; // what systick value when Risk Addressed State was last triggered.

    #ifdef __cplusplus
}
    #endif

#endif // SHARED_RAM_H