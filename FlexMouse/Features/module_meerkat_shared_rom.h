/**
  ********************************************************************************************************************************************
  * @file    shared_rom.h 
  * @author  Justin Moon
  * @version V1.0
  * @date    06-19-2020
  * @brief   Flashable Meerkat Configuration Parameters 
  * @note    These ROM located parameters are intended to allow the application firmware to configure the limits of the safety core.
  *             Values are set in the '.c' file, 'extern' objects for import into other files are declared in the .h file.
  *******************************************************************************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SHARED_ROM_H
    #define SHARED_ROM_H

    #ifdef __cplusplus
extern "C" {
    #endif

#include <stdint.h> // REVIEW: Did not have to import this in similar files, uint32_t error if not imported in this file

// Shared ROM Checksum Area
extern __root const uint32_t Config_CRC @ "shared_configuration_crc32_rom"; // = 0x00000099; // This is a placeholder for Settings Checksum (CRC32)
// -- checksum is generated and placed using IAR's ielftool(Polynomial 0x04C11DB7)
// --- post-build script will generate a warning if this value is not set to 0x00000000

// Shared ROM Area (Objects/ Values)
// - These items are placed by the linker file. Their ordering is not critical in this file (but is in the .c file).
extern __root const uint32_t MeerkatConfig_ClockCheck_IdealLsiCounts_u32; // @ "shared_configuration_rom" = 3788;

extern __root const float MeerkatConfig_ClockCheck_AcceptableVariance_f; // @ "shared_configuration_rom" = 0.35; // 0.35 = +-35%

extern __root const uint8_t MeerkatConfig_ClockCheck_HysteresisLsiTick_u8; // @ "shared_configuration_rom" = 3;
extern __root const uint8_t MeerkatConfig_ClockCheck_HysteresisSysTick_u8; // @ "shared_configuration_rom" = 20;
extern __root const uint8_t MeerkatConfig_ADCCheck_Hysteresis_VRef_u8; // @ "shared_configuration_rom" = 10; 
extern __root const uint8_t MeerkatConfig_ADCCheck_Hysteresis_Mux_u8; // @ "shared_configuration_rom" = 10;
extern __root const uint8_t MeerkatConfig_ADCCheck_Hysteresis_Shunt_current_u8; // @ "shared_configuration_rom" = 10; 
extern __root const uint8_t MeerkatConfig_ADCCheck_Hysteresis_Locked_Rotor_u8; // @ "shared_configuration_rom" = 5;
extern __root const uint8_t MeerkatConfig_ADCCheck_Hysteresis_Coherence_u8; // @ "shared_configuration_rom" = 3; 
extern __root const uint8_t MeerkatConfig_ADCCheck_Hysteresis_Over_Speed_u8; // @ "shared_configuration_rom" = 3;

extern __root const uint16_t MeerkatConfig_ADCCheck_MinVref_u16; // @ "shared_configuration_rom" = 0x0001;
extern __root const uint16_t MeerkatConfig_ADCCheck_MaxVref_u16; // @ "shared_configuration_rom" = 0xFFFE;

extern __root const uint16_t MeerkatConfig_ADCCheck_CurrentVarianceTolerance_u16; // @ "shared_configuration_rom" = 0x03FF;
extern __root const uint16_t MeerkatConfig_ADCCheck_MultiplexerVarianceRequired_u16; // @ "shared_configuration_rom" = 0x00FF;

extern __root const int16_t MeerkatConfig_ADCCheck_MinCurrent_i16; // @ (SHARED_CONFIG_ROM_START+24) = -32000;
extern __root const int16_t MeerkatConfig_ADCCheck_MaxCurrent_i16; // @ (SHARED_CONFIG_ROM_START+26) = 32000;
extern __root const uint32_t MeerkatConfig_ADCCheck_MinCurrentSampleSize_u32; // @ (SHARED_CONFIG_ROM_START+28) = 4;

extern __root const uint32_t MeerkatConfig_LockedRotorCheck_BemfDiff_u32; // @ (SHARED_CONFIG_ROM_START+52) = 65000;
extern __root const int32_t MeerkatConfig_OverSpeedCheck_MaxSpeed_u32; // @ (SHARED_CONFIG_ROM_START+56) = 10000;
extern __root const int32_t MeerkatConfig_OverSpeedCheck_MinSpeed_u32; // @ (SHARED_CONFIG_ROM_START+60) = -10000;
extern __root const uint32_t MeerkatConfig_OverSpeedCheck_MinTorque_u32; // @ (SHARED_CONFIG_ROM_START+64) = 800; 
extern __root const uint16_t MeerkatConfig_ADCCoherence_MinSpeedRpm; // @ (THIS_BLOCK_ADDR+0) = 10; 

extern __root const uint32_t MeerkatConfig_RAMCheck_NumBlocks_u32; // @ "shared_configuration_rom" = 90;

extern __root const uint32_t MeerkatConfig_ROMCheck_BlocksPerIteration_u32; // @ (SHARED_CONFIG_ROM_START+28) = 256;

extern __root const uint8_t MeerkatConfig_ROMCheck_Hysteresis_u8; // @ (SHARED_CONFIG_ROM_START+28) = 3;
extern __root const uint8_t MeerkatConfig_FaultClearTimeS_u8; // This is added to the min time RISK_ADDRESSED_STATE_MIN_TIMEOUT_MS
extern __root const uint8_t MeerkatConfig_FaultClearLimit_u8; // number of times safety faults can clear before device reboots

extern __root const uint32_t MeerkatConfig_ROMCheck_AppStartAddress_u32;
extern __root const uint32_t MeerkatConfig_ROMCheck_AppEndAddress_u32;

#define FIXED_ROM_ADDRESS_APPLICATION_CRC  (MeerkatConfig_ROMCheck_AppEndAddress_u32)
#define FIXED_ROM_APPLICATION_CRC (*(volatile uint32_t *) MeerkatConfig_ROMCheck_AppEndAddress_u32)

    #ifdef __cplusplus
}
    #endif

#endif // SHARED_ROM_H
