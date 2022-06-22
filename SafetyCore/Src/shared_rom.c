/**
  ********************************************************************************************************************************************
  * @file    shared_rom.c
  * @author  Justin Moon
  * @version V1.0
  * @date    06-19-2020
  * @brief   Flashable Meerkat Configuration Parameters 
  * @note    These ROM located parameters are intended to allow the application firmware to configure the limits of the safety core.
  *             Default values are written by this firmware, but the application firmware is expected to overwrite these locations.
  *             Values are set in the '.c' file, 'extern' objects for import into other files are declared in the .h file.
  *******************************************************************************************************************************************
  */

#include "shared_rom.h"

// Shared ROM Checksum Area
//__root const uint32_t Application_CRC @ "application_crc32_rom" = 0x00000000; // This is a placeholder for the Application Checksum
__root const uint32_t Config_CRC @ "shared_configuration_crc32_rom" = 0x00000000; // This is a placeholder for Settings Checksum (CRC32)
// -- checksum is generated and placed using IAR's ielftool(Polynomial 0x04C11DB7)
// --- post-build script will generate a warning if this value is not set to 0x00000000

// Shared ROM Area (Objects/ Values)
// - These items are placed by the linker file. Their ordering is critical.
// -- Do not re-organize this area or the addresses of each item will change.
// --- All items must be declared __root to ensure that they are not optimized out by the compiler.
// ---- Note addressing on stm32 devices is blocked in 32-bit chunks. If you do the chunk the space is wasted.
// ---- Example: if you start a chunk with a single u8 then add a u32 after it, the u8 will be allotted four bytes of space.
// ----- as a u32 object cannot begin the middle of a chunk.

// Note: All Addresses below must match designated area from linker file.
// - Flash Word Size
#define SHARED_CONFIG_ROM_BLOCK_SIZE 0x04 
// - Flash ROM Addresses
#define SHARED_CONFIG_ROM_START 0x0800D800 // (REVIEW: Can we link this to linker file and still 'address' the consts below?)
#define SHARED_CONFIG_ROM_END 0x0800E000 // Exclusive: This value is not included in the address space
#define SHARED_CONFIG_ROM_CRC_ADDR 0x0800DFFC // For Reference
#define SHARED_CONFIG_ROM_LAST_BLOCK_START (SHARED_CONFIG_ROM_END-2*SHARED_CONFIG_ROM_BLOCK_SIZE) 
#define SHARED_CONFIG_ROM_LAST_BLOCK_END (SHARED_CONFIG_ROM_END-1*SHARED_CONFIG_ROM_BLOCK_SIZE) // Exclusive: This value is not included in the address space


//#define SHARED_CONFIG_ROM_START &Config_CRC // must match linker file  REVIEW: better connection
// #define SHARED_CONFIG_ROM_START (&LNK_CFG_ROM_START); // REVIEW: works if defining a const, but not in #define macros
// - note: automatic addressing as shown in the commented example below, results in the linker arbitrarily re-organizing ROM
// -- so we must specify the address specifically instead of using this 
// --- NO:  __root const uint32_t MeerkatConfig_ClockCheck_IdealLsiCounts_u32 @ "shared_configuration_rom" = 3788; 
// --- YES: __root const uint32_t MeerkatConfig_ClockCheck_IdealLsiCounts_u32 @ (SHARED_CONFIG_ROM_START) = 3788;
// - Address = ("shared_configuration_rom_start") + 0x00
// chunk 0: u32
#define THIS_BLOCK_ADDR SHARED_CONFIG_ROM_LAST_BLOCK_START
__root const uint32_t MeerkatConfig_ClockCheck_IdealLsiCounts_u32 @ (THIS_BLOCK_ADDR) = 3788;
// chunk 1: float (u32)
#undef THIS_BLOCK_ADDR
#define THIS_BLOCK_ADDR (SHARED_CONFIG_ROM_LAST_BLOCK_START - (1 * SHARED_CONFIG_ROM_BLOCK_SIZE))
__root const float MeerkatConfig_ClockCheck_AcceptableVariance_f @ (THIS_BLOCK_ADDR) = 0.35; // 0.35 = +-35%, float is 4 bytes

// chunk 2: u8, u8, u8, u8
#undef THIS_BLOCK_ADDR
#define THIS_BLOCK_ADDR (SHARED_CONFIG_ROM_LAST_BLOCK_START - (2 * SHARED_CONFIG_ROM_BLOCK_SIZE))
__root const uint8_t MeerkatConfig_ClockCheck_HysteresisLsiTick_u8 @ (THIS_BLOCK_ADDR+0) = 3;
__root const uint8_t MeerkatConfig_ClockCheck_HysteresisSysTick_u8 @ (THIS_BLOCK_ADDR+1) = 20;
__root const uint8_t MeerkatConfig_ADCCheck_Hysteresis_VRef_u8 @ (THIS_BLOCK_ADDR+2) = 10; 
__root const uint8_t MeerkatConfig_ADCCheck_Hysteresis_Mux_u8 @ (THIS_BLOCK_ADDR+3) = 10;

// chunk 3: u8, u8, u8, u8
#undef THIS_BLOCK_ADDR
#define THIS_BLOCK_ADDR (SHARED_CONFIG_ROM_LAST_BLOCK_START - (3 * SHARED_CONFIG_ROM_BLOCK_SIZE))
__root const uint8_t MeerkatConfig_ADCCheck_Hysteresis_Shunt_current_u8 @ (THIS_BLOCK_ADDR+0) = 25;
__root const uint8_t MeerkatConfig_ADCCheck_Hysteresis_Locked_Rotor_u8 @ (THIS_BLOCK_ADDR+1) = 5;
__root const uint8_t MeerkatConfig_ADCCheck_Hysteresis_Coherence_u8 @ (THIS_BLOCK_ADDR+2) = 3; 
__root const uint8_t MeerkatConfig_ADCCheck_Hysteresis_Over_Speed_u8 @ (THIS_BLOCK_ADDR+3) = 3;


// - Address = ("shared_configuration_rom_start") + 0x10
// chunk 4: u16, u16
#undef THIS_BLOCK_ADDR
#define THIS_BLOCK_ADDR (SHARED_CONFIG_ROM_LAST_BLOCK_START - (4 * SHARED_CONFIG_ROM_BLOCK_SIZE))
__root const uint16_t MeerkatConfig_ADCCheck_MinVref_u16 @ (THIS_BLOCK_ADDR+0) = 0x0700;
__root const uint16_t MeerkatConfig_ADCCheck_MaxVref_u16 @ (THIS_BLOCK_ADDR+2) = 0x0900;

// chunk 5: u16, u16
#undef THIS_BLOCK_ADDR
#define THIS_BLOCK_ADDR (SHARED_CONFIG_ROM_LAST_BLOCK_START - (5 * SHARED_CONFIG_ROM_BLOCK_SIZE))
__root const uint16_t MeerkatConfig_ADCCheck_CurrentVarianceTolerance_u16 @ (THIS_BLOCK_ADDR+0) = 3000;
__root const uint16_t MeerkatConfig_ADCCheck_MultiplexerVarianceRequired_u16 @ (THIS_BLOCK_ADDR+2) = 0x00FF;

// chunk 6: i16, i16
#undef THIS_BLOCK_ADDR
#define THIS_BLOCK_ADDR (SHARED_CONFIG_ROM_LAST_BLOCK_START - (6 * SHARED_CONFIG_ROM_BLOCK_SIZE))
//this value is the maximum rated current for IGBT module
__root const int16_t MeerkatConfig_ADCCheck_MaxRatedCurrent_i16 @ (THIS_BLOCK_ADDR+0) = 32000;
//this value is the minimu rated speed for the motor * 0.8
__root const int16_t MeerkatConfig_ADCCheck_MinRatedSpeed_i16 @ (THIS_BLOCK_ADDR+2) = 1296;  

// chunk 7: float
#undef THIS_BLOCK_ADDR
#define THIS_BLOCK_ADDR (SHARED_CONFIG_ROM_LAST_BLOCK_START - (7 * SHARED_CONFIG_ROM_BLOCK_SIZE))
//this value is ((maxCurrent-minCurrent)/(minSpeed - maxSpeed)) where:
// maxSpeed = eerkatConfig_ADCCheck_MaxRatedSpeed_i16
// minSpeed = MeerkatConfig_ADCCheck_MinRatedSpeed_i16
// maxCurrent = maxCurrentPhaseU_i16 * 1.1 when the motor is running at minSpeed = 10375.7 * 1.1 = 11413.3
// minCurrent = maxCurrentPhaseU_i16 *1.1 when the motor is running at maxSpeed =  7984 * 1.1 = 8782.4
__root const float MeerkatConfig_ADCCheck_CurrentSpeedSlope @ (THIS_BLOCK_ADDR+0) = -3.9; 

// chunk 8: float
#undef THIS_BLOCK_ADDR
#define THIS_BLOCK_ADDR (SHARED_CONFIG_ROM_LAST_BLOCK_START - (8 * SHARED_CONFIG_ROM_BLOCK_SIZE))
//this value is (minCurrent - slope * minSpeed) where:
// minCurrent = maxCurrentPhaseU_i16 + 15% when the motor is running at maxSpeed
// slope = MeerkatConfig_ADCCheck_SpeedCurrentSlope
// minSpeed = MeerkatConfig_ADCCheck_MinRatedSpeed_i16
__root const float MeerkatConfig_ADCCheck_SpeedCurrentYIntercept @ (THIS_BLOCK_ADDR+0) = 16472.1;

// chunk 9: u32
#undef THIS_BLOCK_ADDR
#define THIS_BLOCK_ADDR (SHARED_CONFIG_ROM_LAST_BLOCK_START - (9 * SHARED_CONFIG_ROM_BLOCK_SIZE))
__root const uint32_t MeerkatConfig_ADCCheck_MinCurrentSampleSize_u32 @ (THIS_BLOCK_ADDR+0) = 450;
//>>> (1/(400/60.))/10 = 0.015ms >>>>> .015/(1/30000.) = 449.99

// chunk 10: u32 
#undef THIS_BLOCK_ADDR
#define THIS_BLOCK_ADDR (SHARED_CONFIG_ROM_LAST_BLOCK_START - (10 * SHARED_CONFIG_ROM_BLOCK_SIZE))
__root const uint32_t MeerkatConfig_LockedRotorCheck_BemfDiff_u32 @ (THIS_BLOCK_ADDR+0) = 4500;
// chunk 11: u32 
#undef THIS_BLOCK_ADDR
#define THIS_BLOCK_ADDR (SHARED_CONFIG_ROM_LAST_BLOCK_START - (11 * SHARED_CONFIG_ROM_BLOCK_SIZE))
__root const int32_t MeerkatConfig_OverSpeedCheck_MaxSpeed_u32 @ (THIS_BLOCK_ADDR+0) = 10000;
// chunk 12: u32 
#undef THIS_BLOCK_ADDR
#define THIS_BLOCK_ADDR (SHARED_CONFIG_ROM_LAST_BLOCK_START - (12 * SHARED_CONFIG_ROM_BLOCK_SIZE))
__root const int32_t MeerkatConfig_OverSpeedCheck_MinSpeed_u32 @ (THIS_BLOCK_ADDR+0) = -10000;
// chunk 13: u32 
#undef THIS_BLOCK_ADDR
#define THIS_BLOCK_ADDR (SHARED_CONFIG_ROM_LAST_BLOCK_START - (13 * SHARED_CONFIG_ROM_BLOCK_SIZE))
__root const uint32_t MeerkatConfig_OverSpeedCheck_MinTorque_u32 @ (THIS_BLOCK_ADDR+0) = 800; 
// - if above this limit, the device is 'loaded' and the 'no load' speed check does not execute
// chunk 14: u16, u16 
#undef THIS_BLOCK_ADDR
#define THIS_BLOCK_ADDR (SHARED_CONFIG_ROM_LAST_BLOCK_START - (14 * SHARED_CONFIG_ROM_BLOCK_SIZE))
__root const uint16_t MeerkatConfig_ADCCoherence_MinSpeedRpm @ (THIS_BLOCK_ADDR+0) = 10; 
__root const uint16_t MeerkatConfig_Unused_u16 @ (THIS_BLOCK_ADDR+2) = 0; 
// - if speed is below this limit, safety core will not run coherence checks.
// -- This is implemented to avoid adjusting to startup scenarios where running at less than 1rpm where hundreds of thousands
// --- of samples need to be considered in a single decision.
// __root const uint16_t MeerkatConfig_ @ (THIS_BLOCK_ADDR+2) = 0; 


// chunk 15: u32
// - RAM Check compares two 512 Byte blocks fof RAM. If "LEN_OF_BLOCK" is 4, then this value should be 128 
#undef THIS_BLOCK_ADDR
#define THIS_BLOCK_ADDR (SHARED_CONFIG_ROM_LAST_BLOCK_START - (15 * SHARED_CONFIG_ROM_BLOCK_SIZE))
__root const uint32_t MeerkatConfig_RAMCheck_NumBlocks_u32 @ (THIS_BLOCK_ADDR+0) = 4;
// chunk 16: u32
//__root const uint32_t MeerkatConfig_ROMCheck_BlocksPerIteration_u32 @ (SHARED_CONFIG_ROM_START+28) = 256; // 1.3ms execution time
#undef THIS_BLOCK_ADDR
#define THIS_BLOCK_ADDR (SHARED_CONFIG_ROM_LAST_BLOCK_START - (16 * SHARED_CONFIG_ROM_BLOCK_SIZE))
__root const uint32_t MeerkatConfig_ROMCheck_BlocksPerIteration_u32 @ (THIS_BLOCK_ADDR+0) = 16; // 16: 89us execution time 
// chunk 17: u32
#undef THIS_BLOCK_ADDR
#define THIS_BLOCK_ADDR (SHARED_CONFIG_ROM_LAST_BLOCK_START - (17 * SHARED_CONFIG_ROM_BLOCK_SIZE))
__root const uint32_t MeerkatConfig_ROMCheck_AppStartAddress_u32 @ (THIS_BLOCK_ADDR+0) = 0x08000000;
// chunk 18: u32
#undef THIS_BLOCK_ADDR
#define THIS_BLOCK_ADDR (SHARED_CONFIG_ROM_LAST_BLOCK_START - (18 * SHARED_CONFIG_ROM_BLOCK_SIZE))
__root const uint32_t MeerkatConfig_ROMCheck_AppEndAddress_u32 @ (THIS_BLOCK_ADDR+0) = 0x0800DFFC; // This should be the start address of the checksum
// chunk 19: u8, u8, u8, u8
#undef THIS_BLOCK_ADDR
#define THIS_BLOCK_ADDR (SHARED_CONFIG_ROM_LAST_BLOCK_START - (19 * SHARED_CONFIG_ROM_BLOCK_SIZE))
__root const uint8_t MeerkatConfig_ROMCheck_Hysteresis_u8 @ (THIS_BLOCK_ADDR+0) = 3;
__root const uint8_t MeerkatConfig_FaultClearTimeS_u8 @ (THIS_BLOCK_ADDR+1) = 10; // This is added to the min time RISK_ADDRESSED_STATE_MIN_TIMEOUT_MS
__root const uint8_t MeerkatConfig_FaultClearLimit_u8 @ (THIS_BLOCK_ADDR+2) = 3; // number of times safety faults can clear before device reboots
__root const uint8_t MeerkatConfig_Unused_u8 @ (THIS_BLOCK_ADDR+3) = 0;

 
