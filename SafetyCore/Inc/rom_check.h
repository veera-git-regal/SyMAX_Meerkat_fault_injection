/**
  ********************************************************************************************************************************************
  * @file    rom_check.h 
  * @author  Pamela Lee
  * @version V1.0
  * @date    03-09-2020
  * @brief   Static ROM Check of Safety Core
  * @note    Header for periodically checking Safety Core ROM
  *******************************************************************************************************************************************
  */
#ifndef _ROM_CHECK_H_
#define _ROM_CHECK_H_

#include <stdint.h>

#include "shared_ram.h"

// Test Module Performance
// Measured Execution Time: 1320us, when BlocksPerIteration = 256 (1kB checked per iteration because block size must be 4 for crc32)
// Measured Execution Time: 171us, when BlocksPerIteration_u32 = 32 (128B checked per iteration (1s pass time at 2ms call rate)
// - Expected Execution Time (us) (motor stopped): t = 1.28236*(4*BlocksPerIteration) + 6.857
// -- Calculated Test Setup Time (us) - 6.857us
// -- Calculated Slope (us per byte checked) = 1.28236us
// --- Note: Expected Execution time to scan all of ROM in a single call 1.28236*65535 + 6.857 = 84,046us (84ms)
// --- Note: Expected Execution time to scan all ROM (16 Blocks Per Iteration)  (1.28236*64 + 6.857)*65535/64 = 84,046us (91ms)
// - Expected Full Scan Time (ms): t = (65535/(4*BlocksPerIteration))*CALL_RATE (measured very close +-10ms)


// #define SAFETY_ROM_CHECKSUM_START_ADDR 0x0800FFFC // Must match __ICFEDIT_region_ROMCRC_start__ in '.icf' linker file

#define ROM_CHECK_CALL_RATE  (4) // How frequently a module is called (in milliseconds)
// - 5ms call rate leads to 5s pass time when 64kB ROM is measured in three phases (Config, App, Safety Core)
#define ROM_CHECK_CALL_RATE_IDLE (18) // How frequently to call this module after ROM Check has passed has passed.

#define ROM_CHECK_PASS_REQUIREMENT   (1) // How many times module must pass before pass is reported


#define ERR_CODE_OK          (0)                             // error code for no errors
#define ERROR_CODE_ROM_CHECK_SAFETY 0x12
#define ERROR_CODE_ROM_CHECK_CONFIG 0x13
#define ERROR_CODE_ROM_CHECK_APPLICATION 0x14

#define ROM_CHECK_HYSTERESIS (MeerkatConfig_ROMCheck_Hysteresis_u8)                             // number of hysteresis loops before asserting a failure
#define ROM_CHECK_DEFAULT_BLOCKSIZE (MeerkatConfig_ROMCheck_BlocksPerIteration_u32) // number of u32's to checksum per call of MeerkatCore_SupervisorRomCheck
#define APPLICATION_ROM_START_ADDR (MeerkatConfig_ROMCheck_AppStartAddress_u32)
#define APPLICATION_ROM_END_ADDR (MeerkatConfig_ROMCheck_AppEndAddress_u32) // Exclusive (last address is not included in calculations)
// -- Note: doing 64-kB ( blocksize 16k ) takes at least 17ms (19 Cycles per Byte)
// --- Blocksize 256 (1024 bytes) is estimated 0.25ms execution time


/*** Public variable structure looks like this
typedef struct
{
  uint8_t	moduleID_u8;                                                       //moduleID defined above
  uint16_t	callRate_u16;
  uint32_t	faultCount_u32;               
  uint32_t	passCount_u32;
  uint32_t	errorCode_u32; 
}Module_Public_OTYP;
End of public variable structure ****/

// variable structure
typedef struct {
    uint32_t CRC_u32;
    uint32_t blockSize_u32;
    uint32_t blockAddrPtr;
    uint32_t index_u32;
    uint8_t bitCount_u8;
    uint8_t stage_u8; // next stage of module
} romCheck_private_OTYP;

// function prototypes
void MeerkatCore_SupervisorRomCheck(void);
void MeerkatCore_ROMCheck_UpdateCRC_NextU32(void);

#endif
