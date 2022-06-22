/**
  ********************************************************************************************************************************************
  * @file    ram_check.h 
  * @author  Myron Mychal
  * @version V1.0
  * @date    03-09-2020
  * @brief   Static RAM Check of Safety Core
  * @note    Header for periodically checking Safety Core RAM
  *******************************************************************************************************************************************
  */
#include <stdint.h>

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _RAM_CHECK_H_
    #define _RAM_CHECK_H_

    // Test Module Performance
    // Measured Execution Time: 175us, when LEN_OF_BLOCK = 128, and MeerkatConfig_RAMCheck_NumBlocks_u32 = 4
    // Measured Execution Time: 46us, when LEN_OF_BLOCK = 32, and MeerkatConfig_RAMCheck_NumBlocks_u32 = 4
    // - Expected Execution Time (us) (motor stopped): t = 0.336*(LEN_OF_BLOCK*BlocksPerIteration) + 3
    // -- Calculated Test Setup Time (us) - 3us
    // -- Calculated Slope (us per byte checked) = 0.336us
    // - Expected Full Scan Time (ms): t = (512/(LEN_OF_BLOCK*MeerkatConfig_RAMCheck_NumBlocks_u32))*CALL_RATE
    // -- 512 is the size of Meerkat's 'Positive' RAM (0x200)


    /* Includes ------------------------------------------------------------------*/
    #include "shared_ram.h"

    // Common Test Parameters
    #define RAM_CHECK_CALL_RATE (200) // How frequently a module is called (in milliseconds)
    // - at 200ms call rate test executes in about 1s 
    #define RAM_CHECK_PASS_REQUIREMENT   (1) // How many times module must pass before pass is reported

    // Test Time Configuration (Single Calls)
    #define LEN_OF_BLOCK         (16)  // length of block (in bytes) to check on each iteration of for loop
    #define NUM_OF_BLOCKS        (MeerkatConfig_RAMCheck_NumBlocks_u32) // number of blocks to check each iteration of the test


    #define RAM_GOOD    (0xFF) // RAM is good because PRAM ^ NRAM = all 1's
    #define ERR_CODE_OK (0)    // error code for no errors
    //#define	ERR_CODE_BAD_RAM		(1234)							// error code for RAM error
    #define START_OF_RAM (SAFETY_RAM_START_ADDR) // beginning address for Safety Core RAM structure
    //#define END_OF_RAM				(SAFETY_RAM_END_ADDR)			// last address for Safety Cor RAM structure
    //#define LEN_RAM_BYTES			((END_OF_RAM)-(START_OF_RAM))	// length of RAM in bytes


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
    uint32_t blockLength_u32;     // length of block to check on this iteration
    uint32_t currentAddr_u32;     // current address of PRAM being checked
    uint32_t currentBlock_u32;    // The block of RAM that is currently being checked
    uint32_t index_u32;           // loop index
    uint32_t remainingBlocks_u32; // The number of blocks remaining to be checked for this pass
    uint8_t nRAMData_u8;          // a byte of data from the NRAM
    uint8_t pRAMData_u8;          // a byte of data from the PRAM
    uint8_t stage_u8;             // next stage of module
} ramCheck_private_OTYP;

// function prototypes
void MeerkatCore_SupervisorRamCheck(void);

#endif
