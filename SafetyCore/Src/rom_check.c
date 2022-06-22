/**
  ********************************************************************************************************************************************
  * @file    Staticrom_check.c 
  * @author  Pamela Lee
  * @version V1.0
  * @date    03-09-2020
  * @brief   Static ROM Check of Safety Core
  * @note    Module for periodically checking Safety Core ROM
  *******************************************************************************************************************************************
  */
#include "check_ids.h"
#include "rom_check.h"
#include "safety_core_ram_structures.h"
#include "safety_core_macros.h"

// Macro Setup for this Module
// - Undefine macro names from safety_core_macros.h
#undef PUBLIC_STRUCT_POS
#undef PUBLIC_STRUCT_NEG
#undef PRIVATE_STRUCT_POS
#undef PRIVATE_STRUCT_NEG

// Redefine macro names based on structures in this file.
// - These are required for the shadow ram writing macros to function. (ASSIGN*, GET_PRIVATE_MEMBER_POS, etc.)
#define PUBLIC_STRUCT_POS  Safety_RAMStruct_P.romCheck_public  // Assign Module Struct to Test Macros (Public Positive RAM)
#define PUBLIC_STRUCT_NEG  Safety_RAMStruct_N.romCheck_public  // Assign Module Struct to Test Macros (Public Positive RAM)
#define PRIVATE_STRUCT_POS Safety_RAMStruct_P.romCheck_private // Assign Module Struct to Test Macros (Public Positive RAM)
#define PRIVATE_STRUCT_NEG Safety_RAMStruct_N.romCheck_private // Assign Module Struct to Test Macros (Public Positive RAM)

// Enumerate Test State Machine
enum {
    // Default APPs/Driver stage template
    INIT_STAGE,
    PRE_ROM_CHECK_STAGE,
    ROM_CHECK_STAGE,
    CONFIG_ROM_STAGE,
    APPLICATION_ROM_STAGE,
    SLEEP_STAGE,
    KILL_MODULE = 255
};
/* Private define ------------------------------------------------------------*/
// - ROM Addresses (These must match the declarations in the linker file)
// -- the last four bytes are reserved for the CRC itself, which is why end of checking is *FFFC and not *FFFF
#define ROM_START_ADDR (0x0800E800) // REVIEW: Must match linker file
#define ROM_END_ADDR   (0x0800FFFC) // Exclusive (last address is not included in calculations)
#define CONFIG_ROM_START_ADDR (0x0800E000) 
#define CONFIG_ROM_END_ADDR   (0x0800E7FC) // Exclusive (last address is not included in calculations)


// -- Checksum Addresses (0x0800FFFC-0x0800FFFF)
//#define APPLICATION_ROM_START_ADDR (0x08000000)
//#define APPLICATION_ROM_END_ADDR (0x0800CFFC) // Exclusive (last address is not included in calculations)
// -- Checksum Addresses (0x0800D7FC-0x0800D7FF)

// - Checksum Configuration
#define CRC_InitValue 0xFFFFFFFF // This must match the value set in the post-build script.
//#define Polynomial32 0x04C11DB7 // this is embedded in the 'NibbleTable'
// - ROM Nibble Table for quicker 32-bit checksum calculation
static const uint32_t ROMCheck_CRC32_NibbleTable[16] = {
    // - This is from https://www.iar.com/support/tech-notes/general/checksum-generation/
    0x00000000, 0x04C11DB7, 0x09823B6E, 0x0D4326D9, 0x130476DC, 0x17C56B6B, 0x1A864DB2, 0x1E475005,
    0x2608EDB8, 0x22C9F00F, 0x2F8AD6D6, 0x2B4BCB61, 0x350C9B64, 0x31CD86D3, 0x3C8EA00A, 0x384FBDBD,
};

// - Checksum Value
__root const uint32_t Safety_CRC @ "safety_crc32_rom" = 0x00000000; // This is a placeholder for Safety Core Checksum (CRC32)
#define READ_SAFETY_CRC (*((uint32_t *)ROM_END_ADDR)) // Patch: Optimizations are creating errors when directly reading Safety_CRC, so we read the address

// REVIEW: if this is user definable, can we really define this here? What happens if user is in a different space?
__root const uint32_t Application_CRC @ "application_crc32_rom" = 0x00000000; // This is a placeholder for the Application Checksum
//#define READ_APPLICATION_CRC (*((uint32_t *)APPLICATION_ROM_END_ADDR)) // Application area is user definable, so we do not use this macro here

#define READ_CONFIG_CRC (*((uint32_t *)CONFIG_ROM_END_ADDR)) 
// Patch: Optimizations are creating errors when directly reading Safety_CRC, so we read the address
// - this issue did not directly occur with Config, but due to the similarities between the Safety CRC and this one,
// -- we implemented this patch here as well.

// -- checksum is generated and placed using IAR's ielftool(Polynomial 0x04C11DB7)
// --- post-build script will generate a warning if Safety_CRC is not set to 0x00000000

// Functions
// - Assign these functions to be Linked to the 'Meerkat' ROM Area
#pragma default_function_attributes = @ "meerkat_rom"
void MeerkatCore_RomCheck_IncFaultCounter(uint8_t error_code_u8);


/**
  ********************************************************************************************************************************
  * @brief  MeerkatCore_SupervisorRomCheck: Contains a state machine that initializes and executes testing of ROM
  * @details 
  * - The test computes a checksum of all ROM locations used by the safety core and compares them with the value written
  * - to the last u32 of the ROM Area location.
  * - detected errors are stored in the 'faultCount_u32' variable of this test's RAM structure
  * - required ROM regions are: Safety Core, Safety Core Settings, Application Firmware, Application Settings
  ********************************************************************************************************************************
  */
void MeerkatCore_SupervisorRomCheck(void) {
    switch (GET_PRIVATE_MEMBER_POS(stage_u8)) {
        case INIT_STAGE: // Initialize all test related variables.
        {
            // public structure assignments
            // - Initialize module ID and call rate
            SET_PUBLIC_MEMBER_VALUE_U8(moduleID_u8, ROM_CHECK_ID);
            SET_PUBLIC_MEMBER_VALUE_U16(callRate_u16, ROM_CHECK_CALL_RATE);
            // - Initialize test structure counters
            SET_PUBLIC_MEMBER_VALUE_U32(faultCount_u32, 0);
            SET_PUBLIC_MEMBER_VALUE_U32(passCount_u32, 0);
            SET_PUBLIC_MEMBER_VALUE_U32(errorCode_u32, ERR_CODE_OK);
            SET_PRIVATE_MEMBER_VALUE_U8(stage_u8, PRE_ROM_CHECK_STAGE);


            break;
        } // INIT_STAGE

        case PRE_ROM_CHECK_STAGE:
        {
            // private structure assignments
            // - Initialize test properties and memory pointers
            // -- blockSize: number of u32's to checksum per call of MeerkatCore_SupervisorRomCheck
            SET_PRIVATE_MEMBER_VALUE_U32(blockSize_u32, ROM_CHECK_DEFAULT_BLOCKSIZE);
            SET_PRIVATE_MEMBER_VALUE_U32(blockAddrPtr, ROM_START_ADDR);
            // - reset test state machine and counters for running the test again
            SET_PRIVATE_MEMBER_VALUE_U8(stage_u8, ROM_CHECK_STAGE);
            SET_PRIVATE_MEMBER_VALUE_U32(CRC_u32, CRC_InitValue);
            break;
        }
        
        case ROM_CHECK_STAGE: // Check one 'block' of ROM, if errors are found, update faultCount_u32.
        {
            SET_PRIVATE_MEMBER_VALUE_U8(stage_u8,
                                        ROM_CHECK_STAGE); //loop until the end of the CRC check (error states will overwrite this)
            SET_PRIVATE_MEMBER_VALUE_U32(index_u32, 0);   // Reset the Block Counter
            for (; GET_PRIVATE_MEMBER_POS(index_u32) < GET_PRIVATE_MEMBER_POS(blockSize_u32);
                 INC_PRIVATE_MEMBER_VALUE(index_u32)) { // for each u32 in the block
                // Update Checksum with the next u32 of rom data.
                if (GET_PRIVATE_MEMBER_POS(blockAddrPtr) < ROM_END_ADDR) // Address is Valid
                {
                    MeerkatCore_ROMCheck_UpdateCRC_NextU32(); // read one u32 of data and update stored crc based on it's contents
                } else // Address out of Range -> CRC Calculation is complete
                {
                    if (READ_SAFETY_CRC == GET_PRIVATE_MEMBER_POS(CRC_u32)) // if calculated value matches the expected result
                    {
                        // SET_PRIVATE_MEMBER_VALUE_U8(stage_u8, SLEEP_STAGE);
                        // - Get Ready to run the 'Configuration ROM Check'
                        SET_PRIVATE_MEMBER_VALUE_U8(stage_u8, CONFIG_ROM_STAGE); // Set Test Stage
                        SET_PRIVATE_MEMBER_VALUE_U32(CRC_u32, CRC_InitValue); // Initialize CRC Value
                        SET_PRIVATE_MEMBER_VALUE_U32(blockAddrPtr, CONFIG_ROM_START_ADDR); // Initialize Address Pointer
                        break; // exit the for index in blockSize loop
                    } else     // values do not match
                    {
                        // Record Test Failure
                        MeerkatCore_RomCheck_IncFaultCounter(ERROR_CODE_ROM_CHECK_SAFETY);
                        // INC_PUBLIC_MEMBER_VALUE(faultCount_u32);           // set the fault status
                        // SET_PUBLIC_MEMBER_VALUE_U32(errorCode_u32, ERROR_CODE_ROM_CHECK_SAFETY);  // if need to put in the error code
                        // SET_PRIVATE_MEMBER_VALUE_U8(stage_u8, INIT_STAGE); // fault found and do the check again from init
                        break;                                             // exit the for index in blockSize loop
                    }
                }
            }
            break; // exit the case
        }          // ROM_CHECK_STAGE

       case CONFIG_ROM_STAGE: { 
            SET_PRIVATE_MEMBER_VALUE_U8(stage_u8,
                                        CONFIG_ROM_STAGE); //loop until the end of the CRC check (error states will overwrite this)
            SET_PRIVATE_MEMBER_VALUE_U32(index_u32, 0);   // Reset the Block Counter
            for (; GET_PRIVATE_MEMBER_POS(index_u32) < GET_PRIVATE_MEMBER_POS(blockSize_u32);
                 INC_PRIVATE_MEMBER_VALUE(index_u32)) { // for each u32 in the block
                // Update Checksum with the next u32 of rom data.
                if (GET_PRIVATE_MEMBER_POS(blockAddrPtr) < CONFIG_ROM_END_ADDR) // Address is Valid
                {
                    MeerkatCore_ROMCheck_UpdateCRC_NextU32(); // read one u32 of data and update stored crc based on it's contents
                } else // Address out of Range -> CRC Calculation is complete
                {
                    if (READ_CONFIG_CRC == (GET_PRIVATE_MEMBER_POS(CRC_u32))) // if calculated value matches the expected result
                    { 
                        // SET_PRIVATE_MEMBER_VALUE_U8(stage_u8, SLEEP_STAGE);
                       SET_PRIVATE_MEMBER_VALUE_U8(stage_u8, APPLICATION_ROM_STAGE); // Set Test Stage
                       SET_PRIVATE_MEMBER_VALUE_U32(CRC_u32, CRC_InitValue); // Initialize CRC Value 
                       SET_PRIVATE_MEMBER_VALUE_U32(blockAddrPtr, APPLICATION_ROM_START_ADDR); // Initialize Address Pointer
                        break; // exit the for index in blockSize loop
                    } else     // values do not match
                    {
                        // Record Test Failure
                        MeerkatCore_RomCheck_IncFaultCounter(ERROR_CODE_ROM_CHECK_CONFIG);
                        // INC_PUBLIC_MEMBER_VALUE(faultCount_u32);           // set the fault status
                        // SET_PUBLIC_MEMBER_VALUE_U32(errorCode_u32, ERROR_CODE_ROM_CHECK_CONFIG);  // if need to put in the error code 
                        // SET_PRIVATE_MEMBER_VALUE_U8(stage_u8, INIT_STAGE); // fault found and do the check again from init
                        break;                                             // exit the for index in blockSize loop
                    }
                }
            }
            break; // exit the case
       }

       case APPLICATION_ROM_STAGE: { 
            SET_PRIVATE_MEMBER_VALUE_U8(stage_u8,
                                        APPLICATION_ROM_STAGE); //loop until the end of the CRC check (error states will overwrite this)
            SET_PRIVATE_MEMBER_VALUE_U32(index_u32, 0);   // Reset the Block Counter
            for (; GET_PRIVATE_MEMBER_POS(index_u32) < GET_PRIVATE_MEMBER_POS(blockSize_u32);
                 INC_PRIVATE_MEMBER_VALUE(index_u32)) { // for each u32 in the block
                // Update Checksum with the next u32 of rom data.
                if (GET_PRIVATE_MEMBER_POS(blockAddrPtr) < APPLICATION_ROM_END_ADDR) // Address is Valid
                {
                    MeerkatCore_ROMCheck_UpdateCRC_NextU32(); // read one u32 of data and update stored crc based on it's contents
                } else // Address out of Range -> CRC Calculation is complete
                {
                    if (FIXED_ROM_APPLICATION_CRC == (GET_PRIVATE_MEMBER_POS(CRC_u32))) // if calculated value matches the expected result
                    { 
                        SET_PRIVATE_MEMBER_VALUE_U8(stage_u8, PRE_ROM_CHECK_STAGE); // Test Completed successfully, restart testing
                        // SET_PUBLIC_MEMBER_VALUE_U16(callRate_u16, ROM_CHECK_CALL_RATE_IDLE);
                        INC_PUBLIC_MEMBER_VALUE(passCount_u32); // we've completed a pass of all checks in this module
                        break; // exit the for index in blockSize loop
                    } else     // values do not match
                    {
                        // Record Test Failure
                        MeerkatCore_RomCheck_IncFaultCounter(ERROR_CODE_ROM_CHECK_APPLICATION);    
                        // INC_PUBLIC_MEMBER_VALUE(faultCount_u32);           // set the fault status
                        // SET_PUBLIC_MEMBER_VALUE_U32(errorCode_u32, ERROR_CODE_ROM_CHECK_APPLICATION);  // if need to put in the error code 
                        // SET_PRIVATE_MEMBER_VALUE_U8(stage_u8, INIT_STAGE); // fault found and do the check again from init
                        // REVIEW: Should this re-init the INIT_STAGE after a fail?
                        break;                                             // exit the for index in blockSize loop
                    }
                }
            }
            break; // exit the case
       }

//        case SLEEP_STAGE: { // REVIEW: Can we remove this?
//            SET_PUBLIC_MEMBER_VALUE_U16(callRate_u16, ROM_CHECK_CALL_RATE_IDLE);
//            // SET_PRIVATE_MEMBER_VALUE_U8(stage_u8, SLEEP_STAGE);
//            break;
//        } // SLEEP_STAGE

        case KILL_MODULE: {
            // SET_PRIVATE_MEMBER_VALUE_U8(stage_u8, KILL_MODULE);
            break;
        } // KILL_MODULE

        default: {
            SET_PRIVATE_MEMBER_VALUE_U8(stage_u8, INIT_STAGE);
            break;
        } // default
    }
}

/**
  ********************************************************************************************************************************
  * @brief  MeerkatCore_ROMCheck_UpdateCRC_NextU32: Read one u32 of data and update stored crc based on it's contents then 
  *                                                 update the address pointer for the next iteration.
  * @param none
  * @return none
  ********************************************************************************************************************************
  */

void MeerkatCore_ROMCheck_UpdateCRC_NextU32(void) {
    // Exclusive or the Current u32 to the checksum
    SET_PRIVATE_MEMBER_VALUE_U32(CRC_u32, ((GET_PRIVATE_MEMBER_POS(CRC_u32)) ^
                                            (uint32_t) * (uint32_t *) (GET_PRIVATE_MEMBER_POS(blockAddrPtr))));
    // - Now perform the required bitflip operation on each four bits for the u32 based on the polynomial
    SET_PRIVATE_MEMBER_VALUE_U32(CRC_u32,
                                    (GET_PRIVATE_MEMBER_POS(CRC_u32) << 4) ^
                                        ROMCheck_CRC32_NibbleTable[(GET_PRIVATE_MEMBER_POS(CRC_u32) >> 28)]);
    SET_PRIVATE_MEMBER_VALUE_U32(CRC_u32,
                                    (GET_PRIVATE_MEMBER_POS(CRC_u32) << 4) ^
                                        ROMCheck_CRC32_NibbleTable[(GET_PRIVATE_MEMBER_POS(CRC_u32) >> 28)]);
    SET_PRIVATE_MEMBER_VALUE_U32(CRC_u32,
                                    (GET_PRIVATE_MEMBER_POS(CRC_u32) << 4) ^
                                        ROMCheck_CRC32_NibbleTable[(GET_PRIVATE_MEMBER_POS(CRC_u32) >> 28)]);
    SET_PRIVATE_MEMBER_VALUE_U32(CRC_u32,
                                    (GET_PRIVATE_MEMBER_POS(CRC_u32) << 4) ^
                                        ROMCheck_CRC32_NibbleTable[(GET_PRIVATE_MEMBER_POS(CRC_u32) >> 28)]);
    SET_PRIVATE_MEMBER_VALUE_U32(CRC_u32,
                                    (GET_PRIVATE_MEMBER_POS(CRC_u32) << 4) ^
                                        ROMCheck_CRC32_NibbleTable[(GET_PRIVATE_MEMBER_POS(CRC_u32) >> 28)]);
    SET_PRIVATE_MEMBER_VALUE_U32(CRC_u32,
                                    (GET_PRIVATE_MEMBER_POS(CRC_u32) << 4) ^
                                        ROMCheck_CRC32_NibbleTable[(GET_PRIVATE_MEMBER_POS(CRC_u32) >> 28)]);
    SET_PRIVATE_MEMBER_VALUE_U32(CRC_u32,
                                    (GET_PRIVATE_MEMBER_POS(CRC_u32) << 4) ^
                                        ROMCheck_CRC32_NibbleTable[(GET_PRIVATE_MEMBER_POS(CRC_u32) >> 28)]);
    SET_PRIVATE_MEMBER_VALUE_U32(CRC_u32,
                                    (GET_PRIVATE_MEMBER_POS(CRC_u32) << 4) ^
                                        ROMCheck_CRC32_NibbleTable[(GET_PRIVATE_MEMBER_POS(CRC_u32) >> 28)]);
    // - Update the address pointer for the next iteration
    SET_PRIVATE_MEMBER_VALUE_U32(blockAddrPtr, GET_PRIVATE_MEMBER_POS(blockAddrPtr) + 4);
}


/**
  ********************************************************************************************************************************
  * @brief  MeerkatCore_RomCheck_IncFaultCounter: This function contains common code that executes when any test exceeds 
  *         allowable number of failures 
  * @param error_code_u8 Which test failed
  * @return none
  ********************************************************************************************************************************
  */
void MeerkatCore_RomCheck_IncFaultCounter(uint8_t error_code_u8) {
    // Record Test Failure
    INC_PUBLIC_MEMBER_VALUE(faultCount_u32);           // set the fault status
    SET_PUBLIC_MEMBER_VALUE_U32(errorCode_u32, error_code_u8);  // if need to put in the error code 
    SET_PRIVATE_MEMBER_VALUE_U8(stage_u8, INIT_STAGE); // fault found and do the check again from init
}
