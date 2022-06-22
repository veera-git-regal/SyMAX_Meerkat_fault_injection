/**
  ********************************************************************************************************************************************
  * @file    ram_check.c 
  * @author  Myron Mychal
  * @version V1.0
  * @date    03-09-2020
  * @brief   RAM Check of Safety Core
  * @note    Module for periodically checking Safety Core RAM
  *******************************************************************************************************************************************
  */
 
#include "check_ids.h"
#include	"ram_check.h" 
#include	"safety_core_ram_structures.h" // RAM Locations for variables
#include	"safety_core_macros.h" // Macros for safety modules

// Macro Setup for this Module
// - Undefine macro names from safety_core_macros.h
#undef	PUBLIC_STRUCT_POS
#undef	PUBLIC_STRUCT_NEG
#undef	PRIVATE_STRUCT_POS
#undef	PRIVATE_STRUCT_NEG

// Redefine macro names based on structures in this file.
// - These are required for the shadow ram writing macros to function. 
#define	PUBLIC_STRUCT_POS	Safety_RAMStruct_P.ramCheck_public		// Assign Module Struct to Test Macros (Public Positive RAM)
#define	PUBLIC_STRUCT_NEG	Safety_RAMStruct_N.ramCheck_public		// Assign Module Struct to Test Macros (Public Negative RAM)
#define	PRIVATE_STRUCT_POS	Safety_RAMStruct_P.ramCheck_private		// Assign Module Struct to Test Macros (Private Positive RAM)
#define	PRIVATE_STRUCT_NEG	Safety_RAMStruct_N.ramCheck_private		// Assign Module Struct to Test Macros (Private Negative RAM)


// Enumerate Test State Machine
enum  
{
  // Default APPs/Driver stage template
  INIT_STAGE,
  NEW_BLOCK_STAGE,
  RAM_CHECK_STAGE,
  REPORT_STATUS_STAGE,
  
  SLEEP_STAGE,
  KILL_MODULE = 255
};

// Functions
// - Assign these functions to be Linked to the 'Meerkat' ROM Area
#pragma default_function_attributes = @ "meerkat_rom"
/**
  ********************************************************************************************************************************
  * @brief  MeerkatCore_SupervisorRamCheck: This function contains a state machine that initializes and executes testing of RAM
  * @details 
  * - The test checks compares all RAM used by the safety core with an inverted copy of the RAM (negative/ shadow RAM)
  * - detected errors are stored in the 'faultCount_u32' variable of this test's RAM structure
  ********************************************************************************************************************************
  */
void MeerkatCore_SupervisorRamCheck(void)
{  
  switch (GET_PRIVATE_MEMBER_POS(stage_u8))
  {
  case INIT_STAGE: // Initialize all test related variables.
    {
      // public structure assignments
      // - Initialize module ID and call rate
      SET_PUBLIC_MEMBER_VALUE_U8(moduleID_u8, RAM_CHECK_ID);
      SET_PUBLIC_MEMBER_VALUE_U16(callRate_u16, RAM_CHECK_CALL_RATE);
      // - Initialize test structure counters
      SET_PUBLIC_MEMBER_VALUE_U32(faultCount_u32, 0);   
      SET_PUBLIC_MEMBER_VALUE_U32(passCount_u32, 0);
      SET_PUBLIC_MEMBER_VALUE_U32(errorCode_u32, ERR_CODE_OK);
      
      // private structure assignments			
      // - Initialize test properties and memory pointers
      SET_PRIVATE_MEMBER_VALUE_U32(blockLength_u32, LEN_OF_BLOCK);
      SET_PRIVATE_MEMBER_VALUE_U32(currentAddr_u32, START_OF_RAM);
      SET_PRIVATE_MEMBER_VALUE_U32(currentBlock_u32, START_OF_RAM);
      SET_PRIVATE_MEMBER_VALUE_U32(index_u32, 0);
      SET_PRIVATE_MEMBER_VALUE_U32(remainingBlocks_u32, NUM_OF_BLOCKS);
      // - Reset test state machine and counters for running the test again      
      SET_PRIVATE_MEMBER_VALUE_U8(nRAMData_u8, 0);
      SET_PRIVATE_MEMBER_VALUE_U8(pRAMData_u8, 0);
      SET_PRIVATE_MEMBER_VALUE_U8(stage_u8, RAM_CHECK_STAGE);
      break;
    } // INIT_STAGE
    
  case NEW_BLOCK_STAGE: // Re-Initialize any variables required to re-run the Test
    {                   //- REVIEW: this is called 'NEW_BLOCK_STAGE', but it is set up to restart the scanning of all blocks
      SET_PUBLIC_MEMBER_VALUE_U32(errorCode_u32, ERR_CODE_OK);
      
      // private structure assignments			
      SET_PRIVATE_MEMBER_VALUE_U32(blockLength_u32, LEN_OF_BLOCK);
      // REVIEW:  do not reset current address, but it is not reset elsewhere
      SET_PRIVATE_MEMBER_VALUE_U32(currentAddr_u32, START_OF_RAM);
      SET_PRIVATE_MEMBER_VALUE_U32(currentBlock_u32, START_OF_RAM);
      SET_PRIVATE_MEMBER_VALUE_U32(index_u32, 0);
      // REVIEW: do not reset remainingBLocks, but doesn't repeat ram check if don't 
      SET_PRIVATE_MEMBER_VALUE_U32(remainingBlocks_u32, NUM_OF_BLOCKS);
      // Reset test state machine and counters for running the test again      
      SET_PRIVATE_MEMBER_VALUE_U8(nRAMData_u8, 0);
      SET_PRIVATE_MEMBER_VALUE_U8(pRAMData_u8, 0);
      SET_PRIVATE_MEMBER_VALUE_U8(stage_u8, RAM_CHECK_STAGE);
      break;
    } // NEW_BLOCK_STAGE		
    
  case RAM_CHECK_STAGE: // Check one 'block' of RAM, if errors are found, update faultCount_u32. 
    {
      // scan one block of bytes (exit if errors are encountered)
      while( (GET_PUBLIC_MEMBER_POS(errorCode_u32) == ERR_CODE_OK) && (GET_PRIVATE_MEMBER_POS(blockLength_u32) > 0) )
      {
        SET_PRIVATE_MEMBER_VALUE_U8(pRAMData_u8, (*(uint8_t*)(GET_PRIVATE_MEMBER_POS(currentAddr_u32))) );
        //SET_PRIVATE_MEMBER_VALUE_U8(nRAMData_u8, ((uint32_t)*(uint8_t*)(GET_PRIVATE_MEMBER_NEG(currentAddr_u32))) );
        N_SET_PRIVATE_MEMBER_VALUE_U8(nRAMData_u8, (*(uint8_t*)(GET_PRIVATE_MEMBER_POS(currentAddr_u32)+NRAM_OFFSET)) );
        // if calculated value matches the expected result
        if( ((GET_PRIVATE_MEMBER_POS(pRAMData_u8)) ^ (GET_PRIVATE_MEMBER_POS(nRAMData_u8))) == (RAM_GOOD) ) 
        {	
          // Prepare to read the next RAM address
          INC_PRIVATE_MEMBER_VALUE(currentAddr_u32);		// next address in block
          DEC_PRIVATE_MEMBER_VALUE(blockLength_u32);		// one fewer address in block to check
        }
        else // values do not match 
        {	
          // Record Test Failure
          INC_PUBLIC_MEMBER_VALUE(faultCount_u32);	// increment the fault counter
          SET_PUBLIC_MEMBER_VALUE_U32(errorCode_u32, RAM_CHECK_STAGE);
          // SET_PUBLIC_MEMBER_VALUE_U32(errorCode_u32, GET_PRIVATE_MEMBER_POS(currentAddr_u32));
          SET_PRIVATE_MEMBER_VALUE_U8(stage_u8, INIT_STAGE);	    // Start the RAM test over
        }
      }
      // Get prepared for the next time this state machine is called
      // - Update block counters
      SET_PRIVATE_MEMBER_VALUE_U32(blockLength_u32, LEN_OF_BLOCK);	// reset block counter, do not reset "current address"  
      // - REVIEW: keep track of block?
      DEC_PRIVATE_MEMBER_VALUE(remainingBlocks_u32);					    // dec remaining number of blocks
      // - Mark successful test if we've scanned all blocks, and no errors were encountered 
      if( (GET_PRIVATE_MEMBER_POS(remainingBlocks_u32) <= 0) && (GET_PUBLIC_MEMBER_POS(errorCode_u32) == ERR_CODE_OK) )
      {
        INC_PUBLIC_MEMBER_VALUE(passCount_u32);					// we have completed one full check of the RAM
        SET_PRIVATE_MEMBER_VALUE_U8(stage_u8, NEW_BLOCK_STAGE);		// reset RAM checker to start again at beginning
      } // REVIEW: Just quit running ram checker because of one failure?
      break;
    } // RAM_CHECK_STAGE
    
  case SLEEP_STAGE:
    {
      // SET_PRIVATE_MEMBER_VALUE_U8(stage_u8, SLEEP_STAGE);
      break;
    } // SLEEP_STAGE
    
  default:
    {
      SET_PRIVATE_MEMBER_VALUE_U8(stage_u8, INIT_STAGE);
      break;
    } // default  
  }
}
