/**
********************************************************************************************************************************************
* @file    Safety_register_check.c 
* @author  Regal Ian Cannon
* @version V1.0
* @date    11-20-2019
* @brief   The module of the safety core Register check app
* @note    The result will export to the Module data struct
*          System memory pointer of this module => [Safety_RegisterCheck]
*          local static memory pointer => Safety_RegisterCheck_Private
*******************************************************************************************************************************************
*/

#include "check_ids.h"
#include "register_check.h"
#include "safety_core_ram_structures.h"

// Macro Setup for this Module
#include "safety_core_macros.h"
#undef PUBLIC_STRUCT_POS
#undef PUBLIC_STRUCT_NEG
#undef PRIVATE_STRUCT_POS
#undef PRIVATE_STRUCT_NEG
#define PUBLIC_STRUCT_POS  Safety_RAMStruct_P.registerCheck_public  // Assign Module Struct to Test Macros (Public Positive RAM)
#define PUBLIC_STRUCT_NEG  Safety_RAMStruct_N.registerCheck_public  // Assign Module Struct to Test Macros (Public Negative RAM)
#define PRIVATE_STRUCT_POS Safety_RAMStruct_P.registerCheck_private // Assign Module Struct to Test Macros (Private Positive RAM)
#define PRIVATE_STRUCT_NEG Safety_RAMStruct_N.registerCheck_private // Assign Module Struct to Test Macros (Private Negative RAM)


// UL Note: Stated 100ms max for first pass of Register Check

void RunTimeRegTest(void);


enum // Default APPs/Driver stage template
{ INIT_STAGE,
  REGISTER_CHECK_STAGE,
  SLEEP_STAGE,
  KILL_MODULE = 255 };

enum
{
  REG_CHECK_PASS = 0,
  REG_CHECK_FAIL,
};

volatile uint8_t regCheckStatusFail = REG_CHECK_PASS;
 
 extern void STL_RunTimeCPURegCheck(void);

#pragma default_function_attributes = @ "meerkat_rom"
/**
  ********************************************************************************************************************************
  * @brief  MeerkatCore_SupervisorRegisterCheck: Contains a state machine that initializes and executes testing of Registers
  * @details 
  * - The test checks all safety related registers that are not inherently tested in other test routines. 
  ********************************************************************************************************************************
  */
void MeerkatCore_SupervisorRegisterCheck(void) {
    switch (GET_PRIVATE_MEMBER_POS(stage_u8)) {
        case INIT_STAGE: {
            regCheckStatusFail = REG_CHECK_PASS;
            SET_PUBLIC_MEMBER_VALUE_U16(callRate_u16, REGISTER_CHECK_CALL_RATE);
            SET_PRIVATE_MEMBER_VALUE_U8(stage_u8, REGISTER_CHECK_STAGE);
            break;
        }
        
    case REGISTER_CHECK_STAGE:
         {
           //regCheckStatusFail = REG_CHECK_PASS;
           STL_RunTimeCPURegCheck();
           //STL_RunTimeCPURegR0_R3();    
           if (regCheckStatusFail == REG_CHECK_FAIL)
           {
             // Record Test Failure
             INC_PUBLIC_MEMBER_VALUE(faultCount_u32);           // set the fault status
             SET_PUBLIC_MEMBER_VALUE_U32(errorCode_u32, REGISTER_CHECK_STAGE);  // if need to put in the error code
             SET_PRIVATE_MEMBER_VALUE_U8(stage_u8, SLEEP_STAGE); // fault found and do the check again from init             
           } else {
             regCheckStatusFail = REG_CHECK_PASS;
             //SET_PRIVATE_MEMBER_VALUE_U8(hysteresisDelay_u8, 0);
             SET_PRIVATE_MEMBER_VALUE_U8(stage_u8, REGISTER_CHECK_STAGE);
             INC_PUBLIC_MEMBER_VALUE(passCount_u32); // we've completed a pass of all checks in this module
           }
           break; 
         }
         
        case SLEEP_STAGE: {
            // SET_PRIVATE_MEMBER_VALUE_U8(stage_u8, SLEEP_STAGE);
            break;
        } //SLEEP_STAGE
        case KILL_MODULE: {
            // SET_PRIVATE_MEMBER_VALUE_U8(stage_u8, KILL_MODULE);
            break;
        }

        default: {
            SET_PRIVATE_MEMBER_VALUE_U8(stage_u8, INIT_STAGE);
            break;
        }
    }
}

void RegCheckFail(void)
{
   regCheckStatusFail = REG_CHECK_FAIL;
  
}

