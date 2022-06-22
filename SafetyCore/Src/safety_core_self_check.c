/**
  ********************************************************************************************************************************************
  * @file    safety_core_self_check.c 
  * @author  Myron Mychal
  * @version V1.0
  * @date    11-20-2018
  * @brief   Independent self-test for all Meerkat test functions
  * @note    Module for testing Meerkat functionality independent of application; can run without hardware ties
  *******************************************************************************************************************************************
  */
#include	"safety_core_self_check.h"
#include	"safety_core_ram_structures.h"

//Macro Setup for this Module
#include	"safety_core_macros.h"
// Undefine macro names from safety_core_macros.h
#undef	PUBLIC_STRUCT_POS
#undef	PUBLIC_STRUCT_NEG
#undef	PRIVATE_STRUCT_POS
#undef	PRIVATE_STRUCT_NEG
#define	PUBLIC_STRUCT_POS	Safety_RAMStruct_P.meerkatSelftest_public		// Assign Module Struct to Test Macros (Public Positive RAM)
#define	PUBLIC_STRUCT_NEG	Safety_RAMStruct_N.meerkatSelftest_public		// Assign Module Struct to Test Macros (Public Positive RAM)
#define	PRIVATE_STRUCT_POS	Safety_RAMStruct_P.meerkatSelftest_private		// Assign Module Struct to Test Macros (Public Positive RAM)
#define	PRIVATE_STRUCT_NEG	Safety_RAMStruct_N.meerkatSelftest_private		// Assign Module Struct to Test Macros (Public Positive RAM)

extern	Safety_RAMStruct_OTYP	Safety_RAMStruct_P;
extern	Safety_RAMStruct_OTYP	Safety_RAMStruct_N;

enum  //Default APPs/Driver stage template
{
  INIT_STAGE,
  MEERKAT_SELFTEST_STAGE,
  SLEEP_STAGE,
  KILL_MODULE = 255
};

/**
  ********************************************************************************************************************************
  * @brief  Supervisor_Module_MeerkatSelftest: Contains a state machine that initializes and executes testing of Meerkat 'Self'
  * @details 
  * - This module is currently not-implemented, and it's necessity has not been verified.
  ********************************************************************************************************************************
  */
void Supervisor_Module_MeerkatSelftest(void)
{  
  switch (GET_PRIVATE_MEMBER_POS(stage_u8))
  {
  	case INIT_STAGE:
		{
			// public structure assignments
			SET_PUBLIC_MEMBER_VALUE_U8(moduleID_u8, MEERKAT_SELFTEST_ID);
			SET_PUBLIC_MEMBER_VALUE_U16(callRate_u16, MEERKAT_SELFTEST_CALL_RATE);
			SET_PUBLIC_MEMBER_VALUE_U32(faultCount_u32, 0);   
			SET_PUBLIC_MEMBER_VALUE_U32(passCount_u32, 0);
			SET_PUBLIC_MEMBER_VALUE_U32(errorCode_u32, ERR_CODE_OK);

			// private structure assignments			
			SET_PRIVATE_MEMBER_VALUE_U32(index_u32, 0);
			break;
		} // INIT_STAGE
		
	case MEERKAT_SELFTEST_STAGE:
		{
			break;
		} // MEERKAT_SELFTEST_STAGE
	
	case SLEEP_STAGE:
    	{
      		// SET_PRIVATE_MEMBER_VALUE_U8(stage_u8, SLEEP_STAGE);
      		break;
    	} // SLEEP_STAGE
  
  	default:
    	{
      		SET_PRIVATE_MEMBER_VALUE_U8(stage_u8, INIT_STAGE);
			break;
    	}    
  }
}

void	Supervisor_Module_PopulateSharedRAM(void)
{
}