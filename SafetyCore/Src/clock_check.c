/**
  ********************************************************************************************************************************************
  * @file    clock_check.c 
  * @author  Myron Mychal
  * @version V1.0
  * @date    03-09-2020
  * @brief   Check processor clock
  * @note    Module for periodically checking processor clock
  *******************************************************************************************************************************************
  */
#include "check_ids.h"
#include "clock_check.h"
#include "safety_core_ram_structures.h"

//Macro Setup for this Module
#include "safety_core_macros.h"
// - Undefine macro names from safety_core_macros.h
#undef PUBLIC_STRUCT_POS
#undef PUBLIC_STRUCT_NEG
#undef PRIVATE_STRUCT_POS
#undef PRIVATE_STRUCT_NEG
// Redefine macro names based on structures in this file.
// - These are required for the shadow ram writing macros to function. (ASSIGN*, GET_PRIVATE_MEMBER_POS, etc.)
#define PUBLIC_STRUCT_POS  Safety_RAMStruct_P.clockCheck_public  // Assign Module Struct to Test Macros (Public Positive RAM)
#define PUBLIC_STRUCT_NEG  Safety_RAMStruct_N.clockCheck_public  // Assign Module Struct to Test Macros (Public Negative RAM)
#define PRIVATE_STRUCT_POS Safety_RAMStruct_P.clockCheck_private // Assign Module Struct to Test Macros (Private Positive RAM)
#define PRIVATE_STRUCT_NEG Safety_RAMStruct_N.clockCheck_private // Assign Module Struct to Test Macros (Private Negative RAM)

//extern	Safety_RAMStruct_OTYP	Safety_RAMStruct_P;
//extern	Safety_RAMStruct_OTYP	Safety_RAMStruct_N;

// Enumerate Test State Machine
enum //Default APPs/Driver stage template
{ INIT_STAGE,
  CLOCK_CHECK_STAGE,
  CLOCK_HYSTERSIS_STAGE,
  REPORT_STATUS_STAGE,

  SLEEP_STAGE,
  KILL_MODULE = 255 };


// Functions
// - Assign these functions to be Linked to the 'Meerkat' ROM Area
#pragma default_function_attributes = @ "meerkat_rom"
void MeerkatCore_ClockCheck_IncFaultCounter(uint8_t error_code_u8);

/**
  ********************************************************************************************************************************
  * @brief  MeerkatCore_SupervisorClockCheck: This function contains a state machine that initializes and executes testing of ROM  
  * @details 
  * - The first part of the test ensures that 1ms System Tick counter is continually changing.
  * - The second part of the test computes an expected ratio of LSI Ticks vs 1ms System Ticks,
  * -- and compares that to actual clock data stored in shared RAM.
  * - detected errors are stored in the 'faultCount_u32' variable of this test's RAM structure
  ********************************************************************************************************************************
  */

void MeerkatCore_SupervisorClockCheck(void) {
    switch (GET_PRIVATE_MEMBER_POS(stage_u8)) {
        case INIT_STAGE: // Initialize all test related variables.
        {
            // public structure assignments
            // - Initialize module ID and call rate
            SET_PUBLIC_MEMBER_VALUE_U8(moduleID_u8, CLOCK_CHECK_ID);
            SET_PUBLIC_MEMBER_VALUE_U16(callRate_u16, CLOCK_CHECK_CALL_RATE);
            SET_PUBLIC_MEMBER_VALUE_U32(errorCode_u32, 0);

            // private structure assignments
            // - Initialize test structure counters
            SET_PRIVATE_MEMBER_VALUE_U32(runningPassCount_u32, 0);
            SET_PRIVATE_MEMBER_VALUE_U32(previousClockCounts_u32, 0xDEADBEEF);
            SET_PRIVATE_MEMBER_VALUE_U32(currentClockCounts_u32, 0);
            SET_PRIVATE_MEMBER_VALUE_U32(idealClockCounts_u32, IDEAL_LSI_CLOCK_COUNTS);
            SET_PRIVATE_MEMBER_VALUE_U32(maxClockCounts_u32, (uint32_t)(MAX_LSI_CLOCK_COUNTS)); // force threshold to be an integer
            SET_PRIVATE_MEMBER_VALUE_U32(minClockCounts_u32, (uint32_t)(MIN_LSI_CLOCK_COUNTS)); // force threshold to be an integer
            SET_PRIVATE_MEMBER_VALUE_U8(ring_buffer_position_u8, 0);
            SET_PRIVATE_MEMBER_VALUE_U32(ringBufferLSIdelta_u32, 0);
            // - Reset test state machine and counters for running the test again
            SET_PRIVATE_MEMBER_VALUE_U8(hysteresisCount_u8, 0);
            SET_PRIVATE_MEMBER_VALUE_U8(applicationHysteresisCount_u8, 0);
            SET_PRIVATE_MEMBER_VALUE_U8(stage_u8, CLOCK_CHECK_STAGE);
            break;
        } // INIT_STAGE

        case CLOCK_CHECK_STAGE: // Check one 'block' of Clock Data, if errors are found, update faultCount_u32.
        {
            // Clock Check
            // - Setup
            // -- Update currentTime for time hysteresis comparison
            SET_PRIVATE_MEMBER_VALUE_U32(currentClockCounts_u32, meerkatCore_SysTickMs_u32);
            // - Testing
            // -- Test 1 -> Systick Test: Verify that the 1ms System Tick Counter is incrementing periodically.
            if ((GET_PRIVATE_MEMBER_POS(previousClockCounts_u32)) == (GET_PRIVATE_MEMBER_POS(currentClockCounts_u32))) // If value has not changed
            {
                INC_PRIVATE_MEMBER_VALUE(applicationHysteresisCount_u8); 
                // SET_PRIVATE_MEMBER_VALUE_U32(previousClockCounts_u32, GET_PRIVATE_MEMBER_POS(currentClockCounts_u32)); // Removed: Already know these are equal
                if ((GET_PRIVATE_MEMBER_POS(applicationHysteresisCount_u8)) > (CLOCK_CHECK_HYSTERESIS_SYSTICK)) // Declare Error, if does not increment for a long period of time.
                {
                    MeerkatCore_ClockCheck_IncFaultCounter(CLOCK_SYSTIC_CHECK);
                    SET_PRIVATE_MEMBER_VALUE(applicationHysteresisCount_u8,0);
                }
            } else // If value has changed, reset hysteresis counter
            {
                DEC_PRIVATE_MEMBER_VALUE(applicationHysteresisCount_u8); 
                SET_PRIVATE_MEMBER_VALUE_U32(previousClockCounts_u32, GET_PRIVATE_MEMBER_POS(currentClockCounts_u32));
            }

            // -- Test 2 -> LSI Sample Test - calculate the difference between each available sample of LSI Ticks,
            // --- ensure it is in range of expected value
            // ---- note: rollover for currentClockCounts is 49.7 days.
            // ----- rollover merely causes this part of the clock test to be paused for 400ms
            if ((GET_PRIVATE_MEMBER_POS(currentClockCounts_u32)) >
                LSI_RING_BUFFER_SAMPLE_PERIOD_MS * LENGTH_OF_LSI_RING_BUFFER) // Do not run test until sample buffer is full
            {
                for (SET_PRIVATE_MEMBER_VALUE_IN_LOOP(indexN_u32, 0); GET_PRIVATE_MEMBER_POS(indexN_u32) < (NUM_LSI_COUNTS_IN_BUFFER);
                     INC_PRIVATE_MEMBER_VALUE(indexN_u32)) // For each sample of LSI Ticks
                {
                    // Calculate the difference between the last two LSI Clock Tick Samples
                    if ((GET_PRIVATE_MEMBER_POS(indexN_u32)) == 0) // First sample must look at the last reading from the last iteration of this fucntion
                    //  since sample may no longer exist
                    {
                        SET_PRIVATE_MEMBER_VALUE_U32(ringBufferLSIdelta_u32,
                                   (meerkatCore_LSIClockBuffer_u32[GET_PRIVATE_MEMBER_POS(indexN_u32)]) - GET_PRIVATE_MEMBER_POS(lastClockReading_u32));
                    } else {
                        SET_PRIVATE_MEMBER_VALUE_U32(ringBufferLSIdelta_u32,
                                   (meerkatCore_LSIClockBuffer_u32[GET_PRIVATE_MEMBER_POS(indexN_u32)] - meerkatCore_LSIClockBuffer_u32[GET_PRIVATE_MEMBER_POS(indexN_u32) - 1]));
                    }

                    // Check to see if difference is in expected range for the elapsed period of time
                    if ((GET_PRIVATE_MEMBER_POS(ringBufferLSIdelta_u32) > (GET_PRIVATE_MEMBER_POS(maxClockCounts_u32))) ||
                        ((GET_PRIVATE_MEMBER_POS(ringBufferLSIdelta_u32)) < (GET_PRIVATE_MEMBER_POS(minClockCounts_u32)))) {
                        // Check for 'wrap-around' case: in this case we are essentially going backward by 7 iterations
                        // - value will be around (0xFFFFFFFF - ((NUM_LSI_COUNTS_IN_BUFFER-1)*IDEAL_LSI_CLOCK_COUNTS)
                        if (GET_PRIVATE_MEMBER_POS(ringBufferLSIdelta_u32) <
                                0xFFFFFFFF - (NUM_LSI_COUNTS_IN_BUFFER - 1) * (GET_PRIVATE_MEMBER_POS(maxClockCounts_u32)) ||
                            (GET_PRIVATE_MEMBER_POS(ringBufferLSIdelta_u32)) >
                                0xFFFFFFFF - (NUM_LSI_COUNTS_IN_BUFFER - 1) * (GET_PRIVATE_MEMBER_POS(minClockCounts_u32))) {
                            // invalid case: check hysteresis of clock math
                            INC_PRIVATE_MEMBER_VALUE(hysteresisCount_u8);
                            if ((GET_PRIVATE_MEMBER_POS(hysteresisCount_u8)) > (CLOCK_CHECK_HYSTERESIS_LSITICK)) {
                                MeerkatCore_ClockCheck_IncFaultCounter(CLOCK_LIS_CHECK);
                                SET_PRIVATE_MEMBER_VALUE(hysteresisCount_u8,0);
                            }
                        } else // if value is in expected range for a wrap-around case
                        {
                            DEC_PRIVATE_MEMBER_VALUE(hysteresisCount_u8); 
                        }
                    } else // if value is in expected range
                    {
                        DEC_PRIVATE_MEMBER_VALUE(hysteresisCount_u8); 
                    }
                } // end of for loop
            }     // end of LSI Comparison Test

            // Get prepared for the next time this state machine is called
            // - Update Counters/ Stored Variables
            SET_PRIVATE_MEMBER_VALUE_U32(lastClockReading_u32,
                       meerkatCore_LSIClockBuffer_u32[NUM_LSI_COUNTS_IN_BUFFER - 1]); // keep last clock reading for next pass
            ADD_TO_PRIVATE_MEMBER_VALUE(runningPassCount_u32, (NUM_LSI_COUNTS_IN_BUFFER));   // accumulate pass counter
            // - Mark successful test if we've scanned all samples, and no errors were encountered
            if (GET_PRIVATE_MEMBER_POS(runningPassCount_u32) >= (TIME_TO_ACKNOWLEGE_A_PASS))
            // TODO: Don't acknowledge a pass until after LSI Testing begins
            {
                SET_PRIVATE_MEMBER_VALUE_U32(runningPassCount_u32, 0);
                INC_PUBLIC_MEMBER_VALUE(passCount_u32); // we've completed a pass of clock check
            }
            break;
        } // RAM_CHECK_STAGE

        case SLEEP_STAGE: {
            // SET_PRIVATE_MEMBER_VALUE_U8(stage_u8, SLEEP_STAGE);
            break;
        } // SLEEP_STAGE

        default: {
            SET_PRIVATE_MEMBER_VALUE_U8(stage_u8, INIT_STAGE);
            break;
        }
    }
}

/**
  ********************************************************************************************************************************
  * @brief  MeerkatCore_AddClockSample: This function provides access to load tick count data for evaluation 
  * @param lsi_tick_count_u32 current tick count
  * @return none
  ********************************************************************************************************************************
  */
__root void MeerkatCore_AddClockSample(uint32_t lsi_tick_count_u32) {
    meerkatCore_LSIClockBuffer_u32[GET_PRIVATE_MEMBER_POS(ring_buffer_position_u8)] = lsi_tick_count_u32;
    INC_PRIVATE_MEMBER_VALUE(ring_buffer_position_u8); 
    if (PRIVATE_MEMBER_VALUE_IS_GREATER_OR_EQUAL(ring_buffer_position_u8,LENGTH_OF_LSI_RING_BUFFER)) {
        SET_PRIVATE_MEMBER_VALUE_U8(ring_buffer_position_u8,0);
    }
}

/**
  ********************************************************************************************************************************
  * @brief  MeerkatCore_ClockCheck_IncFaultCounter: This function contains common code that executes when any test exceeds 
  *         allowable number of failures 
  * @param error_code_u8 Which test failed
  * @return none
  ********************************************************************************************************************************
  */
void MeerkatCore_ClockCheck_IncFaultCounter(uint8_t error_code_u8) {
    INC_PUBLIC_MEMBER_VALUE(faultCount_u32);                                     // increment the fault counter
    SET_PUBLIC_MEMBER_VALUE_U32(errorCode_u32, error_code_u8); 
    SET_PRIVATE_MEMBER_VALUE_U8(stage_u8, INIT_STAGE);
}

