/**
********************************************************************************************************************************************
* @file    adc_check.c 
* @author  Regal Kyle McBrady
* @version V1.0
* @date    11-20-2019
* @brief   The module of the safety core ADC check
* @note    The result will export to the Module data struct
*******************************************************************************************************************************************
*/
#include <stdlib.h> // for abs
#include "adc_check.h"
#include "check_ids.h"
#include "safety_core_ram_structures.h"
#include "shared_rom.h"

// Macro Setup for this Module
#include "safety_core_macros.h"
#undef PUBLIC_STRUCT_POS
#undef PUBLIC_STRUCT_NEG
#undef PRIVATE_STRUCT_POS
#undef PRIVATE_STRUCT_NEG
#define PUBLIC_STRUCT_POS  Safety_RAMStruct_P.adcCheck_public  // Assign Module Struct to Test Macros (Public Positive RAM)
#define PUBLIC_STRUCT_NEG  Safety_RAMStruct_N.adcCheck_public  // Assign Module Struct to Test Macros (Public Negative RAM)
#define PRIVATE_STRUCT_POS Safety_RAMStruct_P.adcCheck_private // Assign Module Struct to Test Macros (Private Positive RAM)
#define PRIVATE_STRUCT_NEG Safety_RAMStruct_N.adcCheck_private // Assign Module Struct to Test Macros (Private Negative RAM)
// REVIEW: ST Motor Control r3_1 shunt configurations, only read in SECTOR5, is this normal, why even have three shunts?

enum //Default APPs/Driver stage template
{ 
  INIT_STAGE,
  V_REF_STAGE,
  CURRENT_STAGE,
  MUX_STAGE,
  SLEEP_STAGE,
  KILL_MODULE = 255 };

#pragma default_function_attributes = @ "meerkat_rom"
void MeerkatCore_ADCCheck_ShuntCurrentCheck(void);
void MeerkatCore_ADCCheck_OverloadCheck(void);
void MeerkatCore_ADCCheck_ClearShuntCurrentBuffers(void);
void MeerkatCore_ADCCheck_IncFaultCounter(uint8_t error_code_u8);

/**
  ********************************************************************************************************************************
  * @brief  MeerkatCore_SupervisorAdcCheck: This function contains a state machine that initializes and executes testing of Safety
  * - related data that is read by the Analog-to-Digital Converter module. 
  * @details 
  * - ADC Values are read from Fixed RAM Locations in the 'Shared RAM Area'.
  * - This test ensures proper value ranges for Temperature and Current Sensors read by the ADC
  * - This test ensures that ADC's Multiplexer is effectively changing channels
  * - This test ensures proper value range for a voltage reference
  ********************************************************************************************************************************
  */
void MeerkatCore_SupervisorAdcCheck(void) {
    // Always run current checks, since data comes in so fast when the motor is running
    MeerkatCore_ADCCheck_ShuntCurrentCheck();
    // REVIEW: How often to run Overload Check? Current runs every iteration.
    MeerkatCore_ADCCheck_OverloadCheck();
  
    // Run other checks in stages
    switch (GET_PRIVATE_MEMBER_POS(stage_u8)) {
        case INIT_STAGE: 
        {
            SET_PUBLIC_MEMBER_VALUE_U16(callRate_u16, ADC_CHECK_CALL_RATE);
            SET_PUBLIC_MEMBER_VALUE_U32(errorCode_u32, 0);
            SET_PRIVATE_MEMBER_VALUE_U8(index_u8,0);
            while (GET_PRIVATE_MEMBER_POS(index_u8) < ADC_MAX_CHECK) {
                SET_PRIVATE_MEMBER_VALUE_U8(hysteresisDelay_u8[GET_PRIVATE_MEMBER_POS(index_u8)], 0);
                INC_PRIVATE_MEMBER_VALUE(index_u8);
            } 
            SET_PRIVATE_MEMBER_VALUE_U16(previousCurrentCounterReading_u16, 0);
            SET_PRIVATE_MEMBER_VALUE_U8(stage_u8, V_REF_STAGE);
            // - Initialize Parameters Used by Current Checks
            MeerkatCore_ADCCheck_ClearShuntCurrentBuffers();
            break;
        } //INITIAL_STAGE


        case V_REF_STAGE: // MUX Test Phase 1: Ensure stability of MUX
        {   
            // next stage unless something fails
             SET_PRIVATE_MEMBER_VALUE_U8(stage_u8, CURRENT_STAGE); 
             
            //check for stagnancy using a typically volatile ADC reading
            SET_PRIVATE_MEMBER_VALUE_U16(presentADCReading_u16, meerkatCore_Vbus_u16); 
            if(GET_PRIVATE_MEMBER_POS(presentADCReading_u16) == GET_PRIVATE_MEMBER_POS(previousADCReading_u16)) { // check for stagnancy
                INC_PRIVATE_MEMBER_VALUE(hysteresisDelay_u8[ADC_STAGNANCY_CHECK]);
            }
            else {
              SET_PRIVATE_MEMBER_VALUE_U8(hysteresisDelay_u8[ADC_STAGNANCY_CHECK],0);
            }     
            // Check for excessive Failures
            if (GET_PRIVATE_MEMBER_POS(hysteresisDelay_u8[ADC_STAGNANCY_CHECK]) > (V_REF_HYSTERESIS_LIMIT * 4)) {
              // Record Test Failure
                MeerkatCore_ADCCheck_IncFaultCounter(ADC_STAGNANCY_CHECK);
            }  
             //set comparison variable for future use
            SET_PRIVATE_MEMBER_VALUE_U16(previousADCReading_u16, GET_PRIVATE_MEMBER_POS(presentADCReading_u16));           

            
            //check to ensure known stable adc reading is in range
            SET_PRIVATE_MEMBER_VALUE_U16(presentADCReading_u16, meerkatCore_Vref_u16);
            if(GET_PRIVATE_MEMBER_POS(presentADCReading_u16) < V_REF_LIMIT_MIN){ // check lower limit
                INC_PRIVATE_MEMBER_VALUE(hysteresisDelay_u8[ADC_VREF_CHECK]);
            } else if(GET_PRIVATE_MEMBER_POS(presentADCReading_u16) > V_REF_LIMIT_MAX) { // check upper limit
                INC_PRIVATE_MEMBER_VALUE(hysteresisDelay_u8[ADC_VREF_CHECK]);
            } else {
                DEC_PRIVATE_MEMBER_VALUE(hysteresisDelay_u8[ADC_VREF_CHECK]);
            }           
            // Check for excessive Failures
            if (GET_PRIVATE_MEMBER_POS(hysteresisDelay_u8[ADC_VREF_CHECK]) > V_REF_HYSTERESIS_LIMIT) {
              // Record Test Failure
                MeerkatCore_ADCCheck_IncFaultCounter(ADC_VREF_CHECK);
            }            
            
            break;
        } //V_REF_STAGE

        case CURRENT_STAGE:
        {
            MeerkatCore_ADCCheck_CoherenceCheck();
            break;
        } //CURRENT_STAGE

        case MUX_STAGE:  // MUX Test Phase 2: Voltage Stayed in Range, Current should vary by more than allowable range for Vref
        { 
          if (meerkatCore_ShuntCurrentSampleCounter_u32 >= MeerkatConfig_ADCCheck_MinCurrentSampleSize_u32 && labs(meerkatCore_SpeedRpm_s32) > MIN_SPEED_COHERENCE_CHECK_RPM) {
                // Analog Multiplexer Test: Current readings of a running motor will vary much more than our Voltage Reference.
                // - Therefore we can test the functioning of the multiplexer by comparing the difference of our Current Min's and Max's
                // -- to the maximum allowable difference of the Voltage Reference.
                // --- If the difference is less than that, then we can see that the Multiplexer is not working
                if((uint16_t)(GET_PRIVATE_MEMBER_POS(maxCurrentPhaseU_i16) - GET_PRIVATE_MEMBER_POS(minCurrentPhaseU_i16)) <= V_REF_TORERANCE)
                {   // Potential Failure: Current does not vary enough to signfiy a running motor so check MUX_HYSTERESIS_LIMIT times
                    INC_PRIVATE_MEMBER_VALUE(hysteresisDelay_u8[ADC_MUX_CHECK]); 
                } else {
                    DEC_PRIVATE_MEMBER_VALUE(hysteresisDelay_u8[ADC_MUX_CHECK]);
                }                                       
                // REVIEW: Alternate implementation, store min and max for V_REF.If this happens, when are the saved min and max for V_REF reset?               
            } 
            
            // Record Test Failure
            if (GET_PRIVATE_MEMBER_POS(hysteresisDelay_u8[ADC_MUX_CHECK])>MUX_HYSTERESIS_LIMIT){
                MeerkatCore_ADCCheck_IncFaultCounter(ADC_MUX_CHECK);  
            } else {
                SET_PRIVATE_MEMBER_VALUE_U8(stage_u8, V_REF_STAGE);
            }
            
            //check if all tests passed
            SET_PRIVATE_MEMBER_VALUE_U8(index_u8,0);
            while (GET_PRIVATE_MEMBER_POS(index_u8) < ADC_MAX_CHECK){
                if (GET_PRIVATE_MEMBER_POS(hysteresisDelay_u8[GET_PRIVATE_MEMBER_POS(index_u8)]) > 0 ){
                    break;
                } 
                INC_PRIVATE_MEMBER_VALUE(index_u8);
            } 
            if (GET_PRIVATE_MEMBER_POS(index_u8) == ADC_MAX_CHECK){
                //all hysteresisDelay were 0 so all tests passed
                INC_PUBLIC_MEMBER_VALUE(passCount_u32);
            }
            
            // Clear Coherence Check Trackers (moved until after 'mux check')
            // - Initialize Parameters Used by Current Checks
            MeerkatCore_ADCCheck_ClearShuntCurrentBuffers();
            break;
        } //MUX_STAGE

        case SLEEP_STAGE: {
            // Do nothing, wait for reset
            break;
        } //SLEEP_STAGE

        case KILL_MODULE: {
            // Add note to get rid of compiler warnings (20200511)
            break;
        } //KILL_MODULE

        default: {
            SET_PRIVATE_MEMBER_VALUE_U8(stage_u8, INIT_STAGE);
        }
    }
}

                         
/**
  ********************************************************************************************************************************
  * @brief  MeerkatCore_ADCCheck_CoherenceCheck: This function checks the difference in current reading between
  *          all phases if enough current samples have been provided.
  * @param none
  * @return none
  ********************************************************************************************************************************
  */ 
__root void MeerkatCore_ADCCheck_CoherenceCheck(void) {
    // Output Phase Coherence Check
    // - Requires that we've collected at least one full current cycle of data.
    // -- This is guaranteed by requiring a 'Minimum Speed' and a 'Minimum Number of Collected Samples' before running this test
  
   if (meerkatCore_ShuntCurrentSampleCounter_u32 < MeerkatConfig_ADCCheck_MinCurrentSampleSize_u32 || labs(meerkatCore_SpeedRpm_s32) < MIN_SPEED_COHERENCE_CHECK_RPM ){
        // REVIEW: Should we look at if motor state = ON and have a hysteresis for that?
        // Go back to V_REF Stage, can't do mux stage without current readings
        SET_PRIVATE_MEMBER_VALUE_U8(stage_u8, V_REF_STAGE);             
        return;
    }

    // Perform Coherence Check
    // - Compare Max Current Values for each phase
    if (abs(GET_PRIVATE_MEMBER_POS(maxCurrentPhaseU_i16) - GET_PRIVATE_MEMBER_POS(maxCurrentPhaseV_i16)) > CURRENT_TOLERANCE) {
        INC_PRIVATE_MEMBER_VALUE(hysteresisDelay_u8[ADC_CURRENT_CHECK]); // Max: Phase U vs V Failure
    } else if (abs(GET_PRIVATE_MEMBER_POS(maxCurrentPhaseU_i16) - GET_PRIVATE_MEMBER_POS(maxCurrentPhaseW_i16)) > CURRENT_TOLERANCE) {
        INC_PRIVATE_MEMBER_VALUE(hysteresisDelay_u8[ADC_CURRENT_CHECK]); // Max: Phase U vs W Failure
    } else if (abs(GET_PRIVATE_MEMBER_POS(maxCurrentPhaseV_i16) - GET_PRIVATE_MEMBER_POS(maxCurrentPhaseW_i16)) > CURRENT_TOLERANCE) {
        INC_PRIVATE_MEMBER_VALUE(hysteresisDelay_u8[ADC_CURRENT_CHECK]); // Max: Phase V vs W Failure
    } 
    // - Compare Min Current Values for each phase
    else if (abs(GET_PRIVATE_MEMBER_POS(minCurrentPhaseU_i16) - GET_PRIVATE_MEMBER_POS(minCurrentPhaseV_i16)) > CURRENT_TOLERANCE) {
        INC_PRIVATE_MEMBER_VALUE(hysteresisDelay_u8[ADC_CURRENT_CHECK]); // Min: Phase U vs V Failure
    } else if (abs(GET_PRIVATE_MEMBER_POS(minCurrentPhaseU_i16) - GET_PRIVATE_MEMBER_POS(minCurrentPhaseW_i16)) > CURRENT_TOLERANCE) {
        INC_PRIVATE_MEMBER_VALUE(hysteresisDelay_u8[ADC_CURRENT_CHECK]); // Min: Phase U vs W Failure
    } else if (abs(GET_PRIVATE_MEMBER_POS(minCurrentPhaseV_i16) - GET_PRIVATE_MEMBER_POS(minCurrentPhaseW_i16)) > CURRENT_TOLERANCE) {
        INC_PRIVATE_MEMBER_VALUE(hysteresisDelay_u8[ADC_CURRENT_CHECK]); // Min: Phase V vs W Failure
    } 
    //no failures on this pass
    else {
        DEC_PRIVATE_MEMBER_VALUE(hysteresisDelay_u8[ADC_CURRENT_CHECK]);
    }

    //Check for and record excessive faults
    if(GET_PRIVATE_MEMBER_POS(hysteresisDelay_u8[ADC_CURRENT_CHECK]) > COHERENCE_HYSTERESIS_LIMIT) {
        // Record Test Failure
        MeerkatCore_ADCCheck_IncFaultCounter(ADC_CURRENT_CHECK);                  
    } 
    
    //Move on if iterative pass counts exceed combined fail counts
    if(GET_PRIVATE_MEMBER_POS(hysteresisDelay_u8[ADC_CURRENT_CHECK]) == 0) {
        SET_PRIVATE_MEMBER_VALUE_U8(stage_u8, MUX_STAGE);                  
    }  
}

/**
  ********************************************************************************************************************************
  * @brief  MeerkatCore_ADCCheck_ClearShuntCurrentBuffers: This function sets the min max current such that any current is loaded
  *                                                        when new values are sent.
  * @param none
  * @return none
  ********************************************************************************************************************************
  */  
__root void MeerkatCore_ADCCheck_ClearShuntCurrentBuffers(void) {
    meerkatCore_ShuntCurrentSampleCounter_u32 = 0;
    SET_PRIVATE_MEMBER_VALUE_I16(minCurrentPhaseU_i16, INT16_MAX);
    SET_PRIVATE_MEMBER_VALUE_I16(maxCurrentPhaseU_i16, INT16_MIN);
    SET_PRIVATE_MEMBER_VALUE_I16(minCurrentPhaseV_i16, INT16_MAX);
    SET_PRIVATE_MEMBER_VALUE_I16(maxCurrentPhaseV_i16, INT16_MIN);
    SET_PRIVATE_MEMBER_VALUE_I16(minCurrentPhaseW_i16, INT16_MAX); 
    SET_PRIVATE_MEMBER_VALUE_I16(maxCurrentPhaseW_i16, INT16_MIN);    
}

/**
  ********************************************************************************************************************************
  * @brief  MeerkatCore_AddShuntCurrentSample: This function saves parameters to private variable list in ADC counts
  * @param phase_current_ia phase A current
  * @param phase_current_ib phase B current
  * @param phase_current_ic phase C current
  * @return none
  ********************************************************************************************************************************
  */            

__root void MeerkatCore_AddShuntCurrentSample(int16_t phase_current_ia, int16_t phase_current_ib, int16_t phase_current_ic) {
    meerkatCore_ShuntCurrentSampleCounter_u32 += 1;
    // --- U
    if(phase_current_ia > GET_PRIVATE_MEMBER_POS(maxCurrentPhaseU_i16)) {
        SET_PRIVATE_MEMBER_VALUE_I16(maxCurrentPhaseU_i16, phase_current_ia);
    } 
    if (phase_current_ia < GET_PRIVATE_MEMBER_POS(minCurrentPhaseU_i16)) {
        SET_PRIVATE_MEMBER_VALUE_I16(minCurrentPhaseU_i16, phase_current_ia);
    }
    // --- V
    if(phase_current_ib > GET_PRIVATE_MEMBER_POS(maxCurrentPhaseV_i16)) {
        SET_PRIVATE_MEMBER_VALUE_I16(maxCurrentPhaseV_i16, phase_current_ib);
    } 
    if (phase_current_ib < GET_PRIVATE_MEMBER_POS(minCurrentPhaseV_i16)) {
        SET_PRIVATE_MEMBER_VALUE_I16(minCurrentPhaseV_i16, phase_current_ib);
    }
    // --- W
    if(phase_current_ic > GET_PRIVATE_MEMBER_POS(maxCurrentPhaseW_i16)) {
        SET_PRIVATE_MEMBER_VALUE_I16(maxCurrentPhaseW_i16, phase_current_ic);
    } 
    if (phase_current_ic < GET_PRIVATE_MEMBER_POS(minCurrentPhaseW_i16)) {
        SET_PRIVATE_MEMBER_VALUE_I16(minCurrentPhaseW_i16, phase_current_ic);
    }

}


/**
  ********************************************************************************************************************************
  * @brief  MeerkatCore_ADCCheck_ShuntCurrentCheck: This function will monitor current for a minimum of one current cycle
  * @param none
  * @return none
  * @details - Ts = ( 1 / ( SPEED_RPM / 60 ) ) / (2*POLE_PAIRS)
  *          - at a minimum speed of 400 RPM, and 4 pole pairs, the period would be 15ms
  *          - at a speed of 3000 RPM, and 4 pole pairs, the period would be 2ms
  ********************************************************************************************************************************
  */                           
void MeerkatCore_ADCCheck_ShuntCurrentCheck(void) { 
    // Check for current limits
  SET_PRIVATE_MEMBER_VALUE_I16(maxCurrent_i16, (int16_t)( (CURRENT_SPEED_SLOPE * (float)labs(meerkatCore_SpeedRpm_s32)  ) + CURRENT_SPEED_B ));
  meerkatCore_TestValue1_i32 = GET_PRIVATE_MEMBER_POS(maxCurrent_i16);
  meerkatCore_TestValue2_i32 = (GET_PRIVATE_MEMBER_POS(maxCurrentPhaseU_i16) + GET_PRIVATE_MEMBER_POS(maxCurrentPhaseV_i16) + GET_PRIVATE_MEMBER_POS(maxCurrentPhaseW_i16) )/3;
    //Calculate max current based on speed if speed is in range
  if ( labs(meerkatCore_SpeedRpm_s32)  > MIN_SPEED ){
    SET_PRIVATE_MEMBER_VALUE_I16(minCurrent_i16, GET_PRIVATE_MEMBER_POS(maxCurrent_i16) * -1);
    // - Maximum Current
    if ( GET_PRIVATE_MEMBER_POS(maxCurrentPhaseU_i16) > GET_PRIVATE_MEMBER_POS(maxCurrent_i16)  ||  
                                                 GET_PRIVATE_MEMBER_POS(maxCurrentPhaseV_i16) > GET_PRIVATE_MEMBER_POS(maxCurrent_i16)   ||  
                                                                     GET_PRIVATE_MEMBER_POS(maxCurrentPhaseW_i16) > GET_PRIVATE_MEMBER_POS(maxCurrent_i16)  ) {
        INC_PRIVATE_MEMBER_VALUE(hysteresisDelay_u8[ADC_SHUNT_CURRENT_CHECK]);
        // Record Test Failure
        if (GET_PRIVATE_MEMBER_POS(hysteresisDelay_u8[ADC_SHUNT_CURRENT_CHECK])>SHUNT_CURRENT_HYSTERESIS_LIMIT){
            MeerkatCore_ADCCheck_IncFaultCounter(ADC_SHUNT_CURRENT_CHECK);  //REVIEW Is it necessary to distinguish between over current and under current faults?
        }
    }
    // - Minimum Current
    else if ( GET_PRIVATE_MEMBER_POS(minCurrentPhaseU_i16) < GET_PRIVATE_MEMBER_POS(minCurrent_i16)   ||  
                    GET_PRIVATE_MEMBER_POS(minCurrentPhaseV_i16) < GET_PRIVATE_MEMBER_POS(minCurrent_i16)   ||  
                                  GET_PRIVATE_MEMBER_POS(minCurrentPhaseW_i16) < GET_PRIVATE_MEMBER_POS(minCurrent_i16)  ) {
        INC_PRIVATE_MEMBER_VALUE(hysteresisDelay_u8[ADC_SHUNT_CURRENT_CHECK]);
        // Record Test Failure
        if (GET_PRIVATE_MEMBER_POS(hysteresisDelay_u8[ADC_SHUNT_CURRENT_CHECK])>SHUNT_CURRENT_HYSTERESIS_LIMIT){
            MeerkatCore_ADCCheck_IncFaultCounter(ADC_SHUNT_CURRENT_CHECK | ERROR_FLAG_NEGATIVE_CURRENT);  //REVIEW Is it necessary to distinguish between over current and under current faults?
        }
    }    
    // - Within limits
    else {
        DEC_PRIVATE_MEMBER_VALUE(hysteresisDelay_u8[ADC_SHUNT_CURRENT_CHECK]);
    }
  } else {
    if ( GET_PRIVATE_MEMBER_POS(maxCurrentPhaseU_i16) > MODULE_MAX_CURRENT  ||  
                    GET_PRIVATE_MEMBER_POS(maxCurrentPhaseV_i16) > MODULE_MAX_CURRENT   ||  
                                  GET_PRIVATE_MEMBER_POS(maxCurrentPhaseW_i16) > MODULE_MAX_CURRENT  ) {
      MeerkatCore_ADCCheck_IncFaultCounter(ADC_MAX_STARTUP_CURRENT_CHECK);   
    }
  }  
}



/**
  ********************************************************************************************************************************
  * @brief  MeerkatCore_ADCCheck_OverloadCheck: This function will check for Locked Rotor and No Load (overspeed) Conditions
  * @param none
  * @return none
  ********************************************************************************************************************************
  */                         
void MeerkatCore_ADCCheck_OverloadCheck(void) {
    if (Safety_RAMStruct_P.motorState_u8 <= 0) { 
        return;
    }
    // Locked Rotor Detection: Expected bemf vs observed bemf
    // - when observed bemf is much lower than expected bemf, then the rotor is locked/ stalled
    if (meerkatCore_ExpectedBEMF_s32 > meerkatCore_ObservedBEMF_s32) {
        if (meerkatCore_ExpectedBEMF_s32 - meerkatCore_ObservedBEMF_s32 >= MeerkatConfig_LockedRotorCheck_BemfDiff_u32) {
            INC_PRIVATE_MEMBER_VALUE(hysteresisDelay_u8[ADC_LOCK_ROTOR_CHECK]);
            // Record Test Failure
            if (GET_PRIVATE_MEMBER_POS(hysteresisDelay_u8[ADC_LOCK_ROTOR_CHECK])>LOCKED_ROTOR_HYSTERESIS_LIMIT){
                MeerkatCore_ADCCheck_IncFaultCounter(ADC_LOCK_ROTOR_CHECK);  
            }          
        } else {
            DEC_PRIVATE_MEMBER_VALUE(hysteresisDelay_u8[ADC_LOCK_ROTOR_CHECK]);
        }
    } else {
        DEC_PRIVATE_MEMBER_VALUE(hysteresisDelay_u8[ADC_LOCK_ROTOR_CHECK]);
    }

    // No Load Over Speed Detection: If torque is below limit, and speed is above limit
    // - speed is considered 'out of control'
     if (meerkatCore_MeasuredTorque_s32 < MeerkatConfig_OverSpeedCheck_MinTorque_u32) {
         if (meerkatCore_SpeedRpm_s32 > MeerkatConfig_OverSpeedCheck_MaxSpeed_u32 || meerkatCore_SpeedRpm_s32 < MeerkatConfig_OverSpeedCheck_MinSpeed_u32) {
            INC_PRIVATE_MEMBER_VALUE(hysteresisDelay_u8[ADC_OVER_SPEED_CHECK]);
            // Record Test Failure
            if (GET_PRIVATE_MEMBER_POS(hysteresisDelay_u8[ADC_OVER_SPEED_CHECK])>NO_LOAD_OVER_SPEED_HYSTERESIS_LIMIT){
                MeerkatCore_ADCCheck_IncFaultCounter(ADC_OVER_SPEED_CHECK);  
            }            
        } else {
            DEC_PRIVATE_MEMBER_VALUE(hysteresisDelay_u8[ADC_OVER_SPEED_CHECK]);
        }
     } else {
         DEC_PRIVATE_MEMBER_VALUE(hysteresisDelay_u8[ADC_OVER_SPEED_CHECK]);
     }
}

/**
  ********************************************************************************************************************************
  * @brief  MeerkatCore_ADCCheck_IncFaultCounter: This function contains common code that executes when any test exceeds 
  *         allowable number of failures 
  * @param error_code_u8 Which test failed
  * @return none
  ********************************************************************************************************************************
  */
void MeerkatCore_ADCCheck_IncFaultCounter(uint8_t error_code_u8) {
    INC_PUBLIC_MEMBER_VALUE(faultCount_u32);	// increment the fault counter
    SET_PUBLIC_MEMBER_VALUE_U32(errorCode_u32, error_code_u8);
    SET_PRIVATE_MEMBER_VALUE_U8(stage_u8, SLEEP_STAGE);	   // don't start over, let the failure ride
}


