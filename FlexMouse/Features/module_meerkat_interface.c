/**
  ********************************************************************************************************************************
  * @file    module_meerkat_interface.c 
  * @author  Justin Moon
  * @brief   Main functions for running the Meerkat Safety Core
  * @details Get processor status from safety core firmware, responds to any detected errors.
  ********************************************************************************************************************************
  */

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "stm32f3xx_ll_iwdg.h" // IWDG Reload (only used when Security Core is disabled)
#include "driver_rtc.h" // This module is dependent on rtc module set to LSI clock source
#include "main.h" // !review: just for hrtc and because it imports stm32f3xx_ll_rcc
#include "mc_api.h" // used by Risk Addressed State and ismotor Running REVIEW: Remove this dependency if possible
#include "module_meerkat_interface.h" 
#include "module_meerkat_interface_config.h"
#include "ui_task.h" // !error Temporary for bemf readings and locked rotor detection
#include "module_meerkat_shared_rom.h" // !error temporary for bemf detection prototyping
#include "regular_conversion_manager.h" 
#include "parameters_conversion.h"
#include "drive_parameters.h"

// Meerkat Inteface Description
// - INITIALIZATION
// -- As soon as processor enters main function, MeerkatInterface_StartupInit should be called.
// --- It must be called at least once before MeerkatInterface_RunStateMachine is run.
// --- Beyond that the RunStateMachine will handle other Initialization like processes.

// - RUN
// -- Inteface Module should be run like any other Flexmouse module. This module calls 'Run' of Safety Core Checks.
// --- "Safety Core Run" will return a status value. 
// --- If Status Value is non-zero,
// ---- Meerkat Interface should call a 'Risk Addressed State Set Callback'
// ---- If risk addressed state was not the previously known state, special actions can be triggered.
// ----- This function should check the meerkatCore_ResetPending flag and trigger a 'Risk Addressed Set Will Reset Callback'

// --- If Status Value is zero
// ---- Meerkat Interface should call a 'Risk Addressed State Cleared Callback'
// ---- If 0 was not the previous state, then 


/* Content ---------------------------------------------------------------------------------------------------------------------*/
#define MEERKAT_FAULT_CODE_NO_FAULTS 0
#define MEERKAT_FAILED_TEST_ID_NONE -1

#define RISK_ADDR_STATE_OFF 0
#define RISK_ADDR_STATE_ON 1
#define RISK_ADDR_STATE_RESET_PENDING 2

extern MCI_Handle_t *pMCI[1];
extern MCT_Handle_t *pMCT[1];
extern uint32_t wConfig[1];

// Test Related Data Storage
volatile bool meerkatInterface_Ready = false; // Limits calls to MeerkatCore_SupervisorRun to 1/ms. volatile: changes value in systick interrupt.
volatile uint8_t meerkatInterface_RiskAddrState = RISK_ADDR_STATE_OFF; // local copy of the last known risk addressed state
int meerkatInterface_FailedTestId = MEERKAT_FAILED_TEST_ID_NONE; // -1 = no failed test, 0=adc_check, etc...
int meerkatInterface_FailedTestCode = 0; // Varies by test
volatile uint64_t meerkatInterface_FaultCode = MEERKAT_FAULT_CODE_NO_FAULTS;
uint32_t meerkatInterface_ClockSampleBuffer_u32 = NO_CLOCK_SAMPLES; // Clock Check: Clock update may not update Meerkat Shared RAM during interrupt.

// - This buffer allows us to hold one sample and patch it in during 'RUN_MODULE'
static uint8_t meerkatInterface_ClockSampleCounter_u8 = 0; // used to ensure LSI Readings are only recorded every 100ms
static uint32_t meerkatInterface_TimeMs_u32 = 0; // Meerkat's perceived running time in milliseconds

//This is the handle for the VREF ADC conversion and the structure used to pass data
uint8_t vref_adc_callback_handle_u8 = 255;
RegConv_t VrefRegConv = {.regADC = ADC1,.channel = MC_ADC_CHANNEL_18,.samplingTime = LL_ADC_SAMPLINGTIME_7CYCLES_5};

// ======= Main State Machine =START=
// Local stage(state) name define in enum
enum { // Default APPs/Driver stage template
    INITIALIZE_MODULE,
    RUN_MODULE,
    // any other stage in here !!!

    // above 200 will be all interrupt for this APP
    INTERRUPT_MODULE = 200,
    KILL_MODULE = 255
};

/**
  ********************************************************************************************************************************
  * @brief  MeerkatInterface_RunStateMachine: State machine for triggering Safety Core Routines
  * @details 
  * - This function is designed to be called by FlexMouse 'scheduler'
  * - Meerkat safety core routines are located in an isolated firmware partition
  * - Meerkat functions are accessed by address
  ********************************************************************************************************************************
  */
uint8_t MeerkatInterface_RunStateMachine(uint8_t driver_identifier_u8, uint8_t previous_state_u8, uint8_t next_state_u8, uint8_t interrupt_identifier_u8) {
     //HAL_IWDG_Refresh(&hiwdg);//put counter to zero to not reach 6553,6 ms for 64 prescaler (Moved to Meerkat's 'MeerkatCore_SupervisorRun')
#if DISABLE_MEERKAT_SAFETY_MODULE == 0
    uint8_t return_stage_u8 = INITIALIZE_MODULE;
    switch (next_state_u8) {
        case INITIALIZE_MODULE: { // initialization stage - prepare tests
#if DISABLE_MEERKAT_SAFETY_MODULE_INIT == 0
            MeerkatCore_SupervisorInit(); // Core
            if (meerkatInterface_FaultCode == 0 && !meerkatInterface_RiskAddrState) {
#if DISABLE_MEERKAT_AUTO_START == 0
           //    MeerkatInterface_RestartMotor();
#endif // DISABLE_MEERKAT_AUTO_START == 0
            }
#endif //DISABLE_MEERKAT_SAFETY_MODULE_INIT == 0
            
            //register VREF ADC callback 
            vref_adc_callback_handle_u8 = RCM_RegisterRegConv(&VrefRegConv);
            if ( vref_adc_callback_handle_u8 > 4 /*RCM_MAX_CONV*/ ){
              return_stage_u8 = KILL_MODULE;
            } 
            return_stage_u8 = RUN_MODULE;
            break;
        }
        case RUN_MODULE: { 
            // main stage - run tests
            return_stage_u8 = MeerkatInterface_RunTests();
            break;
        }
        case INTERRUPT_MODULE: {
            return_stage_u8 = RUN_MODULE;
            break;
        }
        case KILL_MODULE: {
            return_stage_u8 = INITIALIZE_MODULE;
            break;
        }
        default: {
            return_stage_u8 = KILL_MODULE;
            break;
        }
    }
    return return_stage_u8;
    #else // DISABLE_MEERKAT_SAFETY_MODULE > 0
    LL_IWDG_ReloadCounter(IWDG); // Kick the watchdog
    return RUN_MODULE;
    #endif // DISABLE_MEERKAT_SAFETY_MODULE > 0
}
// ======= Main State Machine ==END==

// ======= Safety Interface - Run Test =START=
/**
  ********************************************************************************************************************************
  * @brief  MeerkatInterface_RunTests triggers single execution of the Safety Core's Run routine.
  * @details 
  * - This function looks at an 'enable' flag so that it is called a maximum of once per ms.
  ********************************************************************************************************************************
  */
uint8_t MeerkatInterface_RunTests(void) {
    // TODO: Tests Passed - Add Monitoring of both the 'Tests Passed' and the 'Tests Failed' here.
    // - Add this capability to our monitoring software.
   //get the Vref
    RCM_UserConvState_t Udrc_state = RCM_GetUserConvState();
    if ( Udrc_state == RCM_USERCONV_IDLE ){
      RCM_RequestUserConv(vref_adc_callback_handle_u8);
    } else if ( Udrc_state == RCM_USERCONV_EOC ){
      uint16_t adc_vref_u16 = RCM_GetUserConv();
      if ( adc_vref_u16 != 0xFFFF){
       //make left justified data from ADC into actual value by shifting right 4
       meerkatCore_Vref_u16 = adc_vref_u16 >> 4;
      }   
    }     
    
    //execute tests every millisecond
    if (meerkatInterface_Ready) {
        meerkatInterface_Ready = false; // reset the flag to ensure this isn't called more than once per millisecond

        // Determine if the motor is running and pass the value to the safety core
        volatile uint8_t motor_is_running; 
        motor_is_running = MeerkatInterface_MotorIsRunning();

        // Debug: Safety Core Timing by GPIO Tracking =START=
        // #if ENABLE_TIMING_DEBUG_PIN >= 1
        // LL_GPIO_SetOutputPin(TIMING_DEBUG_PORT, TIMING_DEBUG_PIN);
        // #endif //ENABLE_TIMING_DEBUG_PIN >= 1
        // Debug: Safety Core Timing by GPIO Tracking ==END==

        // Determine motor speed+torque and pass the values to the safety core
        int32_t speed_rpm = (int32_t)((MC_GetMecSpeedAverageMotor1() * _RPM)/SPEED_UNIT);
        MeerkatInterface_AddMeasuredSpeedSample(speed_rpm);
        if (motor_is_running) {
            MeerkatInterface_AddBemfTorqueSamples();
        } else {
            // REVIEW: Should we add zero samples when the motor is stopped?
        }

        // Run the Safety Core (single iteration)
        meerkatInterface_FaultCode = MeerkatCore_SupervisorRun(motor_is_running);
        //LL_IWDG_ReloadCounter(IWDG);       
        // Debug: Safety Core Timing by GPIO Tracking =START=
        // #if ENABLE_TIMING_DEBUG_PIN >= 1
        // LL_GPIO_ResetOutputPin(TIMING_DEBUG_PORT, TIMING_DEBUG_PIN);
        // #endif //ENABLE_TIMING_DEBUG_PIN >= 1
        // Debug: Safety Core Timing by GPIO Tracking ==END==
        
        // Check Safety Core Status
        // - Note: Safety Core will repeatedly Shut-down PWM by itself if a Safety Related Fault is identified.
        // -- You may perform additional actions here, if desired.
        if (meerkatInterface_FaultCode > 0) { // || meerkatInterface_RiskAddrState) { // If a safety related fault was detected
            MeerkatInterface_RiskAddrStateSetCallback(); // perform additional actions
            meerkatInterface_FailedTestId =  meerkatInterface_FaultCode >> MEERKAT_STATUS_TEST_ID_SHIFT;
            meerkatInterface_FailedTestCode = meerkatInterface_FaultCode & MEERKAT_STATUS_TEST_ERROR_CODE_FLAGS;
            return INTERRUPT_MODULE; // REVIEW INTERRUPT can have other ramifications
        } else {
            MeerkatInterface_RiskAddrStateClearedCallback();
            return RUN_MODULE;
        }
    } else {
        return RUN_MODULE;            
    }
}
// ======= Safety Interface - Run Test ==END==


// ======= Safety Interface - Local Response To Test Results =START=
/**
  ********************************************************************************************************************************
  * @brief  MeerkatInterface_RiskAddrStateSetCallback allows local response to errors reported by Safety Core
  * @details 
  ********************************************************************************************************************************
  */
void MeerkatInterface_RiskAddrStateSetCallback(void) {  
    // Safety Core has already shut down the PWM. 
    // - The Safety core manages resetting it's own faults
    // -- This routine is only for application specific responses.

    // Store Risk Addressed State Value so that we can detected when it is cleared
    meerkatInterface_RiskAddrState = RISK_ADDR_STATE_ON; // TODO: Magic number

    // Tell the Motor Control State Machine to stop, so that it is not out of sync when the fault is released.
    MC_StopMotor1(); // - tell motor control system to stop trying to turn the motor (pwm is already disabled so it is futile)
}

/**
  ********************************************************************************************************************************
  * @brief  MeerkatInterface_RiskAddrStateSetCallback allows local response to when Safety Core errors are cleared.
  * @details 
  ********************************************************************************************************************************
  */
void MeerkatInterface_RiskAddrStateClearedCallback(void) {
    if (meerkatInterface_RiskAddrState > RISK_ADDR_STATE_OFF) { // Risk Addressed State was just cleared.
        meerkatInterface_FaultCode = 0;
        meerkatInterface_RiskAddrState = RISK_ADDR_STATE_OFF; // Clear the local copy of RiskAddressedState
        meerkatInterface_FailedTestId = -1;
        meerkatInterface_FailedTestCode = 0;
        meerkatInterface_ClockSampleCounter_u8 = 0;
        meerkatInterface_TimeMs_u32 = 0;
        // Run single time operations here, if desired.
        #if DISABLE_MEERKAT_AUTO_START == 0
            // REVIEW: Could add 'Advanced Retry' - (signal Restart of Motor by 'modular' compatible means)
            MeerkatInterface_RestartMotor();
        #endif // DISABLE_MEERKAT_AUTO_START == 0
    } else { // Risk Addressed State is off but was already off  
        // Run recurring operations here, if desired
    }
}
// ======= Safety Interface - Local Response To Test Results ==END==

// ======= Safety Interface - Interrupt Service Routine Extensions =START=
/**
  ********************************************************************************************************************************
  * @brief  MeerkatInterface_SysTickCallback should be called via interrupt every 1ms (from SysTick Callback)
  * @details 
  * - This function calculated and stores the data required by the Meerkat Safety Core Clock Check
  * - It is responsible for maintaining an LSI Tick Counter that is compared to the standard SysTick Counter (HSI/HSE) in 
  * - safety core clock checks.
  ********************************************************************************************************************************
  */
void MeerkatInterface_SysTickCallback(void) { // Clock Data Input
    // Meerkat Clock Check: SysTick/ Running Time
    meerkatInterface_Ready = true; // allow calls to MeerkatCore_SupervisorRun
    // - Record SysTick to Shared RAM
    meerkatInterface_TimeMs_u32 += 1;         // - Update System Time Locally
    meerkatCore_SysTickMs_u32 = meerkatInterface_TimeMs_u32;   // - Record System Time to shared ram for use with meerkat clock check
    
    // Meerkat Clock Check
    // - FlexMouse Application Side -> Keep Track of SysTick, Record LSI clock ticks and SysTick to Shared RAM
    // -- accumulate LSI clock ticks over 100 ms before storing them into one byte of the shared RAM ring buffer
    // --- LSI clock tick counter should accumulate, so it continually increases {4000, 8000, 12000, 16000, 20000, 24000, 28000, 32000}
    // ---- Meerkat will handle overflow cases .
    // -- with LSI frequency of 40kHz, LSI period = 1/40000 = 0.000025 sec;
    // --- Safety core expects each LSI sample to have around 4000 counts.
    #if DISABLE_MEERKAT_SAFETY_MODULE == 0
    // Store Sample in Buffer (samples blocked from write in a previous call to this function)
    if (meerkatInterface_ClockSampleBuffer_u32 != NO_CLOCK_SAMPLES) {
        if (meerkatCore_ActiveTest_u32 != MEERKAT_TEST_ID_CLOCKCHECK) { // do not write to Clock Data RAM while Clock Test is Running
            // write completed sum to RAM
            MeerkatCore_AddClockSample(meerkatInterface_ClockSampleBuffer_u32);
            meerkatInterface_ClockSampleBuffer_u32 = NO_CLOCK_SAMPLES; // clear the buffer
            // do not reset active buffer, that was done earlier in the 'Send the current sample' routine below
        }
    }

    // Store New Sample (every 100ms)
    if (++meerkatInterface_ClockSampleCounter_u8 >= MEERKAT_CLOCK_TEST_SAMPLE_TIME_MS) { // Time to record a sample
        uint32_t this_sample_u32 = RTC_UpdateLsiTickCounter(); // get lsi tick value as a u32
        meerkatInterface_ClockSampleCounter_u8 = 0; // reset sample counter
        if (meerkatCore_ActiveTest_u32 != MEERKAT_TEST_ID_CLOCKCHECK) { // Case: Buffer is Free, can write now
            // write completed sum to RAM
            MeerkatCore_AddClockSample(this_sample_u32);
            meerkatInterface_ClockSampleBuffer_u32 = NO_CLOCK_SAMPLES; // clear the buffer
        } else { // Case: Buffer may be busy because Clock Check security routine is currently active
                // do not write to Meerkat's Clock Data RAM while Clock Test is Running
                // - doing so could cause faults in the RAM Test due to incorrectly written shadow RAM.
                meerkatInterface_ClockSampleBuffer_u32 = this_sample_u32; // add to buffer for writing later        
        }
    }
    #else // DISABLE_MEERKAT_SAFETY_MODULE > 0
    return; // KILL_MODULE;
    #endif // DISABLE_MEERKAT_SAFETY_MODULE > 0
}
// ======= Safety Interface - Interrupt Service Routine Extensions ==END==


// ======= Safety Interface - Data Input to Core =START=

/**
  ********************************************************************************************************************************
  * @brief  MeerkatInterface_AddShuntCurrentSample is a simple function to allow the Application firmware to send current data to the 
  * - Meerkat Safety Core.
  * @details 
  * - This function is designed to be called whenever currents are read from the ADC Value Register
  * - In ST Motor Control SDK, it is part of the 'High Frequency Task Callback', which is triggered via the PWM Interrupt
  ********************************************************************************************************************************
  */
void MeerkatInterface_AddShuntCurrentSample(int16_t phase_current_ia, int16_t phase_current_ib, int16_t phase_current_ic ) {
#if DISABLE_MEERKAT_SAFETY_MODULE >= 1
    return;
#endif //DISABLE_MEERKAT_SAFETY_MODULE >= 1
    // Meerkat ADC Check: Shunt Currents
    // - Note this is called once every pwm cycle, so if pwm is 30kHz, this is called every 33us
    #if ENABLE_TIMING_DEBUG_PIN >= 1  
    LL_GPIO_SetOutputPin(TIMING_DEBUG_PORT, TIMING_DEBUG_PIN);
    #endif //ENABLE_TIMING_DEBUG_PIN >= 1  
   // MeerkatCore_AddShuntCurrentSample(phase_current_ia, phase_current_ib, phase_current_ic);
    #if ENABLE_TIMING_DEBUG_PIN >= 1  
    LL_GPIO_ResetOutputPin(TIMING_DEBUG_PORT, TIMING_DEBUG_PIN);
    #endif //ENABLE_TIMING_DEBUG_PIN >= 1
    return;

}

/**
  ********************************************************************************************************************************
  * @brief  MeerkatInterface_AddTemperature1Sample is a simple function to allow the Application firmware to send temperature 
  * - data to the Meerkat Safety Core.
  * @details 
  * - This function is designed to be called whenever temperature is read from the ADC Value Register
  ********************************************************************************************************************************
  */
void MeerkatInterface_AddTemperature1Sample(uint16_t adc_value) {
    meerkatCore_TemperatureValue1_u16 = adc_value;
}

/**
  ********************************************************************************************************************************
  * @brief  MeerkatInterface_AddTemperature2Sample is a simple function to allow the Application firmware to send temperature 
  * - data from a second temperature sensor to the Meerkat Safety Core.
  * @details 
  * - This function is designed to be called whenever temperature is read from the ADC Value Register
  ********************************************************************************************************************************
  */
void MeerkatInterface_AddTemperature2Sample(uint16_t adc_value) {
    meerkatCore_TemperatureValue2_u16 = adc_value;
}

/**
  ********************************************************************************************************************************
  * @brief  MeerkatInterface_AddVrefSample is a simple function to allow the Application firmware to send reference voltage 
  * - data to the Meerkat Safety Core.
  * @details 
  * - The voltage reference input is an integral part of the 'Multiplexer' check.
  ********************************************************************************************************************************
  */
void MeerkatInterface_AddVrefSample(uint16_t adc_value) {
    meerkatCore_Vref_u16 = adc_value; //meerkatCore_Vbus_u16;// 
}

/**
  ********************************************************************************************************************************
  * @brief  MeerkatInterface_AddMeasuredSpeedSample is a simple function to allow the Application firmware to send  
  * - the current measured speed in RPMto the Meerkat Safety Core .
  * @details 
  * - The voltage reference input is an integral part of the 'Multiplexer' check.
  ********************************************************************************************************************************
  */
void MeerkatInterface_AddMeasuredSpeedSample(int32_t speed_rpm) {
    meerkatCore_SpeedRpm_s32 = speed_rpm;  
}

void MeerkatInterface_AddBemfTorqueSamples(void) { // LockedRotor/ NoLoad detection 
    // Locked Rotor Detection: Expected bemf vs observed bemf
    // - when observed bemf is much lower than expected bemf, then the rotor is locked
    // - Get values from Motor Control System
    MCP_Handle_t* pHandle = GetMCP();
    meerkatCore_ObservedBEMF_s32 = UI_GetReg( &pHandle->_Super, MC_PROTOCOL_REG_OBS_BEMF_LEVEL, MC_NULL );
    meerkatCore_ExpectedBEMF_s32 = UI_GetReg( &pHandle->_Super, MC_PROTOCOL_REG_EST_BEMF_LEVEL, MC_NULL );

    // No Load Detection:
    // - If speedf greater than limit and torque less than threshold, it is a 'no load overspeed' failure.
    // - Get values from Motor Control System
    meerkatCore_MeasuredTorque_s32 = UI_GetReg( &pHandle->_Super, MC_PROTOCOL_REG_TORQUE_MEAS, MC_NULL );
}

/**
  ********************************************************************************************************************************
  * @brief  MeerkatInterface_MotorIsRunning checks if the motor is currently running
  * @details 
  * - Returns 0 if motor is idle
  * - Returns 1 if motor is running
  ********************************************************************************************************************************
  **/
uint32_t meerkatCore_TestInterface_u32;

uint8_t MeerkatInterface_MotorIsRunning(void) { 
    State_t motor_state = MC_GetSTMStateMotor1(); //MCI_GetSTMState(pMCI);
  //  meerkatCore_TestInterface_u32 = motor_state;
    
    if (motor_state == RUN) {
        return 1;
    } else {
        return 0;
    }
}
// ======= Safety Interface - Data Input to Core ==END==


// ======= Safety Interface - Custom to this Project =START=
void MeerkatInterface_RestartMotor(void) { // ERM Gen0 specific
   // MCP_Handle_t* pHandle = GetMCP();
    MCI_ExecSpeedRamp( pMCI[M1], 1800, 500 ); // speed_rpm + duration
    MC_StartMotor1();
}
// ======= Safety Interface - Custom to this Project ==END==
