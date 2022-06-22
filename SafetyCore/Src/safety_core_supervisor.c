/**
  ***************************************************************************************************
  * @file    safety_core_supervisor.c 
  * @author  Regal Pamela Lee
  * @version V1.0
  * @date    11-17-2018
  * @brief   Main c++ function/s for System control (thin kernel)
  * @note    1)modules scheduling are happen here
  * @        2)Supervisor only share memory in the safety_moduleData_public[lastModule] struct array
  ***************************************************************************************************
  *  releast note:
  *     1, changed for the execute time period to auto adapted time compensation 
  *     2, change the SysTick from system to Safety specific: by the variable is located in the Supervisor private attribute
  */
#include "stm32f3xx_ll_tim.h" // REVIEW: pwmShutdown
#include "safety_core_supervisor.h"
#include "safety_core_ram_structures.h"
#include "main.h"
#include "stm32f3xx_ll_iwdg.h" // REVIEW stm32f3xx_ll_iwdg is not modular
#include "stm32f3xx_hal_rcc.h" // IWDG Reset
#include "safety_core_macros.h"


// REVIEW: Test Division - In some modules, it feels that too many tests are grouped together (analog test), 
// - also, test divisions could be more clear as to how applies to UL (ie. 'overload_check.h')
// REVIEW: Create 'overload_check' module, and move those checks out of adc_check?

#define TIM1                ((TIM_TypeDef *) TIM1_BASE) // REVIEW: PWM Shutdown      
      
__no_init Safety_RAMStruct_OTYP Safety_RAMStruct_P @SAFETY_RAM_MEERKAT_POSITIVE_RAM_START;
__no_init Safety_RAMStruct_OTYP Safety_RAMStruct_N @SAFETY_RAM_MEERKAT_NEGATIVE_RAM_START;

#pragma default_function_attributes = @ "supervisor_rom" // REVIEW: why is supervisor_rom necessary?
// #pragma default_function_attributes = @ "meerkat_rom"
/**
  ********************************************************************************************************************************
  * @brief  Provides a fixed location function wrapper for Running the Safety Supervisor (to be called by application firmware)  
  * @details This should be called in the application firmware's main loop.
  ********************************************************************************************************************************
  */
__root uint64_t SupervisorRun_wrapper(uint8_t motor_running) {
    return MeerkatCore_SupervisorRun(motor_running);
}
/**
  ********************************************************************************************************************************
  * @brief  Provides a fixed location function wrapper for Initializing the Safety Supervisor (to be called by application firmware)  
  * @details This function should be called in an application firmware's 'init' routine after all
  *  peripherals have been initialized
  ********************************************************************************************************************************
  */
__root void SupervisorInitialize_wrapper(void) {
    MeerkatCore_SupervisorInit();
    return;
}
/**
  ********************************************************************************************************************************
  * @brief  Provides a fixed location function wrapper for Initializing the Safety Supervisor's Shared RAM Area 
  * @details This function should be called in an application firmware's 'init' routine after all
  *  peripherals have been initialized
  ********************************************************************************************************************************
  */
__root void SupervisorInitializeSharedRam_wrapper(void) {
    MeerkatCore_SupervisorInitSharedRam();
    return;
}

/**
  ********************************************************************************************************************************
  * @brief  Provides a fixed location function wrapper for inputting a current sample into the safety core.
  * @details This function should be called whenever an application firmware reads 'current' throught the A/D Converter
  ********************************************************************************************************************************
  */

__root void Supervisor_ADCCheck_AddCurrentSample_wrapper(int16_t phase_current_ia, int16_t phase_current_ib, int16_t phase_current_ic) {
    MeerkatCore_AddShuntCurrentSample(phase_current_ia, phase_current_ib, phase_current_ic);
    return;
}

/**
  ********************************************************************************************************************************
  * @brief  Provides a fixed location function wrapper for inputting a current sample into the safety core.
  * @details 
  ********************************************************************************************************************************
  */
__root void SupervisorCheckIwdgResetFlag_wrapper(void) {
    MeerkatCore_CheckWatchdogResetFlag();
}

///**
//  ********************************************************************************************************************************
//  * @brief  Provides a fixed location function wrapper for inputting a 'speed' sample into the safety core.
//  * @details This function should be called whenever measured speed is updated by the application firmware.
//  ********************************************************************************************************************************
//  */

//void Supervisor_ADCCheck_AddSpeedSample_wrapper(int16_t speed_rpm) {
//    ADCCheck_AddSpeedSample(speed_rpm);
//    return;
//}

// Only wrappers are in 'supervisor_rom', all function implementations are to be placed in "meerkat_rom"
#pragma default_function_attributes = @ "meerkat_rom"

/**
  ********************************************************************************************************************************
  * @brief  This function executes each of the Safety Check Modules.  
  * @details This function was designed to be called (through the wrapper) in each iteration of the application firmware's main loop
  * - Tests are intended to be tuned such that no single iteration of MeerkatCore_SupervisorRun lasts more than 300us.
  * - This function is solely responsible for resetting the IWDG, which allows the IWDG to serve as proof that the 
  * - routine is running and being addressed properly. IWDG must be initialized in the application firmware.
  ********************************************************************************************************************************
  */
__root uint64_t MeerkatCore_SupervisorRun(uint8_t motor_running_u8) // run each module (in their call period interval)
{
    // Watchdog as Proof of Security Core Execution
    // - Application Firmware Initializes the Watchdog
    // -- This firmware, the security core, kicks the watchdog
    // --- If security core does not run, watchdog timer will reset, and mcu will reset
    LL_IWDG_ReloadCounter(IWDG); // Kick the watchdog

    //meerkatCore_ActiveTest_u32 = LASTMODULE; // Allow the Application to know that no module is running
    // Supervisor checks the value of InitVal if not 0x7391 magic number means, this first time running
    if (Safety_RAMStruct_P.safety_supervisor_private.InitVal != KEY_SAFETY_CORE_INITIALIZED) // First Time Running
    {
        MeerkatCore_Supervisor_WatchdogInit();
        MeerkatCore_SupervisorInit();

        // - Store Motor Running State so modules can see it
        Safety_RAMStruct_P.motorState_u8 = motor_running_u8;
        Safety_RAMStruct_N.motorState_u8 = ~motor_running_u8;
    } else // Normal Operation
    {
        // - Store Motor Running State so modules can see it
        Safety_RAMStruct_P.motorState_u8 = motor_running_u8;
        Safety_RAMStruct_N.motorState_u8 = ~motor_running_u8;
        // - Run Individual Test Modules
        for (Safety_RAMStruct_P.safety_supervisor_private.SuIndex_u8 = 0;
             Safety_RAMStruct_P.safety_supervisor_private.SuIndex_u8 < LASTMODULE;
             Safety_RAMStruct_P.safety_supervisor_private.SuIndex_u8++,
            Safety_RAMStruct_N.safety_supervisor_private.SuIndex_u8 = ~Safety_RAMStruct_P.safety_supervisor_private.SuIndex_u8) {
            if ((Safety_RAMStruct_P.moduleInfo[Safety_RAMStruct_P.safety_supervisor_private.SuIndex_u8].moduleStatus_u8 ==
                 0x00) &&
                (Safety_RAMStruct_P.safety_supervisor_private.SuSaSysTickCount_u32 -
                     Safety_RAMStruct_P.moduleInfo[Safety_RAMStruct_P.safety_supervisor_private.SuIndex_u8].moduleNextExe_u32 <
                 0x80000000)) {
                // Allow the Application Firmware to know which test module is running
                meerkatCore_ActiveTest_u32 = Safety_RAMStruct_P.safety_supervisor_private.SuIndex_u8;
                // Run the Current Module
                (*Safety_RAMStruct_P.moduleInfo[Safety_RAMStruct_P.safety_supervisor_private.SuIndex_u8]
                      .modulePtr)(); // run the current module
                                     // Allow the Application to know that no test module is running
                meerkatCore_ActiveTest_u32 = LASTMODULE;
                // Update Module Time Stamp and Time it should be executed next
                Safety_RAMStruct_P.moduleInfo[Safety_RAMStruct_P.safety_supervisor_private.SuIndex_u8].moduleNextExe_u32 +=
                    Safety_RAMStruct_P.moduleInfo[Safety_RAMStruct_P.safety_supervisor_private.SuIndex_u8]
                        .moduleExePeriod_u16; // update time stamp for this execution
                Safety_RAMStruct_N.moduleInfo[Safety_RAMStruct_P.safety_supervisor_private.SuIndex_u8].moduleNextExe_u32 =
                    ~Safety_RAMStruct_P.moduleInfo[Safety_RAMStruct_P.safety_supervisor_private.SuIndex_u8].moduleNextExe_u32;
            }
        }
        MeerkatCore_SupervisorUpdateCheckSchedule();
        MeerkatCore_SupervisorFaultCheck();
        MeerkatCore_Supervisor_PassCheck(); // not critical, only done when called.
    }
    MeerkatCore_Supervisor_IncSysTick();

    uint64_t return_value = meerkatCore_FaultCode_u32;
    return return_value;
}

#pragma default_function_attributes = @ "meerkat_rom"
/**
  ********************************************************************************************************************************
  * @brief  MeerkatCore_StartupInit Initializes Safety Core RAM and checks for an Independent Watchdog Timer Reset Flag.
  * @details 
  * - If Wathdog Reset detected, declares a Risk-Addressed-State to ensure the motor gets shut down from previous errors
  * -- that may have not been handled by the safety core.
  * - This function should be called by the processor Init Routine, as early as possible in it's 'main' function.
  ********************************************************************************************************************************
  */
__root void MeerkatCore_StartupInit(void) {
    MeerkatCore_SupervisorInit(); // REVIEW: Redundant to call in module_meerkat state machine, primarily to clear ADC Sample Buffer.
    meerkatCore_RiskAddrRetryCount_u16 = 0; // REVIEW: Placement
    // - Trigger Initialization of Safety Core Variables
    MeerkatCore_SupervisorInitSharedRam();
    // - See if IWDG was triggered. If so, trigger an 
    MeerkatCore_CheckWatchdogResetFlag();
 
    meerkatCore_MajorVersion_u16 = MEERKAT_FW_VERSION_MAJOR;
    meerkatCore_MedianVersion_u16 = MEERKAT_FW_VERSION_MEDIAN;
    meerkatCore_MinorVersion_u16 = MEERKAT_FW_VERSION_MINOR;
}


#pragma default_function_attributes = @ "meerkat_rom"
/**
  ********************************************************************************************************************************
  * @brief  This function initializes each of the Safety Check Modules and the Shared RAM Space used by both Safety and Application.  
  * @details 
  ********************************************************************************************************************************
  */
__root void MeerkatCore_SupervisorInit(void) {
    meerkatCore_ActiveTest_u32 = LASTMODULE; // Allow the Application to know that no module is running

    // Initialize all Safety RAM
    // - Setup TempCounter to be our 'Current Address' in the initialization routine.
    // -- Note: TempCounter is the first u32 of Safety ram, so it must be skipped. 
    // --- because it is used to store the current address for this init routine.
    Safety_RAMStruct_P.safety_supervisor_private.TempCounter = (uint8_t *) SAFETY_RAM_MEERKAT_POSITIVE_RAM_START + 4; // +4: Skip TempCounter
    // - Initialize Positive RAM
    do { 
        *(Safety_RAMStruct_P.safety_supervisor_private.TempCounter++) = SAFETY_RAM_POSITIVE_RAM_INIT_VAL;
    } while (Safety_RAMStruct_P.safety_supervisor_private.TempCounter < (uint8_t *) SAFETY_RAM_MEERKAT_NEGATIVE_RAM_START);
    // - Initialize Negative RAM (Shadow RAM)
    do {
        *(Safety_RAMStruct_P.safety_supervisor_private.TempCounter++) = SAFETY_RAM_NEGATIVE_RAM_INIT_VAL;
    } while (Safety_RAMStruct_P.safety_supervisor_private.TempCounter < (uint8_t *) SAFETY_RAM_END_ADDR); // REVIEW (uint8_t *) casting of U32?
    // - We are done with TempCounter, Initialize it's value to 0 (the init value for Positive RAM)
    // -- note: we set a u32 here, the routine prior to this point addresses everything as u8.
    Safety_RAMStruct_P.safety_supervisor_private.TempCounter = SAFETY_RAM_POSITIVE_RAM_INIT_VAL; // note: we set a u32 here

    // Initialize All Safety Core Parameters
    // - Set the Magic Key to signify that the Safety Core has been initialized.
    Safety_RAMStruct_P.safety_supervisor_private.InitVal = KEY_SAFETY_CORE_INITIALIZED; 
    Safety_RAMStruct_N.safety_supervisor_private.InitVal = ~(KEY_SAFETY_CORE_INITIALIZED);

    // Setup the Module system data
    for (; Safety_RAMStruct_P.safety_supervisor_private.SuIndex_u8 < (uint8_t) LASTMODULE;
         Safety_RAMStruct_P.safety_supervisor_private.SuIndex_u8++, Safety_RAMStruct_N.safety_supervisor_private.SuIndex_u8--) {
        // - Module ID
        Safety_RAMStruct_P.moduleInfo[Safety_RAMStruct_P.safety_supervisor_private.SuIndex_u8].moduleID_u8 =
            Safety_RAMStruct_P.safety_supervisor_private.SuIndex_u8;
        Safety_RAMStruct_N.moduleInfo[Safety_RAMStruct_P.safety_supervisor_private.SuIndex_u8].moduleID_u8 =
            ~Safety_RAMStruct_P.safety_supervisor_private.SuIndex_u8;
        // - Module Execution Period
        Safety_RAMStruct_P.moduleInfo[Safety_RAMStruct_P.safety_supervisor_private.SuIndex_u8].moduleExePeriod_u16 = DEFAULT_MODULE_EXE_PERIOD;
        Safety_RAMStruct_N.moduleInfo[Safety_RAMStruct_P.safety_supervisor_private.SuIndex_u8].moduleExePeriod_u16 = ~DEFAULT_MODULE_EXE_PERIOD;
        // - Module Status
        Safety_RAMStruct_P.moduleInfo[Safety_RAMStruct_P.safety_supervisor_private.SuIndex_u8].moduleStatus_u8 = DEFAULT_MODULE_STATUS;
        Safety_RAMStruct_N.moduleInfo[Safety_RAMStruct_P.safety_supervisor_private.SuIndex_u8].moduleStatus_u8 = ~DEFAULT_MODULE_STATUS;
        // - Module - Next Execution Time
        Safety_RAMStruct_P.moduleInfo[Safety_RAMStruct_P.safety_supervisor_private.SuIndex_u8].moduleNextExe_u32 =
            Safety_RAMStruct_P.safety_supervisor_private.SuIndex_u8;
        Safety_RAMStruct_N.moduleInfo[Safety_RAMStruct_P.safety_supervisor_private.SuIndex_u8].moduleNextExe_u32 =
            Safety_RAMStruct_N.safety_supervisor_private.SuIndex_u8;
    }

    //Setup Test Module Pointers
    Safety_RAMStruct_P.moduleInfo[ADCCHECK].modulePtr = &MeerkatCore_SupervisorAdcCheck;
    Safety_RAMStruct_N.moduleInfo[ADCCHECK].modulePtr = ((ModulePtr)(~(uint32_t) &MeerkatCore_SupervisorAdcCheck));

    Safety_RAMStruct_P.moduleInfo[REGISTERCHECK].modulePtr = &MeerkatCore_SupervisorRegisterCheck;
    Safety_RAMStruct_N.moduleInfo[REGISTERCHECK].modulePtr = ((ModulePtr)(~(uint32_t) &MeerkatCore_SupervisorRegisterCheck));

    Safety_RAMStruct_P.moduleInfo[CLOCKCHECK].modulePtr = &MeerkatCore_SupervisorClockCheck;
    Safety_RAMStruct_N.moduleInfo[CLOCKCHECK].modulePtr = ((ModulePtr)(~(uint32_t) &MeerkatCore_SupervisorClockCheck));

    Safety_RAMStruct_P.moduleInfo[RAMCHECK].modulePtr = &MeerkatCore_SupervisorRamCheck;
    Safety_RAMStruct_N.moduleInfo[RAMCHECK].modulePtr = ((ModulePtr)(~(uint32_t) &MeerkatCore_SupervisorRamCheck));

    Safety_RAMStruct_P.moduleInfo[ROMCHECK].modulePtr = &MeerkatCore_SupervisorRomCheck;
    Safety_RAMStruct_N.moduleInfo[ROMCHECK].modulePtr = ((ModulePtr)(~(uint32_t) &MeerkatCore_SupervisorRomCheck));

    // Reset and copy the Index value
    // - this value stores which module is slated to be executed next. 
    Safety_RAMStruct_P.safety_supervisor_private.SuIndex_u8 = 0;
    Safety_RAMStruct_N.safety_supervisor_private.SuIndex_u8 = ~0;
}

__root void MeerkatCore_SupervisorInitSharedRam(void) {
    // Reset Clock Check Buffer
    // - SysTick
    meerkatCore_SysTickMs_u32 = 0;
    // - LSI Ticks
    for (uint8_t buffer_index = 0; buffer_index < LENGTH_OF_LSI_RING_BUFFER; buffer_index++) {
        meerkatCore_LSIClockBuffer_u32[buffer_index] = 0;
    }
    // - Back EMF and Torque Readings
    meerkatCore_ExpectedBEMF_s32 = 0;
    meerkatCore_ObservedBEMF_s32 = 0;
    meerkatCore_MeasuredTorque_s32 = 0;

    // Reset Analog Reading Buffers
    // - Current
    // uint16_t *pSharedRAM_u16 = (uint16_t *) (FIXED_RAM_ADDRESS_CURRENT_BUFFER_U_FIRST);
    // for (uint8_t buffer_index = 0; buffer_index < LENGTH_OF_CURRENT_RING_BUFFER; buffer_index++) {
    //     pSharedRAM_u16[buffer_index] = 0;
    // }
    // pSharedRAM_u16 = (uint16_t *) (FIXED_RAM_ADDRESS_CURRENT_BUFFER_V_FIRST);
    // for (uint8_t buffer_index = 0; buffer_index < LENGTH_OF_CURRENT_RING_BUFFER; buffer_index++) {
    //     pSharedRAM_u16[buffer_index] = 0;
    // }
    // pSharedRAM_u16 = (uint16_t *) (FIXED_RAM_ADDRESS_CURRENT_BUFFER_W_FIRST);
    // for (uint8_t buffer_index = 0; buffer_index < LENGTH_OF_CURRENT_RING_BUFFER; buffer_index++) {
    //     pSharedRAM_u16[buffer_index] = 0;
    // }

    // - Single Sample Buffers
    meerkatCore_Vref_u16 = 0;
    meerkatCore_TemperatureValue1_u16 = 0;
    meerkatCore_Vbus_u16 = 0;
    
    // - Motor State - Speed
    meerkatCore_SpeedRpm_s32 = 0;

    // Reset Test Reporting Results
    meerkatCore_FaultCode_u32 = 0;
    meerkatCore_ActiveTest_u32 = LASTMODULE;
    meerkatCore_PassedTests_u32 = 0;
}

#pragma default_function_attributes = @ "meerkat_rom"
/**
  ********************************************************************************************************************************
  * @brief  This function updates the supervisor's stored 'callRate' for each test module based on the value stored in the 
  * - test modules public structure.  
  * @details 
  ********************************************************************************************************************************
  */
void MeerkatCore_SupervisorUpdateCheckSchedule(void) { // REVIEW: These values are set but don't appear to be used
    //refer to user manual
    Safety_RAMStruct_P.moduleInfo[ADCCHECK].moduleExePeriod_u16 = Safety_RAMStruct_P.adcCheck_public.callRate_u16;
    Safety_RAMStruct_N.moduleInfo[ADCCHECK].moduleExePeriod_u16 = ~Safety_RAMStruct_P.adcCheck_public.callRate_u16;

    Safety_RAMStruct_P.moduleInfo[REGISTERCHECK].moduleExePeriod_u16 = Safety_RAMStruct_P.registerCheck_public.callRate_u16;
    Safety_RAMStruct_N.moduleInfo[REGISTERCHECK].moduleExePeriod_u16 = ~Safety_RAMStruct_P.registerCheck_public.callRate_u16;

    Safety_RAMStruct_P.moduleInfo[CLOCKCHECK].moduleExePeriod_u16 = Safety_RAMStruct_P.clockCheck_public.callRate_u16;
    Safety_RAMStruct_N.moduleInfo[CLOCKCHECK].moduleExePeriod_u16 = ~Safety_RAMStruct_P.clockCheck_public.callRate_u16;

    Safety_RAMStruct_P.moduleInfo[RAMCHECK].moduleExePeriod_u16 = Safety_RAMStruct_P.ramCheck_public.callRate_u16;
    Safety_RAMStruct_N.moduleInfo[RAMCHECK].moduleExePeriod_u16 = ~Safety_RAMStruct_P.ramCheck_public.callRate_u16;

    Safety_RAMStruct_P.moduleInfo[ROMCHECK].moduleExePeriod_u16 = Safety_RAMStruct_P.romCheck_public.callRate_u16;
    Safety_RAMStruct_N.moduleInfo[ROMCHECK].moduleExePeriod_u16 = ~Safety_RAMStruct_P.romCheck_public.callRate_u16;
}

#pragma default_function_attributes = @ "meerkat_rom"
/**
  ********************************************************************************************************************************
  * @brief  This function updates the 'Error Code' stored by the Safety Core. A '0' is stored if no errors have occurred.
  * @details The 'Error Code' is designed to signify a Safety Core Fault, which in turn must trigger a 'Risk Addressed State'
  * - A 'Risk Addressed State' must in turn trigger a shutdown of all motor functions.
  ********************************************************************************************************************************
  */
void MeerkatCore_SupervisorFaultCheck(void) {
    // If Supervisor has detected a fault, set risk addressed state and notify the application.
    // REVIEW: This only outputs one error. It could output all errors and just only one 'specific' error code'.

    if (Safety_RAMStruct_P.adcCheck_public.faultCount_u32) {
        meerkatCore_FaultCode_u32 = (Safety_RAMStruct_P.adcCheck_public.errorCode_u32 & 0x00FFFFFF) + ((uint32_t)(ADCCHECK << 24));
    }
    else if (Safety_RAMStruct_P.registerCheck_public.faultCount_u32) {
        meerkatCore_FaultCode_u32 =
            (Safety_RAMStruct_P.registerCheck_public.errorCode_u32 & 0x00FFFFFF) + ((uint32_t)(REGISTERCHECK << 24));
    }
    else if (Safety_RAMStruct_P.clockCheck_public.faultCount_u32) {
        meerkatCore_FaultCode_u32 = (Safety_RAMStruct_P.clockCheck_public.errorCode_u32 & 0x00FFFFFF) + ((uint32_t)(CLOCKCHECK << 24));
    }
    else if (Safety_RAMStruct_P.ramCheck_public.faultCount_u32) {
        meerkatCore_FaultCode_u32 = (Safety_RAMStruct_P.ramCheck_public.errorCode_u32 & 0x00FFFFFF) + ((uint32_t)(RAMCHECK << 24));
    }
    else if (Safety_RAMStruct_P.romCheck_public.faultCount_u32) {
        meerkatCore_FaultCode_u32 = (Safety_RAMStruct_P.romCheck_public.errorCode_u32 & 0x00FFFFFF) + ((uint32_t)(ROMCHECK << 24));
    }
    
    MeerkatCore_SupervisorFaultResponse();
}

#pragma default_function_attributes = @ "meerkat_rom"
void MeerkatCore_SupervisorFaultResponse(void) {
    if (meerkatCore_FaultCode_u32 > 0) { // Safety Core has active Fault Reported
        MeerkatCore_Supervisor_DisablePwm(); // Disable Motor

        // Handle Clearing of the UL Fault Flag
        if (meerkatCore_LastFaultCode_u32 == 0) { // If this is the first time we've seen a fault since faults were last cleared.  TODO: 0=MEERKAT_FAULT_CODE_NO_FAULTS
            meerkatCore_RiskAddrStateSetTime_u32 = Safety_RAMStruct_P.safety_supervisor_private.SuSaSysTickCount_u32;
        }
        else {
            if (Safety_RAMStruct_P.safety_supervisor_private.SuSaSysTickCount_u32 - meerkatCore_RiskAddrStateSetTime_u32 > 
                                                RISK_ADDRESSED_STATE_MIN_TIMEOUT_MS + MeerkatConfig_FaultClearTimeS_u8*1000) {
                // clear the fault flag or restart the device
                if (meerkatCore_RiskAddrRetryCount_u16 > MeerkatConfig_FaultClearLimit_u8) {
                    MeerkatCore_RebootByWatchdogTimerReset();
                    meerkatCore_RiskAddrRetryCount_u16 = 0;
                } else {
                    MeerkatCore_ClearRiskAddressedState();
                }
                meerkatCore_RiskAddrRetryCount_u16 = meerkatCore_RiskAddrRetryCount_u16 + 1;
            } 
        }
    } else {
        // no faults, no specific action to execute.
    }
    meerkatCore_LastFaultCode_u32 = meerkatCore_FaultCode_u32;
}

#pragma default_function_attributes = @ "meerkat_rom"
void MeerkatCore_ClearRiskAddressedState(void) {
    MeerkatCore_SupervisorInit(); // TODO: REVIEW: do we have to do this much?
    MeerkatCore_SupervisorInitSharedRam(); // Clears Fault Code and most test buffers
    MeerkatCore_ADCCheck_ClearShuntCurrentBuffers(); // clear adc buffers so they don't fail with old data
}

#pragma default_function_attributes = @ "meerkat_rom"
/**
  ********************************************************************************************************************************
  * @brief  This function clears a Risk Addressed state 
  * @details Resets meerkatCore_FaultCode_u32 and all tests.
  * - A 'Risk Addressed State' must in turn trigger a shutdown of all motor functions.
  ********************************************************************************************************************************
  */
__root void MeerkatCore_RebootByWatchdogTimerReset(void) {
    while(1) {
        asm("");
    }
}

#pragma default_function_attributes = @ "meerkat_rom"
/**
  ********************************************************************************************************************************
  * @brief  This function updates the Supervisor SysTick. This stores how many times the MeerkatCore_SupervisorRun function has executed.
  * @details 
  ********************************************************************************************************************************
  */
void MeerkatCore_Supervisor_IncSysTick(void) {
    Safety_RAMStruct_P.safety_supervisor_private.SuSaSysTickCount_u32++; // REVIEW: Should this use a macro for shadow ram?
    Safety_RAMStruct_N.safety_supervisor_private.SuSaSysTickCount_u32--;
}

#pragma default_function_attributes = @ "meerkat_rom"
/**
  ********************************************************************************************************************************
  * @brief  This function initializes the Independent Watchdog Timer.
  * @details 
  ********************************************************************************************************************************
  */
void MeerkatCore_Supervisor_WatchdogInit(void) { // REVIEW: This is not currently used, as watchdog is initialized in application.
    LL_IWDG_Enable(IWDG);
    LL_IWDG_EnableWriteAccess(IWDG);
    LL_IWDG_SetPrescaler(IWDG, LL_IWDG_PRESCALER_4);
    LL_IWDG_SetWindow(IWDG, 4095);
    LL_IWDG_SetReloadCounter(IWDG, 4095);
    while (LL_IWDG_IsReady(IWDG) != 1) {
    }
    LL_IWDG_ReloadCounter(IWDG);
}

#pragma default_function_attributes = @ "meerkat_rom"
/**
  ********************************************************************************************************************************
  * @brief  MeerkatCore_Supervisor_PassCheck updates the 'pass count' of each individual test module
  * @details The 'pass counts' are intended to provide an overall 'tests passed' metric for the application firmware.
  ********************************************************************************************************************************
  */
void MeerkatCore_Supervisor_PassCheck(void) { 
    // returns 0 if all checks passed, returns 1 for any checks that have not passed
    if (Safety_RAMStruct_P.adcCheck_public.passCount_u32 >= ADC_CHECK_PASS_REQUIREMENT) {
        meerkatCore_PassedTests_u32 |= 0x01 << ADCCHECK;
    }
    if (Safety_RAMStruct_P.registerCheck_public.passCount_u32 >= REGISTER_CHECK_PASS_REQUIREMENT) {
        meerkatCore_PassedTests_u32 |= 0x01 << REGISTERCHECK;
    }
    if (Safety_RAMStruct_P.clockCheck_public.passCount_u32 >= CLOCK_CHECK_PASS_REQUIREMENT) {
        meerkatCore_PassedTests_u32 |= 0x01 << CLOCKCHECK;
    }
    if (Safety_RAMStruct_P.ramCheck_public.passCount_u32 >= RAM_CHECK_PASS_REQUIREMENT) {
        meerkatCore_PassedTests_u32 |= 0x01 << RAMCHECK;
    }
    if (Safety_RAMStruct_P.romCheck_public.passCount_u32 >= ROM_CHECK_PASS_REQUIREMENT) {
        meerkatCore_PassedTests_u32 |= 0x01 << ROMCHECK;
    }    
    
    // TODO: Reimplement PassCheck
//    if (Safety_RAMStruct_P.safety_supervisor_private.lastPassCount_u32[ADCCHECK]  >= ADC_CHECK_PASS_REQUIREMENT) {
//        meerkatCore_PassedTests_u32 |= 0x01 << ADCCHECK;
//    }   
//    if (Safety_RAMStruct_P.safety_supervisor_private.lastPassCount_u32[REGISTERCHECK]  >= REGISTER_CHECK_PASS_REQUIREMENT) {
//        meerkatCore_PassedTests_u32 |= 0x01 << REGISTERCHECK;
//    }   
//    if (Safety_RAMStruct_P.safety_supervisor_private.lastPassCount_u32[CLOCKCHECK]  >= CLOCK_CHECK_PASS_REQUIREMENT) {
//        meerkatCore_PassedTests_u32 |= 0x01 << CLOCKCHECK;
//    }   
//    if (Safety_RAMStruct_P.safety_supervisor_private.lastPassCount_u32[RAMCHECK]  >= RAM_CHECK_PASS_REQUIREMENT) {
//        meerkatCore_PassedTests_u32 |= 0x01 << RAMCHECK;
//    }   
//    if (Safety_RAMStruct_P.safety_supervisor_private.lastPassCount_u32[ROMCHECK]  >= ROM_CHECK_PASS_REQUIREMENT) {
//        meerkatCore_PassedTests_u32 |= 0x01 << ROMCHECK;
//    }   
}

#pragma default_function_attributes = @ "meerkat_rom"
void MeerkatCore_Supervisor_DisablePwm(void) {
    LL_TIM_DisableIT_UPDATE(TIM1);
    LL_TIM_DisableAllOutputs(TIM1);
    LL_TIM_ClearFlag_UPDATE(TIM1);
    while (((READ_BIT(TIM1->SR, TIM_SR_UIF) == (TIM_SR_UIF)) ? 1UL : 0UL) == 0) { // LL_TIM_IsActiveFlag_UPDATE(TIM1)
    }
    LL_TIM_ClearFlag_UPDATE(TIM1);
}

#pragma default_function_attributes = @ "meerkat_rom"
__root void MeerkatCore_CheckWatchdogResetFlag(void) {
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
        meerkatCore_FaultCode_u32 = ((uint32_t)IWDGRESETCHECK << 24);
        meerkatCore_RiskAddrStateSetTime_u32 = Safety_RAMStruct_P.safety_supervisor_private.SuSaSysTickCount_u32;
        //SupervisorAssesRiskAddressedState(); // trigger reaction based on meerkatCore_FaultCode_u32
        // If we were reset by watchdog, then a critical error has occurred in the previous run of this firmware
        // - stop the firmware from running again (Risk Addressed State is only relieived if device is completely powered down.)
        // -- typically, this can take up to 30 seconds due to the large capacitors on the PCB.
    }
}
