/**
  ***************************************************************************************************
  * @file    safety_core_supervisor.h 
  * @author  Regal Pamela Lee
  * @version V1.0
  * @date    20-Oct-2016
  * @brief   Header of c++ function/s for System control (thin kernel) 
  * @note    1)Apps/Drivers scheduling is happen here
  * @        2)System resource request functions like pipe or shared memory
  ***************************************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SUPERVISOR_H
#define __SUPERVISOR_H

/* Includes ------------------------------------------------------------------*/
#include "stm32F3xx.h"
#include "stm32f3xx_ll_iwdg.h"
//#include "core_cm0.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SUPERVISOR_TICK 1

// #define RISK_ADDRESSED_STATE_RETRY_LIMIT 3 // TODO: Add to Shared ROM
#define RISK_ADDRESSED_STATE_MIN_TIMEOUT_MS 20000
// All Modules Performance
// Notes: All Tests Running (rom test complete), 72MHz
// - Motor Running at 15000 RPM
// -- max execution time = 212us
// -- min execution time = 11.25us (no test executed)
// - Motor Stopped
// -- max execution time = 57us
// -- min execution time = 11.25us (no test executed)


//^^^^^^^^^^^^^^^ Generate safety module ID for each module ^^^^^^^^^^^^^^^^
//^ All safety module should be here to generate a module ID for system kernel    ^
//^ Note : Higher priority module should place as small module ID                ^
//^** Tips: module adding process example step1  [refer to user manual )     	   ^
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

#define KEY_SAFETY_CORE_INITIALIZED (0x7391)
#define DEFAULT_MODULE_EXE_PERIOD (10)
#define DEFAULT_MODULE_STATUS (0)

enum {
    ADCCHECK,
    REGISTERCHECK,
    CLOCKCHECK,
    RAMCHECK,
    ROMCHECK,
    LASTMODULE //total number of module
};
#define IWDGRESETCHECK LASTMODULE // not a module, just a check at startup, so takes id of lastmodule

typedef void (*ModulePtr)(void); //safety module function pointer type definition

typedef struct {
    uint8_t moduleID_u8;          //defined in moduleID
    ModulePtr modulePtr;          //Apps function pointer
    uint16_t moduleExePeriod_u16; //How often this module run
    uint32_t moduleNextExe_u32;   //store the next system tick time to execute this module
    uint8_t moduleStatus_u8;      //This module is 0x00= valid, 0xff= killed, 0x05 = sleep
} ModuleInfo_OTYP;

//static ModuleInfo moduleInfo[lastModule];

typedef struct { //Private Static data for this module
    uint8_t *TempCounter;
    uint16_t InitVal; //if the value == KEY_SAFETY_CORE_INITIALIZED indicate supervisor has already initialized
    uint8_t SuIndex_u8;
    uint32_t SuSaSysTickCount_u32;
    uint32_t NRamPtr;
    uint32_t lastPassCount_u32[LASTMODULE];
} Safety_Supervisor_Private_OTYP; // = {0,12};

//^^^^^^^^^^^^^^^^^^^^^^ Module Function prototypes ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^ All module is a function which call by the Mouse OS as a pointer of function ^
//^** Tips: APPs/Drivers adding process example step2  [refer to user manual )   ^
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
void MeerkatCore_SupervisorAdcCheck(void);
void MeerkatCore_SupervisorRegisterCheck(void);
void MeerkatCore_SupervisorClockCheck(void);
void MeerkatCore_SupervisorRamCheck(void);
void MeerkatCore_SupervisorRomCheck(void);
//^^^^^^^^^^^^^^^^^^^^^^ Supervisor function prototypes ^^^^^^^^^^^^^^^^^^^^^^^^^
//^                                                                              ^
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
void SupervisorInitialize_wrapper(void);
void MeerkatCore_SupervisorInit(void);
void SupervisorInitializeSharedRam_wrapper(void);
void MeerkatCore_SupervisorInitSharedRam(void);
void MeerkatCore_Supervisor_WatchdogInit(void);
uint64_t SupervisorRun_wrapper(uint8_t motor_running); //run all modules in there call period interval
void Supervisor_ADCCheck_AddCurrentSample_wrapper(int16_t phase_current_ia, int16_t phase_current_ib, int16_t phase_current_ic);
void MeerkatCore_AddShuntCurrentSample(int16_t phase_current_ia, int16_t phase_current_ib, int16_t phase_current_ic);
void MeerkatCore_AddClockSample(uint32_t lsi_tick_count_u32);
void Supervisor_ADCCheck_AddSpeedSample_wrapper(int16_t speed_rpm);
void ADCCheck_AddSpeedSample(int16_t speed_rpm);
void SupervisorCheckIwdgResetFlag_wrapper(void);
void MeerkatCore_CheckWatchdogResetFlag(void); 

//uint64_t SupervisorRun_wrapper(void); //run all modules in there call period interval
//uint64_t MeerkatCore_SupervisorRun(void);         //run all modules in there call period interval
uint64_t MeerkatCore_SupervisorRun(uint8_t motor_running);         //run all modules in there call period interval
void MeerkatCore_SupervisorUpdateCheckSchedule(void);
void MeerkatCore_SupervisorFaultCheck(void);
void MeerkatCore_RebootByWatchdogTimerReset(void);
void MeerkatCore_Supervisor_IncSysTick(void);
void MeerkatCore_Supervisor_PassCheck(void);
void MeerkatCore_Supervisor_DisablePwm(void);
void MeerkatCore_SupervisorFaultResponse(void);
void MeerkatCore_ClearRiskAddressedState(void);

extern void MeerkatCore_ADCCheck_ClearShuntCurrentBuffers(void); // this is defined in adc_check REVIEW: extern import like this

#ifdef __cplusplus
}
#endif

#endif /* __SYSCON_H */
