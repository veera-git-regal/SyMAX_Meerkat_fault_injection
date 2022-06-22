/**
  ********************************************************************************************************************************
  * @file    scheduler.h 
  * @author  Pamela Lee
  * @brief   Header of c++ function/s for the kernel scheduler. 
  * @details Modules and drivers are collectively known as processes. This file contains function for scheduling processes. The
  *             file also contains functions for performing system resource requests such as creating and destroying sequential
  *             and/or structured memory instances. The function scheduler_run is the 'main loop' of the FlexMouse architecture.
  ********************************************************************************************************************************
  */

/* Define to prevent recursive inclusion ---------------------------------------------------------------------------------------*/
#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include "main.h"
//#include "stm32f4xx_hal.h" 
#include "stm32f3xx_hal.h" //SPA

/*USE_FULL_LL_DRIVER
HSE_VALUE=8000000
HSE_STARTUP_TIMEOUT=100
LSE_STARTUP_TIMEOUT=5000
LSE_VALUE=32768
EXTERNAL_CLOCK_VALUE=12288000
HSI_VALUE=16000000
LSI_VALUE=32000
VDD_VALUE=3300
PREFETCH_ENABLE=0
INSTRUCTION_CACHE_ENABLE=1
DATA_CACHE_ENABLE=1*/

#include "../Regal/typedef.h"

#include "sequential_memory.h"
#include "structured_memory.h"

/* Content ---------------------------------------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
  
//static uint64_t tickCounter = 0; //SPA

/* global parameters -----------------------------------------------------------------------------------------------------------*/  

/* User parameters -------------------------------------------------------------------------------------------------------------*/
// General
extern __root const uint32_t App_CRC @ "app_crc32_rom";// = 0x00000000; // This is a placeholder for App Firmware Checksum (CRC32)

/** pam procedure #1 of Module insertion  :  to define the default task parameters **/
#define MODULE_APP_ID                   MODULE_APP                     //Pam: name the module and define in procedure #2 as the Task ID
#define MODULE_APP_FUNCTION_POINTER     &p_moduleApp_u32               //Pam: name the module function name in procedure #?? to let kernel to execute the task
#define MODULE_APP_TOTAL_SEQ        0                                  //Pam: assign how many memory of seqential-memory for this module for interacting with other module\s   
#define MODULE_APP_TOTAL_STRUCT     1                                  //Pam: assign how many structure of structured-memory for this module for share common variable with other module\s
#define MODULE_APP_PREV_STATE       0                                  //Pam: kernal use only, only set as zero 
#define MODULE_APP_NEXT_STATE       0                                  //Pam: kernal use only, only set as zero
#define MODULE_APP_IRQ_STATUS     DEFAULT_IRQ_STATE                    //Pam: set your software interrupt service point/state 
#define MODULE_APP_PROCESS_STATUS   0x00                               //Pam: all module can in either one of these status:- 
                                                                                      // -PROCESS_STATUS_RUNNING 0x00     <Normal running state>
                                                                                      // -PROCESS_STATUS_KILLED  0xFF     <This module will never execute>
                                                                                      // -PROCESS_STATUS_PAUSED  0x05     <This module is paused waiting for other module/s to resume running>
#define MODULE_APP_MASTER_SHARED_MEM 0                           //Pam: Attach the structured-memory pointer after created to kernal module/task list. Any other module\s going to use or link to 
                                                                            //this module can search the kernal module/task list with the App ID, then find out the structured-memory as the entry point 
/** pam procedure #1 of Module insertion  :  to define the default task parameters end **/

#define MODULE_FLASH_ID                    MODULE_FLASH
#define MODULE_FLASH_FUNCTION_POINTER      &moduleFlash_u32
#define MODULE_FLASH_TOTAL_SEQ        0
#define MODULE_FLASH_TOTAL_STRUCT     1
#define MODULE_FLASH_PREV_STATE       0
#define MODULE_FLASH_NEXT_STATE           0
#define MODULE_FLASH_IRQ_STATUS     DEFAULT_IRQ_STATE
#define MODULE_FLASH_PROCESS_STATUS       0
#define MODULE_FLASH_MASTER_SHARED_MEM 0

#define MODULE_USART2_ID                   MODULE_USART2
#define MODULE_USART2_FUNCTION_POINTER     &moduleUsart2
#define MODULE_USART2_TOTAL_SEQ        3
#define MODULE_USART2_TOTAL_STRUCT     1
#define MODULE_USART2_PREV_STATE       0
#define MODULE_USART2_NEXT_STATE           0
#define MODULE_USART2_IRQ_STATUS     DEFAULT_IRQ_STATE
#define MODULE_USART2_PROCESS_STATUS       0
#define MODULE_USART2_MASTER_SHARED_MEM 0

#define MODULE_I2C_ID                   MODULE_I2C
#define MODULE_I2C_FUNCTION_POINTER     &moduleI2c_u32
#define MODULE_I2C_TOTAL_SEQ        2
#define MODULE_I2C_TOTAL_STRUCT     1
#define MODULE_I2C_PREV_STATE       0
#define MODULE_I2C_NEXT_STATE       0
#define MODULE_I2C_IRQ_STATUS     DEFAULT_IRQ_STATE
#define MODULE_I2C_PROCESS_STATUS   0
#define MODULE_I2C_MASTER_SHARED_MEM 0

#define MODULE_MC_STATEMACHINE_ID                   MODULE_MC_STATEMACHINE 
#define MODULE_MC_STATEMACHINE_FUNCTION_POINTER     &module_Mc_StateMachine_u32 
#define MODULE_MC_STATEMACHINE_TOTAL_SEQ        0
#define MODULE_MC_STATEMACHINE_TOTAL_STRUCT     1
#define MODULE_MC_STATEMACHINE_PREV_STATE       0
#define MODULE_MC_STATEMACHINE_NEXT_STATE       0
#define MODULE_MC_STATEMACHINE_IRQ_STATUS     DEFAULT_IRQ_STATE
#define MODULE_MC_STATEMACHINE_PROCESS_STATUS   0
#define MODULE_MC_STATEMACHINE_MASTER_SHARED_MEM 0

#define MODULE_EEP_CMD_ID                   MODULE_EEP_CMD
#define MODULE_EEP_CMD_FUNCTION_POINTER     &moduleEEP
#define MODULE_EEP_CMD_TOTAL_SEQ        0
#define MODULE_EEP_CMD_TOTAL_STRUCT     1
#define MODULE_EEP_CMD_PREV_STATE       0
#define MODULE_EEP_CMD_NEXT_STATE       0
#define MODULE_EEP_CMD_IRQ_STATUS     DEFAULT_IRQ_STATE
#define MODULE_EEP_CMD_PROCESS_STATUS   0
#define MODULE_EEP_CMD_MASTER_SHARED_MEM 0

#define MODULE_DYNAMIC_ID                   MODULE_DYNAMIC
#define MODULE_DYNAMIC_FUNCTION_POINTER     &module_dynamic 
#define MODULE_DYNAMIC_TOTAL_SEQ        0
#define MODULE_DYNAMIC_TOTAL_STRUCT     1
#define MODULE_DYNAMIC_PREV_STATE       0
#define MODULE_DYNAMIC_NEXT_STATE       0
#define MODULE_DYNAMIC_IRQ_STATUS     DEFAULT_IRQ_STATE
#define MODULE_DYNAMIC_PROCESS_STATUS   0
#define MODULE_DYNAMIC_MASTER_SHARED_MEM 0


#define MODULE_ERR_LOGHANDLE_ID                MODULE_ERR_LOGHANDLE
#define MODULE_ERR_LOGHANDLE_FUNCTION_POINTER     &module_err_log_u32
#define MODULE_ERR_LOGHANDLE_TOTAL_SEQ        0
#define MODULE_ERR_LOGHANDLE_TOTAL_STRUCT     1
#define MODULE_ERR_LOGHANDLE_PREV_STATE       0
#define MODULE_ERR_LOGHANDLE_NEXT_STATE       0
#define MODULE_ERR_LOGHANDLE_IRQ_STATUS     DEFAULT_IRQ_STATE
#define MODULE_ERR_LOGHANDLE_PROCESS_STATUS   0
#define MODULE_ERR_LOGHANDLE_MASTER_SHARED_MEM 0

#define MODULE_ECM_ICL_ID                   MODULE_ECM_ICL                     
#define MODULE_ECM_ICL_FUNCTION_POINTER     &p_moduleECM_ICL_u32              
#define MODULE_ECM_ICL_TOTAL_SEQ        	0                                     
#define MODULE_ECM_ICL_TOTAL_STRUCT     	1                                  
#define MODULE_ECM_ICL_PREV_STATE       	0                                   
#define MODULE_ECM_ICL_NEXT_STATE       	0                              
#define MODULE_ECM_ICL_IRQ_STATUS     		DEFAULT_IRQ_STATE                    
#define MODULE_ECM_ICL_PROCESS_STATUS   	0x00                               
#define MODULE_ECM_ICL_MASTER_SHARED_MEM 	0                          

#define MODULE_VOLTAGE_DOUBLER_ID                   MODULE_VOLTAGE_DOUBLER
#define MODULE_VOLTAGE_DOUBLER_FUNCTION_POINTER     &p_module_voltage_doubler_u32
#define MODULE_VOLTAGE_DOUBLER_TOTAL_SEQ        	0
#define MODULE_VOLTAGE_DOUBLER_TOTAL_STRUCT     	1
#define MODULE_VOLTAGE_DOUBLER_PREV_STATE       	0
#define MODULE_VOLTAGE_DOUBLER_NEXT_STATE       	0                                  
#define MODULE_VOLTAGE_DOUBLER_IRQ_STATUS     		DEFAULT_IRQ_STATE
#define MODULE_VOLTAGE_DOUBLER_PROCESS_STATUS   	0x00
#define MODULE_VOLTAGE_DOUBLER_MASTER_SHARED_MEM	0



#define MODULE_MODBUS_ID                MODULE_MODBUS
#define MODULE_MODBUS_FUNCTION_POINTER  &moduleModbus
#define MODULE_MODBUS_TOTAL_SEQ         0
#define MODULE_MODBUS_TOTAL_STRUCT      1
#define MODULE_MODBUS_PREV_STATE        0
#define MODULE_MODBUS_NEXT_STATE        0
#define MODULE_MODBUS_IRQ_STATUS        DEFAULT_IRQ_STATE
#define MODULE_MODBUS_PROCESS_STATUS    0
#define MODULE_MODBUS_MASTER_SHARED_MEM 0

#define MODULE_TEST_ID                 MODULE_TEST
#define MODULE_TEST_FUNCTION_POINTER   &moduleTest_u32
#define MODULE_TEST_TOTAL_SEQ          0
#define MODULE_TEST_TOTAL_STRUCT       1
#define MODULE_TEST_PREV_STATE         0
#define MODULE_TEST_NEXT_STATE         0
#define MODULE_TEST_IRQ_STATUS         DEFAULT_IRQ_STATE
#define MODULE_TEST_PROCESS_STATUS     0
#define MODULE_TEST_MASTER_SHARED_MEM  0

#define MODULE_RTC_ID                   MODULE_RTC
#define MODULE_RTC_FUNCTION_POINTER     &p_moduleRTC_u32
#define MODULE_RTC_TOTAL_SEQ            0
#define MODULE_RTC_TOTAL_STRUCT         0
#define MODULE_RTC_PREV_STATE           0
#define MODULE_RTC_NEXT_STATE           0
#define MODULE_RTC_IRQ_STATUS           DEFAULT_IRQ_STATE
#define MODULE_RTC_PROCESS_STATUS       0
#define MODULE_RTC_MASTER_SHARED_MEM    0

#define MODULE_MEERKAT_ID                MODULE_MEERKAT_SAFETY_CORE
#define MODULE_MEERKAT_FUNCTION_POINTER  &MeerkatInterface_RunStateMachine
#define MODULE_MEERKAT_TOTAL_SEQ         0
#define MODULE_MEERKAT_TOTAL_STRUCT      0
#define MODULE_MEERKAT_PREV_STATE        0
#define MODULE_MEERKAT_NEXT_STATE        0
#define MODULE_MEERKAT_IRQ_STATUS        DEFAULT_IRQ_STATE
#define MODULE_MEERKAT_PROCESS_STATUS    0
#define MODULE_MEERKAT_MASTER_SHARED_MEM 0


// Process Status
#define PROCESS_STATUS_RUNNING 0x00
#define PROCESS_STATUS_KILLED  0xFF
#define PROCESS_STATUS_PAUSED  0x05

// Process Ranges
#define MIN_MODULE_ID     0
#define MAX_MODULE_ID     191
#define MAX_NUM_OF_MODULES MAX_MODULE_ID - MIN_MODULE_ID + 1
#define MIN_IRQ_ID     0
#define MAX_IRQ_ID     254
#define MAX_NUM_OF_DRVS MAX_DRV_ID - MIN_DRV_ID + 1
#define MAX_PROCESS_ID    255

// Interrupt Parameters
#define SHIFTER                 1
#define GPIO_SOFTWARE_IRQ 0x02
#define PROCESS_IRQ_ID    0
#define NO_IRQS           0
#define DEFAULT_IRQ_STATE 200

// Default States
#define DEFAULT_PREV_STATE 0
#define DEFAULT_NEXT_STATE     0

#define INDEX_NOT_FOUND 255

#define KILL_APP 255

// Watch Dog Timer
#define NUM_OF_625_MS_INC 6

/**
  ********************************************************************************************************************************
  * @brief   Generate Process ID for each module and/or driver.
  * @details 1. All module and drivers should be declared here.
  *          2. The position in the enum defines the process ID for each module/driver.
  *          3. The lower the position the higher the priority.
  *                 Position 0 is the highest module priority.
  *                 Position 192 is the highest driver priority.
  *          4. Modules should be declared at positions 0 to 191.
  *          5. Drivers should be declared at positions 192 to 254.
  *          6. END_OF_PROCESS_ID is reserved.
  ********************************************************************************************************************************
  */
/** pam procedure #3 of Module insertion  :  add up total number of module below **/
#define TOTAL_NUM_OF_PROCESSES 14       // Pam: Let kernal assign total number of process to execute
/** pam procedure #3of Module insertion  :  add up total number of module below end  **/

/** pam procedure #2 of Module insertion  :  add/define the module ID **/
enum Processes { 
    MODULE_FLASH= MIN_MODULE_ID,
    MODULE_USART2,
    MODULE_I2C,
    MODULE_MC_STATEMACHINE,
    MODULE_APP,                         //Pam: kernal will auto asign a module/process/task ID for this module
    MODULE_EEP_CMD,
    MODULE_DYNAMIC,
    MODULE_ERR_LOGHANDLE,
    MODULE_MODBUS,
     MODULE_ECM_ICL,
     MODULE_VOLTAGE_DOUBLER,
    MODULE_TEST,
     MODULE_MEERKAT_SAFETY_CORE,
    MODULE_RTC,
    END_OF_PROCESS_ID = MAX_PROCESS_ID,
};
/** pam procedure #2 of Module insertion  :  add/define the module ID  end **/

/**
  ********************************************************************************************************************************
  * @brief   Generate Memory Instance ID for each Structured Memory Instance
  * @details 1. Used to generate Array Sizes and for checking the full array of memory instances
  *          2. The position is not used as of 10/5/2020
  ********************************************************************************************************************************
  */
/** pam procedure #4 #5 of Module insertion  :  add up total number of structured-memory and assign with an ID below **/
// Cannot use Enum, because need TOTAL_NUM_OF_STRUCT_MEM_INSTANCES pre-compiler for compiler directives
// - REVIEW: TOTAL_NUM_OF_STRUCT_MEM_INSTANCES only used pre-comile for Array Zero Detect and Size double check
#define STRUCT_MEM_ID_MODULE_FLASH_BUFFER      0
#define STRUCT_MEM_ID_MODULE_USART2_BUFFER     1
#define STRUCT_MEM_ID_MODULE_I2C_BUFFER        2
#define STRUCT_MEM_ID_MODULE_MC_STATEMACHINE_BUFFER 3
#define STRUCT_MEM_ID_MODULE_APP_BUFFER        4  //Pam: Add the structured-memory instance ID
#define STRUCT_MEM_ID_MODULE_EEP_CMD           5
#define STRUCT_MEM_ID_MODULE_DYNAMIC           6
#define STRUCT_MEM_ID_MODULE_ERR_LOGHANDLE     7
#define STRUCT_MEM_ID_MODULE_MODBUS            8
#define STRUCT_MEM_ID_MODULE_TEST              9
#define TOTAL_NUM_OF_STRUCT_MEM_INSTANCES      10  // should always be the last member of this enum

// Ensure that we do not declare arrays of size 0, if Struct Memory is not used in a project
#if TOTAL_NUM_OF_STRUCT_MEM_INSTANCES == 0
#define STRUCT_MEM_ARRAY_SIZE 1
#else
#define STRUCT_MEM_ARRAY_SIZE TOTAL_NUM_OF_STRUCT_MEM_INSTANCES
#endif

// Generate a warning if TOTAL_NUM_OF_STRUCT_MEM_INSTANCES != Amount declared in the Module Setup Tables
#define STRUCT_MEM_SETUP_TABLE_COUNT (MODULE_APP_TOTAL_STRUCT + MODULE_MC_STATEMACHINE_TOTAL_STRUCT + MODULE_EEP_CMD_TOTAL_STRUCT + MODULE_DYNAMIC_TOTAL_STRUCT + MODULE_I2C_TOTAL_STRUCT + \
                                  MODULE_ERR_LOGHANDLE_TOTAL_STRUCT + MODULE_USART2_TOTAL_STRUCT + MODULE_FLASH_TOTAL_STRUCT + MODULE_MODBUS_TOTAL_STRUCT + MODULE_TEST_TOTAL_STRUCT)
                                        
#if STRUCT_MEM_SETUP_TABLE_COUNT != TOTAL_NUM_OF_STRUCT_MEM_INSTANCES
#warning TOTAL_NUM_OF_STRUCT_MEM_INSTANCES does not equal amount derived from tables (STRUCT_MEM_SETUP_TABLE_COUNT)!
//printf("%d\n", XSTR(TOTAL_NUM_OF_STRUCT_MEM_INSTANCES))
#endif
/** pam procedure #4 #5 of Module insertion  :  add up total number of structured-memory and assign with an ID below end **/
/**
  ********************************************************************************************************************************
  * @brief   Generate Memory Instance ID for each Sequential Memory Instance
  * @details 1. Used to generate Array Sizes and for checking the full array of memory instances
  *          2. The position is not used as of 10/5/2020
  ********************************************************************************************************************************
  */
/** pam procedure #6 #7 of Module insertion  :  add up total number of Sequential Memory and assign with an ID below below **/
// Cannot use Enum, because need TOTAL_NUM_OF_SEQ_MEM_INSTANCES pre-compiler for compiler directives
// - REVIEW: TOTAL_NUM_OF_SEQ_MEM_INSTANCES only used pre-comile for Array Zero Detect and Size double check
#define SEQ_MEM_ID_MODULE_USART2_INTERNAL       0
#define SEQ_MEM_ID_MODULE_USART2_MODBUS         1
#define SEQ_MEM_ID_MODULE_USART2_TX             2
#define SEQ_MEM_ID_MODULE_I2C_TX                3
#define SEQ_MEM_ID_MODULE_I2C_RX                4
#define TOTAL_NUM_OF_SEQ_MEM_INSTANCES 5 // should always be the last member of this enum
//};
      
// Ensure that we do not declare arrays of size 0, if Struct Memory is not used in a project
#if TOTAL_NUM_OF_SEQ_MEM_INSTANCES == 0
#define SEQ_MEM_ARRAY_SIZE 1
#else
#define SEQ_MEM_ARRAY_SIZE TOTAL_NUM_OF_SEQ_MEM_INSTANCES
#endif
// Generate a warning if TOTAL_NUM_OF_SEQ_MEM_INSTANCES != Amount declared in the Module Setup Tables
#define SEQ_MEM_SETUP_TABLE_COUNT (MODULE_MC_STATEMACHINE_TOTAL_SEQ + MODULE_EEP_CMD_TOTAL_SEQ + MODULE_DYNAMIC_TOTAL_SEQ + MODULE_I2C_TOTAL_SEQ + \
                                  MODULE_ERR_LOGHANDLE_TOTAL_SEQ + MODULE_USART2_TOTAL_SEQ + MODULE_FLASH_TOTAL_SEQ + MODULE_MODBUS_TOTAL_SEQ + \
                                  MODULE_TEST_TOTAL_SEQ + MODULE_MEERKAT_SAFETY_CORE_TOTAL_SEQ  + MODULE_RTC_TOTAL_SEQ)
#if SEQ_MEM_SETUP_TABLE_COUNT != TOTAL_NUM_OF_SEQ_MEM_INSTANCES
#warning TOTAL_NUM_OF_SEQ_MEM_INSTANCES does not equal amount derived from tables (SEQ_MEM_SETUP_TABLE_COUNT)!
#endif


/** pam procedure #6 #7 of Module insertion  :  add up total number of Sequential Memory and assign with an ID below end **/
/**
  ********************************************************************************************************************************
  * @brief   
  * @details 
  * @param   moduleId_u8
  * @param   prevState_u8
  * @param   nextState_u8
  * @param   irqId_u8
  * @return  
  ********************************************************************************************************************************
  */
/** pam procedure #8 of Module insertion  :  declear a function prototype for the module body **/
uint8_t p_moduleApp_u32(uint8_t moduleId_u8, uint8_t prevState_u8, uint8_t nextState_u8,
                        uint8_t irqId_u8);
/** pam procedure #8 of Module insertion  :  declear a function prototype for the module body end **/

/**
  ********************************************************************************************************************************
  * @brief   
  * @details 
  * @param   drv_id_u8
  * @param   prevState_u8
  * @param   nextState_u8
  * @param   irqId_u8
  * @return  
  ********************************************************************************************************************************
  */
uint8_t module_Mc_StateMachine_u32(uint8_t drv_id_u8, uint8_t prevState_u8, uint8_t nextState_u8,
                           uint8_t irqId_u8);


/**
  ********************************************************************************************************************************
  * @brief   
  * @details 
  * @param   moduleId_u8
  * @param   prevState_u8
  * @param   nextState_u8
  * @param   irqId_u8
  * @return  
  ********************************************************************************************************************************
  */
uint8_t p_moduleECM_ICL_u32(uint8_t moduleId_u8, uint8_t prevState_u8, uint8_t nextState_u8,
                        uint8_t irqId_u8);

/**
  ********************************************************************************************************************************
  * @brief   
  * @details 
  * @param   moduleId_u8
  * @param   prevState_u8
  * @param   nextState_u8
  * @param   irqId_u8
  * @return  
  ********************************************************************************************************************************
  */
uint8_t p_module_voltage_doubler_u32(uint8_t moduleId_u8, uint8_t prevState_u8, uint8_t nextState_u8,
                        uint8_t irqId_u8);



/**
  ********************************************************************************************************************************
  * @brief   
  * @details 
  * @param   drv_id_u8
  * @param   prevState_u8
  * @param   nextState_u8
  * @param   irqId_u8
  * @return  
  ********************************************************************************************************************************
  */
uint8_t moduleEEP(uint8_t drv_id_u8, uint8_t prevState_u8, uint8_t nextState_u8,
                           uint8_t irqId_u8);


/**
  ********************************************************************************************************************************
  * @brief   
  * @details 
  * @param   drv_id_u8
  * @param   prevState_u8
  * @param   nextState_u8
  * @param   irqId_u8
  * @return  
  ********************************************************************************************************************************
  */
uint8_t module_err_log_u32(uint8_t drv_id_u8, uint8_t prevState_u8, uint8_t nextState_u8,
                           uint8_t irqId_u8);

/**
  ********************************************************************************************************************************
  * @brief   
  * @details 
  * @param   drv_id_u8
  * @param   prevState_u8
  * @param   nextState_u8
  * @param   irqId_u8
  * @return  
  ********************************************************************************************************************************
  */
uint8_t module_dynamic(uint8_t drv_id_u8, uint8_t prevState_u8, uint8_t nextState_u8,
                           uint8_t irqId_u8);

/**
  ********************************************************************************************************************************
  * @brief   
  * @details 
  * @param   drv_id_u8
  * @param   prevState_u8
  * @param   nextState_u8
  * @param   irqId_u8
  * @return  
  ********************************************************************************************************************************
  */
uint8_t moduleUsart2(uint8_t drv_id_u8, uint8_t prevState_u8, uint8_t nextState_u8,
                           uint8_t irqId_u8);

/**
  ********************************************************************************************************************************
  * @brief   
  * @details 
  * @param   drv_id_u8
  * @param   prevState_u8
  * @param   nextState_u8
  * @param   irqId_u8
  * @return  
  ********************************************************************************************************************************
  */
uint8_t moduleFlash_u32(uint8_t drv_id_u8, uint8_t prevState_u8, uint8_t nextState_u8,
                           uint8_t irqId_u8);

/**
  ********************************************************************************************************************************
  * @brief   
  * @details 
  * @param   drv_id_u8
  * @param   prevState_u8
  * @param   nextState_u8
  * @param   irqId_u8
  * @return  
  ********************************************************************************************************************************
  */
uint8_t moduleI2c_u32(uint8_t drv_id_u8, uint8_t prevState_u8, uint8_t nextState_u8,
                           uint8_t irqId_u8);

/**
  ********************************************************************************************************************************
  * @brief   
  * @details 
  * @param   drv_id_u8
  * @param   prevState_u8
  * @param   nextState_u8
  * @param   irqId_u8
  * @return  
  ********************************************************************************************************************************
  */
uint8_t p_moduleRTC_u32(uint8_t drv_id_u8, uint8_t prevState_u8, uint8_t nextState_u8,
                                 uint8_t irqId_u8); 

/**
  ********************************************************************************************************************************
  * @brief   
  * @details 
  * @param   drv_id_u8
  * @param   prevState_u8
  * @param   nextState_u8
  * @param   irqId_u8
  * @return  
  ********************************************************************************************************************************
  */
uint8_t MeerkatInterface_RunStateMachine(uint8_t drv_id_u8, uint8_t prevState_u8, uint8_t nextState_u8,
                                 uint8_t irqId_u8); // TODO: These are 'functions' and should not follow 'pointer' naming conventions


/**
  ********************************************************************************************************************************
  * @brief   
  * @details 
  * @param   drv_id_u8
  * @param   prevState_u8
  * @param   nextState_u8
  * @param   irqId_u8
  * @return  
  ********************************************************************************************************************************
  */
uint8_t p_moduleMeerkatSafetyCore_u32(uint8_t drv_id_u8, uint8_t prevState_u8, uint8_t nextState_u8,
                                 uint8_t irqId_u8); // TODO: These are 'functions' and should not follow 'pointer' naming conventions

/**
  ********************************************************************************************************************************
  * @brief   
  * @details 
  * @param   drv_id_u8
  * @param   prevState_u8
  * @param   nextState_u8
  * @param   irqId_u8
  * @return  
  ********************************************************************************************************************************
  */
uint8_t moduleADC1_u32(uint8_t drv_id_u8, uint8_t prevState_u8, uint8_t nextState_u8,
                                 uint8_t irqId_u8); // TODO: These are 'functions' and should not follow 'pointer' naming conventions

/**
  ********************************************************************************************************************************
  * @brief   
  * @details 
  * @param   drv_id_u8
  * @param   prevState_u8
  * @param   nextState_u8
  * @param   irqId_u8
  * @return  
  ********************************************************************************************************************************
  */

uint8_t moduleModbus(uint8_t drv_identifier_u8, uint8_t previous_state_u8, uint8_t next_state_u8,
                               uint8_t irq_identifier_u8);


/**
********************************************************************************************************************************
* @brief   State machine for Test Module
* @details
* @retval  return_state_u8
********************************************************************************************************************************
*/
uint8_t moduleTest_u32(uint8_t drv_identifier_u8, uint8_t previous_state_u8, uint8_t next_state_u8,
                            uint8_t irq_identifier_u8);

/**
  ********************************************************************************************************************************
  * @brief   Module Callback Intended to be calaled by the SysTick Interrupt
  * @details  Used by the Meerkat Clock Test
  * @param  
  ********************************************************************************************************************************
 */
void Meerkat_SafetyCore_SysTickCallback(void);

/**
  ********************************************************************************************************************************
  * @brief   Function pointer type definition for modules/drivers.
  * @details 
  ********************************************************************************************************************************
  */

typedef uint8_t (*p_process_u8)(uint8_t moduleId_u8, uint8_t prevState_u8, uint8_t nextState_u8,
                                uint8_t irqId_u8);

#pragma pack(1) // Specifies the packing alignment for data to be 1 byte, instead of the default value of 8 bytes, to save memory.

/**
  ********************************************************************************************************************************
  * @brief   Structure for all module/driver system control parameters.
  * @details The strucure for modules and drivers share the same space in memory. The parameters are similar, but not identical.
  *          Note on modules:
  *             Each module can be called by more than one driver's interrupt. (One 64-bit bit-oriented  parameter per driver.)
  *          Note on drivers:
  *             The driver interrupt will call the module defined by that driver's attachedModuleID
  *             The driver interrupt will pass control to the irqState_u8 of the associated module.
  ********************************************************************************************************************************
  */
typedef union {
    /**
    ******************************************************************************************************************************
    * @brief   Structure parameters for Sched_ModuleData struct.
    * @param   moduleId_u8     ID of the module currently being run by the scheduler.
    * @param   p_module_u32             Pointer to the module currently being run.
    * @param   totalSeqMemInstances_u8 Number of sequential memory instances allocated to the module currently being run.
    * @param   totalStructMemInstances_u8 Number of structured memory instances allocated to the module currently being run.
    * @param   prevState_u8        Reference to the last stage/state that this module ran.
    * @param   nextState_u8            Reference to the next stage/state that this module will run.
    * @param   irqState_u8       Reference to the interrupt stage/state for the module being run.
    * @param   processStatus_u8        This process is 0x00= running, 0xff= deleted, 0x05 = paused
    * @param   p_masterSharedMem_u32   Pointer to structured memory instance of the module being run.
    * @param   irqType_u8       this will indicate what kind of interrupt (for details please read the software interrupt document) 
    * @param   irqDat_u8         interrupt triggered module to pass data to response module
    * @param   irqDat1_len_u8    Parameter for interrupt triggered module to pass data to response module (multi-byte data use 
    *                                       data pointer of irqDatPt_u8 andthis byte will be data length of irqDatPt_u8 pointer) .
    * @param   irqDatPt_u8      Data pointer,usually set as NULL pointer for no extended data [for example only one byte if data] 
    *                                          (else irqDat_len_u8 = data length of this data pointer)
    ******************************************************************************************************************************
    */
    struct {
        uint8_t moduleId_u8;
        p_process_u8 p_module_u32;
        uint8_t totalSeqMemInstances_u8;
        uint8_t totalStructMemInstances_u8;
        uint8_t prevState_u8;
        uint8_t nextState_u8;
        uint8_t irqState_u8;
        uint8_t processStatus_u8;
        Ram_Buf_Handle p_masterSharedMem_u32;
        uint8_t irqType_u8;
        uint8_t irqDat_u8;
        uint8_t irqDat1_len_u8;
        uint8_t* irqDatPt_u8;
    } Sched_ModuleData;

    /**
    ******************************************************************************************************************************
    * @brief   Structure parameters for Sched_DrvData struct. 
    * @param   drvId_u8          ID of the driver currently being run by the scheduler. Same memory location as moduleId_u8.
    * @param   p_drv_u32         Pointer to the driver currently being run. Same memory location as p_module_u32.
    * @param   totalSeqMemInstances_u8       Number of sequential memory instances allocated to the driver currently being run.
    * @param   totalStructMemInstances_u8      Number of structured memory instances allocated to the driver currently being run.
    * @param   prevState_u8  Reference to the last stage/state that this driver ran.
    * @param   nextState_u8      Reference to the next stage/state that this driver will run.
    * @param   irqState_u8 Reference to the interrupt stage/state for the driver being run.
    * @param   processStatus_u8  This process is 0x00= running, 0xff= deleted, 0x05 = paused
    * @param   p_masterSharedMem_u32 Pointer to structured memory instance of the driver being run.
    * @param   irqType_u8       this will indicate what kind of interrupt (for details please read the software interrupt document) 
    * @param   irqDat_u8         interrupt triggered module to pass data to response module
    * @param   irqDat1_len_u8    Parameter for interrupt triggered module to pass data to response module (multi-byte data use 
    *                                       data pointer of irqDatPt_u8 andthis byte will be data length of irqDatPt_u8 pointer) .
    * @param   irqDatPt_u8      Data pointer,usually set as NULL pointer for no extended data [for example only two bytes if data] 
    *                                          (else irqDat_len_u8 = data length of this data pointer)
    ******************************************************************************************************************************
    */
    struct {
        uint8_t drvId_u8;
        p_process_u8 p_drv_u32;
        uint8_t totalSeqMemInstances_u8;
        uint8_t totalStructMemInstances_u8;
        uint8_t prevState_u8;
        uint8_t nextState_u8;
        uint8_t irqState_u8;
        uint8_t processStatus_u8;
        Ram_Buf_Handle p_masterSharedMem_u32;
        uint8_t irqType_u8;
        uint8_t irqDat_u8;
        uint8_t irqDat1_len_u8;
        uint8_t* irqDatPt_u8;
    } Sched_DrvData;

} ProcessInfo;

#pragma pack() // Declaration that the packing alignment return to the default of 8 bytes, from the temporary value of 1 byte.

/**
  ********************************************************************************************************************************
  * @brief   Initializes shared sequential and structured memory.
  * @details Frees previously allocated memory.
  ********************************************************************************************************************************
  */
uint8_t Sched_Initialize();

/**
  ********************************************************************************************************************************
  * @brief   Not used.
  * @details 
  ********************************************************************************************************************************
  */
uint8_t Sched_InitializeDrvs();

/**
  ********************************************************************************************************************************
  * @brief   Kernel scheduler main loop.
  * @details 
  ********************************************************************************************************************************
  */
void Sched_Run();

/**
  ********************************************************************************************************************************
  * @brief   Returns the index of processInfoTable corresponding to the specified moduleId_u8.
  * @details For each process, if the process is running, run the process and update the previous state and next state of that
  *             process.
  * @param   moduleId_u8
  * @return  Return the index of processInfoTable.
  ********************************************************************************************************************************
  */
uint8_t getProcessInfoIndex(uint8_t moduleId_u8);

/**
  ********************************************************************************************************************************
  * @brief   System Callback called by the SysTick Interrupt
  * @details  Used by the Meerkat Clock Test
  * @param  
  ********************************************************************************************************************************
  */
void HAL_SYSTICK_Callback(void);

/**
  ********************************************************************************************************************************
  * @brief   Initializes and starts the watchdog.
  * @details (IWDG_PR, IWDG_RLR and IWDG_WINR registers)
  * @param   timeout_u8
  ********************************************************************************************************************************
  */
 void Watchdog_Initialize(uint8_t timeout_u8);
/**
  ********************************************************************************************************************************
  * @brief   read system tick.
  * @details 
  * @param   
  ********************************************************************************************************************************
  */
uint64_t getSysCount(void);

/**
  ********************************************************************************************************************************
  * @brief   read minute counter
  * @details 
  * @param   
  ********************************************************************************************************************************
  */
uint64_t getMinCount(void);

/**
  ********************************************************************************************************************************
  * @brief   Reload (kick) the watchdog.
  * @details 
  * @param  
  ********************************************************************************************************************************
  */
void Watchdog_Reload(void);

/**
  ********************************************************************************************************************************
  * @brief   Setup a software interrupt 
  * @details find out and set the correct bit in the software interrupt bit table SoftwareIrqBitPt[IrqGroupIndx_u8] 
  * @param  
  ********************************************************************************************************************************
  */
void setupSoftwareIRQ(uint8_t SENDER_MODULE_ID, uint8_t RECEIVER_MODULE_ID, uint8_t _irqType_u8, uint8_t _irqDat_u8, uint8_t _irqDat1_len_u8, uint8_t * _irqDatPt_u8);

/**
  ********************************************************************************************************************************
  * @brief   CRC calculation
  * @details 
  * @param  
  ********************************************************************************************************************************
  */
uint16_t Calculate_CRC(uint16_t BufSize, unsigned char* aDataBuf);

/**
  ********************************************************************************************************************************
  * @brief   CRC calculation
  * @details 
  * @param  
  ********************************************************************************************************************************
  */
uint32_t Calculate_CRC32(uint16_t BufSize, unsigned char* aDataBuf);
/**
  ********************************************************************************************************************************
  * @brief   Danamic memory handling
  * @details 
  * @param  
  ********************************************************************************************************************************
  */
uint8_t reallocErrorINC(uint8_t addCount);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _SCHEDULER_H_ */