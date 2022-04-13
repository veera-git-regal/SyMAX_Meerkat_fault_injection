/**
  ********************************************************************************************************************************
  * @file    module_gpio.c 
  * @author  Pamela Lee
  * @brief   Main driver module for GPIO
  * @details    
  ********************************************************************************************************************************
  */

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "driver_rtc.h"

/* scheduler handle declaration */
#include "scheduler.h"
// #include "sequential_memory.h"
// #include "structured_memory.h"

/* Content ---------------------------------------------------------------------------------------------------------------------*/
extern Ram_Buf_Handle sharedMemoryArray[TOTAL_NUM_OF_STRUCT_MEM_INSTANCES];
extern ProcessInfo processInfoTable[];

enum {
    INITIALIZE_MODULE,
    RUN_MODULE,
    // additional states to be added here as necessary.
    INTERRUPT_MODULE = DEFAULT_IRQ_STATE,
    KILL_MODULE = KILL_APP
};

uint8_t p_moduleRTC_u32(uint8_t driver_identifier_u8, uint8_t previous_state_u8, uint8_t next_state_u8,
                         uint8_t interrupt_identifier_u8) {
    uint8_t return_state_u8 = INITIALIZE_MODULE;
    switch (next_state_u8) {
        case INITIALIZE_MODULE: {
            // Initialize Real Time Clock.
            RTC_Init();

            // // Find the structured memory for the GPIO driver module, by searching for the GPIO onwer id.
            // Ram_Buffer_Handle this_ram_buffer_u32;
            // for (uint8_t i = 0; i < TOTAL_NUMBER_OF_STRUCTURED_MEMORY_INSTANCES; i++) {
            //     this_ram_buffer_u32 = &sharedMemoryArray[i];
            //     if (RamBuffer_GetOwner(this_ram_buffer_u32) == driver_identifier_u8) {
            //         gpio_shared_memory_location_u32 = &sharedMemoryArray[i];
            //     }
            // }

            // // Attach the structured memory to the process's master shared memory.
            // uint8_t index = getProcessInfoIndex(driver_identifier_u8);
            // if (index != INDEX_NOT_FOUND) {
            //     processInfoTable[index].Sched_DriverData.interruptState_u8 = PROCESS_STATUS_KILLED;
            //     processInfoTable[index].Sched_DriverData.p_masterSharedMemory_u32 = gpio_shared_memory_location_u32;
            //     processInfoTable[index].Sched_DriverData.attachedModuleID = MODULE_APP;
            // }

            return_state_u8 = RUN_MODULE;
            break;
        }
        case RUN_MODULE: {
            return_state_u8 = KILL_MODULE;
            break;
        }
        case KILL_MODULE: {
            // The GPIO driver module must only be executed once.
            // Setting processStatus_u8 to PROCESS_STATUS_KILLED prevents the scheduler main loop from calling this module again.
            uint8_t table_index_u8 = getProcessInfoIndex(driver_identifier_u8);
            if (table_index_u8 != INDEX_NOT_FOUND) {
                processInfoTable[table_index_u8].Sched_DrvData.processStatus_u8 = PROCESS_STATUS_KILLED;
            }
            return_state_u8 = KILL_MODULE;
            break;
        }
        default:
            return_state_u8 = 10;
            break;
    }
    return return_state_u8;
}