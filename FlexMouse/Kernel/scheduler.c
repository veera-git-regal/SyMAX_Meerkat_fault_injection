/**
  ********************************************************************************************************************************
  * @file    scheduler.c
  * @author  Pamela Lee
  * @brief   Implementation of c++ function/s for the kernel scheduler. 
  * @details Modules and drivers are collectively known as processes. This file contains the implementation of function related to
  *             scheduling processes and handling their respective driver interrupts, if any. The function scheduler_run is the
  *             'main loop' of the FlexMouse architecture.
  ********************************************************************************************************************************
  */

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "scheduler.h"

#include "main.h"
#include "mc_api.h"
#include "module_meerkat_interface_config.h"
//#define stackTest
#include "stm32f3xx_ll_iwdg.h"
//volatile uint64_t tickCounter = 0;
__root const uint32_t App_CRC @ "app_crc32_rom" = 0x00000000; // This is a placeholder for App Firmware Checksum (CRC32)

/*Software interrupt variable in bit oriented
  * SoftwareIrqBitPt[0] of binary IRQ process (from 0 to 63), 
  * SoftwareIrqBitPt[1] of binary IRQ process (from 64 to 127)
  * SoftwareIrqBitPt[2] of binary IRQ process (from 128 to 191)
  * SoftwareIrqBitPt[3] of binary IRQ process (from 192 to 254)
*/
uint64_t SoftwareIrqBitPt[] = { 0,0,0,0 };        
uint8_t  IrqTrigProcessID = 255;        //current module/process (ID) trigged IRQ 

uint64_t tickCounter; 

/* house keeping variable ------------------------------------------------------------------------------------------------------*/
uint64_t moduleStartTime;       //keep the the start time of systick for measure the run period of this module
uint64_t moduleRunTime;
#define maxModuleRunTimeLimit   200     // maxmium execution time for a module 200ms 
uint64_t IrqModuleStartTime;    //keep the the start time of systick for measure the run period of the IRQ response module
uint64_t IrqModuleRunTime;

uint64_t minCounter = 0;                //can store 584,542,046years in 64bit 

uint8_t  maxTimeModuleId = 255;
uint64_t maxModuleTimeUsage = 0;
uint8_t  maxTimeIRQ_ModuleId = 255;
uint64_t maxIRQTimeUsage = 0;
void Watchdog_Initialize(uint8_t timeout_u8) ;

// REVIEW: Placement

/* Content ---------------------------------------------------------------------------------------------------------------------*/
    /** pam procedure #9 of Module insertion  :  add the module/s into the kernal list **/
    /** @caution: please put the modules-parameters in the same sequence as the enum defined in the header file for consistance ID number reflect in processInfoTable[MODULE_ID] below**/
ProcessInfo processInfoTable[TOTAL_NUM_OF_PROCESSES] = {
    // Driver Modules
    {MODULE_USART1_ID, MODULE_USART1_FUNCTION_POINTER, MODULE_USART1_TOTAL_SEQ, MODULE_USART1_TOTAL_STRUCT, MODULE_USART1_PREV_STATE,
     MODULE_USART1_NEXT_STATE, MODULE_USART1_IRQ_STATUS, MODULE_USART1_PROCESS_STATUS, MODULE_USART1_MASTER_SHARED_MEM},
	    
     {MODULE_FLASH_ID, MODULE_FLASH_FUNCTION_POINTER, MODULE_FLASH_TOTAL_SEQ, MODULE_FLASH_TOTAL_STRUCT, MODULE_FLASH_PREV_STATE,
     MODULE_FLASH_NEXT_STATE, MODULE_FLASH_IRQ_STATUS, MODULE_FLASH_PROCESS_STATUS, MODULE_FLASH_MASTER_SHARED_MEM},
 
     {MODULE_I2C_ID, MODULE_I2C_FUNCTION_POINTER, MODULE_I2C_TOTAL_SEQ, MODULE_I2C_TOTAL_STRUCT, MODULE_I2C_PREV_STATE,
    MODULE_I2C_NEXT_STATE, MODULE_I2C_IRQ_STATUS, MODULE_I2C_PROCESS_STATUS, MODULE_I2C_MASTER_SHARED_MEM},
     
     // Application Modules
    {MODULE_MC_STATEMACHINE_ID , MODULE_MC_STATEMACHINE_FUNCTION_POINTER, MODULE_MC_STATEMACHINE_TOTAL_SEQ, MODULE_MC_STATEMACHINE_TOTAL_STRUCT, MODULE_MC_STATEMACHINE_PREV_STATE,
     MODULE_MC_STATEMACHINE_NEXT_STATE, MODULE_MC_STATEMACHINE_IRQ_STATUS, MODULE_MC_STATEMACHINE_PROCESS_STATUS, MODULE_MC_STATEMACHINE_MASTER_SHARED_MEM},
     
    {MODULE_APP_ID, MODULE_APP_FUNCTION_POINTER, MODULE_APP_TOTAL_SEQ, MODULE_APP_TOTAL_STRUCT, MODULE_APP_PREV_STATE,
     MODULE_APP_NEXT_STATE, MODULE_APP_IRQ_STATUS, MODULE_APP_PROCESS_STATUS, MODULE_APP_MASTER_SHARED_MEM},
     
    {MODULE_SHORT_CMD_ID, MODULE_SHORT_CMD_FUNCTION_POINTER, MODULE_SHORT_CMD_TOTAL_SEQ, MODULE_SHORT_CMD_TOTAL_STRUCT, MODULE_SHORT_CMD_PREV_STATE,
     MODULE_SHORT_CMD_NEXT_STATE, MODULE_SHORT_CMD_IRQ_STATUS, MODULE_SHORT_CMD_PROCESS_STATUS, MODULE_SHORT_CMD_MASTER_SHARED_MEM},
     
    {MODULE_EEP_CMD_ID, MODULE_EEP_CMD_FUNCTION_POINTER, MODULE_EEP_CMD_TOTAL_SEQ, MODULE_EEP_CMD_TOTAL_STRUCT, MODULE_EEP_CMD_PREV_STATE,
    MODULE_EEP_CMD_NEXT_STATE, MODULE_EEP_CMD_IRQ_STATUS, MODULE_EEP_CMD_PROCESS_STATUS, MODULE_EEP_CMD_MASTER_SHARED_MEM}, 
   
     {MODULE_REPLY_CMD_ID, MODULE_REPLY_CMD_FUNCTION_POINTER, MODULE_REPLY_CMD_TOTAL_SEQ, MODULE_REPLY_CMD_TOTAL_STRUCT, MODULE_REPLY_CMD_PREV_STATE,
     MODULE_REPLY_CMD_NEXT_STATE, MODULE_REPLY_CMD_IRQ_STATUS, MODULE_REPLY_CMD_PROCESS_STATUS, MODULE_REPLY_CMD_MASTER_SHARED_MEM},
     
    {MODULE_FLASH_UPDATE_CMD_ID, MODULE_FLASH_UPDATE_CMD_FUNCTION_POINTER, MODULE_FLASH_UPDATE_CMD_TOTAL_SEQ, MODULE_FLASH_UPDATE_CMD_TOTAL_STRUCT, MODULE_FLASH_UPDATE_CMD_PREV_STATE,
     MODULE_FLASH_UPDATE_CMD_NEXT_STATE, MODULE_FLASH_UPDATE_CMD_IRQ_STATUS, MODULE_FLASH_UPDATE_CMD_PROCESS_STATUS, MODULE_FLASH_UPDATE_CMD_MASTER_SHARED_MEM},
     
    {MODULE_FLASH_REGISTER_CMD_ID, MODULE_FLASH_REGISTER_CMD_FUNCTION_POINTER, MODULE_FLASH_REGISTER_CMD_TOTAL_SEQ, MODULE_FLASH_REGISTER_CMD_TOTAL_STRUCT, MODULE_FLASH_REGISTER_CMD_PREV_STATE,
     MODULE_FLASH_REGISTER_CMD_NEXT_STATE, MODULE_FLASH_REGISTER_CMD_IRQ_STATUS, MODULE_FLASH_REGISTER_CMD_PROCESS_STATUS, MODULE_FLASH_REGISTER_CMD_MASTER_SHARED_MEM},
     
     {MODULE_DEBUG_MODE_CMD_ID, MODULE_DEBUG_MODE_CMD_FUNCTION_POINTER, MODULE_DEBUG_MODE_CMD_TOTAL_SEQ, MODULE_DEBUG_MODE_CMD_TOTAL_STRUCT, MODULE_DEBUG_MODE_CMD_PREV_STATE,
     MODULE_DEBUG_MODE_CMD_NEXT_STATE, MODULE_DEBUG_MODE_CMD_IRQ_STATUS, MODULE_DEBUG_MODE_CMD_PROCESS_STATUS, MODULE_DEBUG_MODE_CMD_MASTER_SHARED_MEM},
     
    {MODULE_AUTOACK_ID, MODULE_AUTOACK_FUNCTION_POINTER, MODULE_AUTOACK_TOTAL_SEQ, MODULE_AUTOACK_TOTAL_STRUCT, MODULE_AUTOACK_PREV_STATE,
     MODULE_AUTOACK_NEXT_STATE, MODULE_AUTOACK_IRQ_STATUS, MODULE_AUTOACK_PROCESS_STATUS, MODULE_AUTOACK_MASTER_SHARED_MEM},

    {MODULE_ERR_LOGHANDLE_ID, MODULE_ERR_LOGHANDLE_FUNCTION_POINTER, MODULE_ERR_LOGHANDLE_TOTAL_SEQ, MODULE_ERR_LOGHANDLE_TOTAL_STRUCT, MODULE_ERR_LOGHANDLE_PREV_STATE,
     MODULE_ERR_LOGHANDLE_NEXT_STATE, MODULE_ERR_LOGHANDLE_IRQ_STATUS, MODULE_ERR_LOGHANDLE_PROCESS_STATUS, MODULE_ERR_LOGHANDLE_MASTER_SHARED_MEM},     

    {MODULE_ECM_ICL_ID, MODULE_ECM_ICL_FUNCTION_POINTER, MODULE_ECM_ICL_TOTAL_SEQ, MODULE_ECM_ICL_TOTAL_STRUCT, MODULE_ECM_ICL_PREV_STATE,
     MODULE_ECM_ICL_NEXT_STATE, MODULE_ECM_ICL_IRQ_STATUS, MODULE_ECM_ICL_PROCESS_STATUS, MODULE_ECM_ICL_MASTER_SHARED_MEM},

    {MODULE_VOLTAGE_DOUBLER_ID, MODULE_VOLTAGE_DOUBLER_FUNCTION_POINTER, MODULE_VOLTAGE_DOUBLER_TOTAL_SEQ, MODULE_VOLTAGE_DOUBLER_TOTAL_STRUCT, MODULE_VOLTAGE_DOUBLER_PREV_STATE,
     MODULE_VOLTAGE_DOUBLER_NEXT_STATE, MODULE_VOLTAGE_DOUBLER_IRQ_STATUS, MODULE_VOLTAGE_DOUBLER_PROCESS_STATUS, MODULE_VOLTAGE_DOUBLER_MASTER_SHARED_MEM},	 
         
        
     {MODULE_MEERKAT_ID, MODULE_MEERKAT_FUNCTION_POINTER, MODULE_MEERKAT_TOTAL_SEQ, MODULE_MEERKAT_TOTAL_STRUCT, MODULE_MEERKAT_PREV_STATE,
     MODULE_MEERKAT_NEXT_STATE, MODULE_MEERKAT_IRQ_STATUS, MODULE_MEERKAT_PROCESS_STATUS, MODULE_MEERKAT_MASTER_SHARED_MEM},     

    {MODULE_RTC_ID, MODULE_RTC_FUNCTION_POINTER, MODULE_RTC_TOTAL_SEQ, MODULE_RTC_TOTAL_STRUCT, MODULE_RTC_PREV_STATE,
     MODULE_RTC_NEXT_STATE, MODULE_RTC_IRQ_STATUS, MODULE_RTC_PROCESS_STATUS, MODULE_RTC_MASTER_SHARED_MEM},   
     
   };
/** pam procedure #9 of Module insertion  :  add the module/s into the kernal list end **/

uint64_t interruptRequestRegister_u64; // 64-Bit data structure for for storing big-wise driver interrupt requests flags.

uint8_t Sched_Initialize()  {
    StructMem_InitBufs();
    SeqMem_InitBufs();
   
    interruptRequestRegister_u64 = 0;

  Watchdog_Initialize(NUM_OF_625_MS_INCREMENTS);

    return TRUE;
}

uint64_t shifter_u8;
uint8_t IRQgroupItem;
uint8_t IrqGroupIndx_u8;
void Sched_Run()  {
    while (TRUE) {
        // Run each process with nextState_u8, save nextState_u8 as prevState_u8, and update nextState_u8 using return value.
        for (uint8_t table_index_u8 = 0; table_index_u8 < TOTAL_NUM_OF_PROCESSES; table_index_u8++) 
        {
            if (processInfoTable[table_index_u8].Sched_ModuleData.processStatus_u8 == PROCESS_STATUS_RUNNING) 
            {
                uint8_t current_state_u8 = processInfoTable[table_index_u8].Sched_ModuleData.nextState_u8;
                moduleStartTime = getSysCount();        //acquire time before execute the module//house keeping code                            //house keeping code 
                processInfoTable[table_index_u8].Sched_ModuleData.nextState_u8 =
                    (*processInfoTable[table_index_u8].Sched_ModuleData.p_module_u32)(
                        processInfoTable[table_index_u8].Sched_ModuleData.moduleId_u8,
                        processInfoTable[table_index_u8].Sched_ModuleData.prevState_u8,
                        processInfoTable[table_index_u8].Sched_ModuleData.nextState_u8,
                        processInfoTable[table_index_u8].Sched_ModuleData.irqState_u8);
               processInfoTable[table_index_u8].Sched_ModuleData.prevState_u8 = current_state_u8;     
               moduleRunTime = getSysCount() - moduleStartTime;                                             //get this module run time          //house keeping code      
               if( moduleRunTime > maxModuleTimeUsage) {                                                    //store the max run time module ID  //house keeping code  
                  maxModuleTimeUsage = moduleRunTime;                                                                                           //house keeping code  
                  maxTimeModuleId = table_index_u8;                                                                                             //house keeping code  
               }
            }
         //   ITM_EVENT8_WITH_PC(1,processInfoTable[table_index_u8].Sched_ModuleData.moduleId_u8);
            // Iterate through the interrupt register when an interrupt is present.
            // Handle each event sequentially by calling the interrupt handler of the driver's respective associated module.
            // Clear each event from the register after being handled.
            // After the event is handled, the module's previousStage_u8 remains unchanged, so normal operation can resume.
            for(IrqGroupIndx_u8 = 0; IrqGroupIndx_u8 < 4 ; IrqGroupIndx_u8++)
            {
              if( SoftwareIrqBitPt[IrqGroupIndx_u8] != 0)
              {
                shifter_u8 = SHIFTER;
                for(IRQgroupItem = 0; IRQgroupItem < 64 ; IRQgroupItem++)
                {
                  if (SoftwareIrqBitPt[IrqGroupIndx_u8] & shifter_u8) 
                    {
                      uint8_t effectiveID = IRQgroupItem + (IrqGroupIndx_u8 * 64);
                      IrqModuleStartTime = getSysCount();        //acquire time before execute the IRQ //house keeping code                    //house keeping code 
                        if (getProcessInfoIndex(effectiveID) != INDEX_NOT_FOUND) 
                        {
                            (*processInfoTable[effectiveID].Sched_ModuleData.p_module_u32)(
                                 processInfoTable[effectiveID].Sched_ModuleData.moduleId_u8,      //which software isr module ID
                                 processInfoTable[effectiveID].Sched_ModuleData.irqState_u8,                                                           //this meaningless for prevState in interrupt
                                 processInfoTable[effectiveID].Sched_ModuleData.irqState_u8,      //entry point/state for interrupt call back
                                 processInfoTable[effectiveID].Sched_ModuleData.irqState_u8);                                             //the interrupt triggered module 
                            SoftwareIrqBitPt[IrqGroupIndx_u8] &= ~shifter_u8;
                        }
                        IrqModuleRunTime = getSysCount() - IrqModuleStartTime;                                                   //get this IRQ run time   rollover(tested) //house keeping code 
                        if( IrqModuleRunTime > maxIRQTimeUsage) {                                                                //store the max IRQ run time and ID //house keeping code  
                            maxIRQTimeUsage = IrqModuleRunTime;                                                                                                      //house keeping code  
                            maxTimeIRQ_ModuleId = effectiveID;                                                                                                       //house keeping code  
                        }
                    }  
                  if((shifter_u8 <<= 1)  == 0) shifter_u8 = SHIFTER;
                }     
              }              
            }         
        }
        if(!meerkat_fault_occured)
        {         
         Watchdog_Reload(); // Reload IWDG counter.
         HAL_Delay(10);
        }
    }
}


uint8_t getProcessInfoIndex(uint8_t moduleId_u8)                                   //return Process index from processInfo array with the appID
{
    uint8_t idValue= 255;
    for(int i =0; i < TOTAL_NUM_OF_PROCESSES ; i++)
    {
       if(processInfoTable[i].Sched_ModuleData.moduleId_u8 == moduleId_u8)                               //find system appInfo of this driver
       {
          idValue = i;
       }
    }
    return idValue;                                                               //not found
}
/**
  * @brief  This function performs CRC calculation on BufSize bytes from input data buffer aDataBuf.
  * @param  BufSize Nb of bytes to be processed for CRC calculation
  * @retval 16-bit CRC value computed on input data buffer
  */
uint16_t Calculate_CRC(uint16_t BufSize, unsigned char* aDataBuf)              //Pam Tested
{
  register uint16_t index = 0;
  LL_CRC_ResetCRCCalculationUnit(CRC);
  /* Compute the CRC of Data Buffer array*/
  for (index = 0; index < BufSize ; index++)
  {
    LL_CRC_FeedData8(CRC,aDataBuf[index] );
  }
  /* Return computed CRC value */
  return (LL_CRC_ReadData16(CRC));
}

/**
  *************************************************************************************************************************************************************
  * @brief   Setup a software interrupt 
  * @details find out and set the correct bit in the software interrupt bit table SoftwareIrqBitPt[IrqGroupIndx_u8] 
  *             parameters:     SENDER_MODULE_ID        the module ID for the source of this interrupt
  *                             RECIVER_MODULE_ID       the module ID for the responding this interrupt
  *                             _irqType_u8             interrupt category of this interrupt
  *                             _irqDat_u8              data pass to the responding module
  *                             _irqDat1_len_u8         if _irqDatPt_u8 not equal to NULL this is the second byte of data pass to the responding module
  *                             _irqDatPt_u8            if data more than 2 byte can wrap it as pointer and use _irqDat1_len_u8 as the length of this data set
  * @return  
  *************************************************************************************************************************************************************
  */
void setupSoftwareIRQ(uint8_t SENDER_MODULE_ID, uint8_t RECIVER_MODULE_ID, uint8_t _irqType_u8, uint8_t _irqDat_u8, uint8_t _irqDat1_len_u8, uint8_t * _irqDatPt_u8) 
{ /**prepare software interrupt for the Ack timeout module**/                  
  uint8_t SoftwareIrqBitPtIndx = RECIVER_MODULE_ID / 64;     // get the interrupt pointer group of software IRQ point index
  uint64_t IrqBitTempry = 0x01;
  SoftwareIrqBitPt[SoftwareIrqBitPtIndx] |= IrqBitTempry << (RECIVER_MODULE_ID - (SoftwareIrqBitPtIndx * 64)); //set software interrupt trigger bit
  IrqTrigProcessID = SENDER_MODULE_ID;                                     /**set current module ID to let the IRQ response module know who triggered this interrupt  **/          
  //find out the ISR module and enter all the parameter for it to respone the interrupt, "this ACK time out error"
  uint8_t table_index_u8 = getProcessInfoIndex(RECIVER_MODULE_ID);  
  if (table_index_u8 != INDEX_NOT_FOUND) {
    processInfoTable[table_index_u8].Sched_DrvData.irqType_u8 = _irqType_u8;                     /**inform the interrupt response module this is an error message**/
    processInfoTable[table_index_u8].Sched_DrvData.irqDat_u8 = _irqDat_u8;                
    processInfoTable[table_index_u8].Sched_DrvData.irqDat1_len_u8 = _irqDat1_len_u8;      
    processInfoTable[table_index_u8].Sched_DrvData.irqDatPt_u8 = _irqDatPt_u8;                          //if no extend data so point to NULL                  
  }
}


#ifdef stackTest
/** @Note this routine is only for testing stack pointer for the max value **/
volatile char * SP_Ptr;
volatile uint32_t maxSP_Value = 0x20001760;

#pragma diag_suppress=Pe940 
#pragma inline = forced // Optimization at HIGH 
inline char * get_SP( void )
{
 /* On function exit,
 function return value should be present in R0 */
 asm( "MOV R0, SP" );
} 

char * Test_SP_Code_Ptr(void)
{
 char *Ptr;
 Ptr = get_SP();
 return Ptr;
} 

void HAL_SYSTICK_Callback(void) { //Using SysTick_Handler() instead //SPA
//    tickCounter++; //SPA
    if(!(tickCounter++ % 60000)) 
      minCounter++;
    SP_Ptr = Test_SP_Code_Ptr();
 //   maxSP_Value = (uint32_t)(SP_Ptr);
    if((uint32_t)SP_Ptr < maxSP_Value)  maxSP_Value = (uint32_t)SP_Ptr;
    
    #if DISABLE_MEERKAT_SAFETY_MODULE <= 0
    MeerkatInterface_SysTickCallback();
    #endif // DISABLE_MEERKAT_SAFETY_MODULE <= 0
}
#else
void HAL_SYSTICK_Callback(void) { //Using SysTick_Handler() instead //SPA
//    tickCounter++; //SPA
    if(!(tickCounter++ % 60000)) 
      minCounter++;
     #if DISABLE_MEERKAT_SAFETY_MODULE <= 0
   // MeerkatInterface_SysTickCallback();
    #endif // DISABLE_MEERKAT_SAFETY_MODULE <= 0

}
#endif

#if 0
void HAL_SYSTICK_Callback(void) {
    //tickCounter++;
#if DISABLE_MEERKAT_SAFETY_MODULE <= 0
    MeerkatInterface_SysTickCallback();
#endif // DISABLE_MEERKAT_SAFETY_MODULE <= 0

}
#endif
uint64_t getSysCount(void) 
{
  return tickCounter; 
}

uint64_t getMinCount(void) //minute counter start counting from power up
{
  return minCounter;
}
  

void Watchdog_Initialize(uint8_t timeout_u8) {
    LL_IWDG_Enable(IWDG);                             // Start the Independent Watchdog.
    LL_IWDG_EnableWriteAccess(IWDG);                  // Enable write access to IWDG registers.
    LL_IWDG_SetPrescaler(IWDG, LL_IWDG_PRESCALER_4);  // IWDG timer clock will be (LSI / 32).
    LL_IWDG_SetReloadCounter(IWDG, timeout_u8 * 625); // (timeout_s * 625) must be between Min_Data=0 and Max_Data=0x0FFF
    while (LL_IWDG_IsReady(IWDG) != TRUE)             // Wait for the registers to be updated
    {
    }
    LL_IWDG_ReloadCounter(IWDG); // Reload the IWDG counter (kick the dog for once!).
}

void Watchdog_Reload(void) {
    LL_IWDG_ReloadCounter(IWDG);
}

