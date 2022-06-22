/**
  ***************************************************************************************************
  * @file    module_EEP.c 
  * @author  Doug Oda
  * @version V1.0
  * @date    11-May-2022
  * @brief   read and write up to 8 registers ( 16 bytes ) of eeprom
  * @note    
  ***************************************************************************************************
  */

#include "module_EEP.h"
#include "driver_I2c.h"
#include "module_i2c.h"
#include "scheduler.h"

extern ProcessInfo processInfoTable[];
ModuleEEPROM_Control moduleEEPROM_Control;

#define WAIT_I2C_FEEDBACK 1000

enum AppStates {
    INIT_MEMORY,
    INIT_APP,
    RUN_APP_WRITE_EEPROM,
    // additional states to be added here as necessary.
    RUN_APP_READ_EEPROM,
    STOP_APP = KILL_APP
};

/**
********************************************************************************
* @brief   Periodic function used to handle EEPROM read/write
* @details 
* @param   drv_id_u8 The var used by the sceduler to identify this module
*          prev_state_u8 Unused by this module
*          next_state_u8 State to be executed INIT_APP, RUN_APP_WRITE_EEPROM, RUN_APP_READ_EEPROM or KILL_MODULE
*          irq_id_u8 Unused by this module
* @return  Current State INIT_APP, RUN_APP_WRITE_EEPROM, RUN_APP_READ_EEPROM or KILL_MODULE
* @note Unused parameters are maintained because this is called by a pointer whose
*       definition has four uint8_t parameters.
********************************************************************************
*/
uint8_t moduleEEP(uint8_t module_id_u8, uint8_t prev_state_u8, uint8_t next_State_u8, uint8_t irq_id_u8)                 
{ 
  static int32_t waitI2cFeedbackCounter_s32 = -1;
  static uint8_t writeByteIndex_u8 =0;
  static I2c_Control* i2cControl_EEP;
  uint8_t returnStage = INIT_APP;  
  
  switch (next_State_u8)
    {
      case INIT_MEMORY:                                                             
        {  
          moduleEEPROM_Control.moduleEEPROM_Data.EEPROMEraseKey_u16 = EEPROM_ERASE_KEY;
          //Attach I2c shared memory into this App
          uint8_t I2cIndex  = getProcessInfoIndex(MODULE_I2C);              //return Process index from processInfo array with the Uart2 driver
          i2cControl_EEP = (I2c_Control*) ((*(processInfoTable[I2cIndex].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8);  
          //move on
          returnStage = INIT_APP ;
          break;
        }        
      
      case INIT_APP:                                                             
        { 
          waitI2cFeedbackCounter_s32 = -1;
          writeByteIndex_u8 =0;
          //init some modbus variables
          moduleEEPROM_Control.moduleEEPROM_Settings.adminFlags01_u16.is_ready = TRUE;
          moduleEEPROM_Control.moduleEEPROM_Settings.WriteRegisterCount_u16 = 0; 
          moduleEEPROM_Control.moduleEEPROM_Settings.ReadRegisterCount_u16 = 0;
          moduleEEPROM_Control.moduleEEPROM_Data.RegistersReadCount_u16 = 0;
          moduleEEPROM_Control.moduleEEPROM_Settings.EEPROMErase_u16 = 0;  
          //clear some memory
          RingBuf_ClearContents(i2cControl_EEP->SeqMemTX_u32);
          RingBuf_ClearContents(i2cControl_EEP->SeqMemRX_u32);
          //move on
          returnStage = RUN_APP_WRITE_EEPROM ;
          break;
        }       
      case RUN_APP_WRITE_EEPROM:
        { 
          returnStage = RUN_APP_WRITE_EEPROM;
          //check for writing data
          if( moduleEEPROM_Control.moduleEEPROM_Settings.WriteRegisterCount_u16 > 8 ){
            //error start over
            returnStage = INIT_APP;
          } else if(moduleEEPROM_Control.moduleEEPROM_Settings.WriteRegisterCount_u16 > 0){ 
            //let anyone who cares to check that EEPROM handler is busy
            moduleEEPROM_Control.moduleEEPROM_Settings.adminFlags01_u16.is_ready = FALSE;
            //check to ensuer driver is idle
            if (I2CIsReady()){
              uint16_t address_u16 = (uint16_t)writeByteIndex_u8 + moduleEEPROM_Control.moduleEEPROM_Settings.WriteAddress_u16; 
              uint8_t tmpryBuf[] = {0x02, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                                        (uint8_t)(address_u16), (uint8_t)(address_u16>>8),
                                         (uint8_t)(moduleEEPROM_Control.moduleEEPROM_Settings.WriteData[writeByteIndex_u8]),
                                          (uint8_t)(moduleEEPROM_Control.moduleEEPROM_Settings.WriteData[writeByteIndex_u8]>>8),0x03};
              unsigned DataLen2 = sizeof(tmpryBuf);
              RingBuf_WriteBlock( i2cControl_EEP->SeqMemRX_u32, tmpryBuf, &DataLen2);

              //keep writing bytes until all have been sent, 2 bytes at a time to the EEPROM
              //only do this a byte at a time because the driver is using hard delay loops
              if ( writeByteIndex_u8++ >= moduleEEPROM_Control.moduleEEPROM_Settings.WriteRegisterCount_u16){
                returnStage = INIT_APP;
              }
            }
          } else {
            //reading data
            if( moduleEEPROM_Control.moduleEEPROM_Settings.ReadRegisterCount_u16 > 8 ){
              //error start over
              returnStage = INIT_APP;
            } else if(moduleEEPROM_Control.moduleEEPROM_Settings.ReadRegisterCount_u16 > 0){ 
                uint8_t tmpryBuf[] = {0x02, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                                     (uint8_t)(moduleEEPROM_Control.moduleEEPROM_Settings.ReadAddress_u16),
                                     (uint8_t)(moduleEEPROM_Control.moduleEEPROM_Settings.ReadAddress_u16>>8),  module_id_u8, 0x03};
                unsigned DataLen2 = sizeof(tmpryBuf);
                RingBuf_WriteBlock( i2cControl_EEP->SeqMemRX_u32, tmpryBuf, &DataLen2); 
                //setup read back counter for time out the read try
                waitI2cFeedbackCounter_s32 = WAIT_I2C_FEEDBACK + getSysCount();             
                //jump to RUN_APP_READ_EEPROM stage for response
                returnStage = RUN_APP_READ_EEPROM;      
              }

             //erasing eeprom   
            if ( moduleEEPROM_Control.moduleEEPROM_Settings.EEPROMErase_u16  == EEPROM_KEY ){
              uint8_t tmpryBuf[] = {0x02, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03};
              unsigned DataLen2 = sizeof(tmpryBuf);
              RingBuf_WriteBlock( i2cControl_EEP->SeqMemRX_u32, tmpryBuf, &DataLen2); 
            }
          }
          break;
        }
      case RUN_APP_READ_EEPROM:
        { 
          returnStage = RUN_APP_READ_EEPROM;               
          //wait for the response of the Fixed parameter 
          if(RingBuf_GetUsedNumOfElements(i2cControl_EEP->SeqMemTX_u32) >= 13 ) { 
            uint8_t registerRdBuff[33];
            unsigned DataLen2 = sizeof(registerRdBuff);
            //check for any protenial result
            RingBuf_Observe(i2cControl_EEP->SeqMemTX_u32, registerRdBuff, 0, &DataLen2);  
           //verify that data is returned correctly                                                                                                         
            if(((( ((((uint16_t)registerRdBuff[3]) << 8)+ registerRdBuff[2]) == 0x00) && 
                               (registerRdBuff[12] == (uint8_t)moduleEEPROM_Control.moduleEEPROM_Settings.ReadAddress_u16)) && 
                               (registerRdBuff[13] == (uint8_t)(moduleEEPROM_Control.moduleEEPROM_Settings.ReadAddress_u16 >> 8))) && 
                               (registerRdBuff[14] == module_id_u8)) 
            { 
              //a valid reply
              moduleEEPROM_Control.moduleEEPROM_Settings.adminFlags01_u16.is_ready = TRUE;
              waitI2cFeedbackCounter_s32 = -1;
              
              DataLen2 = registerRdBuff[1] + 13;
              RingBuf_ReadBlock(i2cControl_EEP->SeqMemTX_u32, registerRdBuff, &DataLen2); //extract the whole frame
              
              // populate modbus input
              moduleEEPROM_Control.moduleEEPROM_Data.RegistersReadCount_u16 = 8;
              moduleEEPROM_Control.moduleEEPROM_Data.AddressRead_u16 = registerRdBuff[12];
              moduleEEPROM_Control.moduleEEPROM_Data.AddressRead_u16 |= (uint16_t)registerRdBuff[13] << 8; 
              for (uint8_t index_u8 = 0; index_u8 < 8; index_u8 ++){
                moduleEEPROM_Control.moduleEEPROM_Data.Data[index_u8] = registerRdBuff[(index_u8 * 2) + 15] | (uint16_t)registerRdBuff[(index_u8 * 2) + 16] << 8;
              }

            } 
            returnStage = INIT_APP; 
          }
          if ( waitI2cFeedbackCounter_s32 > 0 ){
            if( getSysCount() > waitI2cFeedbackCounter_s32) { 
              returnStage = INIT_APP;
            }
          }
          break;
        }
      default:
      case STOP_APP:
        {
          returnStage = STOP_APP;
          break;
        }  
    }
  return returnStage;
}
