/**
  ********************************************************************************************************************************
  * @file    module_i2c.h 
  * @author  Pamela Lee
  * @version V1.0
  * @date    15-Mar-2021
  * @brief   Main driver module for I2C Communication.
  * @details This module initializes the I2C port and attaches the pre-selected fixed memory allocation to the module.
  ********************************************************************************************************************************
  */

/* Define to prevent recursive inclusion ---------------------------------------------------------------------------------------*/
#ifndef _MODULE_I2C_H_
#define _MODULE_I2C_H_

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "main.h"
#include "typedef.h"

#include "scheduler.h"
#include "sequential_memory.h"
#include "structured_memory.h"

#define eepromSyncChr 0x02 // sync byte form eeprom frame (0x02 start of text)
#define eepromEndSyncChr 0x03  // end sync byte form eeprom frame (0x03 end of text)
#define DATLOGGER_HEADER_SIZE 0x0C
#define eepromInstall 1         //if no EEprom installed, please comment out this line

enum{ /** @caution please don't change the number to the items already with the number 
                   assignment for special arrangement have been make for these parameters **/
    Idfy_logDatAddr_dynPtr 		         	=0    ,	// register number
    //------------------------------------------------------------   

    Idfy_logDatAddr_DeratingOverPower                         , //1
    Idfy_logDatAddr_DeratingTemperature                       , //2
    Idfy_logDatAddr_DeratingOverCurrent                       , //3
    Idfy_logDatAddr_powerOnTimeCount0                   =4    , //4
    Idfy_logDatAddr_powerOnTimeCount1                         , //5
    Idfy_logDatAddr_powerOnTimeCount2                         , //6
    Idfy_logDatAddr_powerOnTimeCount3                         , //7

    Idfy_logDatAddr_MotorStart_RetryNSuccess                  , //8
    Idfy_logDatAddr_MotorStart_RetryNFail                     , //9
    Idfy_logDatAddr_MotorStart_CMDByUser                      , //10
    Idfy_logDatAddr_SpinMotor2TargetSpeedFail                 , //11
    Idfy_logDatAddr_WarmBootByWatchdog                        , //12
    Idfy_logDatAddr_AmbientTemperatureMax                     , //13
    Idfy_logDatAddr_AmbientOverTemperature                    , //14
    Idfy_logDatAddr_BusVoltageMax                       =15   , //15
    Idfy_logDatAddr_ShutdownCount                             , //16
    Idfy_logDatAddr_ReverseSpinCount                          , //17
    Idfy_logDatAddr_ReverseSpinCountNFail                     , //18
    Idfy_logDatAddr_OTFCount                                  , //19
    //-------------------------------------------------------------                        
    Idfy_logDatAddr_ULRamTestFail                             , //20
    Idfy_logDatAddr_ULRomTestFail                             , //21
    Idfy_logDatAddr_ULLostPhaseCurrent                        , //22
    Idfy_logDatAddr_ULClockTestFail                           , //23
    Idfy_logDatAddr_ULADCTestFail                             , //24
    Idfy_logDatAddr_ULWatchdogTimeout                         , //25
    Idfy_logDatAddr_UniProtNackCount                          , //26
    Idfy_logDatAddr_UniProtPacketDropCount                    , //27
    Idfy_logDatAddr_UniProtUnkownCmdCount                     , //28
    Idfy_logDatAddr_UniProtBufferFullCount                    , //29
    Idfy_logDatAddr_UniProtHeartBeatFailCount                 , //30
    Idfy_logDatAddr_STLibFOCDuration                          , //31
    Idfy_logDatAddr_STLibOverVoltage                          , //32
    Idfy_logDatAddr_STLibUnderVoltage                         , //33
    Idfy_logDatAddr_STLibOverHeat                             , //34
    Idfy_logDatAddr_STLibStartUpFailure                       , //35
    Idfy_logDatAddr_STLibSpeedFeedback                        , //36
    Idfy_logDatAddr_STLibOverCurrent                          , //37
    Idfy_logDatAddr_STLibSoftwareError                        , //38
    Idfy_logDatAddr_FlexMouseModuleBufferOverflow             , //39                                          
    Idfy_logDatAddr_FlexMouseFlashUpdateError                 , //40
    Idfy_logDatAddr_FlexMouseModuleExeTimeLimit               , //41
    
    Idfy_logDatAddr_EEPromEraseAll_0                          , //42
    Idfy_logDatAddr_EEPromEraseAll_1                          , //43
    
    Idfy_logDatAddr_EndOfRegister
};
/** @Knowbug as the eeprom page mode problem, please use the random access for each byte before the knownbug fixed, and don't change the 
             sequenceof the fix area,  **/     
/************************************************** Fixed EEprom area offset address ***************82 byte total ***********************/                          
#define  logDatAddr_dynPtr			    0x00						                  //uint16_t  
//------------------------------------------------------ General counter area -----------------------------------------------------------                           
#define  logDatAddr_DeratingOverPower                Idfy_logDatAddr_DeratingOverPower                  * 2	 	  //uint16_t           //54
#define  logDatAddr_DeratingTemperature              Idfy_logDatAddr_DeratingTemperature                * 2	 	  //uint16_t                               
#define  logDatAddr_DeratingOverCurrent              Idfy_logDatAddr_DeratingOverCurrent                * 2	 	  //uint16_t         
#define  logDatAddr_powerOnTimeCount                 Idfy_logDatAddr_powerOnTimeCount0                  * 2	 	  //uint64_t            //46               
//                                                     Idfy_logDatAddr_powerOnTimeCount1                      
//                                                     Idfy_logDatAddr_powerOnTimeCount2         
//                                                     Idfy_logDatAddr_powerOnTimeCount3                
#define  logDatAddr_MotorStart_RetryNSuccess         Idfy_logDatAddr_MotorStart_RetryNSuccess           * 2	 	  //uint16_t                               
#define  logDatAddr_MotorStart_RetryNFail            Idfy_logDatAddr_MotorStart_RetryNFail              * 2	 	  //uint16_t                               
#define  logDatAddr_MotorStart_CMDByUser             Idfy_logDatAddr_MotorStart_CMDByUser               * 2	 	  //uint16_t                               
#define  logDatAddr_SpinMotor2TargetSpeedFail        Idfy_logDatAddr_SpinMotor2TargetSpeedFail          * 2	 	  //uint16_t                               
#define  logDatAddr_WarmBootByWatchdog               Idfy_logDatAddr_WarmBootByWatchdog                 * 2	 	  //uint16_t                               
#define  logDatAddr_AmbientTemperatureMax            Idfy_logDatAddr_AmbientTemperatureMax              * 2	 	  //uint16_t                               
#define  logDatAddr_AmbientOverTemperature           Idfy_logDatAddr_AmbientOverTemperature             * 2	 	  //uint16_t                               
#define  logDatAddr_BusVoltageMax                    Idfy_logDatAddr_BusVoltageMax                      * 2	 	  //uint16_t                               
#define  logDatAddr_ShutdownCount                    Idfy_logDatAddr_ShutdownCount                      * 2	 	  //uint16_t                               
#define  logDatAddr_ReverseSpinCount                 Idfy_logDatAddr_ReverseSpinCount                   * 2	 	  //uint16_t                               
#define  logDatAddr_ReverseSpinCountNFail            Idfy_logDatAddr_ReverseSpinCountNFail              * 2	 	  //uint16_t                               
#define  logDatAddr_OTFCount                         Idfy_logDatAddr_OTFCount                           * 2	 	  //uint16_t      
//----------------------------------------------------------- Fault count area ----------------------------------------------------------                           
//       item name                                   -------------offset from the above item-----------------           item data type                        
#define  logDatAddr_ULRamTestFail                    Idfy_logDatAddr_ULRamTestFail                      * 2		  //uint16_t                                   
#define  logDatAddr_ULRomTestFail                    Idfy_logDatAddr_ULRomTestFail                      * 2	 	  //uint16_t                                         
#define  logDatAddr_ULLostPhaseCurrent               Idfy_logDatAddr_ULLostPhaseCurrent                 * 2	 	  //uint16_t                                         
#define  logDatAddr_ULClockTestFail                  Idfy_logDatAddr_ULClockTestFail                    * 2	 	  //uint16_t                                         
#define  logDatAddr_ULADCTestFail                    Idfy_logDatAddr_ULADCTestFail                      * 2	 	  //uint16_t                                         
#define  logDatAddr_ULWatchdogTimeout                Idfy_logDatAddr_ULWatchdogTimeout                  * 2	 	  //uint16_t                                         
#define  logDatAddr_UniProtNackCount                 Idfy_logDatAddr_UniProtNackCount                   * 2	 	  //uint16_t                                         
#define  logDatAddr_UniProtPacketDropCount           Idfy_logDatAddr_UniProtPacketDropCount             * 2	 	  //uint16_t                                         
#define  logDatAddr_UniProtUnkownCmdCount            Idfy_logDatAddr_UniProtUnkownCmdCount              * 2	 	  //uint16_t                                         
#define  logDatAddr_UniProtBufferFullCount           Idfy_logDatAddr_UniProtBufferFullCount             * 2	 	  //uint16_t                                         
#define  logDatAddr_UniProtHeartBeatFailCount        Idfy_logDatAddr_UniProtHeartBeatFailCount          * 2	 	  //uint16_t                                         
#define  logDatAddr_STLibFOCDuration                 Idfy_logDatAddr_STLibFOCDuration                   * 2	 	  //uint16_t                                         
#define  logDatAddr_STLibOverVoltage                 Idfy_logDatAddr_STLibOverVoltage                   * 2	 	  //uint16_t                                         
#define  logDatAddr_STLibUnderVoltage                Idfy_logDatAddr_STLibUnderVoltage                  * 2	 	  //uint16_t                                         
#define  logDatAddr_STLibOverHeat                    Idfy_logDatAddr_STLibOverHeat                      * 2	 	  //uint16_t                                         
#define  logDatAddr_STLibStartUpFailure              Idfy_logDatAddr_STLibStartUpFailure                * 2	 	  //uint16_t                                         
#define  logDatAddr_STLibSpeedFeedback               Idfy_logDatAddr_STLibSpeedFeedback                 * 2	 	  //uint16_t                                         
#define  logDatAddr_STLibOverCurrent                 Idfy_logDatAddr_STLibOverCurrent                   * 2	 	  //uint16_t                                         
#define  logDatAddr_STLibSoftwareError               Idfy_logDatAddr_STLibSoftwareError                 * 2	 	  //uint16_t                                         
#define  logDatAddr_FlexMouseModuleBufferOverflow    Idfy_logDatAddr_FlexMouseModuleBufferOverflow      * 2               //uint16_t                                   
#define  logDatAddr_FlexMouseFlashUpdateError        Idfy_logDatAddr_FlexMouseFlashUpdateError      	* 2	  	  //uint16_t                                     
#define  logDatAddr_FlexMouseModuleExeTimeLimit      Idfy_logDatAddr_FlexMouseModuleExeTimeLimit     	* 2	  	  //uint16_t        

#define  logDatAddr_EEPromEraseAll_0                 Idfy_logDatAddr_EEPromEraseAll_0                   * 2               //uint16_t    
#define  logDatAddr_EEPromEraseAll_1                 Idfy_logDatAddr_EEPromEraseAll_1                   * 2               //uint16_t                 

#define eepromInstall 1
/* Content ---------------------------------------------------------------------------------------------------------------------*/

int8_t I2C_Write(uint16_t writeAddr,uint8_t* writeBuf, uint8_t NumOfBytes);
void I2C_write_2Page(uint16_t writeFirstAddr, uint16_t writeSecondAddr, uint8_t* writeBuf, uint8_t NumOfBytes);
uint8_t* readBlockDat(uint16_t addr, uint8_t len);

//uint8_t* readBlockDat(uint16_t addr, uint8_t len);
int8_t  writeBlockDat(uint16_t addr, uint8_t *writeBuf, uint8_t len);
uint16_t  writeDynFrame(uint8_t * writeBuf, uint16_t writeLen);
int16_t  readDynFrame(uint16_t addr, uint8_t* readBuf);                //Read the first frame from 'addr' into 'readBuf' 
void EEprom_EraseAll(void);
void EEprom_RegisterUpdate(uint16_t RegisterNum);
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _MODULE_I2C_H_ */