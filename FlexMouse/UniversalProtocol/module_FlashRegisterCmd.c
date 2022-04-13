/**
  ***************************************************************************************************
  * @file    module_FlashUpdateCmd.c 
  * @author  Regal Pamela Lee
  * @version V1.0
  * @date    10-DEC-2020
  * @brief   Decode and perform group 4 CMD
  * @note    This App decode Group4 CMD in Universal protocol for all flash/setting update
  *          Register mode   : Each parameter as a register and register number note in FlashOffsetIndex(Enum)
  *                            @caution For Write-only parameter/s will not active until reset!!!!
  *          Flash Block mode: User should update the whole page of setting in a single/multi Page. 
  *                            when transfer a page of setting will be divided into 32bytes per block
  *                            always end with block0 (when receive block0 will finialize the whole update) 
  *                            and can start with any other blockx, 
  *                            @caution transfer must alsway contain the last block of the page for CRC  
  ***************************************************************************************************
  */

#include "module_FlashRegisterCmd.h"
#include "zz_module_flash.h"
#include "driver_usart1.h"
#include "ab_module_Mc_StateMachine.h"
#include "pmsm_motor_parameters.h"
#include "mc_tasks.h"
#include "mc_type.h"

extern ProcessInfo processInfoTable[];


Usart1_Control* usart1Control_FlashRegisterCmd;

typedef enum                                                            //data request cmd list
{   //match universal protocol (group4)                                 //please assign according to the universal protocol document
  SingleRegRd = 0x70,               //item0
  SingleRegWr,                      //item1
  SingleRegStatus,                  //item2
  GroupRegRd,
  GroupRegWr,
}FlashRegisterCMD;




/****************flash register local variable ********************************/
//#define flashBlockSize 32
//#define FLASH_PAGE_SIZE 0x800
#define FLASH_BLOCK_SETTING_PAGE ADDR_FLASH_PAGE_30
#define MIRROR_FLASH_BLOCK_SETTING_PAGE ADDR_FLASH_PAGE_31



//uint64_t tt_FlashFrameTimeOut;                                //
//#define FlashFrameTimeOutValue 2000000                          //Time out value between the last block Flash frame to the next frame
/****************flash register local variable end*****************************/

/************** periodic register value resent of motor data back to comBoard ***************************/
uint8_t ResendItems_bit = 0x00;                        //8 relatived register resend items, it's in bit mode bit0 = 1 => item0 is occupied .... bit7 = 1 => item7 is occupied
uint16_t RegReSend[] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};    //each of the register number storage array
uint64_t tt_RegPerioidTime[]   = {0,0,0,0,0,0,0,0};  //real time relatived resend value 
uint16_t RegPerioidTimeValue[] = {0,0,0,0,0,0,0,0};  //time period of each resend items in ms
/**************************************************************************************************************************/

typedef enum  
{
    INIT_APP,
    RUN_APP,
    CMDreply,
    // additional states to be added here as necessary.
    IRQ_APP = DEFAULT_IRQ_STATE,
    STOP_APP = KILL_APP
}AppStates;

Module_StateMachineControl*  module_StateMachineControl_FlashRegisterCmd;
uint16_t ResendPeriod = 0;
uint16_t regNum =0xffff;

#define ENABLE_PROTOCOLBUF_FLASHREGISTERCMD_FIXED_LEN 1
#if ENABLE_PROTOCOLBUF_FLASHREGISTERCMD_FIXED_LEN >= 1
  // This is a one-shot buffer, that is written to and read from in single calls.
  // - it does not currently need to be tracked for current index because of this.
  #define RX_FLASHREGISTERCMD_LENGTH 32 // This buffer is only for this command. Currently only about 11 bytes, Group Reads may increase later
  #define FIXED_PROTOCOLBUF_FLASHREGISTERCMD_MAX_LENGTH RX_FLASHREGISTERCMD_LENGTH // Inclusive (this value is accepted) 
  unsigned char fixedProtocolBuf_FlashRegisterCmd_Length = 0;
  unsigned char fixedProtocolBuf_FlashRegisterCmd[FIXED_PROTOCOLBUF_FLASHREGISTERCMD_MAX_LENGTH];
  unsigned char* protocolBuf_FlashRegisterCmd = fixedProtocolBuf_FlashRegisterCmd;
#else // if ENABLE_PROTOCOLBUF_FLASHREGISTERCMD_FIXED_LEN <= 0
  unsigned char* protocolBuf_FlashRegisterCmd;
#endif // if ENABLE_PROTOCOLBUF_FLASHREGISTERCMD_FIXED_LEN <= 0

uint8_t moduleFlashRegisterCmd_u32(uint8_t module_id_u8, uint8_t prev_state_u8, uint8_t next_State_u8, uint8_t irq_id_u8)                
{ 
  AppStates   returnStage = INIT_APP;
  switch (next_State_u8)
    {
      case INIT_APP:                                                              //initial stage
        {     
          /*Attach Uart2 shared memory into this App*/
          uint8_t Usart1index  = getProcessInfoIndex(MODULE_USART1);              //return Process index from processInfo array with the Uart2 driver
          usart1Control_FlashRegisterCmd = (Usart1_Control*) ((*(processInfoTable[Usart1index].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8);
          uint8_t Mc_StateMachineindex  = getProcessInfoIndex(MODULE_MC_STATEMACHINE);              //return Process index from processInfo array with the MC_statemachine module
          module_StateMachineControl_FlashRegisterCmd = (Module_StateMachineControl*) ((*(processInfoTable[Mc_StateMachineindex].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8);

          returnStage = RUN_APP ;
          break;
        }       
      case RUN_APP:
        { 
          unsigned int DataLen2 = (unsigned int)UniHeaderlen;
          if(RingBuf_GetUsedNumOfElements((*usart1Control_FlashRegisterCmd).seqMemRXG4L_u32) >= DataLen2 )
          { /** ------------------------------ pre-determine what state is in for Rx data is valid or not valid between blockmode transfer -------------------------**/
            // Fixed Length: No Need to Length Check here, buffer must be bigger than UniHeaderLen (constant)       
            #if ENABLE_PROTOCOLBUF_FLASHREGISTERCMD_FIXED_LEN <= 0
              if((protocolBuf_FlashRegisterCmd = (unsigned char*) realloc(protocolBuf_FlashRegisterCmd,DataLen2)) == NULL) reallocError++;     
            #endif // if ENABLE_PROTOCOLBUF_FLASHREGISTERCMD_FIXED_LEN <= 0
            RingBuf_Observe((*usart1Control_FlashRegisterCmd).seqMemRXG4L_u32, protocolBuf_FlashRegisterCmd, 0, &DataLen2);  

            //calculate the total number of frame
            DataLen2 = ((unsigned int)protocolBuf_FlashRegisterCmd[1] & 0x3F) + (unsigned int)UniHeaderlen;
            
            #if ENABLE_PROTOCOLBUF_FLASHREGISTERCMD_FIXED_LEN >= 1
              // Check for Buffer Space. Discard Data, if would cause overflow.
              // - Return if invalid.
              if (DataLen2 > FIXED_PROTOCOLBUF_FLASHREGISTERCMD_MAX_LENGTH) { // Normal Case: Buffer Overflow
                // Read All Data (Clear the Buffer)
                while (DataLen2 > 0) {
                  if (DataLen2 > FIXED_PROTOCOLBUF_FLASHREGISTERCMD_MAX_LENGTH) {
                    // REVIEW: Replace with RingBuf_ClearContents? Much less processing
                    unsigned int read_length = FIXED_PROTOCOLBUF_FLASHREGISTERCMD_MAX_LENGTH;
                    RingBuf_ReadBlock((*usart1Control_FlashRegisterCmd).seqMemRXG4L_u32, protocolBuf_FlashRegisterCmd, &read_length); //extract the whole frame
                    // RingBuf_ReadBlock((*usart1Control).seqMemTX_u32, headerFramebuf, &read_length);             //copy the complete frame into buffer
                    DataLen2 -= FIXED_PROTOCOLBUF_FLASHREGISTERCMD_MAX_LENGTH;
                  } else {
                    RingBuf_ReadBlock((*usart1Control_FlashRegisterCmd).seqMemRXG4L_u32, protocolBuf_FlashRegisterCmd, &DataLen2); //extract the whole frame
                    // RingBuf_ReadBlock((*usart1Control).seqMemTX_u32, headerFramebuf, &DataLen2);             //copy the complete frame into buffer
                    DataLen2 = 0;
                  }
                }
                // Exit Gracefully
                returnStage = CMDreply; 
                return returnStage;
              }            
            #else // if ENABLE_PROTOCOLBUF_FLASHREGISTERCMD_FIXED_LEN <= 0
              if((protocolBuf_FlashRegisterCmd = (unsigned char*) realloc(protocolBuf_FlashRegisterCmd,DataLen2)) == NULL) reallocError++;     //allocate the right frame size of memory for buffer
            #endif // if ENABLE_PROTOCOLBUF_FLASHREGISTERCMD_FIXED_LEN <= 0            
            
            RingBuf_ReadBlock((*usart1Control_FlashRegisterCmd).seqMemRXG4L_u32, protocolBuf_FlashRegisterCmd, &DataLen2); //extract the whole fram
            /**---------------------------------------------- decode and perform the CMD function ------------------------------------------**/
            switch((FlashRegisterCMD)protocolBuf_FlashRegisterCmd[2])
            {
              case SingleRegRd: 
                { //Single Register read
                  regNum = (((uint16_t)protocolBuf_FlashRegisterCmd[7])<< 8) +  protocolBuf_FlashRegisterCmd[8];
                  if( ResendPeriod =(((uint16_t)protocolBuf_FlashRegisterCmd[5])<< 8) +  protocolBuf_FlashRegisterCmd[6])
                  {              
                    if( ResendPeriod == 1) 
                      RegisterSend(regNum, RegisterRead(regNum));
                    else
                    { //setup reg resend command
                      if(ResendItems_bit == 0xFF)
                      { //register resend buffer full
                        unsigned char RegSendBufFul[] = {0x55, 0x01, 0x72, 0x00, 0x00, 0xE0, 0xCC, 0xCC}; //auto-resend read registers buffer full
                        unsigned int TxLen = sizeof(RegSendBufFul);
                        RingBuf_WriteBlock((*usart1Control_FlashRegisterCmd).seqMemTX_u32, RegSendBufFul, &TxLen); 
                      } 
                      else
                      { //check duplicate registr number already in buffer
                        uint8_t slotNum = 0;
                        for (; slotNum <= 7; slotNum++)
                        {
                          if(RegReSend[slotNum] == regNum)
                          {  //reNew the resend period
                             RegPerioidTimeValue[slotNum] = ResendPeriod;       //store register resend period in array      
                             break;
                          }
                        }
                        if(slotNum > 7)
                        { //find empty slot in Resend buffer
                          uint8_t tmpryShifter = 0x01;
                          for (slotNum = 0; slotNum <= 7; slotNum++)
                          {
                            if(!(ResendItems_bit & (tmpryShifter << slotNum))) //find empty slot
                            {
                              RegReSend[slotNum] = regNum;                       //store register number in array
                              RegPerioidTimeValue[slotNum] = ResendPeriod;       //store register resend period in array
                              ResendItems_bit |= (tmpryShifter << slotNum);
                              break;
                            }
                          }
                        }
                      }                        
                    }                    
                  }
                  else
                  { //remove register read auto-resend 
                    uint8_t slotNum = 0;
                    for (; slotNum <= 7; slotNum++)
                    { //search for the Register number in the list
                      if(RegReSend[slotNum] == regNum)
                      { //remove the resend from this slot
                        uint8_t tmpryShifter = 0x01;
                        RegReSend[slotNum] = 0xffff;
                        RegPerioidTimeValue[slotNum] = 0;
                        ResendItems_bit &= (~(tmpryShifter << slotNum));
                        break;
                      } 
                    }
                    if(slotNum > 7)
                    { //register number not in resend list
                        unsigned char RegNumNotInBuf[] = {0x55, 0x01, 0x72, 0x00, 0x00, 0xE1, 0xCC, 0xCC}; 
                        unsigned int TxLen = sizeof(RegNumNotInBuf);
                        RingBuf_WriteBlock((*usart1Control_FlashRegisterCmd).seqMemTX_u32, RegNumNotInBuf, &TxLen); 
                        break;
                    } 
                  }
                  break;
                }
              case SingleRegWr: 
                { //Single Register write      
                  regNum = (((uint16_t)protocolBuf_FlashRegisterCmd[5])<< 8) +  protocolBuf_FlashRegisterCmd[6];
                  uint16_t flashDat = (((uint16_t)protocolBuf_FlashRegisterCmd[7])<< 8) +  protocolBuf_FlashRegisterCmd[8];
                  if(FlashDatSet(regNum, flashDat)) //store new flash write value in buffer first
                  { //success store in flash buffer and update in ram
                    /** @todo update in ram function**/
                    //update in ram 
                    Reg2Ram(regNum, flashDat);
                  }
                  if(IsFlashBufFull())
                  { //temporary Flash write buffer is full, perform flash update
                    unsigned char RegflashUpdate2Main[] = {0x55, 0x01, 0x72, 0x00, 0x00, 0x03, 0xCC, 0xCC}; 
                    unsigned int TxLen = sizeof(RegflashUpdate2Main);
                    RingBuf_WriteBlock((*usart1Control_FlashRegisterCmd).seqMemTX_u32, RegflashUpdate2Main, &TxLen); 
                    /** @todo perform flash update**/
         //         uint8_t flashPageUpdate(uint8_t drv_id_u8, uint32_t _FrompageAddress, uint32_t _TopageAddress)
                    
                  }
                              
                  break;
                }
              case SingleRegStatus: 
                { //Register status CMD
                  /*
                  PerioidTimeValue[2] = (uint16_t)protocolBuf_FlashUpdateCmd[5] << 8;
                  PerioidTimeValue[2] += protocolBuf_FlashUpdateCmd[6];
                  if(PerioidTimeValue[2] > 1)
                  {     // if not one off cmd will start to remember the next wakeup time
                    tt_PerioidTime[2] = getSysCount() + PerioidTimeValue[2];                          //store time tick value
                  }*/
                  break;
                }
              case GroupRegRd: 
                { //Register group  Read

                  break;
                }
              case GroupRegWr: 
                { //Register group  write

                  break;
                }
              default:
                break;
            }
          }
          else
          {
//pam tmpry            if(flashBlockWrDat.flashWrState != idle) flashBlkWriteStateMachine_Run(module_id_u8);  //still in block flash stateMachine 
          }
          returnStage = CMDreply;
          break;
        }
      case CMDreply:
        {
          uint8_t CMDindex;
          for(CMDindex = 0; CMDindex < (sizeof(RegPerioidTimeValue)/sizeof(RegPerioidTimeValue[0])); CMDindex++)
          {
            uint8_t tmpryShifter = 0x01;
            if(((getSysCount() >= tt_RegPerioidTime[CMDindex]) && ( RegPerioidTimeValue[CMDindex] != 0)))
            {
                if(ResendItems_bit & (tmpryShifter << CMDindex))
                {  //current buffer ned send data
                    RegisterSend(RegReSend[CMDindex], RegisterRead(RegReSend[CMDindex]));
                    tt_RegPerioidTime[CMDindex] = getSysCount() + RegPerioidTimeValue[CMDindex]; 
                    break;
                }
            }
          }
          returnStage = RUN_APP ;
          break;
        }
      case IRQ_APP:
        {
          //if more than 1 driver interrupt attached to this APP
//           uint8_t index = getProcessInfoIndex(interruptIdentfer);         //return Process index from processInfo array of the driver interrupt call, APP can response respectively
          returnStage = RUN_APP;
          break;
        }               
      case STOP_APP:
        {
          returnStage = INIT_APP;
          break;
        }
      default:
        returnStage = STOP_APP;   
    }
  return returnStage;
}             

uint16_t RegisterRead(uint16_t _registerNum)                   //read 16bit at the register
{
  unsigned char* _pageAddress = (unsigned char*) FLASH_BLOCK_SETTING_PAGE;
  return (FlashRead(_pageAddress, (_registerNum * 2))); 
}

void RegisterSend(uint16_t _registerNumber, uint16_t _Data)
{
  unsigned char RegSendDat[] = {0x55, 0x04, 0x70, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xCC, 0xCC};
  uint32_t TxLen = sizeof(RegSendDat);
  RegSendDat[5] = (unsigned char) ((_registerNumber & 0xff00) >> 8);
  RegSendDat[6] = (unsigned char) _registerNumber & 0xff;
  RegSendDat[7] = (unsigned char) ((_Data & 0xff00) >> 8);
  RegSendDat[8] = (unsigned char) _Data & 0xff;
  RingBuf_WriteBlock((*usart1Control_FlashRegisterCmd).seqMemTX_u32, RegSendDat, &TxLen); 
}
                     
