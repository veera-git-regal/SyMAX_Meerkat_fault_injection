/**
  ***************************************************************************************************
  * @file    module_EEPCmd.c 
  * @author  Regal Pamela Lee
  * @version V1.0
  * @date    11-May-2021
  * @brief   Decode and perform group 5 CMD
  * @note    This App decode Group5 CMD (for EEprom)in Universal protocol
  ***************************************************************************************************
  */
//#include "pmsm_motor_parameters.h"
#include "module_EEPCmd.h"
//#include "mc_api.h"
#include "driver_usart1.h"
#include "driver_I2c.h"
//#include "ab_module_Mc_StateMachine.h"


extern uint64_t getSysCount(void);
/* SysCon handle declaration */
extern ProcessInfo processInfoTable[];

Usart1_Control* usart1Control_EEPCmd;
I2c_Control* i2cControl_EEPCmd;

enum AppStates {
    INIT_APP,
    RUN_APP,
    // additional states to be added here as necessary.
    Read_Fixed_parameter,
    IRQ_APP = DEFAULT_IRQ_STATE,
    STOP_APP = KILL_APP
};

//uint16_t adcFilterVal = 0;
//unsigned char* protocolBuf_EEPCmd[100] ;
uint8_t protocolBuf_EEPCmd[100] ;
#define waitI2cFeedback 1000
int16_t waitI2cFeedbackCounter = 0;
//^**Tips: APPs/Drivers adding process example step7 (Add the Additional funtion itself)
uint8_t moduleEEPCmd_u32(uint8_t module_id_u8, uint8_t prev_state_u8, uint8_t next_State_u8,
                        uint8_t irq_id_u8)                 
{ 
  uint8_t     returnStage = 0;  
  
  switch (next_State_u8)
    {
      case INIT_APP:                                                              //initial stage
        {
          /*Attach Uart2 shared memory into this App*/
          uint8_t Usart1index  = getProcessInfoIndex(MODULE_USART1);              //return Process index from processInfo array with the Uart2 driver
          usart1Control_EEPCmd = (Usart1_Control*) ((*(processInfoTable[Usart1index].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8);
          
          /*Attach I2c shared memory into this App*/
          uint8_t I2cIndex  = getProcessInfoIndex(MODULE_I2C);              //return Process index from processInfo array with the Uart2 driver
          i2cControl_EEPCmd = (I2c_Control*) ((*(processInfoTable[I2cIndex].Sched_ModuleData.p_masterSharedMem_u32)).p_ramBuf_u8);        

          /** @caution this is only for debug testing @MustDelete after test **/
      //      unsigned char I2cRegfeddbackBufDebg[] = {0x55, 0x02, 0x80, 0x00, 0x00, 0x0f, 0x00, 0xCC, 0xCC}; // Test read the BusVoltageMax
     //     unsigned char I2cRegfeddbackBufDebg[] = {0x55, 0x00, 0x8F, 0x00, 0x00, 0xCC, 0xCC}; // Test whole eeprom erase
     //       uint32_t TxLenDebg = sizeof(I2cRegfeddbackBufDebg);
    //        RingBuf_WriteBlock((*usart2Control_EEPCmd).seqMemRXG5_u32, I2cRegfeddbackBufDebg, &TxLenDebg); 
          /** @caution this is only for debug testing @MustDelete after test **/
          
          returnStage = RUN_APP ;
          break;
        }       
      case RUN_APP:
        { 
          unsigned int DataLen2 = (unsigned int)UniHeaderlen;
          returnStage = RUN_APP;
          if(RingBuf_GetUsedNumOfElements((*usart1Control_EEPCmd).seqMemRXG5_u32) >= DataLen2 )
          {        
           // if((protocolBuf_EEPCmd = (unsigned char*) realloc(protocolBuf_EEPCmd,DataLen2)) == NULL) reallocErrorINC(1);     
            RingBuf_Observe((*usart1Control_EEPCmd).seqMemRXG5_u32, protocolBuf_EEPCmd, 0, &DataLen2);  
            //calculate the total number of frame
            DataLen2 = ((unsigned int)protocolBuf_EEPCmd[1] & 0x3F) + (unsigned int)UniHeaderlen;
           // if((protocolBuf_EEPCmd = (unsigned char*) realloc(protocolBuf_EEPCmd,DataLen2)) == NULL) reallocErrorINC(1);     //allocate the right frame size of memory for buffer
            RingBuf_ReadBlock((*usart1Control_EEPCmd).seqMemRXG5_u32, protocolBuf_EEPCmd, &DataLen2); //extract the whole frame
            //decode and perform the CMD function
            switch(protocolBuf_EEPCmd[2])
            { //the command is 0x8X
              case 0x80:
                { //read EEprom fixed area (maintenance parameters)[5]                                          register number lsb     register number msb
                  uint8_t tmpryBuf[] = {0x02, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, protocolBuf_EEPCmd[5],  protocolBuf_EEPCmd[6],  module_id_u8, 0x03};
                  unsigned DataLen2 = sizeof(tmpryBuf);
                  RingBuf_WriteBlock( i2cControl_EEPCmd->SeqMemRX_u32, tmpryBuf, &DataLen2); 

                  waitI2cFeedbackCounter = waitI2cFeedback;             //setup read back counter for time out the read try
                  returnStage = Read_Fixed_parameter;                   //jump to Read_Fixed_parameter stage for response
                  break;
                }
              case 0x81:
                { //write EEprom fixed area as data
                  uint8_t tmpryBuf[] = {0x02, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, protocolBuf_EEPCmd[5],  protocolBuf_EEPCmd[6],  protocolBuf_EEPCmd[7], protocolBuf_EEPCmd[8], 0x03};
                  unsigned DataLen2 = sizeof(tmpryBuf);
                  RingBuf_WriteBlock( i2cControl_EEPCmd->SeqMemRX_u32, tmpryBuf, &DataLen2);                
                  break;
                }
              case 0x82:
                {
                 
                  break;
                }
              case 0x86:
                { 
                  
                  break;
                }
              case 0x84:
                {
               //  int32_t speed_target = protocolBuf_EEPCmd[6];
               //   speed_target += (int16_t) protocolBuf_EEPCmd[5] << 8;
               //   (*module_StateMachineControl_ShortCmd).command_Speed = speed_target;
                  
               //   uint8_t dir_target = (uint8_t) protocolBuf_EEPCmd[7];
                  
                //  if (dir_target == 6)(*module_StateMachineControl_ShortCmd).motorDir = 1;
               //   if (dir_target == 9) (*module_StateMachineControl_ShortCmd).motorDir = -1;

                  break;
                }
              default:
                break;
            }

          }
         // if((protocolBuf_EEPCmd = (unsigned char*) realloc(protocolBuf_EEPCmd,1)) == NULL) reallocErrorINC(1);;
        
          break;
        }
      case Read_Fixed_parameter:
        { //wait for the response of the Fixed parameter 
          //check the ring buffer with the current parameter
          returnStage = Read_Fixed_parameter;                   //jump to Read_Fixed_parameter stage for response
          uint8_t registerRdBuff[32];
          unsigned DataLen2 = sizeof(registerRdBuff);
          if(RingBuf_GetUsedNumOfElements(i2cControl_EEPCmd->SeqMemTX_u32) >= 13 )
          { //check for any protenial result
            RingBuf_Observe(i2cControl_EEPCmd->SeqMemTX_u32, registerRdBuff, 0, &DataLen2);  
                            //Command/type                                                      register(LSB)                                   register(MSB)                        
            if(((( ((((uint16_t)registerRdBuff[3]) << 8)+ registerRdBuff[2]) == 0x00) && (registerRdBuff[12] == protocolBuf_EEPCmd[5])) && (  registerRdBuff[13] == protocolBuf_EEPCmd[6])) && (registerRdBuff[14] == module_id_u8))
            { //a valid replied 
              DataLen2 = registerRdBuff[1] + 13;
              RingBuf_ReadBlock(i2cControl_EEPCmd->SeqMemTX_u32, registerRdBuff, &DataLen2); //extract the whole frame
              
              // send result back through universal procotol
              unsigned char I2cRegfeddbackBuf[] = {0x55, 0x12, 0x80, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xCC, 0xCC};
              uint32_t TxLen = sizeof(I2cRegfeddbackBuf);
              I2cRegfeddbackBuf[5] = registerRdBuff[12];
              I2cRegfeddbackBuf[6] = registerRdBuff[13];  
              I2cRegfeddbackBuf[7] = registerRdBuff[15];
              I2cRegfeddbackBuf[8] = registerRdBuff[16];  
              I2cRegfeddbackBuf[9] = registerRdBuff[17];
              I2cRegfeddbackBuf[10] = registerRdBuff[18];  
              I2cRegfeddbackBuf[11] = registerRdBuff[19];
              I2cRegfeddbackBuf[12] = registerRdBuff[20];  
              I2cRegfeddbackBuf[13] = registerRdBuff[21];
              I2cRegfeddbackBuf[14] = registerRdBuff[22];  
              
              I2cRegfeddbackBuf[15] = registerRdBuff[23];
              I2cRegfeddbackBuf[16] = registerRdBuff[24];  
              I2cRegfeddbackBuf[17] = registerRdBuff[25];
              I2cRegfeddbackBuf[18] = registerRdBuff[26];  
              I2cRegfeddbackBuf[19] = registerRdBuff[27];
              I2cRegfeddbackBuf[20] = registerRdBuff[28];  
              I2cRegfeddbackBuf[21] = registerRdBuff[29];
              I2cRegfeddbackBuf[22] = registerRdBuff[30]; 
              RingBuf_WriteBlock((*usart1Control_EEPCmd).seqMemTX_u32, I2cRegfeddbackBuf, &TxLen); 
              returnStage = RUN_APP;
            }
            else
            { //invalid frame will clear the i2c TX ring buffer and return error to Universal protocol
              RingBuf_ClearContents(i2cControl_EEPCmd->SeqMemTX_u32); //clear all content in buffer   
              // send result back through universal procotol
              unsigned char I2cRegfeddbackBuf[] = {0x55, 0x08, 0x80, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xCC, 0xCC}; //all data = 0xff mean error  
              uint32_t TxLen = sizeof(I2cRegfeddbackBuf);
              RingBuf_WriteBlock((*usart1Control_EEPCmd).seqMemTX_u32, I2cRegfeddbackBuf, &TxLen); 
              returnStage = RUN_APP;
            }     
          }
          if(waitI2cFeedbackCounter-- <= 0)            //check timeout up
          { // timeout, give up return to idle 
            RingBuf_ClearContents(i2cControl_EEPCmd->SeqMemTX_u32); //clear all content in buffer   
            // send result back through universal procotol
            unsigned char I2cRegfeddbackBuf[] = {0x55, 0x08, 0x80, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xCC, 0xCC}; //all data = 0xff mean error  
            uint32_t TxLen = sizeof(I2cRegfeddbackBuf);
            RingBuf_WriteBlock((*usart1Control_EEPCmd).seqMemTX_u32, I2cRegfeddbackBuf, &TxLen); 
            returnStage = RUN_APP;
          }
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

