/**
********************************************************************************************************************************
* @file    module_i2c.c 
* @author  Pamela Lee
* @version V1.00
* @date    15-Mar-2021
* @brief   Main driver module for I2C Communication.
* @details This module initializes the I2C port and attaches the pre-selected fixed memory allocation to the module.
********************************************************************************************************************************
*/

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "module_i2c.h"
#include "motorcontrol.h"
#include "driver_i2c.h"
#include "stm32f3xx_ll_i2c.h"


/* Content ---------------------------------------------------------------------------------------------------------------------*/
/* Uarts handle declaration */
extern Ram_Buf sharedMemArray[STRUCT_MEM_ARRAY_SIZE];
extern ProcessInfo processInfoTable[];
extern Ram_Buf *i2cStructMem_u32;
extern MCT_Handle_t MCT[NBR_OF_MOTORS];

uint8_t readBlockDatState = 0;         /**@Note //readBlockDat() state pointer variable, alway set to zero before calling readBlockDat()**/
uint8_t I2C_write_2PageState = 0;      /**@Note //I2C_write_2Page() state pointer variable, alway set to zero before calling I2C_write_2Page()**/
uint8_t I2C_WriteState = 0;            /**@Note //I2C_Write() state pointer variable, alway set to zero before calling I2C_Write()**/
uint8_t writeBlockDatState = 0;        /**@Note //writeBlockDat() state pointer variable, alway set to zero before calling writeBlockDat()**/
uint8_t writeDynFrameState = 0;        /**@Note //writeDynFrame() state pointer variable, alway set to zero before calling writeDynFrame()**/
uint8_t readDynFrameState = 0;         /**@Note //readDynFrame() state pointer variable, alway set to zero before calling readDynFrame()**/
uint8_t EEprom_EraseAllState = 0;
uint8_t EEprom_RegisterUpdateState = 0;

#define EEpromRamBufSize 200
unsigned char dataloggerRxBuf[EEpromRamBufSize] ;    //receive buffer for input ring buffer

uint64_t tt_I2cDelay = 0;               //EEprom internal read/write delay variable

uint64_t tt_I2cExeLimit = 0;            //variable for EEprom can stay execute for read/write

uint64_t tt_minDelay = 0;               //for minute counter delay
#define minDelayValue 1;               //store data to eeprom every 1 minute
uint8_t periodicUpdatDat[] = {Idfy_logDatAddr_powerOnTimeCount0, Idfy_logDatAddr_BusVoltageMax};
int64_t powOnOffset = -2;   //Change from -1 to -2 for the prevent confuse by the blank EEprom of all data(0xff).

uint16_t curVBus = 0;
uint64_t sumPowOn;
uint16_t* tmpryVBusEE;
uint8_t* tmpryPowOn;
uint8_t *tmpryVerifyDat;
uint16_t tmpryRegisterNum;

/** @Note "EEPROM_ACCESS_TIM_LIMIT" is the first time limit control for an stateMachine function **/
#define EEPROM_ACCESS_TIM_LIMIT 3      //constant of the EEprom can stay execute for read/write

/*
uint8_t  i2cTxBuffer[]  = {0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,\
                           0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f,\
                           0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f};

uint8_t tmpryWrDat[] = {0x02, 0x18, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
                        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x03};

uint8_t tmpryWrDat1[] = {0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
                        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
                        0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04};


uint8_t tmpryWrDat2[] = { 0x02, 0x18, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x34, 0x56, 0x78, 
                          0x9A, 0xBC, 0xDE, 0xF0, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0, 0x12, 0x34, 0x56, 0x78, 
                          0x9A, 0xBC, 0xDE, 0xF0, 0x03};


uint8_t tmpryWrDat3[] = { 0x02, 0x18, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x34, 0x56, 0x78, 
                          0x9A, 0xBC, 0xDE, 0xF0, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0, 0x12, 0x34, 0x56, 0x78, 
                          0x9A, 0xBC, 0xDE, 0xF0, 0x03};


uint8_t tmpryWrDat4[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
*/

uint8_t tmpryWrDat5[] = { 0x02, 0x03, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, Idfy_logDatAddr_ReverseSpinCount, 0x00, 0xAA, 0x03};


//uint8_t tmpryWrDat6[] = { 0x02, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03};

uint16_t lastFrameAddr = 0; //the last frame address read 

uint8_t* readBlockBuf;

uint8_t* readTestDat;
uint16_t FrameI2cCMD;
uint8_t updateParamIndex = 0;






//////debug only ///////   
int16_t validFrameAddr;
uint8_t *tmpryDebugDat;

uint8_t tmpryIndexI2c = 0;
int16_t finalAddr = 0;

uint8_t testCount = 0;
uint8_t debugBuf[180];

//////End of debug only ///////   

extern I2c_Control *i2cControl;

enum {
  INIT_MODULE,
  RUN_MODULE,
  // additional states to be added here as necessary.
  DECODE_MODULE,
  LOGGER_CMD_MODULE,
  STORAGE_CMD_MODULE,
  debug_module,
  CounterUpdate_MODULE,
  busUpdate_MODULE,
  busUpdate1_MODULE,
  pwOnTimUpdate_Module,
  pwOnTimUpdate1_Module,
  IRQ_MODULE = DEFAULT_IRQ_STATE,
  KILL_MODULE = KILL_APP
};

uint8_t i2cSeqMemRX_u32_buf[200];
uint8_t i2cSeqMemTX_u32_buf[200];
uint8_t i2cStructMem_u32_buf[sizeof(I2c_Control)];
uint8_t I2CRecBufA[maxSizeEEpromFrame];

void assign_I2C_ModuleMem(){  
  i2cSeqMemRX_u32 = SeqMem_CreateInstance(MODULE_I2C, 200, 
                              ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);//System call create a buffer for final packet receiver buffer 
  i2cSeqMemRX_u32->p_ringBuf_u8 = i2cSeqMemRX_u32_buf;
  
  i2cSeqMemTX_u32 = SeqMem_CreateInstance(MODULE_I2C, 200 , 
                              ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);//System call create a buffer for Tx data 
  i2cSeqMemTX_u32->p_ringBuf_u8 = i2cSeqMemTX_u32_buf;
  
  i2cStructMem_u32 =  StructMem_CreateInstance(MODULE_I2C, sizeof(I2c_Control), ACCESS_MODE_WRITE_ONLY, NULL, EMPTY_LIST);//System call create a structured memory for this driver [should map it back to this driver local struct]
  i2cStructMem_u32->p_ramBuf_u8 = i2cStructMem_u32_buf;
  
  i2cControl = (I2c_Control*)(*i2cStructMem_u32).p_ramBuf_u8;
  
  /** assign all the new generated sequential-memory of USART2 to the structured-memory **/
  i2cControl->SeqMemTX_u32 = i2cSeqMemTX_u32;
  i2cControl->SeqMemTX_u32->is_OverwrittingAllowed_u8 = FALSE;
  i2cControl->SeqMemRX_u32 = i2cSeqMemRX_u32;
  i2cControl->SeqMemRX_u32->is_OverwrittingAllowed_u8 = FALSE;
  i2cControl->errorCode_u8 = 0;
  i2cControl->I2C_Competed = 0;
  i2cControl->aReceiveBuffer = I2CRecBufA;
}

uint8_t moduleI2c_u32(uint8_t drv_id_u8, uint8_t prev_state_u8, uint8_t next_state_u8, uint8_t irq_id_u8) {
  uint8_t return_state_u8 = INIT_MODULE;
  switch (next_state_u8) 
  {
    case INIT_MODULE: 
    {
      // Initialize I2C
      MX_I2C1_Init();
      assign_I2C_ModuleMem();
     
      // Find the structured memory for the UART2 driver module, by searching for the UART2 onwer id.
      Ram_Buf_Handle this_ram_buf_u32;
      for (uint8_t struct_mem_index_u8 = 0; struct_mem_index_u8 < TOTAL_NUM_OF_STRUCT_MEM_INSTANCES;
      struct_mem_index_u8++) {
        this_ram_buf_u32 = &sharedMemArray[struct_mem_index_u8];
        if (RamBuf_GetOwner(this_ram_buf_u32) == drv_id_u8) {
          i2cStructMem_u32 = &sharedMemArray[struct_mem_index_u8];
        }
      }
      
      // Attach the structured memory to the process's master shared memory.
      uint8_t table_index_u8 = getProcessInfoIndex(drv_id_u8);
      if (table_index_u8 != INDEX_NOT_FOUND) {
        processInfoTable[table_index_u8].Sched_DrvData.irqState_u8 = DEFAULT_IRQ_STATE;
        processInfoTable[table_index_u8].Sched_DrvData.p_masterSharedMem_u32 =
          i2cStructMem_u32;
      }
      //Get structured memory 
      i2cControl = (I2c_Control*)((*(processInfoTable[table_index_u8].Sched_DrvData.p_masterSharedMem_u32)).p_ramBuf_u8);


      
      
    //Testing code below as single block of data the EEprom can cope with in raw
     
      /*
      SetcurrentFramePtr[] = {0, 00}; 
      I2C_WriteState = 0;
      do{
          I2C_Write(fixedAreaStartingAddr + logDatAddr_dynPtr , SetcurrentFramePtr, 2); 
      }while((I2C_WriteState != 0xF1) && (I2C_WriteState != 0xFE));


      writeDynFrameState = 0;
      do{
        writeDynFrame(tmpryWrDat2,sizeof(tmpryWrDat2));
      }while((writeDynFrameState != 0xF1) && (writeDynFrameState != 0xFE));
      
      
      writeDynFrameState = 0;
      do{
        writeDynFrame(tmpryWrDat2,sizeof(tmpryWrDat2));
      }while((writeDynFrameState != 0xF1) && (writeDynFrameState != 0xFE));
      
      
      writeDynFrameState = 0;
      do{
        writeDynFrame(tmpryWrDat2,sizeof(tmpryWrDat2));
      }while((writeDynFrameState != 0xF1) && (writeDynFrameState != 0xFE));

       readBlockDatState = 0;
      do{
        tmpryDebugDat = readBlockDat(00 , 180);
      }while((readBlockDatState != 0xF1) && (readBlockDatState != 0xFE));


      writeBlockDatState = 0;
      do{
        writeBlockDat(fixedAreaStartingAddr, tmpryWrDat4, sizeof(tmpryWrDat4));
      }while((writeBlockDatState != 0xF1) && (writeBlockDatState != 0xFE));

      readBlockDatState = 0;
      do{
        tmpryDebugDat = readBlockDat(fixedAreaStartingAddr , sizeof(tmpryWrDat4));
      }while((readBlockDatState != 0xF1) && (readBlockDatState != 0xFE));
*/
      
 //     unsigned DataLen2 = sizeof(tmpryWrDat5);
 //     RingBuf_WriteBlock(i2cControl->SeqMemRX_u32, tmpryWrDat5, &DataLen2); 
      
/* EEprom_EraseAllState = 0;
          do{
            EEprom_EraseAll();
          }while ((EEprom_EraseAllState != 0xF1) && (EEprom_EraseAllState!= 0xFE));
 */    
      /****** Check the Idfy_logDatAddr_EEPromEraseAll_0 is fill with the magic number for EEprom all erase ******/
      /** @brief  In order to erase the FULL EEprom, User can write the magic number to the 
      *          Idfy_logDatAddr_EEPromEraseAll_0 and Idfy_logDatAddr_EEPromEraseAll_1.
      *          The system will execute the ALL EEprom erase after reset                                        */     
#ifdef eepromInstall 
      
     //  EEprom_EraseAll();
      
      do
      {
        tmpryDebugDat = readBlockDat(( Idfy_logDatAddr_EEPromEraseAll_0 *2) + fixedAreaStartingAddr , 4);
      }while((readBlockDatState != 0xF1) && (readBlockDatState != 0xFE));
      
      if(*((uint32_t*)tmpryDebugDat) == 0x73126351) //compare the magic number
      {
        EEprom_EraseAllState = 0;
        do{
          EEprom_EraseAll();
        }while ((EEprom_EraseAllState != 0xF1) && (EEprom_EraseAllState!= 0xFE));              
      }
      /**********************************************************************************************************/
   
      tt_minDelay = getMinCount() ; 
#endif
      return_state_u8 = RUN_MODULE;
      break;
    }
    case RUN_MODULE: 
    {
      return_state_u8 = RUN_MODULE;
#ifdef eepromInstall 
      unsigned int DataLen = sizeof(dataloggerRxBuf);

      if(RingBuf_GetUsedNumOfElements(i2cControl->SeqMemRX_u32) >= DATLOGGER_HEADER_SIZE )
      {
        RingBuf_Observe(i2cControl->SeqMemRX_u32 , dataloggerRxBuf, 0, &DataLen);                 //get data from input pipe

        uint8_t protenETX;
        do                                                                                        //find and check a vaild frame
        {  
          while(dataloggerRxBuf[tmpryIndexI2c] != 0x02) tmpryIndexI2c++;                                //search for the STX 
          protenETX = dataloggerRxBuf[tmpryIndexI2c +1] + tmpryIndexI2c + DATLOGGER_HEADER_SIZE;
          tmpryIndexI2c++;
        }while((dataloggerRxBuf[protenETX] != 0x03) &&(tmpryIndexI2c <= EEpromRamBufSize-1)); //check this is a valid frame
          
        if(tmpryIndexI2c != 1)
        {//empty the front part of non-frame
          DataLen = tmpryIndexI2c - 1;                                                            //dump all byte before STX                                    
          RingBuf_ReadBlock(i2cControl->SeqMemRX_u32, dataloggerRxBuf, &DataLen); 
        }
        DataLen = dataloggerRxBuf[tmpryIndexI2c] + 13;                                                 //get the frame length
        RingBuf_ReadBlock(i2cControl->SeqMemRX_u32, dataloggerRxBuf, &DataLen);                   //extract the whole frame     
        return_state_u8 = DECODE_MODULE;
      }
      else
      {
        if(getMinCount() >= tt_minDelay)
        {
          tt_minDelay = getMinCount() + minDelayValue; 
          tt_I2cExeLimit = getSysCount() + EEPROM_ACCESS_TIM_LIMIT ; //set the eeprom exe time limit
          return_state_u8 = CounterUpdate_MODULE;
        }
      }
#endif
      break;
    }
    case DECODE_MODULE:
    {   //valid dataLogger frame store in "dataloggerRxBuf[]", length is in dataloggerRxBuf[1]
        return_state_u8 = STORAGE_CMD_MODULE;
        FrameI2cCMD =  (((uint16_t)dataloggerRxBuf[3]) << 8)+ dataloggerRxBuf[2] ;
        if( FrameI2cCMD < 0x100) return_state_u8 = LOGGER_CMD_MODULE;
        tt_I2cExeLimit = getSysCount() + EEPROM_ACCESS_TIM_LIMIT ; //set the eeprom 
        writeDynFrameState = 0;
        
        break;
    }
    case LOGGER_CMD_MODULE:
    {   //process command (for example, storge or increase counter in fixed area, read frame in fixed or dynamic area)
      return_state_u8 = RUN_MODULE;
      switch(FrameI2cCMD)
      {
        /******************************************************************************** EEprom Command 0x00 read register   **************************************************************************************************************************************/
        case 0x00:        //read register from EEprom and send result to SeqMemTX_u32
        {
          tmpryRegisterNum =  (((uint16_t)dataloggerRxBuf[13]) << 8)+ dataloggerRxBuf[12] ;
          readBlockDatState = 0;
          FrameI2cCMD = 0xEE00;
          return_state_u8 = LOGGER_CMD_MODULE;
          break;
        }       
        case 0xEE00:      //extension of command 0x00
        {
          return_state_u8 = LOGGER_CMD_MODULE;
          tmpryDebugDat = readBlockDat(( tmpryRegisterNum *2) + fixedAreaStartingAddr , 16);
          if((readBlockDatState == 0xF1) || (readBlockDatState == 0xFE))
          {// Read finish and send data out to SeqMemTX_u32
            uint8_t tmpryBuf[] = {0x02, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, dataloggerRxBuf[12],  dataloggerRxBuf[13],  dataloggerRxBuf[14], 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x03};
            tmpryBuf[15] = tmpryDebugDat[0];
            tmpryBuf[16] = tmpryDebugDat[1];
            tmpryBuf[17] = tmpryDebugDat[2];
            tmpryBuf[18] = tmpryDebugDat[3];
            tmpryBuf[19] = tmpryDebugDat[4];
            tmpryBuf[20] = tmpryDebugDat[5];
            tmpryBuf[21] = tmpryDebugDat[6];
            tmpryBuf[22] = tmpryDebugDat[7];
            
            tmpryBuf[23] = tmpryDebugDat[8];
            tmpryBuf[24] = tmpryDebugDat[9];
            tmpryBuf[25] = tmpryDebugDat[10];
            tmpryBuf[26] = tmpryDebugDat[11];
            tmpryBuf[27] = tmpryDebugDat[12];
            tmpryBuf[28] = tmpryDebugDat[13];
            tmpryBuf[29] = tmpryDebugDat[14];
            tmpryBuf[30] = tmpryDebugDat[15];
            unsigned DataLen2 = sizeof(tmpryBuf);
            RingBuf_WriteBlock(i2cControl->SeqMemTX_u32, tmpryBuf, &DataLen2); 
            return_state_u8 = RUN_MODULE;            
          }
          break;
        }

        
        /******************************************************************************** EEprom Command 0x01 write register   **************************************************************************************************************************************/
        case 0x01:        //write register from EEprom and send result to SeqMemTX_u32
        {
          tmpryRegisterNum =  (((uint16_t)dataloggerRxBuf[13]) << 8)+ dataloggerRxBuf[12] ;
          writeBlockDatState = 0;
          FrameI2cCMD = 0xEE01;
          return_state_u8 = LOGGER_CMD_MODULE;
          break;
        }       
        case 0xEE01:      //extension of command 0x01
        {
          return_state_u8 = LOGGER_CMD_MODULE;          
          tt_I2cExeLimit = getSysCount() + EEPROM_ACCESS_TIM_LIMIT ; //set the eeprom =
          do
          {//write LSB 
            writeBlockDat(( tmpryRegisterNum *2) + fixedAreaStartingAddr, &dataloggerRxBuf[14], 1);
          }while(((writeBlockDatState != 0xF1) && (writeBlockDatState != 0xFE)) && (getSysCount() < tt_I2cExeLimit));    //loop if not exist exetime limit and not finish read
          if((writeBlockDatState == 0xF1) || (writeBlockDatState == 0xFE)) 
          { //if eeprom write finished
            writeBlockDatState = 0;
            FrameI2cCMD = 0xEE02;
            return_state_u8 = LOGGER_CMD_MODULE;    
          }
          break;
        }
        case 0xEE02:      //further extension of command 0x01
        { // write MSB
          return_state_u8 = LOGGER_CMD_MODULE;         
          tt_I2cExeLimit = getSysCount() + EEPROM_ACCESS_TIM_LIMIT ; //set the eeprom 
          do
          {
            writeBlockDat(( tmpryRegisterNum *2) + fixedAreaStartingAddr +1, &dataloggerRxBuf[15], 1);
          }while(((writeBlockDatState != 0xF1) && (writeBlockDatState != 0xFE)) && (getSysCount() < tt_I2cExeLimit));    //loop if not exist exetime limit and not finish read
          if((writeBlockDatState == 0xF1) || (writeBlockDatState == 0xFE)) 
          { //if eeprom write finished
            return_state_u8 = RUN_MODULE;      
          }
          break;
        }
        
        /******************************************************************************** EEprom Command 0x10 increasement register counter***************************************************************************************************************************/
        case 0x10:      //write register into EEprom
        {
          tmpryRegisterNum = (((uint16_t)dataloggerRxBuf[13]) << 8)+ dataloggerRxBuf[12] ;
          if( tmpryRegisterNum < Idfy_logDatAddr_EndOfRegister)
          {//within the register number range
            if((( tmpryRegisterNum != 0) && ( tmpryRegisterNum != 4)) && ( tmpryRegisterNum != 15)) //check not write the system registers
            {
              FrameI2cCMD = 0xEE10;
              EEprom_RegisterUpdateState = 0;
              return_state_u8 = LOGGER_CMD_MODULE;
            }
          }
          break;
        }
        case 0xEE10:  //extension of command 0x10, the 
        {
          return_state_u8 = LOGGER_CMD_MODULE;
          do{
            EEprom_RegisterUpdate(tmpryRegisterNum);
          }while(((EEprom_RegisterUpdateState != 0xF1) && (EEprom_RegisterUpdateState != 0xFE)) && (getSysCount() < tt_I2cExeLimit));    //loop if not exist exetime limit and not finish read     
          tt_I2cExeLimit = getSysCount() + EEPROM_ACCESS_TIM_LIMIT ; //set the eeprom exe time limit
          if((EEprom_RegisterUpdateState == 0xF1) || (EEprom_RegisterUpdateState == 0xFE)) return_state_u8 = RUN_MODULE;
          break;
        }
        
        /******************************************************************************** EEprom Command 0xFF Rease all data in EEprom   ***************************************************************************************************************************/
        /** @pamNote Disable the above feature of 'All eeprom erase' for more secure method for erase*/
        /*
        case 0xff:          //whole chip erase // { 0x02, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03};
        {
          EEprom_EraseAllState = 0;
          do{
            EEprom_EraseAll();
          }while ((EEprom_EraseAllState != 0xF1) && (EEprom_EraseAllState!= 0xFE));
          break;
        }
        */
        default:
          break;
          //if no such cmd then back to idle
      }
      break;
    }
    case STORAGE_CMD_MODULE:
    {     //storge the frame into EEprom circular buffer
      return_state_u8 = STORAGE_CMD_MODULE;
      do
      {
        finalAddr = writeDynFrame(dataloggerRxBuf,dataloggerRxBuf[1] + DATLOGGER_HEADER_SIZE + 1);                         //write frame process need to run multiple time (as stateMachine) to complete the whole process
        if(getSysCount() >= tt_I2cExeLimit)
        { //write time limit reach.
          tt_I2cExeLimit = getSysCount() + EEPROM_ACCESS_TIM_LIMIT ; //set the eeprom 
          break;
        }
      }while((writeDynFrameState != 0xF1) && (writeDynFrameState != 0xFE)); //write frame process can run multiple times 

      if((writeDynFrameState == 0xF1) || (writeDynFrameState == 0xFE)) return_state_u8 = RUN_MODULE; //debug_module;     //check whole frame has been written if((writeDynFrameState == 0xF1) || (writeDynFrameState == 0xFE)) return_state_u8 = RUN_MODULE;     //check whole frame has been written 
      break;
    }
   
    case debug_module:
    {
      readBlockDatState = 0;
      do{
        tmpryDebugDat = readBlockDat(fixedAreaStartingAddr , 180);
      }while((readBlockDatState != 0xF1) && (readBlockDatState != 0xFE));
      return_state_u8 = RUN_MODULE;
      break;
    }
    /*********************************************************************** periodic update parameters area ********************************************************************/
    case CounterUpdate_MODULE:
    {
      if(updateParamIndex & 0x01)
      {
        return_state_u8 = busUpdate_MODULE;
        readBlockDatState = 0;
      }
      else
      {
        tt_I2cExeLimit = getSysCount() + EEPROM_ACCESS_TIM_LIMIT ; //set the eeprom 
        return_state_u8 = pwOnTimUpdate_Module;
        readBlockDatState = 0;
      }
      break;
    }
    case busUpdate_MODULE:
      { //update bus voltage max  
        return_state_u8 = busUpdate_MODULE; //prepare to loop in this state untill whole Read process finish
        
        tt_I2cExeLimit = getSysCount() + EEPROM_ACCESS_TIM_LIMIT ; //set the eeprom 
        do{
          tmpryVBusEE = (uint16_t*)readBlockDat(periodicUpdatDat[1]*2 + fixedAreaStartingAddr, 2);
        }while(((readBlockDatState != 0xF1) && (readBlockDatState != 0xFE)) && (getSysCount() < tt_I2cExeLimit));    //loop if not exist exetime limit and not finish read
        if((readBlockDatState == 0xF1) || (readBlockDatState == 0xFE)) 
        {
          curVBus = VBS_GetAvBusVoltage_V(PQD_MotorPowMeasM1.pVBS);
          if( curVBus > *tmpryVBusEE)  
          {//current value grater than the old one, write new higher value
            writeBlockDatState = 0;
            return_state_u8 = busUpdate1_MODULE; 
          }
          else
          { //no need to update to eeprom
            updateParamIndex++;
            return_state_u8 = RUN_MODULE;  
          }       
        }
        break;
      }
    case busUpdate1_MODULE:
      {
        return_state_u8 = busUpdate1_MODULE;      //prepare to loop in this state untill whole write process finish
        tt_I2cExeLimit = getSysCount() + EEPROM_ACCESS_TIM_LIMIT ; //set the eeprom 
        uint8_t tmpryOut[] = { (curVBus & 0xff), (curVBus & 0xff00) >> 8};
        do
        {
          writeBlockDat(periodicUpdatDat[1]*2 + fixedAreaStartingAddr, tmpryOut, 2);
        }while(((writeBlockDatState != 0xF1) && (writeBlockDatState != 0xFE)) && (getSysCount() < tt_I2cExeLimit));    //loop if not exist exetime limit and not finish read
        if((writeBlockDatState == 0xF1) || (writeBlockDatState == 0xFE)) 
        { //if eeprom write finished
          updateParamIndex++;
          return_state_u8 = RUN_MODULE;      
        }
        break;
      }
    case pwOnTimUpdate_Module:
      { //update motor power-on counter
        return_state_u8 = pwOnTimUpdate_Module;
        if(powOnOffset == -2) //will only appear at power up is -2
        { //just power up and check this motor have turn on before 
          
          tt_I2cExeLimit = getSysCount() + EEPROM_ACCESS_TIM_LIMIT ; //set the eeprom 
          do{
              tmpryPowOn = readBlockDat(periodicUpdatDat[0]*2 + fixedAreaStartingAddr, 8);
          }while(((readBlockDatState != 0xF1) && (readBlockDatState != 0xFE)) && (getSysCount() < tt_I2cExeLimit));    //loop if not exist exetime limit and not finish read
          
          if((readBlockDatState == 0xF1) || (readBlockDatState == 0xFE)) 
          {
            powOnOffset = 0;
            for (uint8_t tmpryIndx = 0; tmpryIndx < 8; tmpryIndx++)
            {
              if(tmpryPowOn[tmpryIndx] !=0 )
              {
                powOnOffset += ((uint64_t)tmpryPowOn[tmpryIndx]) << (8*tmpryIndx);
              }          
            }       
            if(powOnOffset > 0x1397F2080) powOnOffset = 0x1397F2080;  //10000years or 5,259,600,000Minute      
          }
        }
        else
        { 
          sumPowOn = powOnOffset + getMinCount(); //the first value just store the eeprom value +min count
          if(sumPowOn < 0x1397F2080)              //if powerup time more than the max then stay there
          {
            return_state_u8 = pwOnTimUpdate1_Module;
            writeBlockDatState = 0;        
          }
          else
          {
            updateParamIndex++;
            return_state_u8 = RUN_MODULE;
          }
        }
        break;
      }    
    case pwOnTimUpdate1_Module:
      {
        return_state_u8 = pwOnTimUpdate1_Module;
        tt_I2cExeLimit = getSysCount() + EEPROM_ACCESS_TIM_LIMIT ; //set the eeprom 
        // write the update value to register
        do
        {
          writeBlockDat(periodicUpdatDat[0]*2 + fixedAreaStartingAddr, (uint8_t*) &sumPowOn, 8);
        }while(((writeBlockDatState != 0xF1) && (writeBlockDatState != 0xFE)) && (getSysCount() < tt_I2cExeLimit));    //loop if not exist exetime limit and not finish read
        if((writeBlockDatState == 0xF1) || (writeBlockDatState == 0xFE)) 
        {
            updateParamIndex++;
            return_state_u8 = RUN_MODULE;
        }
        break;
      }
    case KILL_MODULE: {
      // The USART2 driver module must only be executed once.
      // Setting processStatus_u8 to PROCESS_STATUS_KILLED prevents the scheduler main loop from calling this module again.
      uint8_t table_index_u8 = getProcessInfoIndex(drv_id_u8);
      if (table_index_u8 != INDEX_NOT_FOUND) {
        processInfoTable[table_index_u8].Sched_DrvData.processStatus_u8 = PROCESS_STATUS_KILLED;
      }
      return_state_u8 = KILL_MODULE;
      break;
    }
    default: 
    {
      return_state_u8 = KILL_MODULE;
      break;
    }
  }
  return return_state_u8;
}

/** @brief   Write data in EEprom as single byte or page mode(16 bytes MAX)
  * @details EEprom write as byte write or page write mode (according to the number of NumOfBytes)
  *          This routine will also handle the block wrap around.
  * @param   writeAddr : starting address of EEprom write(include block address [last 3 bit in Device addressing field of I2C])
  *          writeBuf  : pointer of the write buffer                     
  *          NumOfBytes: data length will store into EEprom (16 byte Max, it will be truncated) 
  * @return  0 = success, 1= success and wrap around to starting block
**/
enum {
  START_I2C_Write = 0,            //general start state for stateMachine function as 0x00
  PREADDR_I2C_Write,
  PLACE_ADDR_I2C_Write,
  SINGLE_PAGE_I2C_Write,           //single byte or page write
  WITHIN_ONE_PAGE_I2C_Write,
  SEPERATE_2PAGE_I2C_Write,
  TWO_CHUNK_2PAGE_I2C_Write,
  WAIT_I2C_Write,
  ERROR_I2C_Write = 0xFE,         //general state for ERROR as 0xFE
  DONE_I2C_Write = 0xF1           //general state for this stateMachine function finished as 0xF1
};
int8_t I2C_Write(uint16_t writeAddr,uint8_t* writeBuf, uint8_t NumOfBytes)
{
  i2cControl->I2C_Competed = 0;
  if(NumOfBytes > EEpromPageModeSize) NumOfBytes = EEpromPageModeSize;                  //EEprom's page mode can support up to 16 byte.

  uint32_t effectiveAddr = ((writeAddr >> 7)& 0xfE) | SLAVE_OWN_ADDRESS;
  int8_t returnValue = 0;
  switch(I2C_WriteState)
  {
    case START_I2C_Write:
      {
        I2C_ReadAddr(writeAddr);
        tt_I2cDelay = getSysCount() + 1;  
        I2C_WriteState = PREADDR_I2C_Write;
        break;
      }
    case PREADDR_I2C_Write:
      {
        if((!(LL_I2C_IsActiveFlag_BUSY(I2C1))) && (getSysCount() >= tt_I2cDelay))  
        {
          I2C_WriteState = PLACE_ADDR_I2C_Write;
        }  
        break;
      }
    case PLACE_ADDR_I2C_Write:
      {
        LL_I2C_TransmitData8(I2C1, (uint8_t)(writeAddr & 0xff));
        if(NumOfBytes == 1) 
        {
          LL_I2C_EnableIT_TX(I2C1);
          i2cControl->aTransmitBuffer = writeBuf;
          LL_I2C_HandleTransfer(I2C1, effectiveAddr , LL_I2C_ADDRSLAVE_7BIT, 2, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE); 
          tt_I2cDelay = getSysCount() + 5;  
          I2C_WriteState = WAIT_I2C_Write;
        }
        else
        {
          if (!(((uint8_t)(writeAddr & 0xff))% EEpromPageModeSize))
          {
            I2C_WriteState = SINGLE_PAGE_I2C_Write;
          }
          else
          {/** @caution need to do read modify write **/      
            if((writeAddr + NumOfBytes) <= (fixedAreaStartingAddr + fixedAreaLen))                //check the write will exceed the same block 
            { //still within the same block(256byte)
              if((((writeAddr & 0xff) % EEpromPageModeSize ) + NumOfBytes) <= EEpromPageModeSize)     //check if write data plus the last data already in EEprom exceed a page(16 byte)
              { // still within a page(16byte)   
                I2C_write_2PageState = 0;
                I2C_WriteState = WITHIN_ONE_PAGE_I2C_Write;                    
              }
              else
              { //seperate into two page
                I2C_write_2PageState = 0;
                I2C_WriteState = SEPERATE_2PAGE_I2C_Write;    
              }
            }
            else
            {  /** @caution exceed the same block, need to seperate into two chunk of data into two page **/
              I2C_write_2PageState = 0;
              I2C_WriteState = TWO_CHUNK_2PAGE_I2C_Write;                  
            }           
          }
        }
        break;
      }
    case SINGLE_PAGE_I2C_Write:          //single byte or page write
      { //at the top of 16 byte page mode boundry
        //max length is 16 byte so must be within 1 block
        LL_I2C_EnableIT_TX(I2C1);
        i2cControl->aTransmitBuffer = writeBuf;
        LL_I2C_HandleTransfer(I2C1, effectiveAddr , LL_I2C_ADDRSLAVE_7BIT, NumOfBytes + 1 , LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE); 
        tt_I2cDelay = getSysCount() + 5;  
        I2C_WriteState = WAIT_I2C_Write;   
        break;
      }
    case WITHIN_ONE_PAGE_I2C_Write:
      {// still within a page(16byte)   
        I2C_write_2Page(writeAddr , 0xFFFF, writeBuf, NumOfBytes);                            //only a single page needed
        if((I2C_write_2PageState == 0xF1) || (I2C_write_2PageState == 0xFE)) 
        {
          tt_I2cDelay = getSysCount() + 5;
          I2C_WriteState = WAIT_I2C_Write;
        }
        break;
      }
    case SEPERATE_2PAGE_I2C_Write:
      { //seperate into two page
        I2C_write_2Page(writeAddr , writeAddr + EEpromPageModeSize, writeBuf, NumOfBytes);
        if((I2C_write_2PageState == 0xF1) || (I2C_write_2PageState == 0xFE)) 
        {
          tt_I2cDelay = getSysCount() + 5;
          I2C_WriteState = WAIT_I2C_Write;
        }
        break;
      }
    case TWO_CHUNK_2PAGE_I2C_Write:
      { /** @caution exceed the same block, need to seperate into two chunk of data into two page **/
        I2C_write_2Page(writeAddr , 0, writeBuf, NumOfBytes);                         //wrap around to the starting block address block
        if((I2C_write_2PageState == 0xF1) || (I2C_write_2PageState == 0xFE)) 
        {
          tt_I2cDelay = getSysCount() + 5;
          I2C_WriteState = WAIT_I2C_Write;
          returnValue = 1;                                                                   /** @caution this already wrap around to the starting block !!!!! **/
        }
        break;
      }
    case WAIT_I2C_Write:
      {
        if(getSysCount() >= tt_I2cDelay)        //wait for the last process delay 
        {
          I2C_WriteState = DONE_I2C_Write;
        }
        break;        
      }
    case ERROR_I2C_Write:     
      {
        break;
      }
    case DONE_I2C_Write:         
      {
        break;
      }
    default:
      {
        I2C_WriteState = ERROR_I2C_Write;
        break;
      } 
  }
  return(returnValue);
}

/** @brief   Write data in EEprom page mode(16 bytes MAX)
  * @details EEprom write as page write mode 
  * @param   writeFirstAddr : starting address of the first page of EEprom write (include block address [last 3 bit in Device addressing field of I2C])
  *                           This first page need to perform Read modify write 
  *          writeSecondAddr: starting address of EEprom write(include block address [last 3 bit in Device addressing field of I2C])
  *                           This second page no need to perform Read modify write for this always assume this page can be overlap or empty.
  *                           if = 0xffff means no second page write needed. 
  *          writeBuf  : pointer of the write buffer                     
  *          NumOfBytes: data length will store into EEprom (16 byte Max, it will be truncated) 
  * @return  No return;
**/
enum {
  START_I2C_write_2Page = 0,            //general start state for stateMachine function as 0x00
  READ_BLOCK_I2C_write_2Page,
  MARGE_N_WRITE_I2C_write_2Page,
  SECOND_PAGE_I2C_write_2Page,
  WAIT_I2C_write_2Page,
  ERROR_I2C_write_2Page = 0xFE,         //general state for ERROR as 0xFE
  DONE_I2C_write_2Page = 0xF1           //general state for this stateMachine function finished as 0xF1
};

void I2C_write_2Page(uint16_t writeFirstAddr, uint16_t writeSecondAddr, uint8_t* writeBuf, uint8_t NumOfBytes)
{ // the first page need read modify write
  uint16_t fisrtEffectivePageAddr =  ( writeFirstAddr - (writeFirstAddr % EEpromPageModeSize)); //calculate the first page addres starting from 0xXXX00 of that page  (writeFirstAddr & 0x700) +
  uint8_t  firstEffectiveDatLen = EEpromPageModeSize - (writeFirstAddr - fisrtEffectivePageAddr);
  static uint8_t* tmpryRdDat;
  switch(I2C_write_2PageState)
  {
    case START_I2C_write_2Page:
      {
        readBlockDatState = 0;
        tmpryRdDat = readBlockDat(fisrtEffectivePageAddr , EEpromPageModeSize);              //Read the page value into the buffer for Read modify write
        I2C_write_2PageState = READ_BLOCK_I2C_write_2Page;
        break;
      }
    case READ_BLOCK_I2C_write_2Page:     //read the first occupied block for marge with the current data
      { /**@caution this state is calling a lower level stateMachine finunction readBlockDat(), so need to keep checking and calling readBlockDat()
                    until this function done**/
        if((readBlockDatState == 0xF1) || (readBlockDatState == 0xFE)) //check if the block read function has finished
        {
          I2C_write_2PageState = MARGE_N_WRITE_I2C_write_2Page;
        }
        else
        { //clock the readBlockDat stateMachine
          tmpryRdDat = readBlockDat(fisrtEffectivePageAddr , EEpromPageModeSize);              //Read the page value into the buffer for Read modify write
        }
        break;
      }
    case MARGE_N_WRITE_I2C_write_2Page:
      {
        for(int8_t tmpryIndex = (EEpromPageModeSize - firstEffectiveDatLen); tmpryIndex < EEpromPageModeSize; tmpryIndex++)   //marge the input buffer data into the read data
        {
          *(tmpryRdDat + tmpryIndex) = writeBuf[tmpryIndex - (EEpromPageModeSize - firstEffectiveDatLen)];                    //copy the write data and marge with the read data
        }
        LL_I2C_EnableIT_TX(I2C1);
        i2cControl->aTransmitBuffer = tmpryRdDat;
        uint8_t i2cTxLen = firstEffectiveDatLen + (writeFirstAddr % EEpromPageModeSize);
        LL_I2C_TransmitData8(I2C1, (uint8_t)(fisrtEffectivePageAddr & 0xff));
        uint32_t effectiveAddr = ((fisrtEffectivePageAddr >> 7)& 0xfE) | SLAVE_OWN_ADDRESS;
        LL_I2C_HandleTransfer(I2C1, effectiveAddr , LL_I2C_ADDRSLAVE_7BIT, i2cTxLen + 1 , LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE); 
        tt_I2cDelay = getSysCount() + 5;  
        I2C_write_2PageState = SECOND_PAGE_I2C_write_2Page;
        break;
      }
    case SECOND_PAGE_I2C_write_2Page:
      {
        if(getSysCount() >= tt_I2cDelay)
        {
          if(writeSecondAddr != 0xffff)                                      //check need to proceed to the second page write 
          { // the second page don't need to use read modify write
            uint16_t secondEffectivePageAddr =  ( writeSecondAddr - (writeSecondAddr % EEpromPageModeSize));        //calculate the first page addres starting from 0xXXX00 of that page   
            LL_I2C_EnableIT_TX(I2C1);
            i2cControl->aTransmitBuffer = writeBuf + firstEffectiveDatLen;
            uint8_t i2cTxLen = EEpromPageModeSize - firstEffectiveDatLen ;
            LL_I2C_TransmitData8(I2C1, (uint8_t)(secondEffectivePageAddr & 0xff));
            uint32_t effectiveAddr = ((secondEffectivePageAddr >> 7)& 0xfE) | SLAVE_OWN_ADDRESS;
            LL_I2C_HandleTransfer(I2C1, effectiveAddr , LL_I2C_ADDRSLAVE_7BIT, i2cTxLen + 1 , LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);     
            tt_I2cDelay = getSysCount() + 5;  
          }
          I2C_write_2PageState = WAIT_I2C_write_2Page;
        }
        break;
      }
    case WAIT_I2C_write_2Page:
      {
        if(getSysCount() >= tt_I2cDelay)        //wait for the last process delay 
        {
          I2C_write_2PageState = DONE_I2C_write_2Page;
        }
        break;
      }
    case ERROR_I2C_write_2Page:
      {
        break;
      }
    case DONE_I2C_write_2Page:
      {
        break;
      }
    default:
      {
        I2C_write_2PageState = ERROR_I2C_write_2Page;   
      } 
  }
}

/** @brief   Read a block of data from eeprom
  * @details Read a block of data from eeprom
  * @param   addr : address to read from eeprom           
  *          len: length of data read
  * @return  the result pointer or if error return NULL
**/
enum {
  START_readBlockDat = 0,               //general start state for stateMachine function as 0x00
  ADDRESS_readBlockDat,
  READDATA_readBlockDat,
  WAIT4DATA_readBlockDat,
  ERROR_readBlockDat = 0xFE,            //general state for ERROR as 0xFE
  DONE_readBlockDat = 0xF1              //general state for this stateMachine function finished as 0xF1
};
uint8_t* readBlockDat(uint16_t addr, uint8_t len)
{
  if( (addr + len) > (fixedAreaStartingAddr + fixedAreaLen))
  {
    readBlockDatState = ERROR_readBlockDat;
  }
  switch(readBlockDatState)
  {
    case START_readBlockDat:
      {
         I2C_ReadAddr(addr);
         tt_I2cDelay = getSysCount() + 1;  
         readBlockDatState = ADDRESS_readBlockDat;
         break;
      }
    case ADDRESS_readBlockDat:
      {
        if((!(LL_I2C_IsActiveFlag_BUSY(I2C1))) && (getSysCount() >= tt_I2cDelay))  
        {
          I2C_ReadAddr(addr);           //write twice address to flush the buffer
          tt_I2cDelay = getSysCount() + 1;  
          readBlockDatState = READDATA_readBlockDat;
        }    
        break;
      }
    case READDATA_readBlockDat:
      {     
        if((!(LL_I2C_IsActiveFlag_BUSY(I2C1))) && (getSysCount() >= tt_I2cDelay))  
        {
          i2cControl->i2cRdDat = I2C_ReadCurrentAddr(addr, len);
          tt_I2cDelay = getSysCount() + 7;  
          readBlockDatState = WAIT4DATA_readBlockDat;
        }    
        break;    
      }
    case WAIT4DATA_readBlockDat:
      {
        if((!(LL_I2C_IsActiveFlag_BUSY(I2C1))) && (getSysCount() >= tt_I2cDelay))  
        {
          readBlockDatState = DONE_readBlockDat;
          return i2cControl->i2cRdDat;
        }
      }
    case ERROR_readBlockDat:
      {  //just indicate the process error
        break;          
      }
    case DONE_readBlockDat:
      {
        return i2cControl->i2cRdDat;
      }    
    default:
      {
        readBlockDatState = ERROR_readBlockDat;
        break;
      }
  }
  return NULL;        //out off memory error
}

/** @brief   write a block of data to eeprom
  * @details write a block of data to eeprom
  * @param   addr : address of eeprom to write    
  *          writeBuf : a block of data [max 76 byte]
  *          len: length of data write
  * @return  0 = success, negative = error
**/
enum {
  START_writeBlockDat = 0,               //general start state for stateMachine function as 0x00
  MULTI_PAGE_0_writeBlockDat,
  MULTI_PAGE_1_writeBlockDat,
  LAST_CHUNK_0_writeBlockDat,
  LAST_CHUNK_1_writeBlockDat,
  ERROR_writeBlockDat = 0xFE,            //general state for ERROR as 0xFE
  DONE_writeBlockDat = 0xF1              //general state for this stateMachine function finished as 0xF1
};
uint8_t writeBlockDatIndex = 0;         //this variable need to be global for stateMachine
int8_t  writeBlockDat(uint16_t addr, uint8_t *writeBuf, uint8_t len) /**@caution the content of writeBuf will be changed after this function called **/
{
  int8_t returnCode = 0;
  if( (addr + len) > (fixedAreaStartingAddr + fixedAreaLen))
  {
    returnCode = -1;        //out off memory error 
    writeBlockDatState = ERROR_writeBlockDat;
  }
  switch(writeBlockDatState)
  {
    case START_writeBlockDat:
      {
        writeBlockDatIndex = 0;
        if(len >= EEpromPageModeSize)
        {//process multiple page 
          writeBlockDatState = MULTI_PAGE_0_writeBlockDat;
        }
        else
        {
          writeBlockDatState = LAST_CHUNK_0_writeBlockDat;
        }
        break; 
      }    
    case MULTI_PAGE_0_writeBlockDat:
      {
        I2C_WriteState = 0;
        writeBlockDatState = MULTI_PAGE_1_writeBlockDat;
      }
    case MULTI_PAGE_1_writeBlockDat:
      {
        I2C_Write(addr + (writeBlockDatIndex * EEpromPageModeSize) , writeBuf + (writeBlockDatIndex * EEpromPageModeSize), EEpromPageModeSize);   //the 24AA16 cannot accept more then 16 bytes in page mode!!!!
        if((I2C_WriteState == 0xF1) || (I2C_WriteState == 0xFE))                                                        //check the I2C_Write() stateMachine is finish
        { //I2C_Write() stateMachine finished
          if(writeBlockDatIndex++ >= (len/EEpromPageModeSize))                                                           //check the multiple page part has finished
          { //all the multi-page chunk has finished
            writeBlockDatState = LAST_CHUNK_0_writeBlockDat;         
          }
          else
          { //finished current page, but still got page/s to send, then continue at MULTI_PAGE_1_writeBlockDat
            writeBlockDatState = MULTI_PAGE_0_writeBlockDat;
          }
        }
 //       else
 //       { //current page has not finished yet, so continue in MULTI_PAGE_1_writeBlockDat until this page finished
 //         
 //       }
        break;
      }
    case LAST_CHUNK_0_writeBlockDat:
      { //process the last chunk of data (less than a page of byte/s) 
        if(len % EEpromPageModeSize)            //check still got data less the a page of byte/s
        { //still got data less the a page of byte/s
          I2C_WriteState = 0;
          writeBlockDatState = LAST_CHUNK_1_writeBlockDat;
        }
        else
        { //the total data lenght is a multiple of page size, so no last chunk of data (less than a page of byte/s) avaliable
          writeBlockDatState = DONE_writeBlockDat;
        }
        break;
      }
    case LAST_CHUNK_1_writeBlockDat:
      { //
        I2C_Write(addr + (writeBlockDatIndex * EEpromPageModeSize), writeBuf  + (writeBlockDatIndex * EEpromPageModeSize), (len % EEpromPageModeSize));   // process the last group
        if((I2C_WriteState == 0xF1) || (I2C_WriteState == 0xFE))                                                                //check the I2C_Write() stateMachine is finish
        { //I2C_Write() stateMachine finished
          writeBlockDatState = DONE_writeBlockDat;
        }
        break; 
      }
    case ERROR_writeBlockDat:
      {
        break;        
      }
    case DONE_writeBlockDat:
      {
        break;    
      }
    default:
      {
        writeBlockDatState = ERROR_writeBlockDat;    
      }    
  } 
  return (returnCode);
}

/** @brief   Write a frame to the circular area of eeprom
  * @details Write a new frame to the circular area staring from the current frame pointer stored in fixed area
  * @param   writeBuf :            
  *          writeLen :
  * @return  0 =  error , >0 the pointer address
  * @note    if the frame 
**/
enum {
  START_writeDynFrame = 0,               //general start state for stateMachine function as 0x00
  ReadDynPtr_writeDynFrame,
  writeBlockDatLoop_writeDynFrame,
  I2C_WriteLoop0_writeDynFrame,
  I2C_WriteLoop1_writeDynFrame,
  ERROR_writeDynFrame = 0xFE,            //general state for ERROR as 0xFE
  DONE_writeDynFrame = 0xF1              //general state for this stateMachine function finished as 0xF1
};
uint8_t* currentFramePtr;
uint16_t currentPtr;
uint16_t  writeDynFrame(uint8_t * writeBuf, uint16_t writeLen)
{
  // read Dynamic Frame pointer from fixed eeprom location(logDatAddr_dynPtr)
 // uint16_t effectAddr = fixedAreaStartingAddr + logDatAddr_dynPtr;
  switch(writeDynFrameState)
  {
    case START_writeDynFrame:
	{
          readBlockDatState = 0;
          writeDynFrameState = ReadDynPtr_writeDynFrame;
          break;
        }         
    case ReadDynPtr_writeDynFrame:
        {
          currentFramePtr =  readBlockDat(fixedAreaStartingAddr + logDatAddr_dynPtr, 2); //get current frame pointer  
          if((readBlockDatState == 0xF1) || (readBlockDatState == 0xFE)) 
          {
            currentPtr = ((uint16_t)(*(currentFramePtr+1)) << 8) + *currentFramePtr;
            if( (currentPtr + writeLen) > dynamicAreaLen ) currentPtr = dynamicAreaStartingAddr; //check the buffer wraparound
            writeBlockDatState = 0;
            writeDynFrameState = writeBlockDatLoop_writeDynFrame;
          }
          break; 
        }
    case writeBlockDatLoop_writeDynFrame:
	{ //store the event frame into dynamic area
          writeBlockDat(currentPtr , writeBuf, writeLen);
          if((writeBlockDatState == 0xF1) || (writeBlockDatState == 0xFE)) 
          {
            writeDynFrameState = I2C_WriteLoop0_writeDynFrame;
          }
          break;
	}
    case I2C_WriteLoop0_writeDynFrame:
	{//calculate the new starting address in the circular buffer in EEprom and update Dynamic Frame pointer into EEprom
          currentPtr += writeLen; 
          I2C_WriteState = 0;
          writeDynFrameState = I2C_WriteLoop1_writeDynFrame;
          break;
	}
    case I2C_WriteLoop1_writeDynFrame:
	{
          I2C_Write(fixedAreaStartingAddr + logDatAddr_dynPtr ,(uint8_t*)&currentPtr, 2); 
          if((I2C_WriteState == 0xF1) || (I2C_WriteState == 0xFE)) 
          {
            writeDynFrameState = DONE_writeDynFrame;
          }
          break;
	}
    case ERROR_writeDynFrame:
	{
          break;
	}
    case DONE_writeDynFrame:
	{
          break;
	}
    default:
	{
          writeDynFrameState = ERROR_writeDynFrame;
	}   
  }
  return currentPtr;     
}

/** @brief   Read a frame from the circular area of eeprom
  * @details Read the first frame from 'addr' into 'readBuf' 
  * @param   addr     : searching start address within the circular buffer area
  *          readBuf  : the search result data will store into this buffer(no need to assign any memory read NOTE below). 
  *                     [ @caution NOTE  In order to prevent access outside the memory,
  *                                      User only need to assign a uint8_t pointer for 'readBuf'
  *                                      This function will allocate heap memory for the result]                  
  * @return  negative value when error
  *          zero value when cannot found any valid frame within (addr + maxSizeEEpromFrame) area
  *          positive value the starting address of the valid frame
**/
enum {
  START_readDynFrame = 0,               //general start state for stateMachine function as 0x00
  ReadMaxFrame_readDynFrame,
  Search4Sync_readDynFrame,
  searchValidFrame_readDynFrame,
  getFrameNotFm0_readDynFrame,
  getFrame_readDynFrame,
  ERROR_readDynFrame = 0xFE,            //general state for ERROR as 0xFE
  DONE_readDynFrame = 0xF1              //general state for this stateMachine function finished as 0xF1
};
uint8_t* frameResult;
uint8_t searchReadDynIndex = 0;
int16_t returnReadDynFrameValue = 0;
uint8_t SyncCharInFrame = 0;
uint8_t NumOfSyncStart = 0;                //Counter of number of 0x02 whithin this buffer
uint8_t eepromSyncChrPos[20];           //start of text as sync byte , eepromSyncChrPos[0] to eepromSyncChrPos[21]  will be the position of 0x02
uint8_t searchIndxRdFrame = 0;
uint16_t  protentialEndSync;
#pragma diag_suppress=Pe550             //the warning message supressed for the pointer "readBuf" just for return the read data only
int16_t  readDynFrame(uint16_t addr, uint8_t* readBuf)
{  
  switch(readDynFrameState)
  {
    case START_readDynFrame:
    {
      returnReadDynFrameValue = 0;
      SyncCharInFrame = 0;
      readBlockDatState = 0;
      NumOfSyncStart = 0; 
      readDynFrameState = ReadMaxFrame_readDynFrame;
      break;
    }
    case ReadMaxFrame_readDynFrame:
    {   //get a max size of frame into buffer
      frameResult = readBlockDat(addr, maxSizeEEpromFrame);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
      if((readBlockDatState == 0xF1) || (readBlockDatState == 0xFE)) 
      {
        searchReadDynIndex = 0;
        readDynFrameState = Search4Sync_readDynFrame;
      }
      break;
    }   
    case Search4Sync_readDynFrame:
    {   //search for the first sync byte
      for(; searchReadDynIndex < maxSizeEEpromFrame; searchReadDynIndex++)
      {
        if(frameResult[searchReadDynIndex] == eepromSyncChr) //search sync byte 
        {
          eepromSyncChrPos[NumOfSyncStart] = searchReadDynIndex;
          NumOfSyncStart++;
        }
      }
      if(NumOfSyncStart == 0)
      { //sync byte not found
        returnReadDynFrameValue =  0;
        readDynFrameState = DONE_readDynFrame;
      }
      else  //sync byte found
      {
        searchIndxRdFrame = 0;
        readDynFrameState = searchValidFrame_readDynFrame;
      }
      break;
    }   
    case searchValidFrame_readDynFrame:
    {
      returnReadDynFrameValue = 0;
      do
      {                         //   frame data length                           +       offset position               + header size
        protentialEndSync = frameResult[eepromSyncChrPos[searchIndxRdFrame] + 1] + eepromSyncChrPos[searchIndxRdFrame] + DATLOGGER_HEADER_SIZE; //calculate the protential end sync byte
        if(frameResult[protentialEndSync] == eepromEndSyncChr) 
        { //a valid frame found
          if(eepromSyncChrPos[searchIndxRdFrame] == 0)          
          { // syn byte at the very beginning 
            readBuf = frameResult;     //copy the pointer from internal to result pointer
            returnReadDynFrameValue = (addr); //successfully return the first frame into 'readBuf' 
            readDynFrameState = DONE_readDynFrame;
          }
          else
          {// syn byte not at the very beginning 
            readDynFrameState = getFrameNotFm0_readDynFrame;    //reload the valid frame from start of sync
            readBlockDatState = 0 ;
            //the variable 'searchIndxRdFrame' will keep the valid index of the frame
          }
          break;
        }
        searchIndxRdFrame++;
      }while(NumOfSyncStart >= searchIndxRdFrame);       //check out all the sync characters found in buffer
      //No valid frame found
      if(searchIndxRdFrame > searchIndxRdFrame) readDynFrameState = DONE_readDynFrame;
      break;
    }
    case getFrameNotFm0_readDynFrame:   //get the frame from offset
    {   //successfully get sync byte
      frameResult = readBlockDat((addr + eepromSyncChrPos[searchIndxRdFrame]), maxSizeEEpromFrame); //read the frame from the first byte of the frame into buffer again   
      if((readBlockDatState == 0xF1) || (readBlockDatState == 0xFE)) 
      {
        readBuf = frameResult;     //copy the pointer from internal to result pointer
        returnReadDynFrameValue = (addr + eepromSyncChrPos[searchIndxRdFrame]); //successfully return the first frame into 'readBuf' 
        readDynFrameState = DONE_readDynFrame;
      }
      break;
    }    
    case DONE_readDynFrame:
    {
       break;
    }
    case ERROR_readDynFrame:
    {   
        returnReadDynFrameValue =  -1;
        break;
    }   
    default:
    {
      readDynFrameState = ERROR_readDynFrame;
    }    
  }
  return returnReadDynFrameValue;
}
#pragma diag_default=Pe550

/** @brief   Erase the whole eeprom content
  * @details Read a block of data from eeprom
  * @param   addr : address to read from eeprom           
  *          len: length of data read
  * @return  the result pointer or if error return NULL
**/
enum {
  START_EEprom_EraseAll = 0,               //general start state for stateMachine function as 0x00
  blkClr_EEprom_EraseAll,
//  verify_EEprom_EraseAll,
  ERROR_EEprom_EraseAll = 0xFE,            //general state for ERROR as 0xFE
  DONE_EEprom_EraseAll = 0xF1              //general state for this stateMachine function finished as 0xF1
};
uint16_t eraseIndex = 0;

void EEprom_EraseAll(void)
{
  /** @caution if the eeprom page size is lager than 16 byte need to increase the zero with the same size**/
  uint8_t tmpryData[]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; 
  
  switch(EEprom_EraseAllState)
  {
    case START_EEprom_EraseAll:
    {
      eraseIndex = 0;
      I2C_WriteState = 0;
      EEprom_EraseAllState = blkClr_EEprom_EraseAll;
      break;
    }
    case blkClr_EEprom_EraseAll:
    {
      I2C_Write(eraseIndex * EEpromPageModeSize ,tmpryData , EEpromPageModeSize); 
      if((I2C_WriteState == 0xF1) || (I2C_WriteState == 0xFE))
      {
        eraseIndex++;
        I2C_WriteState = 0;
        if( eraseIndex >= (((fixedAreaStartingAddr + fixedAreaLen +1)/ EEpromPageModeSize)))
        {
            EEprom_EraseAllState = DONE_EEprom_EraseAll;            
        }
      }
      break;
    }

    case ERROR_EEprom_EraseAll:
    {
      break;
    }
    case DONE_EEprom_EraseAll:
    {
      break;
    }
    default:
    {
      EEprom_EraseAllState = ERROR_EEprom_EraseAll;
      break;
    } 
  }
}




enum {
  START_EEprom_RegisterUpdate = 0,               //general start state for stateMachine function as 0x00
  RdReg_EEprom_RegisterUpdate,
  WrReg_EEprom_RegisterUpdate,
  WrReg1_EEprom_RegisterUpdate,
  ERROR_EEprom_RegisterUpdate = 0xFE,            //general state for ERROR as 0xFE
  DONE_EEprom_RegisterUpdate = 0xF1              //general state for this stateMachine function finished as 0xF1
};
void EEprom_RegisterUpdate(uint16_t RegisterNum)
{
  static uint16_t newValue;
  switch(EEprom_RegisterUpdateState)
  {
    case START_EEprom_RegisterUpdate:
      {
        readBlockDatState = 0;
        EEprom_RegisterUpdateState = RdReg_EEprom_RegisterUpdate;
        break;
      }
    case RdReg_EEprom_RegisterUpdate:
      {
        tmpryDebugDat = readBlockDat(( tmpryRegisterNum *2) + fixedAreaStartingAddr , 2);
        if((readBlockDatState == 0xF1) || (readBlockDatState == 0xFE))
        {
          
          if(( newValue =(*((uint16_t*)tmpryDebugDat))+ dataloggerRxBuf[14]) <= 0xfff0)
          {
            I2C_WriteState = 0;
            EEprom_RegisterUpdateState = WrReg_EEprom_RegisterUpdate;
          }
        }
        break;
      }
    case WrReg_EEprom_RegisterUpdate:
      {
        I2C_Write(( tmpryRegisterNum *2) + fixedAreaStartingAddr, (uint8_t*)&newValue, 1);
        if((I2C_WriteState == 0xF1) || (I2C_WriteState == 0xFE))
        {
          I2C_WriteState = 0;
          newValue >>= 8;
          EEprom_RegisterUpdateState = WrReg1_EEprom_RegisterUpdate;
        }
        break;        
      }
    case WrReg1_EEprom_RegisterUpdate:
      {
        I2C_Write(( tmpryRegisterNum *2) + fixedAreaStartingAddr +1, (uint8_t*)&newValue, 1);
        if((I2C_WriteState == 0xF1) || (I2C_WriteState == 0xFE))
        {
          EEprom_RegisterUpdateState = DONE_EEprom_RegisterUpdate;
        }
        break;
      }  
    case ERROR_EEprom_RegisterUpdate:
      {
        break;
      }
    case DONE_EEprom_RegisterUpdate:
      {
        break;
      }
    default:
      {
        EEprom_RegisterUpdateState = ERROR_EEprom_RegisterUpdate;
      }
  } 
}