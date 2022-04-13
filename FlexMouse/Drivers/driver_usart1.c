/**
  ********************************************************************************************************************************
  * @file    drv_usart1.c 
  * @author  Pamela Lee
  * @brief   Main Driver function/s for serial protocol with Usart1 hardware
  * @details Protocol Usart1, after decode a whole valid frame from serial port2,
  *          trigger the system control to execute the relative APP in the int stage
  *          the Rx data is in usart1SeqMemRX_u32.
  *          To Transmitt data : put data into usart1SeqMemTX_u32, and call this function
  *                              USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
  ********************************************************************************************************************************
  */

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "driver_usart1.h"
#include "module_AutoAck.h"

#include "main.h"
#include "ui_task.h" // !error TODO: Remove, only for Regal_SendDebugMessage testing

/* Private variables ---------------------------------------------------------*/
//UART_HandleTypeDef huart2; //SPA

Usart1_Control *usart1Control;

AutoAck_Control *autoAckControl_u32;             

__IO ITStatus UartReady = RESET;
uint8_t usart1CaptureLen;                     //default Universal Protocol header length
//uint8_t counter = 0;
uint16_t uwCRCValue = 0;

static uint8_t dataLen = 0;                     //length of data byte/s plus CRC expected
uint8_t UniProtocolState = protocolstart;
//unsigned char protocolBuf[100];                 //for decoding the protocol, assume max size of a universal frame is 64+7+2 = 73 with some left-behind frame from previous, therefore 100 bytes
//unsigned char headerFramebuf[80];               //this buffer can only store up to 73 +7 = 80 and constrianed by the code using this buffer
unsigned char wholeFramebuf[100];               //this buffer for usart TX data input 
/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART1_UART_Init(void)
{

   /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**USART1 GPIO Configuration
  PB6   ------> USART1_TX
  PB7   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);



  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_DisableIT_CTS(USART1);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);

  /* USER CODE BEGIN WKUPType USART1 */

  /* USER CODE END WKUPType USART1 */
  LL_USART_ClearFlag_ORE(USART1);               //reset all usart error bit
  /* Polling USART1 initialisation */
 while((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1))))
 {
 }
  LL_USART_EnableIT_RXNE(USART1);
  /* USER CODE BEGIN USART1_Init 2 */


  /* USER CODE END USART1_Init 2 */

}


void usart1_Init(){
  MX_USART1_UART_Init();
}


/**
  * @brief  Function called from USART IRQ Handler when RXNE flag is set
  *         Function is in charge of reading character received on USART RX line.
  * @param  None
  * @retval None
  */
void USART1_CharReception_Callback(void)
{
  /*
  __IO uint32_t received_char;

  // Read Received character. RXNE flag is cleared by reading of RDR register //
  received_char = LL_USART_ReceiveData8(USART1);
  */
 
  unsigned char rxDat2;
  rxDat2 = LL_USART_ReceiveData8(USART1);

  RingBuf_WriteCharacter((*usart1Control).seqMem_InternalPipe_u32,&rxDat2);
  //RBWrite(usart1InternalSeqMem_u32->systemInstanceIndex_u8,&rxDat2); 
  
  /* Echo received character on TX */
 // LL_USART_TransmitData8(USART1, received_char);
}

/******************* variable for Usart1 TX interrupt **************************/
__IO uint8_t indexTx = 0;
uint8_t ubSizeToSend = 0;

#define ENABLE_WHOLEFRAME_BUF_FIXED_LEN 1
#if ENABLE_WHOLEFRAME_BUF_FIXED_LEN >= 1
  // This is a one-shot buffer, that is written to and read from in single calls.
  // - it does not currently need to be tracked for current index because of this.
  #define FIXED_WHOLEFRAMEBUF_MAX_LENGTH TX_RX_BUF_SIZE // Inclusive (this value is accepted) 
  unsigned char fixedWholeFrameBuf_Length = 0;
  unsigned char fixedWholeFrameBuf[FIXED_WHOLEFRAMEBUF_MAX_LENGTH];
 // unsigned char* wholeFramebuf = fixedWholeFrameBuf;
#else // if ENABLE_WHOLEFRAME_BUF_FIXED_LEN <= 0
  unsigned char* wholeFramebuf;
#endif

/*******************************************************************************/
/**
  * @brief  Function called for achieving next TX Byte sending
  * @param  None
  * @retval None
  */
void USART1_TXEmpty_Callback(void)
{
  /* Fill TDR with a new char */
  LL_USART_TransmitData8(USART1, wholeFramebuf[indexTx++]);
  
  if (indexTx == (ubSizeToSend )) // REVIEW: Potential Crash Point if Interrupt is triggered while Buffer is being filled.
  {
    /* Disable TXE interrupt */
    LL_USART_DisableIT_TXE(USART1);
    indexTx = 0;
  } 
}

/**
  * @brief  Function called at completion of last byte transmission
  * @param  None
  * @retval None
  */
void USART1_CharTransmitComplete_Callback(void)
{
//  if (indexTx == sizeof(ubSizeToSend))
//  {
//    indexTx = 0;

    /* Disable TC interrupt */
//    LL_USART_DisableIT_TC(USART1);
//  }
}


//static uint8_t dataLen = 0;                                         //length of data byte/s plus CRC expected
//uint8_t UniProtocolState = protocolstart;

#define ENABLE_PROTOCOLBUF_FIXED_LEN 1
#if ENABLE_PROTOCOLBUF_FIXED_LEN >= 1
  // This is a one-shot buffer, that is written to and read from in single calls.
  // - it does not currently need to be tracked for current index because of this.
  #define FIXED_PROTOCOLBUF_MAX_LENGTH TX_RX_BUF_SIZE // Inclusive (this value is accepted) 
  unsigned char fixedProtocolBuf_Length = 0;
  unsigned char fixedProtocolBuf[FIXED_PROTOCOLBUF_MAX_LENGTH];
  unsigned char* protocolBuf = fixedProtocolBuf;
#else
  unsigned char* protocolBuf;
#endif



/**
  *@brief   Function for decode a valid frame of universal protocol
  *         the Rx data will put into an internal pipe and process after 
  *         the first 5 byte contain sync byte and then wait for the whole header 
  *         bytes in side the internal pipe, then perform the header veification.
  *         If the the header bytes are valid, then prepare for getting the whole data fields 
  *         and perform CRC check.
  * @param  None
  * @retval None
  */
void protocolHeaderfetch(void)
{
  switch(UniProtocolState)
  {
    case  protocolstart:
     Reentryprotocalstart: 
      { // assume this is very beginning of universal-frame, check and clear any garble before sync byte
        usart1CaptureLen = UniHeaderlen; 
        dataLen = 0;
        unsigned int SyncPosition = RingBuf_Search((*usart1Control).seqMem_InternalPipe_u32, RxSyncChr, 0);              //search sync byte within internal pipe,[return position or 0xf000 == not found 0xffff == error]
        if (SyncPosition != 0) // Sync Position is not at start of packet.
        {
          // Clear All Bytes before the sync byte. Message can be parsed on the next pass.
          // - If no sync byte was found, clear a predetermined number of bytes
          if(SyncPosition & 0xf000)                                                            
          { //sync byte not found or error, then clear the first 5 bytes
            SyncPosition = UniHeaderlen;                                                                // this will clear all 5 byte                                       
          }
          #if ENABLE_PROTOCOLBUF_FIXED_LEN >= 1
              // Clear Pre-Sync Bytes from the buffer by 'Read'ing them from the RingBuf
              fixedProtocolBuf_Length = SyncPosition;
              RingBuf_ReadBlock((*usart1Control).seqMem_InternalPipe_u32, protocolBuf, &SyncPosition);                          //truncate any char before SYNC char 
              // Clear Message Parsing Data and Exit Routine
              usart1CaptureLen = UniHeaderlen; 
              dataLen = 0;
              UniProtocolState = protocolstart;
              break;
          #else // ENABLE_PROTOCOLBUF_FIXED_LEN == 0
            if((protocolBuf = (unsigned char*) realloc(protocolBuf,SyncPosition)) == NULL) reallocError++;
            RingBuf_ReadBlock((*usart1Control).seqMem_InternalPipe_u32, protocolBuf, &SyncPosition);                          //truncate any char before SYNC char 
            if((protocolBuf = (unsigned char*)realloc(protocolBuf,1)) == NULL) reallocError++;
            usart1CaptureLen = UniHeaderlen; 
            UniProtocolState = protocolstart;                                                             // still to back here next stage for complete header frame 
            break;
          #endif // ENABLE_PROTOCOLBUF_FIXED_LEN == 0
        }
        // otherwise mean the the first byte in buffer is sync byte, then can directly process to header Validate state !!!!!!!!!
        UniProtocolState = protocolstart;
      }
    case headerValidate:
      { //check this is a valid header frame in internal pipe
        unsigned char headerBuf[]={ 0, 0, 0, 0, 0, 0, 0, 0, 0};                 //9 byte array  for header including advanced CMD 
        unsigned int headerLen = 9;     
        RingBuf_Observe((*usart1Control).seqMem_InternalPipe_u32, headerBuf, 0, &headerLen);      //get the pipe data without clear internal pipe, headerLen will return number of bytes in pipe if less than 9 byte

        uint8_t dataA ;
        uint8_t headerCRCLen = 7;                                               //default normal frame header + CRC bytes = 7 byte
        uint8_t dataC = (uint8_t) headerBuf[1] & 0xC0;
        if (dataC == 0)                                                         //check this is advanced frame [00]+[data length]
        {
          dataA = (uint8_t)headerBuf[4] & 0xf0 ;                                //this is normal frame, store dataA as the expected Header-end in byte 5
        }
        else
        {  //this is advanced frame [11]+[data length]   or  garbage 
          if (dataC == 0xC0)
          { //this is advanced frame 
            dataA = (uint8_t)headerBuf[6] & 0xf0 ;                              //this is Advanced frame,  store dataA as the expected Header-end in byte 7 
            headerCRCLen = 9;                                                   //header + CRC bytes = 9 byte
          }
          else
          { //this is garbage, because the frame type pointer not valid
            unsigned char tmpbuf3;
            unsigned int truncateLen = 1;   
            RingBuf_ReadBlock((*usart1Control).seqMem_InternalPipe_u32, &tmpbuf3, &truncateLen);                             //truncate the first byte incase the garbage is the same of sync byte
            usart1CaptureLen = UniHeaderlen; 
            UniProtocolState = protocolstart;    
            if( *RingBuf_GetPointerToRingBuf((*usart1Control).seqMem_InternalPipe_u32) >= usart1CaptureLen)                     //after truncase the carbage byte still contain more then header length then goto back to do again
            {
              goto Reentryprotocalstart;                                                                 //this line needed to use goto for finish the frame check in one go
            }
            break;   
          }
        }
        if(dataA == (~((uint8_t)RxSyncChr) & 0xf0))                                  //check header-end byte is valid
        {// this is a valid header        
          dataLen = (uint8_t) headerBuf[1] & 0x3F;     
          usart1CaptureLen = (uint8_t)(dataLen + headerCRCLen) ; // header+ CRC +data length = number of bytes for next capture
          UniProtocolState = frameCRC;
          break;
        }
        else
        {
         // if the header frame is not valid, then delete the first byte only
          unsigned char tmpbuf3;
          unsigned int truncateLen = 1;   
          RingBuf_ReadBlock((*usart1Control).seqMem_InternalPipe_u32, &tmpbuf3, &truncateLen);                             //truncate the first byte incase the garbage is the same of sync byte
          usart1CaptureLen = UniHeaderlen; 
          UniProtocolState = protocolstart;    
          if( *RingBuf_GetPointerToRingBuf((*usart1Control).seqMem_InternalPipe_u32) >= usart1CaptureLen)                     //after truncase the carbage byte still contain more then header length then goto back to do again
          {
            goto Reentryprotocalstart;                  //this line needed to use goto for finish the frame check in one go
          }
          break;
        }
      }
    case frameCRC:
      {
        #if ENABLE_PROTOCOLBUF_FIXED_LEN >= 1
          // Clear Pre-Sync Bytes from the buffer by 'Read'ing them from the RingBuf
          if (usart1CaptureLen <= FIXED_PROTOCOLBUF_MAX_LENGTH) { // Normal Case
            fixedProtocolBuf_Length = usart1CaptureLen;
            // REVIEW: usart1CaptureLen is calculated from Data Length (and not where the CRC was)
          } else { // Overflow Case
            // Read All Data (Clear the Buffer)
            while (usart1CaptureLen > 0) {
              if (usart1CaptureLen > FIXED_PROTOCOLBUF_MAX_LENGTH) {
                // REVIEW: Replace with RingBuf_ClearContents? Much less processing
                unsigned int read_length = FIXED_PROTOCOLBUF_MAX_LENGTH;
                RingBuf_ReadBlock((*usart1Control).seqMem_InternalPipe_u32, protocolBuf, &read_length); //extract the whole frame
                // RingBuf_ReadBlock((*usart1Control).seqMemTX_u32, headerFramebuf, &read_length);             //copy the complete frame into buffer
                usart1CaptureLen -= FIXED_PROTOCOLBUF_MAX_LENGTH;
              } else {
                unsigned int read_length = usart1CaptureLen;
                RingBuf_ReadBlock((*usart1Control).seqMem_InternalPipe_u32, protocolBuf, &read_length); //extract the whole frame
                // RingBuf_ReadBlock((*usart1Control).seqMemTX_u32, headerFramebuf, &DataLen2);             //copy the complete frame into buffer
                usart1CaptureLen = 0;
              }
            }
            // Clear Message Parsing Data and Exit Routine
            usart1CaptureLen = UniHeaderlen; 
            dataLen = 0;
            UniProtocolState = protocolstart;
            break;
          }
        #else // ENABLE_PROTOCOLBUF_FIXED_LEN == 0
          if((protocolBuf = (unsigned char*) realloc(protocolBuf,usart1CaptureLen)) == NULL) reallocError++;        
        #endif

        unsigned int DataLen2 = (unsigned int)usart1CaptureLen;
        RingBuf_Observe((*usart1Control).seqMem_InternalPipe_u32, protocolBuf, 0, &DataLen2);                             //copy the whole frame into buffer
        uint16_t frameCRC = (((uint16_t)protocolBuf[DataLen2 - 2]) << 8) + ((uint16_t)protocolBuf[DataLen2 - 1]) ;
        
        uwCRCValue = Calculate_CRC((DataLen2 - 2) , protocolBuf);                                       //Get calculated CRC of this frame
        if(uwCRCValue == frameCRC)
        { //CRC match put whole frame into RX pipe      
          //decode from the CMD of group(1-2, 3 or 4 )
         // RingBuf_WriteBlock((*usart1Control).seqMemRX_u32, protocolBuf, &DataLen2);                                //put fully checked (Valid) Rx frame into Rx Pipe
          if((protocolBuf[3]) && (protocolBuf[2] != 0x3f))                                              //if auto ACK
          {
            unsigned char ackTx[] = {0x55, 0x00, 0x3F, protocolBuf[3], 0x00, 0xCC, 0xCC};
            unsigned int TxLen = sizeof(ackTx);
            RingBuf_WriteBlock((*usart1Control).seqMemTX_u32, ackTx, &TxLen); 
          }          
          // RPa: to signal that there is valid packet
          Set_ValidRx();
          
          switch(protocolBuf[2] & 0xf0)
          {     //groups decoding  please refer to the document of Universal protocol 
            case 0x00:          //group 1
            case 0x10:          //group 1
            case 0x20:          //group 2
            case 0x30:          //group 2
            case 0xF0:          //group 15: 0xFB is Short Command (Utility Category), TODO: 0xFC-0xFF are 'Advanced Frames'
              {    
                if(protocolBuf[2] == 0x3f)
                {
                  //Auto-Ack feedback from receiver
                  AckDeRegistered((uint8_t)protocolBuf[3]);  /** Pam!!!! this is the only function linking from outside driver_Usart1.c can be replace by using structured-memory for loose coupling **/ 
 //                 (*autoAckControl_u32).RxAckFrameID = (uint8_t)protocolBuf[3];
                }
                else
                {       //group1_2 as module_ShortCmd
                  RingBuf_WriteBlock((*usart1Control).seqMemRXG1_2_u32, protocolBuf, &DataLen2);                                //put fully checked (Valid) Rx frame into Rx Pipe
                }
                break;
              }
            case 0x40:          //group 3
            case 0x50:          //group 3
            case 0x60:            //group 3
              {         //group3 module_ReplyCMD
                RingBuf_WriteBlock((*usart1Control).seqMemRXG3_u32, protocolBuf, &DataLen2);                                //put fully checked (Valid) Rx frame into Rx Pipe
                break;
              }
          
            case 0x70:            //group 4
              {        //group 4 module_FlashCMD 
                if(protocolBuf[2] & 0x08)
                {
                   RingBuf_WriteBlock((*usart1Control).seqMemRXG4H_u32, protocolBuf, &DataLen2);                                //put fully checked (Valid) Rx frame into Rx Pipe
                }
                else
                {
                  RingBuf_WriteBlock((*usart1Control).seqMemRXG4L_u32, protocolBuf, &DataLen2);                                //put fully checked (Valid) Rx frame into Rx Pipe
                }
                break;
              }
            default:              // the rest will be truncase
              {   
                break;
              }
          }
          
          /*********** this part only for testing the Tx message *************/
          /*
          //Echo back the full valid frame to sender
          protocolBuf[DataLen2 - 2] = 0xCC;                                                             //Tx frame always use 0xCC as CRC byte, the Tx sending routine should process it with the final CRC value
          protocolBuf[DataLen2 - 1] = 0xCC;                                                             //Tx frame always use 0xCC as CRC byte, the Tx sending routine should process it with the final CRC value
          protocolBuf[0] = 0x55;                                                                           //Tx frame always use 0x55 as Sync, the Tx sending routine should process it for Master or Slave
          if(protocolBuf[1] & 0xC0)
          {//Advanced frame
            protocolBuf[5] = 0x00;                                                                         //Tx frame always use 0xFF as Source Frame ID for ACK require, and 0x00 as non-ACK frame, the Tx sending routine should process with the value for ACK frame        
            protocolBuf[6] = 0x01;                                                                         //Tx frame slways put motor address to lower nibble, and leave upper nibble as 0 for Tx routine to put the Header-end value to this field
          }
          else
          {//Normal frame
            protocolBuf[3] = 0x00;                                                                         //Tx frame always use 0xFF as Source Frame ID for ACK require, and 0x00 as non-ACK frame, the Tx sending routine should process with the value for ACK frame        
            protocolBuf[4] = 0x01;                                                                         //Tx frame slways put motor address to lower nibble, and leave upper nibble as 0 for Tx routine to put the Header-end value to this field
          }
          RingBuf_WriteBlock(usart1SeqMemTX_u32->systemInstanceIndex_u8, protocolBuf, &DataLen2);        
*/
          /*********** this part only for testing the Tx message End End End *************/
        }                                
        else
        { 
          DataLen2 = UniHeaderlen;                                                                   //only truncase the header         
        }                                                                  

        RingBuf_ReadBlock((*usart1Control).seqMem_InternalPipe_u32, protocolBuf, &DataLen2);                              //extract the whole frame  
#if ENABLE_DEBUG_OUTPUT_VIA_STMC_PORT >= 1
        Regal_SendDebugMessage(protocolBuf, DataLen2);
#endif
        #if ENABLE_PROTOCOLBUF_FIXED_LEN <= 0
        if((protocolBuf = (unsigned char*) realloc(protocolBuf,1)) == NULL) reallocError++;                                          //free heap only leave 1 byte 
        #endif // ENABLE_PROTOCOLBUF_FIXED_LEN <= 0
                
        usart1CaptureLen = UniHeaderlen;
        UniProtocolState = protocolstart;
        break;
      }

    default:
      {
        usart1CaptureLen = UniHeaderlen; 
        dataLen = 0;
        UniProtocolState = protocolstart;
        break;
      } 
    
  }

}


/**
  *@brief   Universal protocol transmit
  * @ usage  To send a frame out user should setup a frame buffer
  *          always start with a 0x55 (no matter Master or slave) 
  *            |    count how many data length
  *            |     |   Command of this frame                 
  *            |     |     |   FrameID as 0x00 = non-ack frame or 0xff = auto ack frame
  *            |     |     |     |    put the motor addres for the low 4bits and always 0x0 for the upper 4 bits 
  *            |     |     |     |     |   data for this frame or omit these two byte if no data
  *            |     |     |     |     |     |     |   if this is Auto ack frame put the module ID here, otherwise always leave it as 0xCC
  *            |     |     |     |     |     |     |     |    Always leave as 0xCC
  *            |     |     |     |     |     |     |     |     |
  *          0x55, 0x02, 0x40, 0x00, 0x00, 0xff, 0xff, 0xCC, 0xCC
  *          Then put them into Tx sequential-memory of Usart1, the system will add all the correct sync byte and CRC.
  *          User can put as many frame as the Tx sequential-memory is not full, so please check the Tx sequential-memory status before append new frame.
  * @param  None
  * @retval None
  */

#define ENABLE_HEADERFRAME_BUF_FIXED_LEN 1
#if ENABLE_HEADERFRAME_BUF_FIXED_LEN >= 1
  // This is a one-shot buffer, that is written to and read from in single calls.
  // - it does not currently need to be tracked for current index because of this.
  // #define FIXED_HEADERFRAMEBUF_MAX_LENGTH 21 // Inclusive (this value is accepted) 
  #define FIXED_HEADERFRAMEBUF_MAX_LENGTH TX_RX_BUF_SIZE // Inclusive (this value is accepted) 
  unsigned char fixedHeaderFrameBuf_Length = 0;
  unsigned char fixedHeaderFrameBuf[FIXED_PROTOCOLBUF_MAX_LENGTH];
  unsigned char* headerFramebuf = fixedHeaderFrameBuf;
#else
  unsigned char* headerFramebuf;
#endif


uint8_t TxProcess(void)
{ //process Tx frame in Tx pipe
  int txHeaderSize = UniHeaderlen;
  uint8_t currentFrameID = 0;
  #if ENABLE_HEADERFRAME_BUF_FIXED_LEN >= 1
    fixedHeaderFrameBuf_Length = txHeaderSize;
  #else // if ENABLE_HEADERFRAME_BUF_FIXED_LEN <= 0
    if((headerFramebuf = (unsigned char*) realloc(headerFramebuf, txHeaderSize)) == NULL) reallocError++;
  #endif
  unsigned int DataLenTx = (unsigned int)txHeaderSize;
  if(indexTx == 0)
  {
    unsigned int HeaderSize = 0;
    if(RingBuf_GetUsedNumOfElements((*usart1Control).seqMemTX_u32) >= txHeaderSize)
    {
      RingBuf_Observe((*usart1Control).seqMemTX_u32, headerFramebuf, 0, &DataLenTx);  // copy the whole header frame into buffer
      if(headerFramebuf[1] & 0xc0) 
      {
        HeaderSize = 9; //Advanced frame +CRC
      }
      else
      {
        HeaderSize = 7; //normal frame +CRC                                                 //put back the lastest frameID into the Tx frame 
      }
      DataLenTx = HeaderSize + ((unsigned int)(headerFramebuf[1] & 0x3F)); 
      if(headerFramebuf[HeaderSize - 4]) 
      { /**check this is auto-ack frame  **/
        if(!IsAckBufFull())
        {
          #if ENABLE_HEADERFRAME_BUF_FIXED_LEN >= 1
            fixedHeaderFrameBuf_Length = DataLenTx;
            if (DataLenTx <= FIXED_HEADERFRAMEBUF_MAX_LENGTH) { // Normal Case: Prepare Message for Send
              RingBuf_Observe((*usart1Control).seqMemTX_u32, headerFramebuf, 0, &DataLenTx);                      //get the whole frame to capture the second last byte of module ID data in the frame
              currentFrameID = AckDatSet((uint8_t)headerFramebuf[2], 0, (uint8_t)headerFramebuf[DataLenTx - 2]);  //put the current Auto Ack frame data to UniversalAckInfo for timout or auto dismiss  
            } else { // Overflow Case: Discard All Data
              // Overflow leads to unknown data state, don't send any of it
              // - Read all data into the temporary header frame buf, so that it is clearly discaraded.
              while (DataLenTx > 0) {
                if (DataLenTx > FIXED_HEADERFRAMEBUF_MAX_LENGTH) {
                  unsigned int read_length = FIXED_HEADERFRAMEBUF_MAX_LENGTH;
                  RingBuf_ReadBlock((*usart1Control).seqMemTX_u32, headerFramebuf, &read_length);             //copy the complete frame into buffer
                  DataLenTx -= FIXED_HEADERFRAMEBUF_MAX_LENGTH;
                } else {
                  RingBuf_ReadBlock((*usart1Control).seqMemTX_u32, headerFramebuf, &DataLenTx);             //copy the complete frame into buffer
                  DataLenTx = 0;
                }
              }
              // Now Exit without sending
              return 1;
            }
          #else // if ENABLE_HEADERFRAME_BUF_FIXED_LEN <= 0
            if((headerFramebuf = (unsigned char*) realloc(headerFramebuf, DataLenTx)) == NULL) reallocError++;       
            RingBuf_Observe((*usart1Control).seqMemTX_u32, headerFramebuf, 0, &DataLenTx);                      //get the whole frame to capture the second last byte of module ID data in the frame
            currentFrameID = AckDatSet((uint8_t)headerFramebuf[2], 0, (uint8_t)headerFramebuf[DataLenTx - 2]);  //put the current Auto Ack frame data to UniversalAckInfo for timout or auto dismiss  
          #endif
        }
        else
        { 
          //if this is auto ack frame but autoACk UniversalAckInfo is full
          //then wait until the buffer clear 
          // -- especially since seqMemTX_u32 is never cleared in this case...
          /**!!!!!!!!!!!!!!!!!! The communication may slow down alot if completely no reply by the other end, need to be carefull !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!**/
          
          #if ENABLE_HEADERFRAME_BUF_FIXED_LEN >= 1
            // REVIEW: This seems like a potential crash point for Tx, since AckBuf might get stuck full.
            return 1;               //Tx buffer full can't accept anymore frame
          #else // if ENABLE_HEADERFRAME_BUF_FIXED_LEN <= 0
            if((headerFramebuf = (unsigned char*) realloc(headerFramebuf, 1)) == NULL) reallocError++;                            //free heap only leave 1 byte 
            return 1;               //Tx buffer full can't accept anymore frame
          #endif 
        }
      }

      // REVIEW: in Fixed Ram Scenario, wholeFrameBuf is just a 'Read' (instead of observed) duplicate of headerFramebuf
      // - Can we replace wholeFramebuf by just using headerFramebuf here?
      #if ENABLE_HEADERFRAME_BUF_FIXED_LEN <= 0
        if((headerFramebuf = (unsigned char*) realloc(headerFramebuf, 1)) == NULL) reallocError++;                            //free heap only leave 1 byte 
      #endif // if ENABLE_HEADERFRAME_BUF_FIXED_LEN <= 0
      #if ENABLE_WHOLEFRAME_BUF_FIXED_LEN <= 0
        if((wholeFramebuf = (unsigned char*) realloc(wholeFramebuf, DataLenTx)) == NULL) reallocError++;       
      #endif // ENABLE_WHOLEFRAME_BUF_FIXED_LEN <= 0

      // REVIEW: If wholeFramebuf transmit is in progress, and we write to wholeFrameBuf, the transmission will be disrupted.
      // - current response is to overwrite the original message mid-transit.
      
      // Note: Here we do not need to check length here because length was checked by headerFramebuf, and wholeFrameBuf must be as big as headerFramebuf
      RingBuf_ReadBlock((*usart1Control).seqMemTX_u32, wholeFramebuf, &DataLenTx);             //copy the complete frame into buffer (clears seqMemTX)
      wholeFramebuf[HeaderSize - 3] = (~TxSyncChr & 0xf0) + (wholeFramebuf[HeaderSize - 3] & 0xf); 
      wholeFramebuf[HeaderSize - 4] = currentFrameID; 
    }   
    wholeFramebuf[0] = TxSyncChr;
    uwCRCValue = Calculate_CRC((DataLenTx - 2) , wholeFramebuf);    
    wholeFramebuf[DataLenTx - 2] = (unsigned char)((uwCRCValue & 0xff00) >> 8);                  //put calculated CRC back into Tx frame
    wholeFramebuf[DataLenTx - 1] = (unsigned char)(uwCRCValue & 0xff) ;                          //put calculated CRC back into Tx frame  
  }    
  
  
  if((!indexTx) && (LL_USART_IsActiveFlag_TXE(USART1)))
  {
    // REVIEW: Another Potential Crash-Point for Tx if indexTx gets stuck above 0.
    ubSizeToSend = DataLenTx;      //set TX length                                                      
    /* Start USART transmission : Will initiate TXE interrupt after TDR register is empty */
    LL_USART_TransmitData8(USART1, wholeFramebuf[indexTx++]);                                       //put buffer in
    /* Enable TXE interrupt */
    LL_USART_EnableIT_TXE(USART1);
  }
  /**********************for TX interrupt still using this variable , so don't free it!!!!!!!*******/
  //wholeFramebuf = (unsigned char*) malloc(1);   //for 
  return 0;
}




/**
  * @brief  Function called in case of error detected in USART IT Handler
  * @param  None
  * @retval None
  */
void Error_Callback(void)
{
  __IO uint32_t isr_reg;

  /* Disable USARTx_IRQn */
  NVIC_DisableIRQ(USART1_IRQn);

  /* Error handling example :
    - Read USART ISR register to identify flag that leads to IT raising
    - Perform corresponding error handling treatment according to flag
  */
  isr_reg = LL_USART_ReadReg(USART1, ISR);
  if (isr_reg & LL_USART_ISR_NE)
  {
    /* case Noise Error flag is raised : ... */
//    LED_Blinking(LED_BLINK_FAST);
  }
  else
  {
    /* Unexpected IT source : Set LED to Blinking mode to indicate error occurs */
//    LED_Blinking(LED_BLINK_ERROR);
  }
}

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 26.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  /* Check RXNE flag value in ISR register */
  if (LL_USART_IsActiveFlag_RXNE(USART1) && LL_USART_IsEnabledIT_RXNE(USART1))
  {
    /* RXNE flag will be cleared by reading of RDR register (done in call) */
    /* Call function in charge of handling Character reception */
    USART1_CharReception_Callback();
  }
  
  if (LL_USART_IsEnabledIT_TXE(USART1) && LL_USART_IsActiveFlag_TXE(USART1))
  {
    /* TXE flag will be automatically cleared when writing new data in TDR register */

    /* Call function in charge of handling empty DR => will lead to transmission of next character */
    USART1_TXEmpty_Callback();
  }

  if (LL_USART_IsEnabledIT_TC(USART1) && LL_USART_IsActiveFlag_TC(USART1))
  {
    /* Clear TC flag */
    LL_USART_ClearFlag_TC(USART1);
    /* Call function in charge of handling end of transmission of sent character
       and prepare next charcater transmission */
    USART1_CharTransmitComplete_Callback();
  }
  
  if (LL_USART_IsEnabledIT_ERROR(USART1) && LL_USART_IsActiveFlag_NE(USART1))
  {
    /* Call Error function */
    Error_Callback();
  }
  /* USER CODE END USART1_IRQn 0 */
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}


