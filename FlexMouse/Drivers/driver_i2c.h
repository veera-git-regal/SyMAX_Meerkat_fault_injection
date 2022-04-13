/**
  ********************************************************************************************************************************
  * @file    driver_I2c.h 
  * @author  Pamela Lee
  * @version V1.0
  * @date    15-Mar-2021
  * @brief   Header of Driver function/s for serial protocol with I2C hardware
  * @details 
  ********************************************************************************************************************************
  */

/* Define to prevent recursive inclusion ---------------------------------------------------------------------------------------*/
#ifndef _DRV_I2C_H_
#define _DRV_I2C_H_

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "typedef.h"

#include "sequential_memory.h"
#include "structured_memory.h"
#include "scheduler.h"

/* Content ---------------------------------------------------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
  
  ////////////// define EEprom memory usage//////////////////
#define EEpromPageModeSize      16             //24AA16 byte per page mode size
#define fixedAreaStartingAddr   0x200
#define fixedAreaLen            0x5FF          //512 byte in fixed area
#define dynamicAreaStartingAddr 0x000          //dynamic area will be treat as a circular buffer
#define dynamicAreaLen          0x1FF          //1.5kbyte in dynamic area
#define dynamicAreaPtr          0x7FE          //dynamic area address pointer 
#define maxSizeEEpromFrame      0xff           //

/* User parameters -------------------------------------------------------------------------------------------------------------*/
#define SLAVE_OWN_ADDRESS       0xA0 
#define I2C_TX_RX_BUF_SIZE      0x40
  
/* Setup -----------------------------------------------------------------------------------------------------------------------*/
static Ring_Buf_Handle i2cSeqMemRX_u32;
static Ring_Buf_Handle i2cSeqMemTX_u32;
static Ram_Buf_Handle i2cStructMem_u32;
static uint8_t i2cBufIndx = 0;

/**
  ********************************************************************************************************************************
  * @brief   I2C Control (inside shared memory)
  * @details 
  * @param   i2cRX_u32 
  * @param   i2cTX_u32 
  * @param   errorCode_u8           Error code of this USART2 module.
  ********************************************************************************************************************************
  */
//******************* i2c Control (inside structured memory) *******************************************************************************************************************************  
typedef struct {
    Ring_Buf_Handle SeqMemRX_u32;
    Ring_Buf_Handle SeqMemTX_u32;
    uint8_t I2c_busy_u8;
    __IO uint8_t I2C_Competed;
    uint8_t* aTransmitBuffer;                   //max 16byte for 24AA16 can accept
    uint8_t* aReceiveBuffer;
    uint8_t*  i2cRdDat;                         //buffer size for max frame read from eeprom
    uint8_t errorCode_u8;
} I2c_Control;
//******************* end of i2c Control (inside structured memory) ******************************************************************************************************************************* 

/* Function Declarations -------------------------------------------------------------------------------------------------------*/
/**
  ********************************************************************************************************************************
  * @brief   I2C initialization function.
  * @details Configure peripheral clocks. Configure hardware TX/RX pins. Configure I2C interrupt.
  ********************************************************************************************************************************
  */
void MX_I2C1_Init(void);

/**
  ********************************************************************************************************************************
  * @brief   Returns 
  * @details
  * @return  Returns
  ********************************************************************************************************************************
  */
void I2C_ReadAddr(uint16_t readAddr);
uint8_t* I2C_ReadCurrentAddr(uint16_t readAddr, uint16_t NumOfBytes);

void Slave_Ready_To_Transmit_Callback(void);
void I2C_Slave_Reception_Callback(void);
void I2C_Slave_Complete_Callback(void);
void I2C_Transfer_Complete_Callback(void);
void I2C_Transfer_Error_Callback(void);
void I2C_Error_Callback(void);
uint8_t IsI2c_transferCompleted(void);

void I2C1_EV_IRQHandler(void);
void I2C1_ER_IRQHandler(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _DRV_I2C_H_ */