/**
  ********************************************************************************************************************************
  * @file    driver_I2c.c
  * @author  Pamela Lee
  * @version V1.0
  * @date    15-Mar-2021
  * @brief   Main Driver function/s for serial protocol with I2C hardware
  * @details 
  ********************************************************************************************************************************
  */

/* Includes --------------------------------------------------------------------------------------------------------------------*/
#include "driver_I2c.h"
#include "main.h"
#include "stm32f3xx_ll_i2c.h"
/* Private variables ---------------------------------------------------------*/
I2c_Control *i2cControl;

/************************************************************************************************************************************************************************/

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
void MX_I2C1_Init(void)
{
  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }
  LL_RCC_HSI_SetCalibTrimming(16);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_HSI);
  /**I2C1 GPIO Configuration  
  PB8   ------> I2C1_SCL
  PB9   ------> I2C1_SDA 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8|LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  /** I2C Initialization 
  */
  LL_I2C_EnableAutoEndMode(I2C1);
  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.Timing =  0x0000020B; //400k 0x2000090E;//100K   
  I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
  I2C_InitStruct.DigitalFilter = 4;
  I2C_InitStruct.OwnAddress1 = SLAVE_OWN_ADDRESS;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_SetOwnAddress2(I2C1, 0, LL_I2C_OWNADDRESS2_NOMASK);
  /* USER CODE BEGIN I2C1_Init 2 */
  
    /* Configure Event IT: Set priority for I2C1_EV_IRQn, Enable I2C1_EV_IRQn */
  NVIC_SetPriority(I2C1_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),4, 0));
  NVIC_EnableIRQ(I2C1_EV_IRQn);
  /* Configure Error IT: Set priority for I2C1_ER_IRQn, Enable I2C1_ER_IRQn */
//  NVIC_SetPriority(I2C1_ER_IRQn, 4);  
//  NVIC_EnableIRQ(I2C1_ER_IRQn);
 
  /* (5) Enable I2C1 **********************************************************/
  LL_I2C_Enable(I2C1);
 // LL_I2C_EnableDMAReq_TX(I2C1);

  /* (6) Enable I2C1 address match/error interrupts:
   *  - Enable Address Match Interrupt
   *  - Enable Not acknowledge received interrupt
   *  - Enable Error interrupts
   *  - Enable Stop interrupt
   */
  LL_I2C_EnableIT_ADDR(I2C1);
  LL_I2C_EnableIT_NACK(I2C1);
  LL_I2C_EnableIT_ERR(I2C1);
  LL_I2C_EnableIT_STOP(I2C1);
  LL_I2C_EnableIT_RX(I2C1);
  /* USER CODE END I2C1_Init 2 */
}

void I2C_ReadAddr(uint16_t readAddr)
{
  i2cControl->I2C_Competed = 0;
  LL_I2C_TransmitData8(I2C1, (uint8_t)(readAddr & 0xff));
  uint32_t effectiveAddr = ((readAddr >> 7)& 0xfE) | SLAVE_OWN_ADDRESS;
  LL_I2C_HandleTransfer(I2C1, effectiveAddr , LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE); 
}

uint8_t* I2C_ReadCurrentAddr(uint16_t readAddr, uint16_t NumOfBytes)
{
  i2cControl->I2C_Competed = 0;
  LL_I2C_TransmitData8(I2C1, (uint8_t)(readAddr & 0xff));
  uint32_t effectiveAddr = ((readAddr >> 7)& 0xfe) | SLAVE_OWN_ADDRESS;
  LL_I2C_HandleTransfer(I2C1, effectiveAddr, LL_I2C_ADDRSLAVE_7BIT, NumOfBytes, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);  
  return i2cControl->aReceiveBuffer;
}

void I2C_ReadEnd(void)
{
  
}

/********************************************************************************************************************************************/

uint8_t IsI2c_transferCompleted(void)
{
  return (i2cControl->I2C_Competed);
}

/**
  * @brief  Function called from I2C IRQ Handler when STOP flag is set
  *         Function is in charge of checking data received,
  *         LED2 is On if data are correct.
  * @param  None
  * @retval None
  */
void I2C_transfer_Complete_Callback(void)
{
  i2cControl->I2C_Competed = 1;
}

/**
  * @brief  Function called from I2C IRQ Handler when TXIS flag is set
  *         Function is in charge of transmit a byte on I2C lines.
  * @param  None
  * @retval None
  */
void Ready_To_Transmit_Callback(void)
{
  LL_I2C_TransmitData8(I2C1, *(i2cControl->aTransmitBuffer++));
}

/**
  * @brief  Function called in case of error detected in I2C IT Handler
  * @param  None
  * @retval None
  */
void I2C_Error_Callback(void)
{
}

/**
  * @brief  Function called from I2C IRQ Handler when RXNE flag is set
  *         Function is in charge of retrieving received byte on I2C lines.
  * @param  None
  * @retval None
  */
void I2C_Reception_Callback(void)
{
  /* Read character in Receive Data register.
  RXNE flag is cleared by reading data in RXDR register */
  *(i2cControl->aReceiveBuffer + i2cBufIndx++) = LL_I2C_ReceiveData8(I2C1);
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/
/**
  * Brief   This function handles I2C1 (Slave) event interrupt request.
  * Param   None
  * Retval  None
  */
void I2C1_EV_IRQHandler(void)
{
  /* Check STOP flag value in ISR register */
  if(LL_I2C_IsActiveFlag_STOP(I2C1))  //else if(LL_I2C_IsActiveFlag_STOP(I2C1))
  {
    /* End of Transfer */
    LL_I2C_ClearFlag_STOP(I2C1);   
    /* Call function transfer Complete Callback */
    LL_I2C_DisableIT_TX(I2C1);
    i2cBufIndx = 0;
  }
  
  /* Check TXIS flag value in ISR register */
  else if(LL_I2C_IsActiveFlag_TXIS(I2C1))
  {
    // Call function Ready to Transmit Callback //
    Ready_To_Transmit_Callback();
  }
  /* Check RXNE flag value in ISR register */
  else if(LL_I2C_IsActiveFlag_RXNE(I2C1))
  {
    /* Call function Reception Callback */
    I2C_Reception_Callback();
  }
  
}

void I2C1_ER_IRQHandler(void)
{
  I2C_Error_Callback();
}